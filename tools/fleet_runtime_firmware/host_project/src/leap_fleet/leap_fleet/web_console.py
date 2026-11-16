"""Local operator console: telemetry and finite velocity runs, no planner."""

import argparse
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
from functools import partial
import json
import math
from pathlib import Path
import secrets
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .config import default_config, load_cars
from .control import Fleet, Velocity
from .motion_channels import MotionChannels
from .trajectory_console import MAX_CSV_BYTES, TrajectoryConsoleMixin


ACTIVE = {'starting', 'running', 'stopping'}


class UnavailableConnections:
    def __init__(self, error):
        self.error = error

    def snapshot(self):
        return {'available': False, 'error': self.error, 'cars': []}

    def request(self, action, cars):
        raise RuntimeError(self.error)

    def busy_for(self, cars):
        return False

    def close(self):
        pass


def number(value, label, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} 必须是有限数字')
    if low is not None and value < low or high is not None and value > high:
        raise ValueError(f'{label} 必须在 {low}～{high} 范围内')
    return float(value)


class Console(TrajectoryConsoleMixin):
    DISCOVERY_TIMEOUT = 5.0
    STOP_TIMEOUT = 3.0
    FEEDBACK_MAX_AGE = 0.3
    STOP_STABLE_TIME = 0.2
    STOP_MIN_SAMPLES = 3
    STOP_V_TOLERANCE = 0.005
    STOP_W_TOLERANCE = 0.02

    def __init__(self, monitor, fleet_factory, *, mode='live', clock=None, connections=None,
                 channels=None, firmware_logger=None):
        self.monitor, self.fleet_factory, self.mode = monitor, fleet_factory, mode
        self.connections = connections
        self.firmware_logger = firmware_logger
        self.clock = clock or time.monotonic
        self.channels = channels if channels is not None else MotionChannels(fleet_factory)
        self._lock = threading.RLock()
        self._job = None
        self._worker = None
        self._cancel = threading.Event()
        self._closed = False
        self._unconfirmed = set()
        self._connection_transition = set()
        self._events = deque(maxlen=100)
        self._init_trajectories()
        self._event('info', '演示模式已就绪，不会连接小车' if mode == 'demo'
                    else '本机控制台已就绪，正在只读检查车辆连接')

    def _event(self, level, message, details=None):
        with self._lock:
            event = {'time': datetime.now(timezone.utc).isoformat(),
                     'level': level, 'message': message}
            if details is not None:
                event['details'] = deepcopy(details)
            self._events.appendleft(event)

    def _job_view(self):
        if self._job is None:
            return None
        job = {key: deepcopy(value) for key, value in self._job.items() if not key.startswith('_')}
        deadline = self._job.get('_deadline')
        job['remaining_s'] = (max(0, deadline - self.clock())
                              if deadline is not None and job['status'] in ACTIVE else 0)
        if job['status'] == 'starting':
            job['remaining_s'] = job['duration']
        return job

    def snapshot(self):
        cars = self.monitor.snapshot()
        with self._lock:
            return {'mode': self.mode, 'cars': cars, 'job': self._job_view(),
                    'connections': self.connections.snapshot() if self.connections else None,
                    'events': list(self._events), 'server_time': time.time(),
                    'motion_channels': self.channels.snapshot(),
                    'trajectory': self.trajectory_view(points=False),
                    'stop_unconfirmed': sorted(self._unconfirmed)}

    def connection_action(self, action, cars):
        """Reserve connection work under the same lock used to start motion."""
        if not isinstance(action, str) or action not in ('check', 'connect', 'configure', 'cancel'):
            raise ValueError('未知连接操作')
        if (not isinstance(cars, list) or not cars or
                any(not isinstance(name, str) for name in cars) or len(cars) != len(set(cars))):
            raise ValueError('cars 必须是非空且不重复的车辆编号列表')
        with self._lock:
            if self._closed:
                raise RuntimeError('控制台已关闭')
            if self.connections is None:
                raise RuntimeError('当前服务未启用连接管理，请重启更新后的网页服务')
            if action != 'check' and self._busy():
                raise RuntimeError('运动任务尚未结束，请先停止本次任务，再更改底盘连接')
            if set(cars).intersection(self._connection_transition):
                raise RuntimeError('正在释放车辆控制连接，请等待处理完成')
            if action != 'check':
                self._connection_transition.update(cars)
        try:
            if action in ('configure', 'cancel'):
                # Release idle writers in their own threads before the existing
                # connection guard checks for competing command publishers.
                self.channels.release(cars, timeout=1.0)
                deadline = self.clock() + 1.0
                while self.clock() < deadline:
                    states = {car['id']: car for car in self.monitor.snapshot()}
                    if all(states.get(name, {}).get('cmd_publishers', 0) == 0 for name in cars):
                        break
                    time.sleep(0.02)
            with self._lock:
                if self._closed:
                    raise RuntimeError('控制台已关闭')
                result = self.connections.request(action, cars)
                self._event('info', f'连接操作：{action} · {", ".join(cars)}',
                            {'action': action, 'cars': cars})
                return {'connections': result}
        finally:
            with self._lock:
                if action != 'check':
                    self._connection_transition.difference_update(cars)

    def _busy(self):
        return self._worker is not None and self._worker.is_alive()

    def start(self, commands, duration, rate=20):
        duration = number(duration, '持续时间（秒）', 0.1, 60)
        rate = number(rate, '发布频率（Hz）', 1, 50)
        if not isinstance(commands, dict) or not commands:
            raise ValueError('请至少选择一辆车')
        normalized = {}
        for car, values in commands.items():
            if not isinstance(values, dict) or set(values) != {'v', 'w'}:
                raise ValueError(f'{car}: 需要 v 和 w 两个速度参数')
            normalized[car] = Velocity(number(values['v'], 'v'), number(values['w'], 'w'))
        with self._lock:
            if self._closed:
                raise RuntimeError('控制台已关闭')
            if self._busy():
                raise RuntimeError('已有任务正在执行或停止，请等待其结束')
            if self._connection_transition.intersection(normalized):
                raise RuntimeError('所选车辆正在释放控制连接，请等待连接操作完成')
            unresolved = set(normalized).intersection(self._unconfirmed)
            if unresolved:
                raise RuntimeError(f'{", ".join(sorted(unresolved))} 停车尚未确认，请先再次停止并检查反馈')
            if self.connections is not None and self.connections.busy_for(list(normalized)):
                raise RuntimeError('所选车辆正在检测或调整连接，请等待连接操作完成')
            states = {car['id']: car for car in self.monitor.snapshot()}
            for car in normalized:
                if car not in states:
                    raise ValueError(f'未知车辆：{car}')
                if states[car]['status'] != 'connected':
                    raise ValueError(f'{car} 尚未确认连接，请查看连接状态')
            job = {'id': secrets.token_hex(6), 'kind': 'velocity', 'status': 'starting',
                   'cars': list(normalized), 'commands': deepcopy(commands),
                   'duration': duration, 'rate': rate, 'error': None, 'started_at': None,
                   'car_results': self._empty_results(normalized)}
            self._job, self._cancel = job, threading.Event()
            self._worker = threading.Thread(target=self._run,
                                            args=(job, self._cancel, normalized), daemon=True)
            self._event('info', f'准备执行：{", ".join(normalized)}，{duration:g} 秒',
                        {'job_id': job['id'], 'commands': commands, 'duration': duration, 'rate': rate})
            self._worker.start()
            return self._job_view()

    @staticmethod
    def _empty_results(names):
        return {name: {'stop_status': 'pending', 'reason': '尚未开始停车确认',
                       'command_count': 0, 'zero_count': 0, 'error': None}
                for name in names}

    def _record_job_firmware(self, job):
        if self.mode != 'demo' and self.firmware_logger is not None:
            job['firmware_log'] = self.firmware_logger.record(
                job['cars'], session_id=job['id'], kind=job['kind'])

    def _run(self, job, cancel, commands):
        error = None
        try:
            self._record_job_firmware(job)
            self.channels.ensure(commands)
            discovery_deadline = self.clock() + self.DISCOVERY_TIMEOUT
            start_error = None
            while not cancel.is_set():
                states = self.channels.snapshot()
                failures = [f'{name}: {states.get(name, {}).get("error")}' for name in commands
                            if states.get(name, {}).get('error') and
                            states.get(name, {}).get('closed')]
                if failures:
                    raise RuntimeError('; '.join(failures))
                if all(states.get(name, {}).get('ready') for name in commands):
                    # start only changes mailboxes. A background discovery probe
                    # can race the snapshot; retry that reservation within the
                    # same discovery budget, without publishing a partial batch.
                    with self._lock:
                        if not cancel.is_set():
                            try:
                                deadline = self.channels.start(commands, job['duration'], job['rate'])
                            except RuntimeError as exc:
                                start_error = str(exc)
                            else:
                                job.update(status='running', started_at=time.time(), _deadline=deadline)
                                self._event('info', f'开始发送速度：{", ".join(commands)}')
                                break
                if self.clock() >= discovery_deadline:
                    missing = [name for name in commands if not states.get(name, {}).get('ready')]
                    raise RuntimeError('速度接口发现超时，未开始运动：' +
                                       (', '.join(missing) or start_error or '控制通道尚未就绪'))
                cancel.wait(0.02)
            while not cancel.is_set():
                states = self.channels.snapshot()
                failures = [f'{name}: {states.get(name, {}).get("error")}' for name in commands
                            if states.get(name, {}).get('error')]
                if failures:
                    raise RuntimeError('; '.join(failures))
                if self.clock() >= job['_deadline']:
                    break
                cancel.wait(min(0.02, max(0, job['_deadline'] - self.clock())))
        except Exception as exc:
            error = str(exc)
        finally:
            self._finish_stop(job, cancel, error)

    def _feedback_result(self, name, channel, state, tracker):
        result = {'stop_status': 'pending', 'reason': '正在持续发送零速度并等待新的里程计反馈',
                  'command_count': channel.get('command_count', 0),
                  'zero_count': channel.get('zero_count', 0), 'error': channel.get('error')}
        first_zero = channel.get('first_zero_at')
        received = state.get('odom_received_at')
        sequence = state.get('odom_sequence')
        age = state.get('age_s')
        v, w = state.get('measured_v'), state.get('measured_w')
        finite = lambda value: (isinstance(value, (int, float)) and
                                not isinstance(value, bool) and math.isfinite(value))
        reason = None
        if channel.get('stop_unconfirmed') or channel.get('releasing') or channel.get('closed'):
            reason = channel.get('stop_reason') or '控制通道未完成本次停车发送'
        elif first_zero is None:
            reason = '尚无零速度发布成功记录'
        elif state.get('status') != 'connected':
            reason = '底盘反馈未连接或已过期'
        elif (not finite(received) or received <= first_zero or
              not finite(age) or not 0 <= age <= self.FEEDBACK_MAX_AGE or
              not isinstance(sequence, int) or sequence <= 0):
            reason = '缺少停止指令之后的新鲜里程计反馈'
        elif not finite(v) or not finite(w):
            reason = '里程计速度无效'
        elif abs(v) > self.STOP_V_TOLERANCE or abs(w) > self.STOP_W_TOLERANCE:
            reason = '里程计仍报告运动'
        if reason:
            tracker.clear()
            result['reason'] = reason
            return result
        if received > tracker.get('last_received', float('-inf')) and sequence != tracker.get('sequence'):
            tracker.setdefault('since', received)
            tracker['count'] = tracker.get('count', 0) + 1
            tracker.update(last_received=received, sequence=sequence)
        stable = (tracker.get('count', 0) >= self.STOP_MIN_SAMPLES and
                  received - tracker.get('since', received) >= self.STOP_STABLE_TIME)
        if stable and not channel.get('stop_pending', True) and not channel.get('publishing'):
            result.update(stop_status='confirmed', reason='连续新鲜里程计反馈已近零（非车端指令回执）')
        return result

    def _finish_stop(self, job, cancel, error=None):
        trackers = {}
        deadline = self.clock() + self.STOP_TIMEOUT
        known_names = set()
        try:
            with self._lock:
                job['status'] = 'stopping'
                self.channels.stop(job['cars'])
            while True:
                with self._lock:
                    # These are cache reads, not ROS I/O. Keep them in the same
                    # reservation as stop(): a repeated stop must not be judged
                    # using the preceding stop window's cached result.
                    channels = self.channels.snapshot()
                    states = {car['id']: car for car in self.monitor.snapshot()}
                    names = list(job['cars'])
                    if set(names) - known_names:
                        deadline = self.clock() + self.STOP_TIMEOUT
                        known_names.update(names)
                    results = {name: self._feedback_result(
                        name, channels.get(name, {}), states.get(name, {}),
                        trackers.setdefault(name, {})) for name in names}
                    job['car_results'] = results
                    all_confirmed = all(item['stop_status'] == 'confirmed' for item in results.values())
                    if all_confirmed or self.clock() >= deadline:
                        for result in results.values():
                            if result['stop_status'] != 'confirmed':
                                result['stop_status'] = 'unconfirmed'
                        self._unconfirmed.difference_update(names)
                        self._unconfirmed.update(name for name, result in results.items()
                                                 if result['stop_status'] != 'confirmed')
                        failures = [f'{name}: {result["error"]}' for name, result in results.items()
                                    if result['error']]
                        if failures:
                            error = '; '.join(filter(None, [error, *failures]))
                        job['error'] = error
                        job['status'] = ('failed' if error else 'unconfirmed' if not all_confirmed
                                         else 'stopped' if cancel.is_set() or job['kind'] not in ('velocity', 'trajectory')
                                         else 'completed')
                        self._event('error' if error else 'info' if all_confirmed else 'warning',
                                    '本次车辆的里程计停车反馈已确认' if all_confirmed else
                                    '部分车辆停车未确认，请查看逐车结果',
                                    {'job_id': job['id'], 'status': job['status'],
                                     'error': error, 'car_results': results})
                        return
                # A cancellation is already being handled; do not busy-spin on
                # its set Event, and keep sampling fresh feedback during stop.
                time.sleep(0.02)
        except Exception as exc:
            with self._lock:
                job.update(status='failed', error='; '.join(filter(None, [error, str(exc)])))
                self._unconfirmed.update(job['cars'])
                for result in job['car_results'].values():
                    result.update(stop_status='unconfirmed', reason='停车处理异常，未确认反馈')
                self._event('error', f'停车处理失败：{job["error"]}')

    def stop(self, scope='job'):
        if scope not in ('job', 'all'):
            raise ValueError('停止范围必须是 job 或 all')
        with self._lock:
            if self._closed:
                raise RuntimeError('控制台已关闭')
            names = ([car['id'] for car in self.monitor.snapshot()] if scope == 'all'
                     else list(self._job['cars']) if self._job else [])
            if not names:
                return {'job': None, 'message': '尚无本次任务车辆'}
            if self._busy() and self._job['status'] in ACTIVE:
                self._cancel.set()
                self._job['status'] = 'stopping'
                for name in names:
                    if name not in self._job['cars']:
                        self._job['cars'].append(name)
                        self._job['car_results'].update(self._empty_results([name]))
                self.channels.stop(names)
            else:
                job = {'id': secrets.token_hex(6), 'kind': 'stop_' + scope, 'status': 'stopping',
                       'cars': names, 'commands': {name: {'v': 0, 'w': 0} for name in names},
                       'duration': 0, 'rate': 20, 'error': None, 'started_at': time.time(),
                       'car_results': self._empty_results(names)}
                self._job, self._cancel = job, threading.Event()
                self._cancel.set()
                self.channels.stop(names)
                self._worker = threading.Thread(target=self._finish_stop,
                                                args=(job, self._cancel), daemon=True)
                self._worker.start()
            self._event('info', f'已请求持续停车：{", ".join(names)}')
            # Do not wait for workers: a blocked car must never delay this API.
            return {'job': self._job_view(), 'message': '停止请求已提交，正在逐车核对停车反馈'}

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._cancel.set()
            worker = self._worker
            if self._job and self._job['status'] in ACTIVE:
                self.channels.stop(self._job['cars'])
        if worker is not None:
            worker.join(timeout=self.STOP_TIMEOUT + 0.5)
        try:
            self.channels.close(timeout=self.STOP_TIMEOUT + 0.5)
        finally:
            try:
                if self.connections is not None:
                    self.connections.close()
            finally:
                self.monitor.close()


class DemoMonitor:
    """Explicit simulation: no ROS imports, discovery, or vehicle traffic."""

    def __init__(self, config):
        self._lock = threading.Lock()
        payload = json.loads(Path(config).read_text())
        entries = payload['cars']
        metadata = {entry['name']: entry for entry in entries}
        self._updated_at = time.monotonic()
        self.cars = {}
        for index, car in enumerate(load_cars(config)):
            state = ('connected' if index < 18 else 'discovered' if index < 21
                     else 'stale' if index < 23 else 'offline')
            self.cars[car.name] = {
                'id': car.name, 'name': car.name, 'ip': metadata[car.name].get('ip', ''),
                'domain_id': car.domain_id, 'status': state,
                'age_s': 0.04 if state == 'connected' else 4.2 if state == 'stale' else None,
                'odom_received_at': time.monotonic() if state == 'connected' else None,
                'odom_sequence': 1 if state == 'connected' else 0,
                'measured_v': 0.0 if state == 'connected' else None,
                'measured_w': 0.0 if state == 'connected' else None,
                'cmd_subscribers': int(state != 'offline'), 'cmd_publishers': 0, 'error': None}
            pose_status = ('fresh' if index < 18 or index >= 23 else
                           'missing' if index < 21 else 'stale')
            position = ({'x': (index % 5 - 2) * 0.6, 'y': (index // 5 - 2) * 0.6,
                         'z': 0.45} if pose_status != 'missing' else None)
            self.cars[car.name]['mocap'] = {
                'status': pose_status, 'topic': metadata[car.name].get('pose_topic'),
                'domain_id': payload.get('mocap_domain_id', 0),
                'frame_id': 'world' if position else None,
                'position': position,
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0} if position else None,
                'yaw_rad': 0.0 if position else None,
                'age_s': 0.04 if pose_status == 'fresh' else 4.2 if position else None,
                'received_at_unix': time.time() if position else None,
                'source_stamp': None, 'publishers': 2 if index == 2 else int(position is not None),
                'error': None,
                'binding_confirmed': metadata[car.name].get('mocap_binding_confirmed', False)}

    def open(self):
        pass

    def snapshot(self):
        with self._lock:
            self._advance()
            return deepcopy(list(self.cars.values()))

    def _advance(self):
        """Integrate only simulated motion; this class never opens a ROS context."""
        now = time.monotonic()
        dt, self._updated_at = max(0.0, now - self._updated_at), now
        for car in self.cars.values():
            if car['status'] == 'connected':
                car.update(odom_received_at=now, age_s=0.0,
                           odom_sequence=car['odom_sequence'] + 1)
            pose = car['mocap']
            if pose['status'] == 'stale':
                pose['age_s'] += dt
            if pose['status'] != 'fresh':
                continue
            pose['received_at_unix'] = time.time() - pose['age_s']
            v, w = car['measured_v'] or 0.0, car['measured_w'] or 0.0
            heading = pose['yaw_rad']
            turn = w * dt
            if abs(w) > 1e-9:
                pose['position']['x'] += v / w * (math.sin(heading + turn) - math.sin(heading))
                pose['position']['y'] += v / w * (math.cos(heading) - math.cos(heading + turn))
            else:
                pose['position']['x'] += v * dt * math.cos(heading)
                pose['position']['y'] += v * dt * math.sin(heading)
            heading = math.atan2(math.sin(heading + turn), math.cos(heading + turn))
            pose['yaw_rad'] = heading
            pose['orientation'].update(z=math.sin(heading / 2), w=math.cos(heading / 2))

    def close(self):
        pass

    def fleet(self, cars):
        monitor = self

        class DemoFleet:
            def open(self):
                pass

            def wait_for_subscribers(self, timeout=1):
                return all(monitor.cars[car]['cmd_subscribers'] > 0 for car in cars)

            def set_velocities(self, commands):
                with monitor._lock:
                    monitor._advance()
                    for name, velocity in commands.items():
                        if monitor.cars[name]['status'] == 'connected':
                            monitor.cars[name].update(measured_v=velocity.v, measured_w=velocity.w)

            def stop(self):
                self.set_velocities({car: Velocity(0, 0) for car in cars})

            def close(self, *, send_stop=True):
                if send_stop:
                    self.stop()

        return DemoFleet()


class ConsoleServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, console, port=8765):
        self.console = console
        self.csrf_token = secrets.token_urlsafe(32)
        self.static_dir = Path(__file__).resolve().parent / 'web_static'
        super().__init__(('127.0.0.1', port), ConsoleHandler)


class ConsoleHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _host_valid(self):
        port = self.server.server_address[1]
        return self.headers.get('Host') in (f'127.0.0.1:{port}', f'localhost:{port}')

    def _send(self, status, data, mime='application/json; charset=utf-8'):
        if not isinstance(data, bytes):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
                         "img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        if not self._host_valid():
            return self._send(403, {'error': '仅支持本机访问'})
        if self.path == '/api/state':
            try:
                return self._send(200, {**self.server.console.snapshot(),
                                        'csrf_token': self.server.csrf_token})
            except Exception as error:
                return self._send(500, {'error': str(error)})
        if self.path == '/api/trajectory':
            return self._send(200, {'trajectory': self.server.console.trajectory_view()})
        assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                  '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                  '/style.css': ('style.css', 'text/css; charset=utf-8')}
        if self.path not in assets:
            return self._send(404, {'error': '路径不存在'})
        name, mime = assets[self.path]
        return self._send(200, (self.server.static_dir / name).read_bytes(), mime)

    def do_POST(self):
        origin = self.headers.get('Origin')
        expected_origin = f'http://{self.headers.get("Host")}'
        token = self.headers.get('X-CSRF-Token', '')
        if (not self._host_valid() or origin not in (None, expected_origin) or
                not secrets.compare_digest(token, self.server.csrf_token)):
            return self._send(403, {'error': '请求来源无效，请从本机控制台操作'})
        if self.headers.get_content_type() != 'application/json':
            return self._send(415, {'error': '需要 JSON 请求'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            limit = MAX_CSV_BYTES * 2 + 65536 if self.path == '/api/trajectory/import' else 65536
            if not 0 < length <= limit:
                raise ValueError('请求长度无效')
            self.connection.settimeout(5)
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError('需要 JSON 对象')
            if self.path == '/api/trajectory/import':
                if set(payload) - {'csv', 'filename'} or 'csv' not in payload:
                    raise ValueError('导入需要 csv，可选 filename')
                return self._send(200, {'trajectory': self.server.console.import_trajectory(
                    payload['csv'], payload.get('filename', 'trajectory.csv'))})
            if self.path in ('/api/trajectory/check', '/api/trajectory/run'):
                if (set(payload) - {'trajectory_id', 'cars', 'operator_confirmed'}
                        or not {'trajectory_id', 'cars'} <= set(payload)):
                    raise ValueError('轨迹操作需要 trajectory_id、cars，可选布尔字段 operator_confirmed')
                args = (payload['trajectory_id'], payload['cars'], payload.get('operator_confirmed', False))
                if self.path.endswith('/check'):
                    return self._send(200, self.server.console.check_trajectory(*args))
                return self._send(200, {'job': self.server.console.start_trajectory(*args)})
            if self.path == '/api/trajectory/generate':
                if set(payload) - {'cars', 'distance', 'duration'} or 'cars' not in payload:
                    raise ValueError('生成轨迹需要 cars，可选 distance 和 duration')
                return self._send(200, self.server.console.generate_trajectory(
                    payload['cars'], payload.get('distance', .5), payload.get('duration', 20.)))
            if self.path == '/api/run':
                if set(payload) - {'commands', 'duration', 'rate'}:
                    raise ValueError('请求包含未知字段')
                job = self.server.console.start(payload.get('commands'), payload.get('duration'),
                                                payload.get('rate', 20))
                return self._send(200, {'job': job})
            if self.path == '/api/stop':
                if set(payload) - {'scope'}:
                    raise ValueError('请求包含未知字段')
                return self._send(200, self.server.console.stop(payload.get('scope', 'job')))
            if self.path == '/api/connections':
                if set(payload) != {'action', 'cars'}:
                    raise ValueError('连接操作需要且仅接受 action 和 cars')
                return self._send(200, self.server.console.connection_action(
                    payload['action'], payload['cars']))
            return self._send(404, {'error': '接口不存在'})
        except (ValueError, TypeError) as error:
            return self._send(400, {'error': str(error)})
        except RuntimeError as error:
            return self._send(409, {'error': str(error)})
        except Exception as error:
            return self._send(500, {'error': str(error)})


def main(argv=None):
    parser = argparse.ArgumentParser(description='本机小车操作台：连接、动捕位置、速度控制与停止')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--demo', action='store_true', help='演示模式：不加载ROS，不连接小车')
    parser.add_argument('--mocap-domain', type=int, help='动捕 ROS 域；默认读取配置，未配置时为0')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('port 必须在1～65535之间')
    if args.mocap_domain is not None and not 0 <= args.mocap_domain <= 99:
        parser.error('mocap-domain 必须在0～99之间')
    config = args.config or default_config()
    if args.demo:
        from .demo_connections import DemoConnections
        monitor = DemoMonitor(config)
        factory = monitor.fleet
        connections = DemoConnections(monitor)
    else:
        from .web_monitor import ConsoleMonitor
        monitor = ConsoleMonitor(config, mocap_domain=args.mocap_domain)
        factory = partial(Fleet, config=config)
        from .connections import ConnectionManager
        try:
            connections = ConnectionManager(config, monitor)
        except (ValueError, OSError, KeyError) as error:
            connections = UnavailableConnections(str(error))
    firmware_logger = None
    if not args.demo:
        from .firmware_session import SessionLogger
        firmware_logger = SessionLogger(config)
        firmware_logger.record(kind='console_start')
        print(f'会话固件日志：{firmware_logger.directory}', flush=True)
    console = Console(monitor, factory, mode='demo' if args.demo else 'live',
                      connections=connections, firmware_logger=firmware_logger)
    server = ConsoleServer(console, args.port)
    def terminate(*_):
        raise KeyboardInterrupt

    old_term = signal.signal(signal.SIGTERM, terminate)
    try:
        monitor.open()
        print(f'小车操作台 http://127.0.0.1:{args.port}/ [{console.mode}]', flush=True)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        console.close()
        if firmware_logger is not None:
            firmware_logger.close()
        signal.signal(signal.SIGTERM, old_term)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
