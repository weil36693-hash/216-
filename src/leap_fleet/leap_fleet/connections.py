"""Per-car ownership of chassis connection processes for the local web console.

Connection actions never publish velocities. Ordinary connect only starts the
existing single-car launcher after read-only checks; applying communication
settings is a separate action against an unchanged, previously shown preview.
"""

from copy import deepcopy
from dataclasses import dataclass, field
import errno
import fcntl
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import uuid
from urllib.request import build_opener, ProxyHandler, Request

from .config import default_config, load_cars


_RUNTIME_KEYS = ('comm_mode', 'microros_agent_ip', 'microros_agent_port')


def _uint32(value):
    return type(value) is int and 0 <= value <= 0xffffffff


def _status_metadata(payload):
    """Keep only bounded, typed firmware/watchdog fields; none are settings."""
    firmware, watchdog = {}, {}
    source = payload.get('firmware')
    if isinstance(source, dict):
        if isinstance(source.get('version'), str):
            firmware['version'] = source['version'][:128]
        if _uint32(source.get('motion_safety_revision')):
            firmware['motion_safety_revision'] = source['motion_safety_revision']
    source = payload.get('cmd_vel_watchdog')
    if isinstance(source, dict):
        for key in ('enabled', 'has_command', 'timed_out'):
            if type(source.get(key)) is bool:
                watchdog[key] = source[key]
        for key in ('timeout_ms', 'received_sequence', 'applied_sequence'):
            if _uint32(source.get(key)):
                watchdog[key] = source[key]
        if 'last_cmd_age_ms' in source and (source['last_cmd_age_ms'] is None or
                                           _uint32(source['last_cmd_age_ms'])):
            watchdog['last_cmd_age_ms'] = source['last_cmd_age_ms']
    return {'firmware': firmware or None, 'cmd_vel_watchdog': watchdog or None}


def _empty_vehicle_status():
    return {'check_state': 'never', 'checked_at_unix': None, 'attempted_at_unix': None,
            'error': None, 'firmware': None, 'cmd_vel_watchdog': None}


def _get_runtime(car):
    request = Request(f"http://{car['ip']}/api/status")
    with build_opener(ProxyHandler({})).open(request, timeout=3.0) as response:
        payload = json.loads(response.read(1024 * 1024))
    runtime = payload['runtime']
    if not isinstance(runtime, dict) or any(key not in runtime for key in _RUNTIME_KEYS):
        raise ValueError('vehicle status lacks communication runtime settings')
    port = runtime['microros_agent_port']
    if isinstance(port, bool):
        raise ValueError('invalid vehicle Agent port')
    result = {key: runtime[key] for key in _RUNTIME_KEYS}
    result['microros_agent_port'] = int(port)
    # Private read-only envelope. The preflight explicitly extracts the three
    # communication keys before comparing or handing configuration to launch.
    result['_status_metadata'] = _status_metadata(payload)
    return result


def _host_ip(car):
    # UDP connect chooses a route without transmitting a packet to the car.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((car['ip'], car['agent_port']))
        address = sock.getsockname()[0]
    return str(ipaddress.IPv4Address(address))


def _external_busy(workspace, car):
    """Observe the launcher's lock and UDP ownership without replacing either."""
    lock_path = workspace / 'log' / f"fleet_domain_{car['domain_id']}.lock"
    if lock_path.exists():
        with lock_path.open('r') as handle:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return 'external', '该车辆的底盘启动器已由其他终端管理'
            finally:
                # Unlocking a descriptor that did not acquire the lock is safe.
                fcntl.flock(handle, fcntl.LOCK_UN)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('0.0.0.0', car['agent_port']))
    except OSError as error:
        if error.errno == errno.EADDRINUSE:
            return 'blocked', f"UDP {car['agent_port']} 已被其他进程占用，不会替换"
        raise
    return None


@dataclass
class _Session:
    car: dict
    state: dict
    cancel: threading.Event = field(default_factory=threading.Event)
    worker: object = None
    process: object = None
    status_file: object = None
    generation: int = 0
    action: object = None
    cleanup_unknown: bool = False


class ConnectionManager:
    """Asynchronous connection checks and independent, owned single-car launches."""

    CANCEL_TIMEOUT = 12.0
    WORKER_JOIN_TIMEOUT = 4.5
    CLOSE_TIMEOUT = 18.0
    LOG_LIMIT = 6000

    def __init__(self, config, monitor, workspace=None):
        self.config = (default_config() if config is None else Path(config).expanduser()).resolve()
        configs = load_cars(self.config)
        payload = json.loads(self.config.read_text(encoding='utf-8'))
        entries = {entry['name']: dict(entry) for entry in payload['cars']}
        ports, addresses = set(), set()
        for item in configs:
            car = entries[item.name]
            if not item.name.isalnum():
                raise ValueError('connection car names must contain only letters and digits')
            if not 1 <= item.domain_id <= 99:
                raise ValueError(f'{item.name}: connection domain_id must be in 1..99')
            try:
                address = ipaddress.IPv4Address(car.get('ip'))
            except (ValueError, TypeError, ipaddress.AddressValueError) as error:
                raise ValueError(f'{item.name}: a valid vehicle IPv4 is required') from error
            if not isinstance(car.get('ip'), str):
                raise ValueError(f'{item.name}: vehicle IP must be an IPv4 string')
            car['ip'] = str(address)
            if car['ip'] in addresses:
                raise ValueError('vehicle IP addresses must be unique')
            addresses.add(car['ip'])
            port = car.get('agent_port')
            if type(port) is not int or not 1024 <= port <= 65535 or port in ports:
                raise ValueError('Agent ports must be unique integers in 1024..65535')
            ports.add(port)
        for car in entries.values():
            if any(7400 + 250 * item.domain_id <= car['agent_port'] <
                   7650 + 250 * item.domain_id for item in configs):
                raise ValueError('Agent port overlaps a configured DDS domain port range')
        self.monitor = monitor
        self.workspace = self._find_workspace(workspace)
        self.available = self.workspace is not None
        self.error = None if self.available else '未找到包含 start_fleet.sh 的工作区，无法管理底盘连接'
        self._lock = threading.RLock()
        self._network_limit = threading.BoundedSemaphore(6)
        self._closed = False
        self._threads = []
        self._log_dir = None
        self._sessions = {}
        for item in configs:
            state = {'id': item.name, 'phase': 'idle', 'message': '尚未检测',
                     'busy': False, 'owned': False, 'current': None, 'target': None,
                     'log_file': None, 'log_tail': '', 'error': None,
                     'vehicle_status': _empty_vehicle_status()}
            self._sessions[item.name] = _Session(entries[item.name], state)

    def _find_workspace(self, explicit):
        selected = explicit if explicit is not None else os.environ.get('LEAP_FLEET_WORKSPACE')
        if selected is not None:
            candidate = Path(selected).expanduser().resolve()
            return candidate if (candidate / 'start_fleet.sh').is_file() else None
        candidates = [Path(__file__).resolve().parents[3], self.config.parent,
                      self.config.parent.parent]
        for candidate in candidates:
            if (candidate / 'start_fleet.sh').is_file():
                return candidate
        return None

    def _names(self, cars):
        if isinstance(cars, str):
            names = [cars]
        else:
            try:
                names = list(cars)
            except TypeError as error:
                raise ValueError('cars must select vehicles by name') from error
        if not names or any(not isinstance(name, str) for name in names):
            raise ValueError('cars must select at least one vehicle by name')
        if len(set(names)) != len(names):
            raise ValueError('cars contains duplicate names')
        missing = set(names) - self._sessions.keys()
        if missing:
            raise ValueError(f'unknown cars: {sorted(missing)}')
        return names

    def _monitor_states(self):
        return {car['id']: car for car in self.monitor.snapshot()}

    def _mutation_guard(self, names):
        try:
            states = self._monitor_states()
        except Exception as error:
            raise RuntimeError(f'无法读取底盘状态，未执行配置或关闭：{error}') from error
        for name in names:
            state = states.get(name, {})
            if state.get('cmd_publishers', 0) > 0:
                raise RuntimeError(f'{name} 存在速度发布者，请先结束控制来源再操作连接')
            for key in ('measured_v', 'measured_w'):
                value = state.get(key)
                if value is not None and (not isinstance(value, (int, float)) or
                                          not math.isfinite(value) or abs(value) > 1.0e-6):
                    raise RuntimeError(f'{name} 里程计速度非零或无效，未执行配置或关闭')

    def _update(self, session, generation, **values):
        with self._lock:
            if session.generation == generation:
                session.state.update(values)

    def _update_vehicle_status(self, session, generation, **values):
        with self._lock:
            if session.generation == generation:
                status = deepcopy(session.state['vehicle_status'])
                status.update(values)
                session.state['vehicle_status'] = status

    def _start_worker(self, session, action, target, *args):
        # Caller holds the manager lock; allocation and submission are atomic.
        previous_worker, previous_action = session.worker, session.action
        session.generation += 1
        generation = session.generation
        session.action = action
        worker = None
        try:
            worker = threading.Thread(target=target, args=(session, generation, *args),
                                      name=f'fleet-connection-{session.car["name"]}', daemon=True)
            session.worker = worker
            self._threads.append(worker)
            worker.start()
        except BaseException:
            if worker is None or worker.ident is None:
                if worker is not None:
                    self._threads.remove(worker)
                session.worker, session.action = previous_worker, previous_action
            else:
                session.cancel.set()
            session.state.update(busy=False, phase='failed', error='无法创建连接检查线程')
            raise

    def request(self, action, cars):
        """Validate the complete request, reserve per-car work, and return immediately."""
        if action not in ('check', 'connect', 'configure', 'cancel'):
            raise ValueError('unknown connection action')
        names = self._names(cars)
        if not self.available:
            raise RuntimeError(self.error)
        if action in ('configure', 'cancel'):
            self._mutation_guard(names)
        states = self._monitor_states()
        with self._lock:
            if self._closed:
                raise RuntimeError('connection manager is closed')
            if action == 'configure':
                for name in names:
                    session = self._sessions[name]
                    if session.worker is not None and session.worker.is_alive():
                        continue
                    if session.state['phase'] != 'needs_config':
                        raise ValueError(f'{name}: 请先检测并核对 needs_config 配置预览')
                    if session.process is not None and session.process.poll() is None:
                        raise RuntimeError(f'{name}: 请先关闭本页已有连接，再重新检测配置')
            for name in names:
                session = self._sessions[name]
                running = session.worker is not None and session.worker.is_alive()
                owned = session.process is not None and session.process.poll() is None
                if action == 'cancel':
                    if running and session.action == 'cancel':
                        continue
                    if not running and not owned:
                        continue
                    previous = session.worker
                    session.cancel.set()
                    if session.state['vehicle_status']['check_state'] == 'checking':
                        self._update_vehicle_status(
                            session, session.generation, check_state='cancelled',
                            error='本次检测已取消，保留上次检测快照')
                    session.state.update(phase='stopping', busy=True, error=None,
                                         message='正在关闭本页连接；这不是停车操作')
                    self._start_worker(session, 'cancel', self._cancel_worker, previous)
                    continue
                if (running or (action == 'connect' and owned) or
                        (action == 'check' and owned and session.state['busy'])):
                    continue
                if action == 'connect' and states.get(name, {}).get('status') == 'connected':
                    session.state.update(phase='external', busy=False, owned=False, error=None,
                                         message='车辆已连接，复用外部管理的底盘进程')
                    continue
                preview = (deepcopy(session.state['current']), deepcopy(session.state['target']))
                session.cancel = threading.Event()
                session.state.update(phase='checking', busy=True, error=None,
                                     message='正在读取车辆通信设置')
                self._start_worker(session, action, self._preflight_worker,
                                   session.cancel, preview)
        return self.snapshot()

    def _cancelled(self, session, generation, cancel):
        with self._lock:
            return self._closed or cancel.is_set() or session.generation != generation

    def _preflight_worker(self, session, generation, cancel, preview):
        name = session.car['name']
        acquired = False
        try:
            with self._lock:
                owned = session.process is not None and session.process.poll() is None
                action = session.action
            if not owned and action in ('connect', 'configure'):
                occupied = _external_busy(self.workspace, session.car)
                if occupied:
                    self._update(session, generation, phase=occupied[0], message=occupied[1],
                                 busy=False, owned=False, error=None)
                    return
            while not self._cancelled(session, generation, cancel):
                acquired = self._network_limit.acquire(timeout=0.1)
                if acquired:
                    break
            if not acquired or self._cancelled(session, generation, cancel):
                return
            try:
                self._update_vehicle_status(session, generation, check_state='checking',
                                            attempted_at_unix=time.time(), error=None)
                report = _get_runtime(session.car)
                current = {key: report[key] for key in _RUNTIME_KEYS}
                metadata = _status_metadata(report.get('_status_metadata', {}))
                target = {'comm_mode': 'micro_ros', 'microros_agent_ip': _host_ip(session.car),
                          'microros_agent_port': session.car['agent_port']}
            finally:
                self._network_limit.release()
                acquired = False
            if self._cancelled(session, generation, cancel):
                return
            self._update(session, generation, current=current, target=target)
            self._update_vehicle_status(session, generation, check_state='ok',
                                        checked_at_unix=time.time(), error=None, **metadata)
            action = session.action
            with self._lock:
                owned = session.process is not None and session.process.poll() is None
            if not owned:
                occupied = _external_busy(self.workspace, session.car)
                if occupied:
                    self._update(session, generation, phase=occupied[0], message=occupied[1],
                                 busy=False, owned=False, error=None)
                    return
            if action == 'configure' and (current, target) != preview:
                self._update(session, generation, phase='needs_config', busy=False, error=None,
                             message='通信参数或目标已变化，已刷新预览；核对后再次应用才会修改')
                return
            if current != target and action != 'configure':
                self._update(session, generation, phase='needs_config', busy=False, error=None,
                             message='通信设置不匹配；普通连接不会修改，请核对后单独应用配置')
                return
            if action == 'check':
                self._update(session, generation, phase='idle', busy=False, error=None,
                             message='通信参数匹配，可连接')
                return
            if action == 'configure':
                self._mutation_guard([name])
            self._spawn(session, generation, cancel, configure=action == 'configure')
        except Exception as error:
            self._update_vehicle_status(session, generation, check_state='failed', error=str(error))
            self._update(session, generation, phase='failed', busy=False,
                         error=str(error), message=f'连接检查失败：{error}')
        finally:
            if acquired:
                self._network_limit.release()
            if self._cancelled(session, generation, cancel):
                self._update(session, generation, phase='stopped', busy=False,
                             message='连接操作已取消，未再启动进程')

    def _spawn(self, session, generation, cancel, *, configure):
        with self._lock:
            # Cancellation and ownership registration share this commit lock.
            if self._cancelled(session, generation, cancel):
                return
            if self._log_dir is None:
                self._log_dir = self.workspace / 'log' / (
                    time.strftime('web_connections_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8])
                self._log_dir.mkdir(parents=True)
            name = session.car['name']
            token = uuid.uuid4().hex[:8]
            log_file = self._log_dir / f'{name}_{token}.log'
            status_file = self._log_dir / f'{name}_{token}.json'
            command = [str(self.workspace / 'start_fleet.sh'), '--config', str(self.config),
                       '--cars', name, '--status-file', str(status_file)]
            if configure:
                command.extend(['--configure', '--agent-ip',
                                session.state['target']['microros_agent_ip']])
            environment = dict(os.environ, ROS_LOCALHOST_ONLY='0')
            with log_file.open('w', encoding='utf-8') as output:
                process = subprocess.Popen(
                    command, cwd=str(self.workspace), env=environment,
                    stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            session.process, session.status_file = process, status_file
            session.cleanup_unknown = False
            session.state.update(phase='starting', owned=True, busy=True, error=None,
                                 log_file=str(log_file), log_tail='',
                                 message='已启动本页单车底盘连接，正在等待实时数据')

    def _cancel_worker(self, session, generation, previous):
        try:
            if previous is not None and previous.is_alive():
                previous.join(timeout=self.WORKER_JOIN_TIMEOUT)
                if previous.is_alive():
                    raise RuntimeError('车辆检查尚未结束；已禁止后续启动，可稍后重试关闭')
            with self._lock:
                process = session.process
            if process is not None:
                if not self._closed:
                    try:
                        self._mutation_guard([session.car['name']])
                    except Exception:
                        # Lifecycle close follows Console motion cleanup. Its
                        # final odometry samples may still report residual speed.
                        if not self._closed:
                            raise
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    code = process.wait(timeout=self.CANCEL_TIMEOUT)
                except subprocess.TimeoutExpired as error:
                    raise RuntimeError('底盘启动器12秒内未完成清理；保留本页所有权，可重试关闭') from error
                if code < 0 and not self._cleanup_proven(self._read_status(session)):
                    session.cleanup_unknown = True
                    raise RuntimeError(self._unknown_cleanup_message(code))
            with self._lock:
                if session.generation == generation:
                    session.process = None
                    session.state.update(phase='stopped', owned=False, busy=False, error=None,
                                         message='本页连接已关闭；未发送停车或速度命令')
        except Exception as error:
            self._update(session, generation, phase='failed', busy=False,
                         owned=session.process is not None and session.process.poll() is None,
                         error=str(error), message=f'关闭连接失败：{error}')

    @classmethod
    def _log_tail(cls, path):
        if not path:
            return ''
        try:
            with Path(path).open('rb') as source:
                source.seek(0, os.SEEK_END)
                source.seek(max(0, source.tell() - cls.LOG_LIMIT))
                return source.read(cls.LOG_LIMIT).decode('utf-8', errors='replace')[-cls.LOG_LIMIT:]
        except OSError:
            return ''

    @staticmethod
    def _read_status(session):
        if session.status_file is not None:
            try:
                with session.status_file.open('rb') as source:
                    status = json.loads(source.read(65536))
                return status if isinstance(status, dict) else None
            except (OSError, ValueError, UnicodeError):
                pass
        return None

    @staticmethod
    def _cleanup_proven(status):
        return status is not None and status.get('phase') in ('stopped', 'failed')

    @staticmethod
    def _unknown_cleanup_message(code):
        return (f'底盘启动器被信号 {-code} 异常终止；子进程清理状态未知，'
                '请检查本次日志及可能残留的 Agent / 底盘进程；未自动接管或终止其他进程')

    def _refresh(self, states):
        with self._lock:
            for name, session in self._sessions.items():
                session.state['log_tail'] = self._log_tail(session.state['log_file'])
                if session.process is None:
                    continue
                if session.worker is not None and session.worker.is_alive():
                    continue
                code = session.process.poll()
                if code is None and session.action == 'cancel' and session.state['phase'] == 'failed':
                    # A failed cancellation retains ownership and its error;
                    # the launcher's old ready JSON cannot undo this result.
                    continue
                status = self._read_status(session)
                if code is not None:
                    if code < 0 and not self._cleanup_proven(status):
                        session.cleanup_unknown = True
                        message = self._unknown_cleanup_message(code)
                        session.state.update(phase='failed', busy=False, owned=False,
                                             error=message, message=message)
                        session.process = None
                        continue
                    if session.action == 'cancel':
                        session.state.update(phase='stopped', busy=False, owned=False,
                                             error=None, message='本页连接进程已结束')
                        session.process = None
                        continue
                    message = status.get('message') if status else None
                    error = status.get('error') if status else None
                    session.state.update(phase='failed', busy=False, owned=False,
                                         error=error or f'底盘启动器已退出（{code}）',
                                         message=message or f'底盘连接进程已退出（{code}）')
                    session.process = None
                    continue
                if not status:
                    continue
                phase = status.get('phase')
                message = str(status.get('message', ''))[:2000]
                if phase == 'ready':
                    connected = states.get(name, {}).get('status') == 'connected'
                    session.state.update(
                        phase='connected' if connected else 'waiting_data', busy=False, owned=True,
                        error=None if connected else '底盘实时数据尚未就绪或已过期；未自动重启',
                        message=message if connected else '本页进程仍在运行，等待底盘数据恢复')
                elif phase in ('checking', 'configuring', 'waiting_raw', 'starting_base',
                               'waiting_filtered'):
                    session.state.update(phase=phase, message=message, busy=True, owned=True,
                                         error=status.get('error'))
                elif phase in ('failed', 'stopped'):
                    session.state.update(phase='failed' if phase == 'failed' else 'stopping',
                                         message=message, busy=phase == 'stopped', owned=True,
                                         error=status.get('error'))

    def snapshot(self):
        try:
            states = self._monitor_states()
        except Exception:
            states = {}
        self._refresh(states)
        with self._lock:
            return {'available': self.available, 'error': self.error,
                    'cars': [deepcopy(session.state) for session in self._sessions.values()]}

    def busy_for(self, cars):
        names = self._names(cars)
        # Refresh local process status without performing network requests.
        self.snapshot()
        with self._lock:
            return any(self._sessions[name].state['busy'] for name in names)

    def close(self):
        """Cancel owned work concurrently; never kill or adopt external processes."""
        failures = []
        with self._lock:
            self._closed = True
            for session in self._sessions.values():
                session.cancel.set()
                running = session.worker is not None and session.worker.is_alive()
                owned = session.process is not None and session.process.poll() is None
                if not running and not owned:
                    continue
                if running and session.action == 'cancel':
                    continue
                previous = session.worker
                session.state.update(phase='stopping', busy=True,
                                     message='正在结束本页拥有的连接')
                try:
                    self._start_worker(session, 'cancel', self._cancel_worker, previous)
                except Exception as error:
                    failures.append(f'{session.car["name"]}: 无法提交关闭任务：{error}')
            workers = list(self._threads)
        deadline = time.monotonic() + self.CLOSE_TIMEOUT
        for worker in workers:
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
        with self._lock:
            for name, session in self._sessions.items():
                if session.worker is not None and session.worker.is_alive():
                    failures.append(f'{name}: 连接工作线程尚未退出')
                if session.process is not None and session.process.poll() is None:
                    failures.append(f'{name}: {session.state["error"] or "本页底盘进程仍未完成清理"}')
                if session.cleanup_unknown:
                    failures.append(f'{name}: {session.state["error"]}')
        if failures:
            raise RuntimeError('连接清理未完成：' + '; '.join(failures))
