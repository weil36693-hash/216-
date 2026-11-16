"""Timed CSV jobs for the local console; import and preparation are read-only.

The mixin uses the console's existing reservation, independent command workers,
and feedback-based stop workflow. It never creates a second command publisher.
"""

from copy import deepcopy
import math
from pathlib import PurePosixPath
import secrets
import threading
import time

from .control import Velocity
from .trajectory import control_velocity, parse_csv, straight_csv
from .trajectory_pose import PoseEstimate


MAX_CSV_BYTES = 8 * 1024 * 1024


def finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class TrajectoryConsoleMixin:
    TRAJECTORY_MAX_W = 0.6
    TRAJECTORY_POSE_AGE = 0.3
    TRAJECTORY_START_DISTANCE = 0.10
    TRAJECTORY_START_HEADING = 0.35

    def _init_trajectories(self):
        self._trajectory = None
        self._trajectory_generation = 0

    def trajectory_view(self, *, points=True):
        with self._lock:
            if self._trajectory is None:
                return None
            summary = self._trajectory['summary']
            if points:
                return deepcopy(summary)
            return {**{k: v for k, v in summary.items() if k != 'cars'},
                    'cars': [{k: deepcopy(v) for k, v in car.items() if k != 'points'}
                             for car in summary['cars']]}

    def import_trajectory(self, csv, filename='trajectory.csv'):
        with self._lock:
            if self._closed or self._busy():
                raise RuntimeError('请等待当前任务结束后再导入轨迹')
            # An invalid replacement must not leave the previous file runnable.
            self._trajectory = None
            self._trajectory_generation += 1
            generation = self._trajectory_generation
        if not isinstance(csv, str) or not csv.strip():
            raise ValueError('CSV 内容不能为空')
        if len(csv.encode('utf-8')) > MAX_CSV_BYTES:
            raise ValueError('CSV 文件不能超过 8 MiB')
        if not isinstance(filename, str) or len(filename) > 200:
            raise ValueError('文件名无效')
        filename = PurePosixPath(filename.replace('\\', '/')).name
        if not filename or any(ord(char) < 32 for char in filename):
            raise ValueError('文件名无效')
        names = [car['id'] for car in self.monitor.snapshot()]
        trajectory = parse_csv(csv, names)
        summary = {'id': secrets.token_hex(12), 'filename': filename,
                   **trajectory.summary()}
        with self._lock:
            if generation != self._trajectory_generation:
                raise RuntimeError('此导入已被更新的文件替代')
            if self._closed or self._busy():
                raise RuntimeError('控制台状态已变化，请重新导入')
            self._trajectory = {'data': trajectory, 'summary': summary}
            self._event('info', f'已解析轨迹 {filename}：{len(trajectory.tracks)} 辆车；尚未启动')
            return deepcopy(summary)

    def _selected_trajectory(self, trajectory_id, cars):
        if (not isinstance(trajectory_id, str) or self._trajectory is None
                or trajectory_id != self._trajectory['summary']['id']):
            raise ValueError('轨迹已失效或未导入，请重新导入 CSV')
        if (not isinstance(cars, list) or not cars
                or any(not isinstance(name, str) for name in cars)
                or len(cars) != len(set(cars))):
            raise ValueError('请选择不重复的轨迹车辆编号')
        tracks = self._trajectory['data'].tracks
        if any(name not in tracks for name in cars):
            raise ValueError('所选车辆不在当前 CSV 中')
        return {name: tracks[name] for name in cars}

    def _pose_reasons(self, name, state, *, frame=None):
        """Only missing/invalid current input prevents calculating feedback."""
        reasons = []
        pose = state.get('mocap') or {}
        if (pose.get('status') != 'fresh' or not finite(pose.get('age_s'))
                or not 0 <= pose['age_s'] <= self.TRAJECTORY_POSE_AGE):
            reasons.append('没有可用的实时动捕位姿，无法计算当前运动指令')
        position = pose.get('position') or {}
        if not all(finite(position.get(key)) for key in ('x', 'y')) or not finite(pose.get('yaw_rad')):
            reasons.append('动捕位置或方向缺失/无效，无法计算运动指令')
        return reasons

    def _trajectory_risks(self, state, *, own_writer=0, frame=None):
        """Advisories never veto a trajectory or become a runtime stop rule."""
        risks = []
        if state.get('status') != 'connected':
            risks.append('底盘连接尚未确认；启动后会尝试等待控制通道')
        if not finite(state.get('age_s')) or not 0 <= state['age_s'] <= self.FEEDBACK_MAX_AGE:
            risks.append('底盘里程计反馈不够新鲜')
        if state.get('cmd_subscribers') != 1:
            risks.append('速度订阅端数量不是 1，请留意重复或尚未就绪的连接')
        publishers = state.get('cmd_publishers', 0)
        if finite(publishers) and publishers > own_writer:
            risks.append('存在其它速度指令发布者，可能相互覆盖指令')
        pose = state.get('mocap') or {}
        if pose.get('publishers') != 1:
            risks.append('动捕位姿来源数量不是 1，请留意重复客户端')
        if not pose.get('binding_confirmed'):
            risks.append('车号与动捕刚体的对应关系尚未标记核对')
        orientation = pose.get('orientation') or {}
        if all(finite(orientation.get(key)) for key in ('x', 'y', 'z', 'w')):
            x, y, z, w = (orientation[key] for key in ('x', 'y', 'z', 'w'))
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = math.asin(max(-1., min(1., 2 * (w * y - z * x))))
            if max(abs(roll), abs(pitch)) > 0.35:
                risks.append('动捕刚体倾斜，请留意 +Z 方向与跟踪状态')
        else:
            risks.append('动捕完整姿态不可用，将使用已提供的 XY 和航向')
        if not isinstance(pose.get('frame_id'), str) or not pose['frame_id'].strip():
            risks.append('动捕未提供世界坐标系名称')
        elif frame is not None and pose['frame_id'] != frame:
            risks.append('动捕坐标系名称发生变化')
        return risks

    def _trajectory_check_locked(self, tracks, operator_confirmed, *, preparing=False, states=None):
        if states is None:
            states = {car['id']: car for car in self.monitor.snapshot()}
        channels = self.channels.snapshot()
        results = []
        frames = set()
        for name, track in tracks.items():
            state = states.get(name, {})
            pose = state.get('mocap') or {}
            errors = [] if preparing else self._pose_reasons(name, state)
            channel = channels.get(name, {})
            own_writer = int(bool(channel) and not channel.get('closed', False))
            warnings = self._trajectory_risks(state, own_writer=own_writer)
            if preparing and self._pose_reasons(name, state):
                warnings.append('动捕暂未恢复，将按启动时定位推算继续')
            if self._closed or (self._busy() and not preparing):
                errors.append('控制台已关闭或其它任务尚未结束')
            if name in self._unconfirmed:
                warnings.append('上次停车反馈未确认')
            if name in self._connection_transition or (self.connections is not None
                    and self.connections.busy_for([name])):
                errors.append('车辆正在变更连接，控制通道暂不可用')
            if (not finite(state.get('measured_v')) or not finite(state.get('measured_w'))
                    or abs(state['measured_v']) > 0.01 or abs(state['measured_w']) > 0.03):
                warnings.append('车辆尚未确认静止')
            position = pose.get('position') or {}
            distance = heading_error = None
            if all(finite(position.get(key)) for key in ('x', 'y')):
                distance = math.hypot(position['x'] - track.start[0], position['y'] - track.start[1])
                if distance > self.TRAJECTORY_START_DISTANCE:
                    warnings.append(f'距轨迹起点 {distance:.2f} m，启动后会从当前位置跟踪')
            if track.initial_yaw is not None and finite(pose.get('yaw_rad')):
                heading_error = angle(track.initial_yaw - pose['yaw_rad'])
                if abs(heading_error) > self.TRAJECTORY_START_HEADING:
                    warnings.append(f'车头与起始前进方向相差 {math.degrees(abs(heading_error)):.1f}°')
            if isinstance(pose.get('frame_id'), str) and pose['frame_id'].strip():
                frames.add(pose['frame_id'])
            results.append({'car_id': name, 'ready': not errors, 'errors': errors,
                            'warnings': warnings, 'reasons': errors + warnings,
                            'start_distance_m': distance, 'heading_error_rad': heading_error})
        if len(frames) > 1:
            for result in results:
                message = '所选车辆的动捕坐标系名称不同，请核对是否使用同一世界坐标系'
                result['warnings'].append(message)
                result['reasons'].append(message)
        return {'ready': all(result['ready'] for result in results), 'cars': results,
                'warnings': [f'{item["car_id"]}：{text}' for item in results for text in item['warnings']],
                'errors': [f'{item["car_id"]}：{text}' for item in results for text in item['errors']],
                'frame_id': next(iter(frames)) if len(frames) == 1 else None,
                'duration': max(track.duration for track in tracks.values()),
                'operator_confirmed': operator_confirmed}

    def check_trajectory(self, trajectory_id, cars, operator_confirmed=False):
        if type(operator_confirmed) is not bool:
            raise ValueError('operator_confirmed 必须为布尔值')
        with self._lock:
            tracks = self._selected_trajectory(trajectory_id, cars)
            return self._trajectory_check_locked(tracks, operator_confirmed)

    def start_trajectory(self, trajectory_id, cars, operator_confirmed=False):
        if type(operator_confirmed) is not bool:
            raise ValueError('operator_confirmed 必须为布尔值')
        with self._lock:
            tracks = self._selected_trajectory(trajectory_id, cars)
            states = {car['id']: car for car in self.monitor.snapshot()}
            sampled_at = time.monotonic()
            check = self._trajectory_check_locked(tracks, operator_confirmed, states=states)
            if not check['ready']:
                raise ValueError('；'.join(check['errors']))
            estimates = {name: PoseEstimate(states[name]['mocap'], sampled_at,
                                           max_age=self.TRAJECTORY_POSE_AGE) for name in tracks}
            summary = self._trajectory['summary']
            job = {'id': secrets.token_hex(6), 'kind': 'trajectory', 'status': 'starting',
                   'cars': list(tracks), 'commands': {}, 'duration': check['duration'],
                   'rate': 20, 'error': None, 'started_at': None,
                   'trajectory_id': trajectory_id, 'trajectory_filename': summary['filename'],
                   'operator_confirmed': operator_confirmed, 'frame_id': check['frame_id'],
                   'warnings': check['warnings'],
                   'car_results': self._empty_results(tracks),
                   'trajectory_progress': {name: {'elapsed_s': 0., 'duration': track.duration,
                       'progress': 0., 'status': 'waiting', 'localization_mode': 'pending',
                       'localization_gap_s': 0.} for name, track in tracks.items()}}
            self._job, self._cancel = job, threading.Event()
            self._worker = threading.Thread(target=self._run_trajectory,
                                            args=(job, self._cancel, tracks, estimates), daemon=True)
            self._event('info', f'准备轨迹 {summary["filename"]}：{", ".join(tracks)}',
                        {'job_id': job['id'], 'trajectory_id': trajectory_id})
            if check['warnings']:
                self._event('warning', '轨迹存在风险提示，按请求继续启动',
                            {'job_id': job['id'], 'warnings': check['warnings']})
            try:
                self._worker.start()
            except Exception as exc:
                self._worker = None
                self._cancel.set()
                job.update(status='failed', error=f'轨迹任务线程未能启动：{exc}', car_results={})
                self._event('error', job['error'])
                raise RuntimeError(job['error']) from exc
            return self._job_view()

    def _run_trajectory(self, job, cancel, tracks, estimates):
        error = None
        session = None
        try:
            self._record_job_firmware(job)
            self.channels.ensure(tracks)
            timeout = time.monotonic() + self.DISCOVERY_TIMEOUT
            while not cancel.is_set():
                with self._lock:
                    check = self._trajectory_check_locked(tracks, True, preparing=True)
                    if not check['ready']:
                        raise RuntimeError('准备状态已变化：' + '；'.join(
                            f'{item["car_id"]}：' + '；'.join(item['errors'])
                            for item in check['cars'] if not item['ready']))
                    channels = self.channels.snapshot()
                    failures = [f'{name}: {channels.get(name, {}).get("error")}' for name in tracks
                                if channels.get(name, {}).get('error') and channels.get(name, {}).get('closed')]
                    if failures:
                        raise RuntimeError('; '.join(failures))
                    if all(channels.get(name, {}).get('ready') for name in tracks) and not cancel.is_set():
                        try:
                            session = self.channels.start_stream(list(tracks), duration=job['duration'],
                                rate=20, ttl=None, continue_on_error=True,
                                durations={name: track.duration for name, track in tracks.items()})
                        except RuntimeError:
                            pass  # A discovery probe may race the cached ready flag.
                        else:
                            job.update(status='running', started_at=time.time(),
                                       _deadline=session['deadline'])
                            self._event('info', f'轨迹统一开始计时：{", ".join(tracks)}')
                            break
                if time.monotonic() >= timeout:
                    raise RuntimeError('轨迹控制通道准备超时，未开始运动')
                cancel.wait(0.02)
            if session is None:
                return
            previous = {name: Velocity(0., 0.) for name in tracks}
            previous_poses = {}
            localization_modes = dict.fromkeys(tracks, 'mocap')
            stop_trackers = {}
            finished = set()
            seen_warnings = set(job['warnings'])
            last = session['started_at']
            while not cancel.is_set():
                now = time.monotonic()
                elapsed, dt = now - session['started_at'], max(1e-4, now - last)
                last = now
                progress_updates, result_updates = {}, {}
                warnings = []
                localization_events = []
                channels = self.channels.snapshot()
                try:
                    states = {state['id']: state for state in self.monitor.snapshot()}
                except Exception as exc:
                    states = {}
                    warnings.append(f'暂时无法读取定位状态，按推算继续：{exc}')
                commands = {}
                for name, track in tracks.items():
                    progress = {'elapsed_s': 0., 'duration': track.duration, 'progress': 0.,
                                'status': 'completed' if name in finished else 'running',
                                'localization_mode': localization_modes[name],
                                'localization_gap_s': job['trajectory_progress'][name].get('localization_gap_s', 0.)}
                    progress_updates[name] = progress
                    progress.update(elapsed_s=min(elapsed, track.duration),
                                    progress=min(1., elapsed / track.duration))
                    if elapsed >= track.duration:
                        if name not in finished:
                            self.channels.stop([name])
                            finished.add(name)
                            progress['status'] = 'completed'
                        result_updates[name] = self._feedback_result(name,
                            self.channels.snapshot().get(name, {}), states.get(name, {}),
                            stop_trackers.setdefault(name, {}))
                        continue
                    state = states.get(name, {})
                    warnings.extend(f'{name}：{text}' for text in
                                    self._trajectory_risks(state, own_writer=1, frame=job['frame_id']))
                    pose = state.get('mocap') or {}
                    estimated = estimates[name].update(pose, previous[name], now)
                    mode = estimated['mode']
                    progress.update(localization_mode=mode, localization_gap_s=estimated['gap_s'])
                    if mode != localization_modes[name]:
                        localization_events.append((name, mode))
                        localization_modes[name] = mode
                    if mode == 'estimated':
                        warnings.append(f'{name}：动捕暂不可用，按最后有效位姿和运动指令推算继续')
                    p, yaw = estimated, estimated['yaw']
                    # Judge jumps between newly received poses, not between
                    # control ticks that can see the same cached sample.
                    received = pose.get('received_at_unix')
                    old = previous_poses.get(name)
                    if mode == 'mocap' and old and finite(received) and received != old[0]:
                        interval = max(0., now - old[1])
                        if math.hypot(p['x'] - old[2], p['y'] - old[3]) > 0.5 * interval + 0.05:
                            warnings.append(f'{name}：动捕位置变化较大，请留意跟踪状态')
                        if abs(angle(yaw - old[4])) > 1.2 * interval + 0.12:
                            warnings.append(f'{name}：动捕方向变化较大，请留意跟踪状态')
                    if mode == 'mocap' and (not old or received != old[0]):
                        previous_poses[name] = (received, now, p['x'], p['y'], yaw)
                    channel = channels.get(name, {})
                    transport_error = channel.get('transport_warning') or channel.get('error')
                    if transport_error:
                        warnings.append(f'{name}：控制通道异常，保持任务并重试：{transport_error}')
                    if channel.get('closed'):
                        warnings.append(f'{name}：控制通道已关闭，当前无法发送；其它车辆继续执行')
                        progress['control_status'] = 'unavailable'
                        continue
                    progress['control_status'] = 'retrying' if transport_error else 'active'
                    try:
                        commands[name] = control_velocity(track, elapsed, p['x'], p['y'], yaw,
                            previous[name], min(dt, .1), max_w=self.TRAJECTORY_MAX_W)
                    except (ValueError, ArithmeticError) as exc:
                        commands[name] = previous[name]
                        warnings.append(f'{name}：当前跟踪计算无效，暂时沿用上一条指令：{exc}')
                    progress['status'] = 'running'
                with self._lock:
                    if cancel.is_set():
                        break
                    for name, mode in localization_events:
                        self._event('warning' if mode == 'estimated' else 'info',
                            f'{name} 动捕中断，推算继续，其他车辆不受影响' if mode == 'estimated' else
                            f'{name} 动捕已恢复，继续当前时间点的轨迹跟踪', {'job_id': job['id']})
                    computed_age = time.monotonic() - now
                    if commands and computed_age > 0.15:
                        warnings.append('轨迹计算耗时超过 150 ms，请留意跟踪延迟')
                    new_warnings = [warning for warning in dict.fromkeys(warnings)
                                    if warning not in seen_warnings]
                    if new_warnings:
                        seen_warnings.update(new_warnings)
                        job['warnings'].extend(new_warnings)
                        self._event('warning', '运行中收到风险提示，轨迹继续执行',
                                    {'job_id': job['id'], 'warnings': new_warnings})
                    job['trajectory_progress'].update(progress_updates)
                    job['car_results'].update(result_updates)
                    if len(finished) == len(tracks):
                        break
                    latest = self.channels.snapshot()
                    commands = {name: value for name, value in commands.items()
                                if latest.get(name, {}).get('active')
                                and latest[name].get('stream_id') == session['session_id']}
                    if commands:
                        try:
                            self.channels.update_stream(session['session_id'], commands)
                        except RuntimeError as exc:
                            # A shorter car may reach its worker deadline between
                            # the snapshot and this atomic mailbox update.
                            latest = self.channels.snapshot()
                            normally_done = [name for name in commands
                                if latest.get(name, {}).get('stop_reason') == 'stream duration complete'
                                and latest.get(name, {}).get('stream_id') == session['session_id']]
                            if not normally_done:
                                self._event('warning', f'轨迹指令暂未更新，将继续尝试：{exc}',
                                            {'job_id': job['id']})
                        else:
                            previous.update(commands)
                            job['commands'] = {name: {'v': value.v, 'w': value.w}
                                               for name, value in commands.items()}
                cancel.wait(max(0., 0.05 - (time.monotonic() - now)))
        except Exception as exc:
            error = str(exc)
        finally:
            with self._lock:
                for progress in job['trajectory_progress'].values():
                    if progress['status'] != 'completed':
                        progress['status'] = 'stopped'
            self._finish_stop(job, cancel, error)

    def generate_trajectory(self, cars, distance=0.5, duration=20.):
        if (not isinstance(cars, list) or not cars or any(not isinstance(name, str) for name in cars)
                or len(cars) != len(set(cars))):
            raise ValueError('请选择需要生成轨迹的车辆')
        if not finite(distance) or not 0.01 <= distance <= 5:
            raise ValueError('直线距离必须在 0.01–5 米之间')
        if not finite(duration) or not 1 <= duration <= 3600:
            raise ValueError('轨迹时间必须在 1–3600 秒之间')
        if (math.ceil(duration / .05) + 1) * len(cars) > 100000:
            raise ValueError('生成的采样点过多，请减少车辆数量或时长')
        states = {car['id']: car for car in self.monitor.snapshot()}
        lines = ['car_id,t,x,y']
        for name in cars:
            if name not in states:
                raise ValueError(f'未知车辆：{name}')
            pose = states[name].get('mocap') or {}
            p = pose.get('position') or {}
            if (pose.get('status') != 'fresh' or not finite(pose.get('age_s'))
                    or pose['age_s'] > self.TRAJECTORY_POSE_AGE or not finite(pose.get('yaw_rad'))
                    or not all(finite(p.get(key)) for key in ('x', 'y'))):
                raise ValueError(f'{name} 需要新鲜动捕位置与方向才能生成轨迹')
            yaw = pose['yaw_rad']
            end = p['x'] + distance * math.cos(yaw), p['y'] + distance * math.sin(yaw)
            lines.extend(straight_csv(name, (p['x'], p['y']), end, duration).splitlines()[1:])
        return {'filename': 'fleet_straight.csv', 'csv': '\n'.join(lines) + '\n'}
