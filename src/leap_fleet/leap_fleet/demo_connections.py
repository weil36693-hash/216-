"""In-memory connection workflow for the explicit demo mode; no ROS or I/O."""

from copy import deepcopy
import threading


class DemoConnections:
    def __init__(self, monitor):
        self.monitor = monitor
        self._lock = threading.RLock()
        self._closed = False
        self._workers, self._cancel, self._states = {}, {}, {}
        self._needs_config, self._unreachable = set(), set()
        for index, car in enumerate(monitor.snapshot()):
            if index == 19:
                self._needs_config.add(car['id'])
            if index == 24:
                self._unreachable.add(car['id'])
            self._states[car['id']] = {
                'id': car['id'], 'phase': 'external' if car['status'] == 'connected' else 'idle',
                'message': '模拟：已连接，由外部管理' if car['status'] == 'connected' else '模拟：尚未检测',
                'busy': False, 'owned': False, 'current': None, 'target': None,
                'log_file': None, 'log_tail': '', 'error': None}

    def snapshot(self):
        with self._lock:
            return {'available': True, 'error': None,
                    'cars': deepcopy(list(self._states.values()))}

    def busy_for(self, cars):
        with self._lock:
            return any(self._states.get(name, {}).get('busy', False) for name in cars)

    def request(self, action, cars):
        if not isinstance(action, str) or action not in ('check', 'connect', 'configure', 'cancel'):
            raise ValueError('未知连接操作')
        if (not isinstance(cars, list) or not cars or
                any(not isinstance(name, str) for name in cars) or len(cars) != len(set(cars))):
            raise ValueError('cars 必须是非空且不重复的车辆编号列表')
        with self._lock:
            if self._closed:
                raise RuntimeError('连接管理已关闭')
            if any(name not in self._states for name in cars):
                raise ValueError('连接目标含未知车辆')
            for name in cars:
                state = self._states[name]
                if state['busy']:
                    if action == 'cancel':
                        self._cancel[name].set()
                    continue
                if action == 'configure' and state['phase'] != 'needs_config':
                    raise ValueError(f'{name}: 请先检测并核对通信参数差异')
            for name in cars:
                state = self._states[name]
                if state['busy']:
                    continue
                if action == 'cancel' and not state['owned']:
                    continue
                state.update(busy=True, phase='stopping' if action == 'cancel' else 'checking',
                             message='模拟：正在关闭本页连接' if action == 'cancel' else '模拟：正在检测通信',
                             error=None)
                event = threading.Event()
                self._cancel[name] = event
                worker = threading.Thread(target=self._run, args=(name, action, event), daemon=True)
                self._workers[name] = worker
                worker.start()
            return self.snapshot()

    def _update(self, name, **values):
        with self._lock:
            state = self._states[name]
            state.update(values)
            if 'message' in values:
                state['log_tail'] = (state['log_tail'] + values['message'] + '\n')[-3000:]

    def _disconnect(self, name):
        with self.monitor._lock:
            self.monitor.cars[name].update(status='offline', age_s=None,
                                           measured_v=None, measured_w=None, cmd_subscribers=0)
        self._update(name, phase='stopped', busy=False, owned=False,
                     message='模拟：本页连接已关闭，动捕观测独立保留')

    def _run(self, name, action, cancel):
        try:
            if cancel.wait(0.2):
                if self._states[name]['owned']:
                    self._disconnect(name)
                else:
                    self._update(name, phase='stopped', busy=False, message='模拟：连接已取消')
                return
            if action == 'cancel':
                self._disconnect(name)
                return
            car = next(car for car in self.monitor.snapshot() if car['id'] == name)
            if car['status'] == 'connected':
                owned = self._states[name]['owned']
                self._update(name, phase='connected' if owned else 'external', busy=False,
                             message='模拟：复用现有连接，没有重复启动')
                return
            if name in self._unreachable:
                self._update(name, phase='failed', busy=False, message='模拟：车辆状态接口不可达',
                             error='请检查这辆车的供电、IP 和局域网')
                return
            target = {'comm_mode': 'micro_ros', 'microros_agent_ip': '10.1.1.249',
                      'microros_agent_port': 20000 + car['domain_id']}
            current = dict(target)
            if name in self._needs_config:
                current['microros_agent_port'] = 8888
            self._update(name, current=current, target=target)
            if name in self._needs_config and action != 'configure':
                self._update(name, phase='needs_config', busy=False,
                             message='模拟：通信参数不匹配，请核对后应用配置')
                return
            if action == 'check':
                self._update(name, phase='idle', busy=False, message='模拟：网络和通信配置正常，可以连接')
                return
            for phase, message in [('configuring' if action == 'configure' else 'starting',
                                    '模拟：应用通信配置' if action == 'configure' else '模拟：启动底盘服务'),
                                   ('waiting_raw', '模拟：等待里程计与 IMU'),
                                   ('waiting_filtered', '模拟：等待底盘融合数据')]:
                self._update(name, phase=phase, message=message)
                if cancel.wait(0.2):
                    self._update(name, phase='stopped', busy=False, owned=False, message='模拟：连接已取消')
                    return
            with self._lock:
                if cancel.is_set() or self._closed:
                    self._update(name, phase='stopped', busy=False, message='模拟：连接已取消')
                    return
                with self.monitor._lock:
                    self.monitor.cars[name].update(status='connected', age_s=0.04,
                                                   measured_v=0.0, measured_w=0.0, cmd_subscribers=1)
                self._needs_config.discard(name)
                self._update(name, phase='connected', busy=False, owned=True, current=target,
                             message='模拟：底盘连接就绪，由本页管理')
        except Exception as error:
            self._update(name, phase='failed', busy=False, error=str(error), message='模拟：连接操作失败')

    def close(self):
        with self._lock:
            self._closed = True
            for event in self._cancel.values():
                event.set()
            workers = list(self._workers.values())
        for worker in workers:
            worker.join(timeout=2)
        for name, state in self._states.items():
            if state['owned']:
                self._disconnect(name)
