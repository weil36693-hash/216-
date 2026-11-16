"""Full timed-job lifecycle and HTTP boundary using simulated cars only."""

from copy import deepcopy
from http.client import HTTPConnection
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from leap_fleet.motion_channels import MotionChannels
from leap_fleet.web_console import Console, ConsoleServer, DemoMonitor
from leap_fleet.trajectory import parse_csv


class _TrajectoryFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        config = Path(self.temp.name) / 'fleet.json'
        config.write_text(json.dumps({'cars': [{'name': name, 'domain_id': index + 61}
            for index, name in enumerate(('car04', 'car06'))]}))
        self.monitor = DemoMonitor(config)
        self.sent = []

        def factory(cars):
            fleet = self.monitor.fleet(cars)
            original = fleet.set_velocities

            def publish(commands):
                self.sent.append((time.monotonic(), dict(commands)))
                original(commands)

            fleet.set_velocities = publish
            return fleet

        self.channels = MotionChannels(factory, stop_duration=.08, stop_rate=50)
        self.console = Console(self.monitor, factory, mode='demo', channels=self.channels)
        self.console.STOP_STABLE_TIME = .04
        self.console.STOP_TIMEOUT = .6
        self.addCleanup(self.console.close)

    def csv(self, durations=(.3, .65)):
        lines = ['car_id,t,x,y']
        for name, duration in zip(('04', 'car06'), durations):
            car = 'car04' if name == '04' else name
            p = self.monitor.cars[car]['mocap']['position']
            lines.extend((f'{name},0,{p["x"]},{p["y"]}',
                          f'{name},{duration},{p["x"] + .03 * duration},{p["y"]}'))
        return '\n'.join(lines)

    def imported(self, durations=(.3, .65)):
        return self.console.import_trajectory(self.csv(durations), '两车.csv')['id']

    def wait(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(predicate(), self.console.snapshot()['job'])

    def done(self):
        self.wait(lambda: self.console.snapshot()['job']['status'] not in
                  ('starting', 'running', 'stopping'))
        return self.console.snapshot()['job']


class TrajectoryConsoleTests(_TrajectoryFixture):
    def test_eight_car_csv_runs_with_one_clock_and_stops_all(self):
        names = ['car03', 'car04', 'car05', 'car06', 'car07', 'car09', 'car10', 'car11']
        path = Path(self.temp.name) / 'eight.json'
        path.write_text(json.dumps({'cars': [{'name': name, 'domain_id': i + 61}
                                            for i, name in enumerate(names)]}))
        monitor = DemoMonitor(path)
        channels = MotionChannels(monitor.fleet, stop_duration=.08, stop_rate=50)
        console = Console(monitor, monitor.fleet, mode='demo', channels=channels)
        console.STOP_STABLE_TIME = .04
        console.STOP_TIMEOUT = .6
        self.addCleanup(console.close)
        lines = ['car_id,t,x,y']
        starts = {}
        for name in names:
            pose = monitor.cars[name]['mocap']
            pose['publishers'] = 1
            x, y = pose['position']['x'], pose['position']['y']
            starts[name] = x
            lines.extend((f'{name},0,{x},{y}', f'{name},0.5,{x + .015},{y}'))
        identifier = console.import_trajectory('\n'.join(lines))['id']
        console.start_trajectory(identifier, names, True)
        deadline = time.monotonic() + 2
        while console.snapshot()['job']['status'] in ('starting', 'running', 'stopping') and time.monotonic() < deadline:
            time.sleep(.01)
        job = console.snapshot()['job']
        self.assertEqual(job['status'], 'completed', job)
        self.assertEqual(len({item['stream_started_at'] for item in channels.snapshot().values()}), 1)
        self.assertTrue(all(item['stop_status'] == 'confirmed' for item in job['car_results'].values()))
        for car in monitor.snapshot():
            self.assertGreater(car['mocap']['position']['x'], starts[car['id']] + .001)
            self.assertEqual(car['measured_v'], 0.)

    def test_thread_start_failure_leaves_a_terminal_job_without_motion(self):
        identifier = self.imported()
        with patch('leap_fleet.trajectory_console.threading.Thread.start', side_effect=RuntimeError('no thread')):
            with self.assertRaises(RuntimeError):
                self.console.start_trajectory(identifier, ['car04'], True)
        self.assertEqual(self.console.snapshot()['job']['status'], 'failed')
        self.assertFalse(self.console._busy())
        self.assertFalse(self.sent)
        self.assertFalse(self.channels.snapshot())

    def test_import_check_and_generate_are_read_only(self):
        identifier = self.imported()
        check = self.console.check_trajectory(identifier, ['car04', 'car06'], True)
        self.assertTrue(check['ready'], check)
        self.assertTrue(self.console.check_trajectory(identifier, ['car04'], False)['ready'])
        generated = self.console.generate_trajectory(['car04', 'car06'], .5, 20)
        data = parse_csv(generated['csv'], ['car04', 'car06'])
        self.assertEqual(set(data.tracks), {'car04', 'car06'})
        self.assertAlmostEqual(data.tracks['car04'].end[0] - data.tracks['car04'].start[0], .5)
        self.assertFalse(self.sent)
        self.assertFalse(self.channels.snapshot())
        self.assertIsNone(self.console.snapshot()['job'])
        self.assertNotIn('points', self.console.snapshot()['trajectory']['cars'][0])
        self.assertIn('points', self.console.trajectory_view()['cars'][0])

    def test_generator_and_stream_have_no_fixed_point_15_speed_cap(self):
        generated = self.console.generate_trajectory(['car04'], .5, 2.2)
        track = parse_csv(generated['csv'], ['car04']).tracks['car04']
        self.assertGreater(track.max_speed, .15)
        identifier = self.console.import_trajectory(generated['csv'])['id']
        self.assertTrue(self.console.check_trajectory(identifier, ['car04'])['ready'])
        self.console.start_trajectory(identifier, ['car04'])
        self.wait(lambda: self.console.snapshot()['job']['status'] not in
                  ('starting', 'running', 'stopping'), timeout=4)
        self.assertEqual(self.console.snapshot()['job']['status'], 'completed')
        self.assertGreater(max(cmd.v for _, commands in self.sent for cmd in commands.values()), .15)
        self.assertEqual(self.monitor.snapshot()[0]['measured_v'], 0.)

    def test_invalid_replacement_revokes_old_import(self):
        identifier = self.imported()
        with self.assertRaises(ValueError):
            self.console.import_trajectory('car_id,t,x,y\nunknown,0,0,0', 'invalid.csv')
        self.assertIsNone(self.console.trajectory_view())
        with self.assertRaises(ValueError):
            self.console.start_trajectory(identifier, ['car04'], True)
        self.assertFalse(self.sent)

    def test_each_car_finishes_on_its_own_timeline(self):
        identifier = self.imported()
        self.console.start_trajectory(identifier, ['car04', 'car06'], True)
        job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertTrue(all(result['stop_status'] == 'confirmed' for result in job['car_results'].values()))
        channels = self.channels.snapshot()
        self.assertEqual(channels['car04']['stream_started_at'], channels['car06']['stream_started_at'])
        first_end = channels['car04']['stream_deadline']
        moving = [(t, name) for t, commands in self.sent for name, command in commands.items()
                  if command.v != 0 or command.w != 0]
        self.assertEqual({name for _, name in moving}, {'car04', 'car06'})
        self.assertTrue(any(name == 'car06' and t > first_end + .05 for t, name in moving))
        self.assertFalse(any(name == 'car04' and t > first_end + .03 for t, name in moving))
        self.assertTrue(all(item['status'] == 'completed' for item in job['trajectory_progress'].values()))

    def test_selected_subset_never_opens_unselected_channel(self):
        identifier = self.imported()
        self.console.start_trajectory(identifier, ['car04'], True)
        self.assertEqual(self.done()['status'], 'completed')
        self.assertEqual(set(self.channels.snapshot()), {'car04'})
        self.assertTrue(all(set(commands) == {'car04'} for _, commands in self.sent))

    def test_missing_or_invalid_current_pose_cannot_start_feedback_controller(self):
        identifier = self.imported()
        states = self.monitor.snapshot()
        changes = [{'status': 'stale'}, {'age_s': .5}, {'yaw_rad': math.nan},
                   {'position': {'x': math.inf, 'y': 0}}, {'position': None}]
        for pose in changes:
            with self.subTest(pose=pose):
                changed = deepcopy(states)
                changed[0]['mocap'].update(pose)
                with patch.object(self.monitor, 'snapshot', return_value=changed):
                    check = self.console.check_trajectory(identifier, ['car04'])
                    self.assertFalse(check['ready'])
                    self.assertTrue(check['errors'])
                    with self.assertRaises(ValueError):
                        self.console.start_trajectory(identifier, ['car04'])
        self.assertFalse(self.sent)

    def test_risk_warnings_do_not_require_confirmation_or_block_execution(self):
        identifier = self.imported()
        with self.assertRaises(ValueError):
            self.console.start_trajectory(identifier, ['car04'], 'true')
        car = self.monitor.cars['car04']
        car.update(status='offline', cmd_publishers=2, cmd_subscribers=2, measured_v=.02)
        car['mocap'].update(publishers=3, yaw_rad=math.pi, binding_confirmed=False)
        car['mocap']['position']['x'] += 1
        self.console._unconfirmed.add('car04')
        self.monitor.cars['car06']['mocap']['frame_id'] = 'another_world'
        check = self.console.check_trajectory(identifier, ['car04', 'car06'])
        self.assertTrue(check['ready'], check)
        self.assertFalse(check['errors'])
        messages = '；'.join(check['warnings'])
        for warning in ('底盘连接', '其它速度', '位姿来源', '停车反馈', '尚未确认静止',
                        '距轨迹起点', '车头', '坐标系名称不同', '对应关系'):
            self.assertIn(warning, messages)
        self.assertFalse(self.sent)
        car.update(status='connected', measured_v=0.)
        self.console.start_trajectory(identifier, ['car04', 'car06'])
        job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertFalse(job['operator_confirmed'])
        self.assertTrue(job['warnings'])
        self.assertTrue(any(cmd.v or cmd.w for _, commands in self.sent for cmd in commands.values()))

    def test_direct_start_needs_no_preflight_and_tolerates_missing_frame_and_orientation(self):
        identifier = self.imported()
        snapshot = self.monitor.snapshot

        def missing_metadata():
            states = snapshot()
            states[0]['mocap'].update(frame_id='', orientation=None)
            return states

        with patch.object(self.monitor, 'snapshot', side_effect=missing_metadata):
            self.console.start_trajectory(identifier, ['car04'])
            job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertIn('世界坐标系名称', '；'.join(job['warnings']))

    def test_mocap_dropout_keeps_both_cars_moving_and_recovers_without_new_job(self):
        identifier = self.imported((1.5, 1.5))
        snapshot = self.monitor.snapshot
        missing = False

        def intermittent_pose():
            states = snapshot()
            if missing:
                states[0]['mocap'] = {'status': 'missing', 'position': None, 'age_s': None}
            return states

        with patch.object(self.monitor, 'snapshot', side_effect=intermittent_pose):
            original = self.console.start_trajectory(identifier, ['car04', 'car06'])
            self.wait(lambda: all(any(commands.get(name) and commands[name].v > 0
                                     for _, commands in self.sent) for name in ('car04', 'car06')))
            started = self.console.snapshot()['job']['started_at']
            deadline = self.channels.snapshot()['car04']['stream_deadline']
            missing, gap_start = True, time.monotonic()
            self.wait(lambda: self.console.snapshot()['job']['trajectory_progress']['car04']['localization_mode'] == 'estimated')
            time.sleep(.4)  # Longer than both the old pose-age cutoff and command TTL.
            running = self.console.snapshot()['job']
            self.assertEqual(running['status'], 'running', running)
            self.assertGreater(running['trajectory_progress']['car04']['localization_gap_s'], .3)
            during = [(t, commands) for t, commands in self.sent if t >= gap_start]
            for name in ('car04', 'car06'):
                values = [commands[name].v for _, commands in during if name in commands]
                self.assertGreater(len(values), 4)
                self.assertTrue(all(v > 0 for v in values), values)
            missing = False
            self.wait(lambda: self.console.snapshot()['job']['trajectory_progress']['car04']['localization_mode'] == 'mocap')
            recovered = self.console.snapshot()['job']
            self.assertEqual(recovered['id'], original['id'])
            self.assertEqual(recovered['started_at'], started)
            self.assertEqual(self.channels.snapshot()['car04']['stream_deadline'], deadline)
            self.assertTrue(any('动捕已恢复' in e['message'] for e in self.console.snapshot()['events']))
            job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertIn('推算继续', '；'.join(job['warnings']))
        self.assertTrue(all(car['measured_v'] == 0 for car in self.monitor.snapshot()))

    def test_invalid_pose_and_odom_staleness_do_not_interrupt_and_manual_stop_wins(self):
        identifier = self.imported((2., 2.))
        snapshot = self.monitor.snapshot
        invalid = False

        def invalid_feedback():
            states = snapshot()
            if invalid:
                states[0].update(status='stale', age_s=10.)
                states[0]['mocap'].update(position={'x': math.nan, 'y': math.inf}, yaw_rad=None)
            return states

        with patch.object(self.monitor, 'snapshot', side_effect=invalid_feedback):
            self.console.start_trajectory(identifier, ['car04', 'car06'])
            self.wait(lambda: any(command.v > 0 for _, commands in self.sent for command in commands.values()))
            invalid = True
            self.wait(lambda: self.console.snapshot()['job']['trajectory_progress']['car04']['localization_mode'] == 'estimated')
            time.sleep(.35)
            self.assertEqual(self.console.snapshot()['job']['status'], 'running')
            stopped_at = time.monotonic()
            self.console.stop()
            invalid = False  # Feedback can recover, but the old job may not resume.
            job = self.done()
        self.assertEqual(job['status'], 'stopped', job)
        self.assertFalse(any(cmd.v or cmd.w for t, commands in self.sent if t > stopped_at + .03
                             for cmd in commands.values()))

    def test_mocap_missing_until_end_still_finishes_on_csv_deadline(self):
        identifier = self.imported((.65, .9))
        self.console.start_trajectory(identifier, ['car04', 'car06'])
        self.wait(lambda: any(command.v > 0 for _, commands in self.sent for command in commands.values()))
        self.monitor.cars['car04']['mocap']['status'] = 'stale'
        self.monitor.cars['car06']['mocap']['status'] = 'stale'
        job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        for name, channel in self.channels.snapshot().items():
            self.assertFalse(any(cmd.v or cmd.w for t, commands in self.sent
                                 if t > channel['stream_deadline'] + .03
                                 for car, cmd in commands.items() if car == name))

    def test_transient_publish_failure_retries_without_stopping_other_car(self):
        factory = self.channels._factory
        fail = False

        def intermittent_factory(cars):
            fleet = factory(cars)
            original = fleet.set_velocities

            def publish(commands):
                if fail and 'car04' in commands and commands['car04'].v > 0:
                    raise RuntimeError('temporary Wi-Fi interruption')
                original(commands)

            fleet.set_velocities = publish
            return fleet

        self.channels._factory = intermittent_factory
        identifier = self.imported((1.5, 1.5))
        self.console.start_trajectory(identifier, ['car04', 'car06'])
        self.wait(lambda: all(any(commands.get(name) and commands[name].v > 0
                                 for _, commands in self.sent) for name in ('car04', 'car06')))
        fail, since = True, time.monotonic()
        self.wait(lambda: bool(self.channels.snapshot()['car04']['transport_warning']))
        time.sleep(.2)
        self.assertEqual(self.console.snapshot()['job']['status'], 'running')
        self.assertTrue(self.channels.snapshot()['car04']['active'])
        other = [commands['car06'].v for t, commands in self.sent if t > since and 'car06' in commands]
        self.assertGreater(len(other), 2)
        self.assertTrue(all(v > 0 for v in other))
        fail = False
        self.wait(lambda: not self.channels.snapshot()['car04']['error'])
        job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertIn('temporary Wi-Fi interruption', '；'.join(job['warnings']))
        self.assertIn('temporary Wi-Fi interruption', self.channels.snapshot()['car04']['last_error'])

    def test_new_competing_writer_warns_and_trajectory_continues(self):
        identifier = self.imported((1.5, 1.5))
        self.console.start_trajectory(identifier, ['car04'], True)
        self.wait(lambda: any(command.v > 0 for _, commands in self.sent for command in commands.values()))
        self.monitor.cars['car04']['cmd_publishers'] = 2
        job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertIn('其它速度', '；'.join(job['warnings']))

    def test_mocap_heading_jump_warns_and_trajectory_continues(self):
        identifier = self.imported((1.5, 1.5))
        self.console.start_trajectory(identifier, ['car04'], True)
        self.wait(lambda: any(command.v > 0 for _, commands in self.sent for command in commands.values()))
        self.monitor.cars['car04']['mocap']['yaw_rad'] = math.pi
        job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertIn('方向变化较大', '；'.join(job['warnings']))

    def test_manual_stop_and_replacement_during_run(self):
        identifier = self.imported((1.5, 1.5))
        self.console.start_trajectory(identifier, ['car04', 'car06'], True)
        self.wait(lambda: self.console.snapshot()['job']['status'] == 'running')
        with self.assertRaises(RuntimeError):
            self.console.import_trajectory(self.csv(), 'another.csv')
        self.assertEqual(self.console.trajectory_view()['id'], identifier)
        self.console.stop('job')
        self.assertEqual(self.done()['status'], 'stopped')
        time.sleep(.1)
        self.assertTrue(all(car['measured_v'] == 0 for car in self.monitor.snapshot()))

    def test_controller_stall_does_not_block_stop_api(self):
        from leap_fleet.trajectory_console import control_velocity
        entered, release = threading.Event(), threading.Event()

        def blocked(*args, **kwargs):
            entered.set()
            release.wait(2)
            return control_velocity(*args, **kwargs)

        identifier = self.imported((1.5, 1.5))
        with patch('leap_fleet.trajectory_console.control_velocity', side_effect=blocked):
            self.console.start_trajectory(identifier, ['car04'], True)
            self.assertTrue(entered.wait(1))
            before = time.monotonic()
            self.console.stop()
            self.assertLess(time.monotonic() - before, .15)
            release.set()
            self.assertEqual(self.done()['status'], 'stopped')
        self.assertFalse(any(cmd.v or cmd.w for _, commands in self.sent for cmd in commands.values()))

    def test_slow_calculation_with_fresh_pose_and_live_stream_only_warns(self):
        from leap_fleet.trajectory_console import control_velocity
        first = True

        def slow_once(*args, **kwargs):
            nonlocal first
            if first:
                first = False
                time.sleep(.17)
            return control_velocity(*args, **kwargs)

        identifier = self.imported((.65, .65))
        with patch('leap_fleet.trajectory_console.control_velocity', side_effect=slow_once):
            self.console.start_trajectory(identifier, ['car04'])
            job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertIn('150 ms', '；'.join(job['warnings']))

    def test_controller_pause_beyond_old_ttl_does_not_end_stream_and_stop_still_wins(self):
        from leap_fleet.trajectory_console import control_velocity
        entered, release = threading.Event(), threading.Event()

        def blocked(*args, **kwargs):
            entered.set()
            release.wait(2)
            return control_velocity(*args, **kwargs)

        identifier = self.imported((1.5, 1.5))
        with patch('leap_fleet.trajectory_console.control_velocity', side_effect=blocked):
            self.console.start_trajectory(identifier, ['car04'], True)
            self.assertTrue(entered.wait(1))
            time.sleep(.4)
            channel = self.channels.snapshot()['car04']
            self.assertTrue(channel['active'])
            self.assertFalse(channel['stream_expired'])
            self.assertFalse(channel['stop_pending'])
            self.console.stop()
            release.set()
            job = self.done()
        self.assertEqual(job['status'], 'stopped', job)
        self.assertFalse(any(cmd.v or cmd.w for _, commands in self.sent for cmd in commands.values()))

    def test_transport_failure_is_recoverable_and_manual_stop_during_calculation_wins(self):
        from leap_fleet.trajectory_console import control_velocity
        publish_entered, publish_release = threading.Event(), threading.Event()
        compute_entered, compute_release = threading.Event(), threading.Event()
        factory = self.channels._factory

        def failing_factory(cars):
            fleet = factory(cars)
            original = fleet.set_velocities

            def publish(commands):
                if any(cmd.v or cmd.w for cmd in commands.values()):
                    publish_entered.set()
                    publish_release.wait(2)
                    raise RuntimeError('synthetic transport failure')
                original(commands)

            fleet.set_velocities = publish
            return fleet

        def blocked(*args, **kwargs):
            if publish_entered.is_set():
                compute_entered.set()
                compute_release.wait(2)
            return control_velocity(*args, **kwargs)

        self.channels._factory = failing_factory
        identifier = self.imported((1.5, 1.5))
        with patch('leap_fleet.trajectory_console.control_velocity', side_effect=blocked):
            self.console.start_trajectory(identifier, ['car04'], True)
            try:
                self.assertTrue(compute_entered.wait(1))
                publish_release.set()
                self.wait(lambda: bool(self.channels.snapshot()['car04']['error']))
                self.wait(lambda: not self.channels.snapshot()['car04']['stop_pending'])
                self.console.stop()
                compute_release.set()
                job = self.done()
            finally:
                publish_release.set()
                compute_release.set()
        self.assertEqual(job['status'], 'stopped', job)
        self.assertIn('synthetic transport failure', self.channels.snapshot()['car04']['last_error'])


class TrajectoryHttpTests(_TrajectoryFixture):
    def setUp(self):
        super().setUp()
        self.server = ConsoleServer(self.console, port=0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close_server)

    def close_server(self):
        self.server.shutdown()
        self.thread.join(2)
        self.server.server_close()

    def request(self, method, path, payload=None, authorized=True):
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        try:
            headers = {'Content-Type': 'application/json'}
            if authorized:
                headers['X-CSRF-Token'] = self.server.csrf_token
            connection.request(method, path, json.dumps(payload) if payload is not None else None, headers)
            result = connection.getresponse()
            return result.status, json.loads(result.read())
        finally:
            connection.close()

    def test_http_import_preflight_run_and_full_preview(self):
        status, data = self.request('POST', '/api/trajectory/import',
                                    {'csv': self.csv(), 'filename': 'two.csv'})
        self.assertEqual(status, 200, data)
        identifier = data['trajectory']['id']
        body = {'trajectory_id': identifier, 'cars': ['car04', 'car06'], 'operator_confirmed': True}
        status, data = self.request('POST', '/api/trajectory/check', body)
        self.assertEqual(status, 200, data)
        self.assertTrue(data['ready'])
        self.assertFalse(self.sent)
        self.assertIn('points', self.request('GET', '/api/trajectory')[1]['trajectory']['cars'][0])
        status, data = self.request('POST', '/api/trajectory/run', body)
        self.assertEqual(status, 200, data)
        self.assertEqual(self.done()['status'], 'completed')

    def test_new_mutating_routes_require_csrf_and_exact_fields(self):
        for route, body in (
                ('import', {'csv': self.csv()}),
                ('generate', {'cars': ['car04']}),
                ('check', {'trajectory_id': 'bad', 'cars': ['car04']}),
                ('run', {'trajectory_id': 'bad', 'cars': ['car04']})):
            self.assertEqual(self.request('POST', '/api/trajectory/' + route, body, False)[0], 403)
            self.assertEqual(self.request('POST', '/api/trajectory/' + route, {**body, 'typo': True})[0], 400)
        self.assertFalse(self.sent)

    def test_http_direct_run_accepts_risks_without_check_or_confirmation(self):
        status, data = self.request('POST', '/api/trajectory/import', {'csv': self.csv()})
        self.assertEqual(status, 200, data)
        self.monitor.cars['car04']['mocap']['publishers'] = 3
        self.monitor.cars['car04']['mocap']['yaw_rad'] = math.pi
        status, data = self.request('POST', '/api/trajectory/run',
            {'trajectory_id': data['trajectory']['id'], 'cars': ['car04']})
        self.assertEqual(status, 200, data)
        job = self.done()
        self.assertEqual(job['status'], 'completed', job)
        self.assertTrue(job['warnings'])

    def test_csv_larger_than_old_json_limit_is_accepted_without_motion(self):
        csv = self.csv((1500, 1500)) + '\n' * 70000
        status, data = self.request('POST', '/api/trajectory/import', {'csv': csv})
        self.assertEqual(status, 200, data)
        self.assertFalse(self.sent)


if __name__ == '__main__':
    unittest.main()
