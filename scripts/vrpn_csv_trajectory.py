#!/usr/bin/env python3
"""Follow the XY geometry of a mocap CSV using live VRPN feedback.

Recording timestamps describe the source file, not the execution speed. The
vehicle first returns to the absolute start, then follows the whole path at a
bounded speed. Ultrasonic data is neither subscribed to nor used.
"""

import argparse
from collections import deque
import json
import math
import multiprocessing as mp
from pathlib import Path
import signal
import sys
import time
import uuid

from recorded_path import PathGeometry, load_recording
from vrpn_straight_goal import Feedback, clip, endpoints, gate_main, laser_clearance, wrap


def steering(path, progress, x, y, yaw, speed):
    """Pure pursuit with rotation before large heading corrections."""
    remaining = max(0.0, path.length - progress)
    lookahead = max(0.12, 1.8 * speed)
    tx, ty = path.point_at(min(path.length, progress + lookahead))
    look_distance = math.hypot(tx - x, ty - y)
    alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
    if abs(alpha) > 0.35:
        return 0.0, clip(1.5 * alpha, -0.18, 0.18)
    curvature = 2 * math.sin(alpha) / max(look_distance, 0.035)
    linear = min(speed, max(0.015, 0.5 * remaining))
    linear = min(linear, 0.18 / max(abs(curvature), 0.001))
    angular = clip(linear * curvature, -0.25, 0.25)
    return linear, angular


class MotionGuard:
    def __init__(self):
        self.previous = None
        self.last_v = 0.0
        self.last_w = 0.0
        self.turn_window = None
        self.probe = None
        self.verified = False
        self.history = deque()

    def observe(self, state, now):
        x, y, yaw, ox, oy, oyaw = state[:6]
        if self.previous:
            elapsed = now - self.previous[0]
            if math.hypot(x - self.previous[1], y - self.previous[2]) > 0.35 * elapsed + 0.008:
                raise RuntimeError('动捕位置跳变或移动速度异常')
            if abs(wrap(yaw - self.previous[3])) > 0.8 * elapsed + 0.02:
                raise RuntimeError('动捕航向跳变')
            if abs(self.last_w) >= 0.04:
                sign = 1 if self.last_w > 0 else -1
                if self.turn_window is None or self.turn_window[3] != sign:
                    self.turn_window = [yaw, oyaw, 0.0, sign, now]
                # Measured wheel/heading response trails command reversals by
                # about 0.35 s. Anchor the direction check after that response
                # interval rather than counting residual motion as reversal.
                if now - self.turn_window[4] < 0.4:
                    self.turn_window[0:3] = [yaw, oyaw, 0.0]
                else:
                    self.turn_window[2] += self.last_w * elapsed
                    if abs(self.turn_window[2]) > 0.035:
                        turns = wrap(yaw - self.turn_window[0]), wrap(oyaw - self.turn_window[1])
                        if any(turn * sign < -0.008 for turn in turns):
                            raise RuntimeError('实际转向与角速度指令相反')
                        if abs(self.turn_window[2]) > 0.12 and min(abs(t) for t in turns) < 0.005:
                            raise RuntimeError('转向没有响应')
            else:
                self.turn_window = None
        self.previous = now, x, y, yaw, oyaw

    def command(self, state, now, progress, linear, angular):
        x, y, yaw, ox, oy = state[:5]
        if linear > 0:
            if not self.verified:
                linear = min(linear, 0.025)
                if self.probe is None:
                    self.probe = now, x, y, yaw, ox, oy
                moved = math.hypot(x - self.probe[1], y - self.probe[2])
                odom_moved = math.hypot(ox - self.probe[4], oy - self.probe[5])
                if moved >= 0.02:
                    travel = math.atan2(y - self.probe[2], x - self.probe[1])
                    if abs(wrap(travel - self.probe[3])) > 0.30:
                        raise RuntimeError('动捕刚体 +X 与底盘前进方向不一致')
                    if not 0.005 <= odom_moved <= 0.08:
                        raise RuntimeError('动捕和底盘里程计位移不一致')
                    self.verified = True
                elif odom_moved > 0.05 or now - self.probe[0] > 3.5:
                    raise RuntimeError('低速方向验证失败或车轮空转')
            self.history.append((now, progress))
            while self.history and now - self.history[0][0] > 3:
                self.history.popleft()
            if self.history and now - self.history[0][0] > 2.8 and progress - self.history[0][1] < 0.01:
                raise RuntimeError('车辆没有沿路径前进')
        else:
            self.history.clear()
            # A later turn changes the starting heading of an unfinished probe.
            if not self.verified:
                self.probe = None
        linear = min(linear, self.last_v + 0.004)
        angular = clip(angular, self.last_w - 0.03, self.last_w + 0.03)
        self.last_v, self.last_w = linear, angular
        return linear, angular


class Controller:
    def __init__(self, pose_topic, ignore_obstacles=False, speed_limit=0.05):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions

        self.rclpy = rclpy
        self.speed_limit = speed_limit
        self.stopping = False
        signal.signal(signal.SIGINT, lambda *_: setattr(self, 'stopping', True))
        signal.signal(signal.SIGTERM, lambda *_: setattr(self, 'stopping', True))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('vrpn_csv_controller_' + uuid.uuid4().hex[:8])
        self.use_laser = not ignore_obstacles
        self.feedback = Feedback(use_ultrasonic=False, use_laser=self.use_laser)
        self.subscriptions = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for key, topic, kind in (
                ('pose', pose_topic, PoseStamped), ('odom', '/odom', Odometry),
                ('scan', '/scan', LaserScan)):
            if key not in self.feedback.limits:
                continue
            self.subscriptions.append(self.node.create_subscription(
                kind, topic, lambda message, k=key: self.feedback.receive(k, message), qos))
        self.gate = None
        self.pipe = None
        self.gate_name = None
        self.sequence = 0
        self.identity = None
        self.pose_identity = None
        self.state = None
        self.log = None

    def tick(self, duration=0.05):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.stopping:
                raise KeyboardInterrupt
            self.rclpy.spin_once(self.node, timeout_sec=min(0.01, max(0, deadline - time.monotonic())))

    def inspect(self):
        self.state = self.feedback.validate(self.node.get_clock().now().nanoseconds / 1e9)
        actual = endpoints(self.node, self.gate_name, use_ultrasonic=False,
                           use_laser=self.use_laser)
        if self.identity is not None and actual != self.identity:
            raise RuntimeError('底盘命令或里程计端点发生变化')
        publishers = self.node.get_publishers_info_by_topic(self.pose_topic)
        if len(publishers) != 1:
            raise RuntimeError('要求唯一的 VRPN 位姿发布者')
        pose_identity = tuple(publishers[0].endpoint_gid)
        if self.pose_identity is not None and self.pose_identity != pose_identity:
            raise RuntimeError('VRPN 发布者发生变化')
        self.pose_identity = pose_identity
        return self.state

    def ready(self, pose_topic, wait):
        self.pose_topic = pose_topic
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            self.tick()
            if self.feedback.limits.keys() <= self.feedback.data.keys():
                break
        self.inspect()
        self.identity = endpoints(self.node, use_ultrasonic=False, use_laser=self.use_laser)
        velocity = self.feedback.data['odom'][0].twist.twist
        if abs(velocity.linear.x) > 0.01 or abs(velocity.angular.z) > 0.03:
            raise RuntimeError('车辆尚未静止')

    def send(self, linear=0.0, angular=0.0):
        if self.gate is None or not self.gate.is_alive():
            raise RuntimeError('独立停车看门狗已退出')
        self.sequence += 1
        self.pipe.send((time.monotonic(), self.sequence, linear, angular))

    def start_gate(self):
        context = mp.get_context('spawn')
        self.pipe, child = context.Pipe()
        self.gate_name = 'vrpn_velocity_gate_' + uuid.uuid4().hex[:8]
        self.gate = context.Process(target=gate_main,
                                    args=(child, self.gate_name, self.speed_limit))
        self.gate.start()
        child.close()
        deadline = time.monotonic() + 6
        while not self.pipe.poll():
            self.tick()
            if not self.gate.is_alive() or time.monotonic() > deadline:
                raise RuntimeError('停车看门狗无法启动')
        if self.pipe.recv() != 'ready':
            raise RuntimeError('停车看门狗握手异常')
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            self.send()
            self.tick()
        self.inspect()
        directory = Path(__file__).resolve().parents[1] / 'log'
        directory.mkdir(exist_ok=True)
        log_path = directory / ('csv_trajectory_' + time.strftime('%Y%m%d_%H%M%S') + '.jsonl')
        self.log = log_path.open('x')
        from firmware_log import record_for_pose
        record_for_pose(self.pose_topic, log_path.with_suffix('.firmware.json'), kind='csv_trajectory')
        print(f'轨迹执行日志：{log_path}', flush=True)

    def follow(self, path, stage, speed):
        progress = 0.0
        guard = MotionGuard()
        last_tick = time.monotonic()
        last_print = -float('inf')
        deadline = last_tick + path.length / 0.015 + 100
        settled = None
        final = path.point_at(path.length)
        print(f'{stage}：路线 {path.length:.3f} m，速度上限 {speed:.3f} m/s', flush=True)
        while time.monotonic() < deadline:
            self.tick()
            now = time.monotonic()
            state = self.inspect()
            guard.observe(state, now)
            x, y, yaw = state[:3]
            candidate, error = path.project(x, y, progress, 0.02 + 0.20 * (now - last_tick))
            last_tick = now
            if error > 0.10 or candidate < progress - 0.035:
                raise RuntimeError(f'偏离轨迹或倒退：横向距离 {error:.3f} m')
            progress = max(progress, candidate)
            final_distance = math.hypot(x - final[0], y - final[1])
            if settled is not None or (path.length - progress < 0.08 and final_distance <= 0.035):
                self.send()
                if settled is None:
                    settled = now, x, y
                if final_distance > 0.05 or math.hypot(x - settled[1], y - settled[2]) > 0.015:
                    raise RuntimeError('到达后未稳定停车')
                if now - settled[0] >= 1.2:
                    velocity = self.feedback.data['odom'][0].twist.twist
                    if abs(velocity.linear.x) > 0.01 or abs(velocity.angular.z) > 0.03:
                        raise RuntimeError('到达后底盘仍在运动')
                    print(f'{stage}完成：({x:.3f}, {y:.3f})，终点误差 {final_distance:.3f} m', flush=True)
                    return
                continue
            linear, angular = steering(path, progress, x, y, yaw, speed)
            if self.use_laser and linear == 0:
                clearance = laser_clearance(self.feedback.data['scan'][0], rotating=True)
                if clearance < 0.23:
                    raise RuntimeError(f'转向空间不足：最近障碍 {clearance:.3f} m')
            linear, angular = guard.command(state, now, progress, linear, angular)
            self.send(linear, angular)
            measured = self.feedback.data['odom'][0].twist.twist
            self.log.write(json.dumps({'time': now, 'stage': stage, 'x': x, 'y': y,
                                       'yaw': yaw, 'progress': progress, 'length': path.length,
                                       'path_error': error, 'v': linear, 'w': angular,
                                       'measured_v': measured.linear.x,
                                       'measured_w': measured.angular.z,
                                       'end_distance': final_distance}) + '\n')
            self.log.flush()
            if now - last_print >= 3:
                print(f'{stage} {100 * progress / path.length:.1f}%：'
                      f'位置 ({x:.3f}, {y:.3f})，偏线 {error:.3f} m，'
                      f'剩余 {path.length - progress:.3f} m', flush=True)
                last_print = now
        raise RuntimeError(f'{stage}超时')

    def close(self):
        if self.pipe is not None:
            try:
                self.pipe.send(None)
            except (EOFError, BrokenPipeError, OSError):
                pass
            self.pipe.close()
        if self.gate is not None:
            self.gate.join(timeout=3)
            if self.gate.is_alive():
                self.gate.terminate()
                self.gate.join(timeout=2)
        if self.log is not None:
            self.log.close()
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--pose-topic', default='/vrpn_mocap/car01/pose')
    parser.add_argument('--speed', type=float, default=0.05)
    parser.add_argument('--wait', type=float, default=15.0, help='等待实时传感器的秒数')
    parser.add_argument('--check', action='store_true', help='只读检查，不创建速度发布者')
    parser.add_argument('--summary', action='store_true', help='仅解析轨迹，不连接 ROS')
    parser.add_argument('--ignore-obstacles', action='store_true',
                        help='不订阅雷达，不检查前方或转向障碍距离')
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.02 <= args.speed <= 0.125:
        parser.error('速度上限必须在 0.02–0.125 m/s')
    if not math.isfinite(args.wait) or not 1 <= args.wait <= 60:
        parser.error('等待时间范围为 1–60 秒')
    controller = None
    try:
        recording = load_recording(args.csv)
        points = recording['points']
        path = PathGeometry(points)
        print(f'记录 {recording["raw_frames"]} 帧，剔除坏帧 {recording["rejected_frames"]}，'
              f'路线 {path.length:.3f} m，{len(points)} 个几何控制点；'
              f'起点 {points[0]}，终点 {points[-1]}。', flush=True)
        print('按绝对 world XY 路线低速行驶；不使用超声波，不重放原始时间和速度。', flush=True)
        if args.ignore_obstacles:
            print('按要求忽略障碍距离；保留动捕、里程计、偏离轨迹校验和独立停车看门狗。', flush=True)
        if args.summary:
            return 0
        controller = Controller(args.pose_topic, ignore_obstacles=args.ignore_obstacles,
                                speed_limit=args.speed)
        controller.ready(args.pose_topic, args.wait)
        x, y = controller.state[:2]
        approach_length = math.hypot(x - points[0][0], y - points[0][1])
        if approach_length > 5:
            raise RuntimeError('当前位置距记录起点超过 5 米，请核实坐标系')
        print(f'当前位置 ({x:.3f}, {y:.3f})，距记录起点 {approach_length:.3f} m', flush=True)
        if args.check:
            print('只读检查通过；未创建速度发布者。', flush=True)
            return 0
        controller.start_gate()
        if approach_length > 0.035:
            controller.follow(PathGeometry([(x, y), points[0]]), '返回起点', min(args.speed, 0.05))
        controller.follow(path, 'CSV轨迹', args.speed)
        print('整条 CSV 路线执行完成，已停车。', flush=True)
        return 0
    except KeyboardInterrupt:
        print('收到中断，停止轨迹并停车。', flush=True)
        return 130
    except Exception as error:
        print(f'轨迹未完成：{error}；保持或执行停车。', file=sys.stderr, flush=True)
        if controller is not None and controller.state is not None:
            print(f'最近位置 ({controller.state[0]:.3f}, {controller.state[1]:.3f})', file=sys.stderr)
        return 1
    finally:
        if controller is not None:
            controller.close()


if __name__ == '__main__':
    sys.exit(main())
