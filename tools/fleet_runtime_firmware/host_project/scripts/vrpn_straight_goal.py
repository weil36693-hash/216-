#!/usr/bin/env python3
"""Bounded differential-drive goal in VRPN world XY; existing base required.

The car01 rigid body's +X must point forward and +Z upward. No navigation or
mapping node is used. An independent process owns cmd_vel and stops if control
updates expire. --check only subscribes and never creates a command publisher.
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


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, low, high):
    return max(low, min(high, value))


def attitude(q):
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(v * v for v in values))
    if not all(math.isfinite(v) for v in values) or not 0.95 < norm < 1.05:
        raise RuntimeError('动捕四元数无效')
    x, y, z, w = (v / norm for v in values)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(clip(2 * (w * y - z * x), -1.0, 1.0))
    if max(abs(roll), abs(pitch)) > 0.35:
        raise RuntimeError('动捕刚体倾斜过大：需要 +Z 朝上的车体坐标系；'
                           f'roll={math.degrees(roll):.2f}°，pitch={math.degrees(pitch):.2f}°')
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def laser_clearance(scan, rotating=False):
    if (scan.header.frame_id != 'laser_frame' or len(scan.ranges) < 180
            or not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_max))
            or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0
            or not 0 < scan.range_min < scan.range_max or scan.range_max < 1.0):
        raise RuntimeError('雷达坐标系或扫描参数异常')
    valid = 0
    front_valid = 0
    front_samples = 0
    nearest = float('inf')
    # Fixed URDF transform: laser_frame -> base_link, no rotation.
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        front = abs(wrap(angle)) <= math.pi / 3
        front_samples += int(front)
        if distance == float('inf'):
            valid += 1
            front_valid += int(front)
            continue
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        valid += 1
        front_valid += int(front)
        x = distance * math.cos(angle) - 0.0227
        y = distance * math.sin(angle) + 0.000062
        # Only mask points inside the known chassis/wheel envelope.
        if abs(x) <= 0.09 and abs(y) <= 0.10:
            continue
        if rotating:
            nearest = min(nearest, math.hypot(x, y))
        elif x > 0 and abs(y) < 0.16:
            nearest = min(nearest, x)
    if valid < 20 or front_samples < 20 or front_valid < max(6, front_samples * 0.1):
        raise RuntimeError('雷达有效扫描不足')
    end_angle = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
    if (end_angle - scan.angle_min < 2 * math.pi - 0.15
            or abs(scan.angle_max - end_angle) > 0.05):
        raise RuntimeError('需要覆盖 360 度的雷达扫描')
    return nearest


def range_distance(message):
    value = message.range
    if (message.header.frame_id != 'ultrasonic_link'
            or not 0 < message.min_range < message.max_range
            or not math.isfinite(value)
            or not message.min_range <= value <= message.max_range):
        raise RuntimeError('超声波距离无效')
    return value


def command_velocity_valid(linear, angular, speed_limit=0.08):
    return (all(math.isfinite(v) for v in (linear, angular, speed_limit))
            and 0 < speed_limit <= 0.125
            and abs(linear) <= speed_limit and abs(angular) <= 0.30)


def gate_main(connection, node_name, speed_limit=0.08):
    """The sole velocity publisher; stale/queued commands cannot renew motion."""
    if not command_velocity_valid(0.0, 0.0, speed_limit):
        raise ValueError('速度看门狗的线速度上限必须在 (0, 0.125] m/s')
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    exiting = [False]
    signal.signal(signal.SIGINT, lambda *_: exiting.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: exiting.__setitem__(0, True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(node_name)
    publisher = node.create_publisher(Twist, '/cmd_vel', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE))
    command = (0.0, 0.0)
    command_time = -float('inf')
    last_sequence = -1
    try:
        connection.send('ready')
        while not exiting[0]:
            if last_sequence >= 0 and time.monotonic() - command_time >= 0.25:
                break
            while connection.poll():
                update = connection.recv()
                if update is None:
                    exiting[0] = True
                    break
                sent, sequence, linear, angular = update
                if (all(math.isfinite(v) for v in (sent, linear, angular))
                        and sequence > last_sequence
                        and 0 <= time.monotonic() - sent < 0.25
                        and command_velocity_valid(linear, angular, speed_limit)):
                    command = linear, angular
                    command_time = sent
                    last_sequence = sequence
                else:
                    exiting[0] = True
                    break
            message = Twist()
            if not exiting[0] and time.monotonic() - command_time < 0.25:
                message.linear.x, message.angular.z = command
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.025)
    except (EOFError, BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            publisher.publish(Twist())
            time.sleep(0.025)
        node.destroy_node()
        rclpy.shutdown()
        connection.close()


class Feedback:
    LIMITS = {'pose': 0.30, 'odom': 0.30, 'scan': 0.60, 'range': 0.40}

    def __init__(self, use_ultrasonic=True, use_laser=True):
        self.data = {}
        self.error = None
        self.limits = {k: v for k, v in self.LIMITS.items()
                       if (k != 'range' or use_ultrasonic)
                       and (k != 'scan' or use_laser)}

    def receive(self, key, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        previous = self.data.get(key)
        if previous and stamp < previous[2]:
            self.error = f'{key} 时间戳倒退'
        if not previous or stamp > previous[2]:
            self.data[key] = message, time.monotonic(), stamp

    def validate(self, ros_now):
        if self.error:
            raise RuntimeError(self.error)
        now = time.monotonic()
        for key, age_limit in self.limits.items():
            if key not in self.data:
                raise RuntimeError(f'没有收到 {key} 数据')
            _, received, stamp = self.data[key]
            if now - received > age_limit or not -0.20 <= ros_now - stamp / 1e9 <= 0.75:
                raise RuntimeError(f'{key} 数据过期或时钟未同步')
        pose = self.data['pose'][0]
        p = pose.pose.position
        if pose.header.frame_id != 'world' or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            raise RuntimeError('动捕位置或 world 坐标系无效')
        yaw = attitude(pose.pose.orientation)
        odom = self.data['odom'][0]
        if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_link':
            raise RuntimeError('底盘里程计坐标系不匹配')
        op = odom.pose.pose.position
        velocity = odom.twist.twist
        if not all(math.isfinite(v) for v in (op.x, op.y, velocity.linear.x, velocity.angular.z)):
            raise RuntimeError('底盘里程计无效')
        if abs(velocity.linear.x) > 0.15 or abs(velocity.angular.z) > 0.60:
            raise RuntimeError(f'实测底盘速度异常：v={velocity.linear.x:.4f} m/s，'
                               f'w={velocity.angular.z:.4f} rad/s')
        odom_yaw = attitude(odom.pose.pose.orientation)
        forward = laser_clearance(self.data['scan'][0]) if 'scan' in self.limits else None
        ultrasound = range_distance(self.data['range'][0]) if 'range' in self.limits else None
        if forward is not None and forward < 0.35:
            raise RuntimeError(f'前方雷达障碍过近：{forward:.3f} m')
        if ultrasound is not None and ultrasound < 0.30:
            raise RuntimeError(f'前方超声波障碍过近：{ultrasound:.3f} m')
        return p.x, p.y, yaw, op.x, op.y, odom_yaw, forward, ultrasound


def endpoints(node, gate_name=None, use_ultrasonic=True, use_laser=True):
    sources = node.get_publishers_info_by_topic('/odom')
    sinks = node.get_subscriptions_info_by_topic('/cmd_vel')
    commands = node.get_publishers_info_by_topic('/cmd_vel')
    if len(sources) != 1 or len(sinks) != 1:
        raise RuntimeError('要求唯一的底盘 /odom 发布者与 /cmd_vel 订阅者')
    identity = lambda info: (info.node_namespace, info.node_name)
    if (identity(sources[0]) != identity(sinks[0])
            or sources[0].topic_type != 'nav_msgs/msg/Odometry'
            or sinks[0].topic_type != 'geometry_msgs/msg/Twist'):
        raise RuntimeError('里程计与速度命令不属于同一底盘')
    if gate_name is None:
        if commands:
            raise RuntimeError('已有 /cmd_vel 控制发布者；请先停止其他控制程序')
    elif len(commands) != 1 or identity(commands[0]) != ('/', gate_name):
        raise RuntimeError('速度控制发布者变化或出现其他控制程序')
    sensor_topics = (['/scan'] if use_laser else []) + (['/ultrasonic'] if use_ultrasonic else [])
    for topic in sensor_topics:
        publishers = node.get_publishers_info_by_topic(topic)
        if len(publishers) != 1 or identity(publishers[0]) != identity(sources[0]):
            raise RuntimeError(f'{topic} 不属于已确认的唯一底盘')
    return tuple(sources[0].endpoint_gid), tuple(sinks[0].endpoint_gid)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True, help='VRPN world 绝对 X，米')
    parser.add_argument('--y', type=float, required=True, help='VRPN world 绝对 Y，米')
    parser.add_argument('--pose-topic', default='/vrpn_mocap/car01/pose')
    parser.add_argument('--speed', type=float, default=0.05, help='最大速度，默认 0.05 m/s')
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--check', action='store_true', help='仅检查连接，不发布任何速度')
    parser.add_argument('--no-ultrasonic', action='store_true', help='不订阅或使用超声波')
    args = parser.parse_args()
    if (not all(math.isfinite(v) for v in (args.x, args.y, args.speed, args.timeout))
            or not 0.02 <= args.speed <= 0.08 or not 5 <= args.timeout <= 180):
        parser.error('目标必须为有限值；速度范围 0.02–0.08 m/s，超时范围 5–180 s')
    return args


def main():
    args = parse_args()
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Range
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('vrpn_straight_controller_' + uuid.uuid4().hex[:8])
    feedback = Feedback(use_ultrasonic=not args.no_ultrasonic)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = []
    for key, topic, kind in (
            ('pose', args.pose_topic, PoseStamped), ('odom', '/odom', Odometry),
            ('scan', '/scan', LaserScan), ('range', '/ultrasonic', Range)):
        if key == 'range' and args.no_ultrasonic:
            continue
        subscriptions.append(node.create_subscription(
            kind, topic, lambda message, k=key: feedback.receive(k, message), qos))
    gate = None
    pipe = None
    log = None
    state = None
    sequence = 0

    def tick(duration=0.05):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if stop[0]:
                raise KeyboardInterrupt
            rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, end - time.monotonic())))

    def send(v=0.0, w=0.0):
        nonlocal sequence
        if gate is None or not gate.is_alive():
            raise RuntimeError('独立停车看门狗已退出')
        sequence += 1
        pipe.send((time.monotonic(), sequence, v, w))

    try:
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            tick()
            if len(feedback.data) == len(feedback.limits):
                break
        state = feedback.validate(node.get_clock().now().nanoseconds / 1e9)
        identity = endpoints(node, use_ultrasonic=not args.no_ultrasonic)
        x0, y0, yaw0 = state[:3]
        dx, dy = args.x - x0, args.y - y0
        length = math.hypot(dx, dy)
        if length > 5:
            raise RuntimeError('单次直线移动限制为 5 米')
        print(f'VRPN 当前 ({x0:.3f}, {y0:.3f})，目标 ({args.x:.3f}, {args.y:.3f})，'
              f'距离 {length:.3f} m', flush=True)
        if args.check:
            print('只读检查通过，未创建速度发布者；不代表整条路径无障碍。')
            return 0
        if length <= 0.035:
            print('已在目标容差 3.5 厘米内，未发送运动指令。')
            return 0
        initial_velocity = feedback.data['odom'][0].twist.twist
        if abs(initial_velocity.linear.x) > 0.01 or abs(initial_velocity.angular.z) > 0.03:
            raise RuntimeError('车辆尚未静止')
        heading = math.atan2(dy, dx)
        ux, uy = dx / length, dy / length
        context = mp.get_context('spawn')
        pipe, child_pipe = context.Pipe()
        gate_name = 'vrpn_velocity_gate_' + uuid.uuid4().hex[:8]
        gate = context.Process(target=gate_main, args=(child_pipe, gate_name))
        gate.start()
        child_pipe.close()
        deadline = time.monotonic() + 6
        while not pipe.poll():
            tick()
            if not gate.is_alive() or time.monotonic() > deadline:
                raise RuntimeError('无法启动独立停车看门狗')
        if pipe.recv() != 'ready':
            raise RuntimeError('停车看门狗握手异常')
        # Allow DDS discovery while commanding only zero.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            send()
            tick()
        if endpoints(node, gate_name, not args.no_ultrasonic) != identity:
            raise RuntimeError('底盘端点已变化')
        log_dir = Path(__file__).resolve().parents[1] / 'log'
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / ('vrpn_goal_' + time.strftime('%Y%m%d_%H%M%S') + '.jsonl')
        log = log_path.open('x')
        from firmware_log import record_for_pose
        record_for_pose(args.pose_topic, log_path.with_suffix('.firmware.json'), kind='vrpn_goal')
        print(f'开始低速闭环，Ctrl+C 停车。日志：{log_path}', flush=True)
        started = time.monotonic()
        last_print = -float('inf')
        previous = None
        previous_v = 0.0
        previous_w = 0.0
        turn_window = None
        history = deque()
        verified = False
        probe = None
        settled = None
        while time.monotonic() - started < args.timeout:
            tick()
            now = time.monotonic()
            state = feedback.validate(node.get_clock().now().nanoseconds / 1e9)
            if endpoints(node, gate_name, not args.no_ultrasonic) != identity:
                raise RuntimeError('底盘端点已变化')
            x, y, yaw, ox, oy, oyaw, laser, ultrasound = state
            distance = math.hypot(args.x - x, args.y - y)
            along = (x - x0) * ux + (y - y0) * uy
            cross = -(x - x0) * uy + (y - y0) * ux
            if abs(cross) > 0.10 or along < -0.035 or along > length + 0.04:
                raise RuntimeError('偏离起点至目标的直线，或出现倒退/越界')
            if previous:
                elapsed = now - previous[0]
                if math.hypot(x - previous[1], y - previous[2]) > 0.35 * elapsed + 0.008:
                    raise RuntimeError('动捕位置跳变或运动速度异常')
                if abs(wrap(yaw - previous[3])) > 0.8 * elapsed + 0.02:
                    raise RuntimeError('动捕朝向跳变')
                vrpn_turn, odom_turn = wrap(yaw - previous[3]), wrap(oyaw - previous[4])
                if abs(vrpn_turn) > 0.008 and abs(odom_turn) > 0.008 and vrpn_turn * odom_turn < 0:
                    raise RuntimeError('动捕与里程计旋转方向相反')
                if abs(previous_w) >= 0.04:
                    sign = 1 if previous_w > 0 else -1
                    if turn_window is None or turn_window[3] != sign:
                        turn_window = [previous[3], previous[4], 0.0, sign]
                    turn_window[2] += previous_w * elapsed
                    if abs(turn_window[2]) > 0.035:
                        turns = wrap(yaw - turn_window[0]), wrap(oyaw - turn_window[1])
                        if any(turn * sign < -0.008 for turn in turns):
                            raise RuntimeError('底盘实际转向与角速度指令相反')
                        if abs(turn_window[2]) > 0.12 and min(abs(t) for t in turns) < 0.005:
                            raise RuntimeError('转向无响应')
                else:
                    turn_window = None
            previous = now, x, y, yaw, oyaw
            if distance <= 0.035 or settled is not None:
                send()
                if settled is None:
                    settled = now, x, y
                if distance > 0.05 or math.hypot(x - settled[1], y - settled[2]) > 0.015:
                    raise RuntimeError('到达后未能稳定停车')
                velocity = feedback.data['odom'][0].twist.twist
                if now - settled[0] >= 1.2:
                    if abs(velocity.linear.x) > 0.01 or abs(velocity.angular.z) > 0.03:
                        raise RuntimeError('到达后底盘仍在运动')
                    print(f'已到达并停车：({x:.3f}, {y:.3f})，误差 {distance:.3f} m。', flush=True)
                    return 0
                continue
            # Fixed start-goal line with bounded cross-track correction.
            desired_yaw = heading - math.atan2(1.5 * cross, 0.30)
            error = wrap(desired_yaw - yaw)
            angular = clip(1.5 * error, -0.25, 0.25)
            if abs(error) > 0.10:
                if laser_clearance(feedback.data['scan'][0], rotating=True) < 0.23:
                    raise RuntimeError('原地转向空间不足')
                linear = 0.0
                history.clear()
            else:
                linear = min(args.speed, max(0.015, 0.5 * max(0.0, length - along)))
                if not verified:
                    linear = min(linear, 0.025)
                    if probe is None:
                        probe = now, x, y, yaw, ox, oy
                    moved = math.hypot(x - probe[1], y - probe[2])
                    odom_moved = math.hypot(ox - probe[4], oy - probe[5])
                    if moved >= 0.020:
                        travel_yaw = math.atan2(y - probe[2], x - probe[1])
                        if abs(wrap(travel_yaw - probe[3])) > 0.30:
                            raise RuntimeError('动捕刚体 +X 未与车头对齐；需要校正外参')
                        if not 0.005 <= odom_moved <= 0.08:
                            raise RuntimeError('动捕位移与轮式里程计不一致')
                        verified = True
                        print('已验证动捕前进方向和底盘里程计。', flush=True)
                    elif odom_moved > 0.05 or now - probe[0] > 3.5:
                        raise RuntimeError('低速方向验证失败或车轮空转')
                history.append((now, along))
                while history and now - history[0][0] > 3:
                    history.popleft()
                if history and now - history[0][0] > 2.8 and along - history[0][1] < 0.01:
                    raise RuntimeError('车辆无前进进展，已停车')
            linear = min(linear, previous_v + 0.004)
            previous_v = linear
            previous_w = angular
            send(linear, angular)
            log.write(json.dumps({'time': now, 'x': x, 'y': y, 'yaw': yaw,
                                  'distance': distance, 'cross': cross, 'v': linear,
                                  'w': angular, 'laser': laser if math.isfinite(laser) else None,
                                  'ultrasound': ultrasound}) + '\n')
            log.flush()
            if now - last_print >= 2:
                print(f'位置 ({x:.3f}, {y:.3f})，剩余 {distance:.3f} m，'
                      f'偏线 {cross:.3f} m', flush=True)
                last_print = now
        raise RuntimeError('超过本次移动时限')
    except KeyboardInterrupt:
        print('收到中断，停车。', flush=True)
        return 130
    except Exception as error:
        print(f'中止并停车：{error}', file=sys.stderr, flush=True)
        if state is not None:
            print(f'最近动捕位置 ({state[0]:.3f}, {state[1]:.3f})', file=sys.stderr)
        return 1
    finally:
        if pipe is not None:
            try:
                pipe.send(None)
            except (BrokenPipeError, EOFError, OSError):
                pass
            pipe.close()
        if gate is not None:
            gate.join(timeout=3)
            if gate.is_alive():
                gate.terminate()
                gate.join(timeout=2)
        if log is not None:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
