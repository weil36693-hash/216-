#!/usr/bin/env python3
"""Read configured fleet VRPN poses. Never publishes vehicle commands."""
import argparse
from contextlib import ExitStack
import csv
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

from fleet_processes import Processes
from fleet_vrpn import VRPNStartLock, discover_vrpn


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/fleet.json'


def load_config(path):
    cars = json.loads(Path(path).read_text(encoding='utf-8'))['cars']
    for key in ('name', 'ip', 'pose_topic'):
        values = [car[key] for car in cars]
        if not values or len(values) != len(set(values)):
            raise ValueError(f'配置 {key} 为空或重复')
    return cars


def main():
    parser = argparse.ArgumentParser(description='25 车动捕坐标，单位米；只读')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--stale', type=float, default=1.0, help='接收超时秒数')
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--csv', type=Path, help='保存每次收到的有效位姿')
    parser.add_argument('--check-config', action='store_true')
    parser.add_argument('--start-vrpn', action='store_true', help='需要时启动动捕客户端；已有客户端或同域启动任务时复用')
    parser.add_argument('--server', default='10.1.1.198')
    parser.add_argument('--port', type=int, default=3883)
    args = parser.parse_args()
    cars = load_config(args.config)
    if not all(math.isfinite(v) and v > 0 for v in (args.stale, args.interval)):
        parser.error('stale 和 interval 必须是有限正数')
    if not 1 <= args.port <= 65535:
        parser.error('port 必须处于 1～65535')
    if args.check_config:
        for car in cars:
            print(car['name'], car['ip'], car['pose_topic'])
        return
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped

    # Metadata queries stay on their own worker, outside pose callbacks.
    from firmware_log import ROOT
    from leap_fleet.firmware_session import SessionLogger
    logger = SessionLogger(args.config, log_dir=ROOT / 'log', kind='positions')
    logger.record(kind='fleet_positions')
    print(f'会话固件日志：{logger.directory}', flush=True)
    try:
        read_positions(rclpy, Node, PoseStamped, qos_profile_sensor_data, args, cars)
    finally:
        logger.close()


def read_positions(rclpy, Node, PoseStamped, qos_profile_sensor_data, args, cars):
    try:
        with ExitStack() as cleanup:
            rclpy.init()
            cleanup.callback(lambda: rclpy.shutdown() if rclpy.ok() else None)
            node = Node('fleet_positions')
            cleanup.callback(node.destroy_node)
            processes = Processes()
            vrpn = None
            if args.start_vrpn:
                domain_id = node.context.get_domain_id()
                log_dir = DEFAULT_CONFIG.parents[1] / 'log'
                lock = VRPNStartLock(log_dir, domain_id)
                cleanup.callback(lock.close)
                # Cleanup runs in reverse order: stop owned children before
                # closing our copy of their inherited lock descriptor.
                cleanup.callback(processes.close)
                if not lock.acquire():
                    print(f'ROS 域 {domain_id} 已有动捕启动任务或客户端持锁，'
                          '复用并等待读取数据，不再启动客户端。', flush=True)
                else:
                    counts, client_found = discover_vrpn(
                        node, [car['pose_topic'] for car in cars], rclpy.spin_once)
                    if counts or client_found:
                        if counts:
                            detail = (f'已发现 {len(counts)} 个配置位姿话题，'
                                      f'单话题最多 {max(counts.values())} 个发布者')
                        else:
                            detail = '已发现 VRPN 客户端节点，暂未发现配置位姿发布者'
                        print(f'ROS 域 {domain_id}：{detail}；复用现有客户端。'
                              '本次只读取数据，不管理或停止已有客户端。', flush=True)
                        lock.close()
                    else:
                        fd, log_path = tempfile.mkstemp(
                            prefix='fleet_vrpn_', suffix='.log', dir=log_dir)
                        os.close(fd)
                        vrpn = processes.start(
                            ['ros2', 'launch', 'vrpn_mocap', 'client.launch.yaml',
                             f'server:={args.server}', f'port:={args.port}'], log_path,
                            pass_fds=(lock.fileno(),))
                        print(f'动捕客户端日志：{log_path}', flush=True)
            run_reader(rclpy, node, PoseStamped, qos_profile_sensor_data, args, cars, vrpn)
    except KeyboardInterrupt:
        pass


def run_reader(rclpy, node, PoseStamped, qos_profile_sensor_data, args, cars, vrpn):
    with ExitStack() as cleanup:
        output = cleanup.enter_context(args.csv.open('w', newline='', encoding='utf-8')) if args.csv else None
        _read(rclpy, node, PoseStamped, qos_profile_sensor_data, args, cars, vrpn, output)


def _read(rclpy, node, PoseStamped, qos_profile_sensor_data, args, cars, vrpn, output):
    latest = {}
    writer = csv.writer(output) if output else None
    if writer:
        writer.writerow(['received_unix', 'car', 'ip', 'frame_id', 'stamp_sec',
                         'stamp_nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def receive(car, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            return
        latest[car['name']] = time.monotonic(), msg
        if writer:
            writer.writerow([time.time(), car['name'], car['ip'], msg.header.frame_id,
                             msg.header.stamp.sec, msg.header.stamp.nanosec, *values])

    subscriptions = [node.create_subscription(
        PoseStamped, car['pose_topic'], lambda msg, car=car: receive(car, msg),
        qos_profile_sensor_data) for car in cars]

    def display():
        if vrpn is not None and vrpn.poll() is not None:
            raise RuntimeError('动捕客户端已退出，请查看客户端日志')
        if sys.stdout.isatty():
            print('\033[2J\033[H', end='')
        print(time.strftime('%H:%M:%S'), '动捕位置 (m)，状态表示接收新鲜度')
        print('车辆    IP              状态          X          Y          Z  frame')
        for car in cars:
            entry = latest.get(car['name'])
            prefix = f"{car['name']:<7} {car['ip']:<15}"
            if entry is None:
                print(prefix, 'WAIT（未收到）', car['pose_topic'])
                continue
            received, msg = entry
            age = time.monotonic() - received
            status = 'OK' if age <= args.stale else f'STALE {age:.1f}s'
            p = msg.pose.position
            print(f'{prefix} {status:<12} {p.x:10.4f} {p.y:10.4f} {p.z:10.4f}  {msg.header.frame_id}')
        if output:
            output.flush()

    node.create_timer(args.interval, display)
    rclpy.spin(node)


def exit_on_signal(signum, _frame):
    # In particular, closing a terminal sends SIGHUP; unwind owned resources.
    raise SystemExit(128 + signum)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, exit_on_signal)
    signal.signal(signal.SIGHUP, exit_on_signal)
    main()
