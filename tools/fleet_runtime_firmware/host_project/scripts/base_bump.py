#!/usr/bin/env python3
"""Start the local micro-ROS base and perform one bounded forward bump.

Run in a normal terminal with LAN access. Leaves the base running until Ctrl+C.
Does not start or modify motion capture. Requires the MCU to target this host:8888.
"""

import math
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

CAR_IP = '10.1.1.220'
SPEED = 0.03
DURATION = 1.0


def main():
    # Refuse before launching anything when the execution environment blocks UDP,
    # or when an existing agent already owns the port. Never replace that agent.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('0.0.0.0', 8888))
    except OSError as error:
        print(f'不能监听 UDP/8888：{error}。底盘未启动，未发送运动指令。', file=sys.stderr)
        return 1

    result = subprocess.run(
        ['ping', '-c', '1', '-W', '2', CAR_IP],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    if result.returncode:
        print(f'无法 ping 通小车 {CAR_IP}，停止试动。', file=sys.stderr)
        return 1

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    workspace = Path(__file__).resolve().parents[1]
    logs = workspace / 'log' / ('base_bump_' + time.strftime('%Y%m%d_%H%M%S'))
    logs.mkdir(parents=True)
    from firmware_log import record
    record([{'name': 'base_bump_target', 'ip': CAR_IP}],
           logs / 'firmware_versions.json', kind='base_bump')
    processes = []
    handles = []
    node = None
    publisher = None
    last_odom = [None]
    position = [None]
    samples = [0]

    def check_processes():
        for label, process in processes:
            if process.poll() is not None:
                raise RuntimeError(f'{label} 已退出，查看 {logs}')
        if stop[0]:
            raise KeyboardInterrupt

    def launch(label, arguments):
        handle = (logs / f'{label}.log').open('w')
        handles.append(handle)
        processes.append((label, subprocess.Popen(
            arguments, stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True)))

    def odom_callback(message):
        x, y = message.pose.pose.position.x, message.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            position[0] = (x, y)
            last_odom[0] = time.monotonic()
            samples[0] += 1

    def verify_endpoints():
        odom_sources = node.get_publishers_info_by_topic('/odom')
        cmd_sinks = node.get_subscriptions_info_by_topic('/cmd_vel')
        cmd_sources = node.get_publishers_info_by_topic('/cmd_vel')
        if len(odom_sources) != 1 or len(cmd_sinks) != 1 or len(cmd_sources) != 1:
            return False
        source, sink = odom_sources[0], cmd_sinks[0]
        return (source.topic_type == 'nav_msgs/msg/Odometry'
                and sink.topic_type == 'geometry_msgs/msg/Twist'
                and (source.node_namespace, source.node_name)
                == (sink.node_namespace, sink.node_name))

    try:
        print(f'目标小车 {CAR_IP}。启动 Agent UDP/8888 与底盘；动捕保持现状。')
        print(f'日志：{logs}', flush=True)
        launch('agent', [str(workspace / 'install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent'),
                         'udp4', '-p', '8888', '-v', '4'])
        launch('bringup', ['ros2', 'launch', 'chuyun_bringup', 'chuyun_bringup.launch.py'])
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('base_bump_test')
        # Volatile and depth 1 prevent a late subscriber receiving stale commands.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        publisher = node.create_publisher(Twist, '/cmd_vel', qos)
        odom_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscription = node.create_subscription(Odometry, '/odom', odom_callback, odom_qos)
        print('等待新鲜 /odom 与同一底盘节点的 /cmd_vel 订阅，最长 30 秒……', flush=True)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            check_processes()
            rclpy.spin_once(node, timeout_sec=0.05)
            if (samples[0] >= 3 and last_odom[0] is not None
                    and time.monotonic() - last_odom[0] < 0.3
                    and verify_endpoints() and publisher.get_subscription_count() == 1):
                break
        else:
            raise RuntimeError('未发现唯一的底盘数据/命令链路。检查 MCU 的 Agent 目标 IP、8888 端口和 ROS_DOMAIN_ID。')

        origin = position[0]
        print('连接已就绪：以 0.03 m/s 前进最多 1 秒，然后发送零速度。', flush=True)
        start = time.monotonic()
        while time.monotonic() - start < DURATION:
            check_processes()
            if time.monotonic() - last_odom[0] > 0.3 or not verify_endpoints():
                raise RuntimeError('里程计过期或命令链路变化，中止试动。')
            if math.hypot(position[0][0] - origin[0], position[0][1] - origin[1]) >= 0.05:
                break
            command = Twist()
            command.linear.x = SPEED
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)

        # Keep issuing stops for a full second after the bounded movement.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        print('试动指令已结束并发送停车。底盘保持启动，按 Ctrl+C 关闭本脚本启动的进程。', flush=True)
        while not stop[0]:
            check_processes()
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f'试动未完成：{error}', file=sys.stderr)
        return 1
    finally:
        if publisher is not None and rclpy.ok():
            for _ in range(10):
                publisher.publish(Twist())
                time.sleep(0.05)
        for _, process in reversed(processes):
            if process.poll() is None:
                try:
                    signal_name = signal.SIGINT
                    import os
                    os.killpg(process.pid, signal_name)
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
                except ProcessLookupError:
                    pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    sys.exit(main())
