#!/usr/bin/env python3
"""Wait for actual per-domain chassis samples and unique endpoints, read-only."""
import argparse
import math
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=45)
    parser.add_argument('--filtered', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('fleet_readiness_probe')
    latest, counts = {}, {}
    topics = {'/odom': Odometry, '/imu': Imu}
    if args.filtered:
        topics['/odometry/filtered'] = Odometry

    def receive(topic, msg):
        if topic == '/imu':
            values = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        else:
            p = msg.pose.pose.position
            values = (p.x, p.y, p.z)
        if all(math.isfinite(v) for v in values):
            latest[topic] = time.monotonic()
            counts[topic] = counts.get(topic, 0) + 1

    subs = [node.create_subscription(kind, topic, lambda m, t=topic: receive(t, m),
                                     qos_profile_sensor_data) for topic, kind in topics.items()]
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if (all(counts.get(t, 0) >= 3 and now - latest[t] < 2 and
                    node.count_publishers(t) == 1 for t in topics)
                    and node.count_subscribers('/cmd_vel') == 1):
                print('READY:', ', '.join(topics), '收到连续数据；/cmd_vel 订阅者唯一', flush=True)
                return 0
        print('超时：', {t: {'samples': counts.get(t, 0),
                           'publishers': node.count_publishers(t)} for t in topics},
              'cmd_vel subscribers=', node.count_subscribers('/cmd_vel'), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
