#!/usr/bin/env python3
"""
health_check.py — System Health Checker
=========================================
Verifies that all required ROS 2 topics and the Nav2 action server
are live before attempting a delivery mission.

Usage:
  ros2 run delivery_robot health_check
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


REQUIRED_TOPICS = {
    '/scan':       LaserScan,
    '/odom':       Odometry,
    '/map':        OccupancyGrid,
    '/amcl_pose':  PoseWithCovarianceStamped,
}

TIMEOUT = 5.0   # seconds per topic


class HealthChecker(Node):
    def __init__(self):
        super().__init__('delivery_health_check')
        self._received: dict[str, bool] = {t: False for t in REQUIRED_TOPICS}
        self._subs = []

        qos_volatile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        for topic, msg_type in REQUIRED_TOPICS.items():
            profile = qos_transient if topic in ['/map', '/amcl_pose'] else qos_volatile
            self._subs.append(
                self.create_subscription(
                    msg_type, topic,
                    lambda msg, t=topic: self._recv(t),
                    profile,
                )
            )

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _recv(self, topic: str) -> None:
        self._received[topic] = True

    def run(self) -> bool:
        print('\n' + '=' * 55)
        print('  🤖  DELIVERY ROBOT — SYSTEM HEALTH CHECK')
        print('=' * 55)

        all_ok = True

        # 1. Check topics
        print('\n[1/2] Checking required topics...')
        import time
        deadline = time.time() + TIMEOUT
        while time.time() < deadline and not all(self._received.values()):
            rclpy.spin_once(self, timeout_sec=0.2)

        for topic, ok in self._received.items():
            status = '✅ OK' if ok else '❌ NOT RECEIVED'
            print(f'  {topic:35s} {status}')
            if not ok:
                all_ok = False

        # 2. Check Nav2 action server
        print('\n[2/2] Checking Nav2 action server...')
        nav_ok = self._nav_client.wait_for_server(timeout_sec=TIMEOUT)
        status = '✅ OK' if nav_ok else '❌ NOT AVAILABLE'
        print(f'  {"navigate_to_pose":35s} {status}')
        if not nav_ok:
            all_ok = False

        # Summary
        print('\n' + '=' * 55)
        if all_ok:
            print('  ✅  All systems GO — ready for delivery mission!')
        else:
            print('  ❌  Some checks FAILED — launch the navigation stack first.')
            print('      ros2 launch delivery_robot navigation_launch.py')
        print('=' * 55 + '\n')

        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = HealthChecker()
    result = node.run()
    node.destroy_node()
    rclpy.shutdown()
    import sys
    sys.exit(0 if result else 1)


if __name__ == '__main__':
    main()
