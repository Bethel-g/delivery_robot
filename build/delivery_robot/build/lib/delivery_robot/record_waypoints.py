#!/usr/bin/env python3
"""
record_waypoints.py — Interactive Waypoint Recorder
=====================================================
Drive the robot to each room, press Enter to record the current pose,
name it, and save all waypoints to delivery_mission.py automatically.

Usage:
  ros2 run delivery_robot record_waypoints
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import os
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class WaypointRecorder(Node):
    """
    Subscribes to /amcl_pose (after AMCL is running) or /odom (during SLAM)
    and lets the operator interactively record named waypoints.
    """

    def __init__(self):
        super().__init__('waypoint_recorder')

        self._waypoints: dict[str, tuple[float, float, float]] = {}
        self._latest_x = 0.0
        self._latest_y = 0.0
        self._latest_yaw = 0.0
        self._pose_source = 'unknown'

        # Try AMCL pose first (navigation mode)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10,
        )

        # Fall back to odometry (SLAM mode)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, odom_qos)

        self.get_logger().info('Waypoint Recorder ready.')
        self.get_logger().info('Drive the robot to each location, then press Enter.')

    def _amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose_source = 'AMCL (map frame)'
        self._latest_x = msg.pose.pose.position.x
        self._latest_y = msg.pose.pose.position.y
        self._latest_yaw = self._quat_to_yaw(msg.pose.pose.orientation)

    def _odom_callback(self, msg: Odometry) -> None:
        if self._pose_source == 'unknown':
            self._pose_source = 'Odometry'
        self._latest_x = msg.pose.pose.position.x
        self._latest_y = msg.pose.pose.position.y
        self._latest_yaw = self._quat_to_yaw(msg.pose.pose.orientation)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """Extract yaw (degrees) from quaternion."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny, cosy))

    def record_interactively(self) -> None:
        """Interactive loop: drive to room, press Enter, give it a name."""
        print('\n' + '=' * 60)
        print('  WAYPOINT RECORDER')
        print('  Pose source:', self._pose_source)
        print('=' * 60)
        print('Commands:')
        print('  <Enter>          — record current pose')
        print('  "done" + Enter   — finish and save')
        print('  "list" + Enter   — show recorded waypoints')
        print('=' * 60 + '\n')

        while True:
            cmd = input(
                f'[{self._pose_source}] '
                f'pos=({self._latest_x:.3f}, {self._latest_y:.3f}, '
                f'{self._latest_yaw:.1f}°) '
                '— press Enter to record, or type command: '
            ).strip().lower()

            rclpy.spin_once(self, timeout_sec=0.1)

            if cmd == 'done':
                break
            elif cmd == 'list':
                self._print_waypoints()
                continue
            elif cmd == 'delete':
                name = input('  Delete room name: ').strip()
                if name in self._waypoints:
                    del self._waypoints[name]
                    print(f'  Deleted: {name}')
                else:
                    print(f'  Not found: {name}')
                continue

            # Record current pose
            name = input(
                f'  Recording ({self._latest_x:.3f}, {self._latest_y:.3f}, '
                f'{self._latest_yaw:.1f}°)\n'
                '  Room name (e.g. room1, base): '
            ).strip()

            if not name:
                print('  Name cannot be empty, skipping.')
                continue

            self._waypoints[name] = (
                round(self._latest_x, 3),
                round(self._latest_y, 3),
                round(self._latest_yaw, 1),
            )
            print(f'  ✓ Saved "{name}"')

        self._print_waypoints()
        self._save_to_python()

    def _print_waypoints(self) -> None:
        print('\nRecorded waypoints:')
        if not self._waypoints:
            print('  (none)')
            return
        for name, (x, y, yaw) in self._waypoints.items():
            print(f'  {name:20s}  x={x:7.3f}  y={y:7.3f}  yaw={yaw:7.1f}°')
        print()

    def _save_to_python(self) -> None:
        """
        Print the DELIVERY_ROOMS dict snippet that should be pasted
        into delivery_mission.py.
        """
        print('\n' + '=' * 60)
        print('Copy this into delivery_mission.py → DELIVERY_ROOMS:')
        print('=' * 60)
        print('DELIVERY_ROOMS: dict[str, tuple[float, float, float]] = {')
        for name, (x, y, yaw) in self._waypoints.items():
            print(f'    "{name}": ({x}, {y}, {yaw}),')
        print('}')
        print('=' * 60 + '\n')

        # Also write a waypoints.yaml for reference
        try:
            pkg = get_package_share_directory('delivery_robot')
            out_path = os.path.join(pkg, '..', '..', '..', '..', 'src',
                                    'delivery_robot', 'config', 'waypoints.yaml')
            out_path = os.path.abspath(out_path)
            with open(out_path, 'w') as f:
                f.write('# Auto-generated by record_waypoints.py\n')
                f.write('waypoints:\n')
                for name, (x, y, yaw) in self._waypoints.items():
                    f.write(f'  {name}:\n')
                    f.write(f'    x: {x}\n')
                    f.write(f'    y: {y}\n')
                    f.write(f'    yaw_deg: {yaw}\n')
            print(f'Also saved to: {out_path}')
        except Exception as e:
            print(f'(Could not auto-save yaml: {e})')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    # Let one pose message arrive
    print('Waiting for pose data...')
    rclpy.spin_once(node, timeout_sec=3.0)

    try:
        node.record_interactively()
    except KeyboardInterrupt:
        print('\nRecorder stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
