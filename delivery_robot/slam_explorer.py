#!/usr/bin/env python3
"""
slam_explorer.py — Autonomous SLAM Coverage Driver
===================================================
Drives the delivery robot through all rooms and corridors in a systematic
pattern optimised for slam_toolbox map quality.

Coverage strategy:
  1. 360 deg scan at base
  2. East corridor → Room 1 perimeter loop
  3. Continue east → Room 2 perimeter loop
  4. Turn north → Room 4 perimeter loop
  5. Drive west → Room 3 perimeter loop
  6. Return south → final east-west corridor pass for loop closure
  7. Back to base — final 360 deg scan

Uses /odom feedback for accurate distance and heading control.
All driving at 0.18 m/s so LIDAR scans stay crisp.

Usage (after slam_launch.py is running):
  ros2 run delivery_robot slam_explorer
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def _vel(linear=0.0, angular=0.0):
    t = Twist()
    t.linear.x = linear
    t.angular.z = angular
    return t


STOP = _vel()


class SlamExplorer(Node):

    LINEAR_SPEED  = 0.18   # m/s — slow enough for clean LIDAR sweeps
    ANGULAR_SPEED = 0.45   # rad/s

    def __init__(self):
        super().__init__('slam_explorer')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, odom_qos)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._odom_ready = False

    # ── Odometry callback ─────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)
        self._odom_ready = True

    def _spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_odom(self):
        while not self._odom_ready:
            self._spin_once()
            time.sleep(0.05)

    # ── Motion primitives (odometry-based) ───────────────────────────────────

    def drive(self, metres):
        """Drive forward (or backward if negative) using odometry distance."""
        self._wait_odom()
        start_x, start_y = self._x, self._y
        target = abs(metres)
        direction = 1.0 if metres >= 0 else -1.0

        while True:
            self._spin_once()
            dist = math.hypot(self._x - start_x, self._y - start_y)
            if dist >= target:
                break
            self._pub.publish(_vel(direction * self.LINEAR_SPEED, 0.0))

        self._pub.publish(STOP)
        time.sleep(0.15)

    def turn(self, degrees):
        """Turn by a precise angle using odometry yaw feedback."""
        self._wait_odom()
        target_rad = math.radians(degrees)
        direction = 1.0 if degrees >= 0 else -1.0
        start_yaw = self._yaw

        while True:
            self._spin_once()
            delta = self._yaw - start_yaw
            # Normalise to [-pi, pi]
            while delta > math.pi:
                delta -= 2 * math.pi
            while delta < -math.pi:
                delta += 2 * math.pi
            if abs(delta) >= abs(target_rad) - 0.02:
                break
            self._pub.publish(_vel(0.0, direction * self.ANGULAR_SPEED))

        self._pub.publish(STOP)
        time.sleep(0.2)

    def pause(self, seconds=0.5):
        """Stop in place and let slam_toolbox process scans."""
        end = time.time() + seconds
        while time.time() < end:
            self._pub.publish(STOP)
            self._spin_once()

    def spin_360(self):
        """Full 360 deg rotation — gives LIDAR a complete view of the area."""
        self.get_logger().info('    [scan] 360 deg spin')
        self.turn(190)   # slightly over 180
        self.turn(190)   # second half — ends close to original heading
        self.pause(0.4)

    def room_perimeter(self, depth, width):
        """
        Drive a rectangle around the inside of a room.
          depth — distance along current heading (forward/back)
          width — distance perpendicular (left/right)
        Returns to the starting position and heading.
        """
        self.drive(depth)
        self.pause(0.3)
        self.turn(90)
        self.drive(width)
        self.pause(0.3)
        self.turn(90)
        self.drive(depth)
        self.pause(0.3)
        self.turn(90)
        self.drive(width)
        self.pause(0.3)
        self.turn(90)    # restored to original heading
        self.pause(0.3)

    # ── Full exploration route ────────────────────────────────────────────────

    def explore(self):
        self.get_logger().info('Waiting for odometry...')
        self._wait_odom()
        time.sleep(3.0)  # let slam_toolbox warm up

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║     SLAM EXPLORER — starting route       ║')
        self.get_logger().info('║  8 phases — ~6 minutes total             ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')

        # ── Phase 1: Base station ─────────────────────────────────────────
        self.get_logger().info('[1/8] Base station — initial 360 scan')
        self.spin_360()
        self.pause(1.0)

        # ── Phase 2: Room 1 ───────────────────────────────────────────────
        self.get_logger().info('[2/8] Drive east to Room 1')
        self.drive(1.8)
        self.pause(0.5)
        self.spin_360()
        self.pause(0.5)
        self.get_logger().info('    Room 1 perimeter loop')
        self.room_perimeter(1.1, 0.9)
        self.spin_360()
        self.pause(0.5)

        # ── Phase 3: Room 2 ───────────────────────────────────────────────
        self.get_logger().info('[3/8] Drive east corridor to Room 2')
        self.drive(4.6)
        self.pause(0.5)
        self.spin_360()
        self.pause(0.5)
        self.get_logger().info('    Room 2 perimeter loop')
        self.room_perimeter(1.1, 0.9)
        self.spin_360()
        self.pause(0.5)

        # ── Phase 4: North corridor + Room 4 ─────────────────────────────
        self.get_logger().info('[4/8] Turn north — drive to Room 4')
        self.turn(90)
        self.pause(0.3)
        self.drive(2.0)
        self.pause(0.4)
        self.spin_360()         # corridor_right midpoint
        self.pause(0.4)
        self.drive(2.0)
        self.pause(0.5)
        self.spin_360()
        self.pause(0.5)
        self.get_logger().info('    Room 4 perimeter loop')
        self.room_perimeter(1.1, 0.9)
        self.spin_360()
        self.pause(0.5)

        # ── Phase 5: West corridor + Room 3 ──────────────────────────────
        self.get_logger().info('[5/8] Drive west to Room 3')
        self.turn(90)
        self.pause(0.3)
        self.drive(2.5)
        self.pause(0.4)
        self.spin_360()         # top corridor midpoint
        self.pause(0.4)
        self.drive(2.5)
        self.pause(0.5)
        self.spin_360()
        self.pause(0.5)
        self.get_logger().info('    Room 3 perimeter loop')
        self.room_perimeter(1.1, 0.9)
        self.spin_360()
        self.pause(0.5)

        # ── Phase 6: South corridor — return to Room 1 level ─────────────
        self.get_logger().info('[6/8] Head south — left corridor')
        self.turn(-90)
        self.pause(0.3)
        self.drive(1.8)
        self.pause(0.4)
        self.spin_360()         # corridor_left midpoint
        self.pause(0.4)
        self.drive(1.8)
        self.pause(0.5)

        # ── Phase 7: East-west corridor pass for loop closure ─────────────
        self.get_logger().info('[7/8] Loop-closure pass: drive corridor east then west')
        self.turn(-90)
        self.pause(0.3)
        self.drive(4.8)         # east
        self.pause(0.4)
        self.spin_360()
        self.pause(0.3)
        self.turn(180)
        self.pause(0.3)
        self.drive(4.8)         # back west
        self.pause(0.4)
        self.spin_360()
        self.pause(0.3)
        self.turn(90)
        self.pause(0.3)

        # ── Phase 8: Return to base ────────────────────────────────────────
        self.get_logger().info('[8/8] Return to base — final scan')
        self.drive(1.8)
        self.pause(0.3)
        self.turn(180)
        self.pause(0.3)
        self.spin_360()
        self.pause(1.0)

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  Exploration complete! Save the map now: ║')
        self.get_logger().info('║                                          ║')
        self.get_logger().info('║  ros2 run nav2_map_server map_saver_cli \\║')
        self.get_logger().info('║    -f ~/delivery_ws/src/delivery_robot/  ║')
        self.get_logger().info('║         maps/office_map                  ║')
        self.get_logger().info('║    --ros-args -p save_map_timeout:=10.0  ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = SlamExplorer()
    try:
        node.explore()
    except KeyboardInterrupt:
        node.get_logger().info('Exploration interrupted by user.')
    finally:
        node._pub.publish(STOP)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
