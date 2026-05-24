#!/usr/bin/env python3
"""
slam_explorer.py
================
Automatically drives the delivery robot through all five rooms of the office
so SLAM Toolbox can build a complete map without manual teleoperation.

Rooms (approx. map coordinates):
  base  → (0.5, 2.0)
  room1 → (2.0, 2.0)
  room2 → (7.5, 2.0)
  room3 → (2.0, 6.0)
  room4 → (7.5, 6.0)

The script sends timed cmd_vel commands (straight + turn) that trace a path
covering all four rooms and returns to base.

Usage (after slam_launch.py is running):
  ros2 run delivery_robot slam_explorer
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ── Primitive motion commands ─────────────────────────────────────────────────

def _vel(linear=0.0, angular=0.0):
    t = Twist()
    t.linear.x = linear
    t.angular.z = angular
    return t


STOP = _vel()


class SlamExplorer(Node):

    LINEAR_SPEED  = 0.20   # m/s  – safe for indoor nav
    ANGULAR_SPEED = 0.50   # rad/s

    def __init__(self):
        super().__init__('slam_explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('SlamExplorer ready – starting room sweep in 3 s')

    # ── Primitives ────────────────────────────────────────────────────────────

    def _drive(self, linear, angular, duration):
        """Publish a constant velocity for `duration` seconds."""
        msg = _vel(linear, angular)
        end = time.time() + duration
        while time.time() < end:
            self.pub.publish(msg)
            time.sleep(0.05)

    def forward(self, metres):
        self._drive(self.LINEAR_SPEED, 0.0,
                    metres / self.LINEAR_SPEED)

    def backward(self, metres):
        self._drive(-self.LINEAR_SPEED, 0.0,
                    metres / self.LINEAR_SPEED)

    def turn(self, degrees):
        """Positive = left (CCW), negative = right (CW)."""
        radians  = math.radians(abs(degrees))
        duration = radians / self.ANGULAR_SPEED
        direction = 1.0 if degrees > 0 else -1.0
        self._drive(0.0, direction * self.ANGULAR_SPEED, duration)

    def stop(self, seconds=0.5):
        end = time.time() + seconds
        while time.time() < end:
            self.pub.publish(STOP)
            time.sleep(0.05)

    def sweep(self, width=1.0):
        """Short side-to-side sweep to cover room width with lidar."""
        self.turn(30)
        self.stop(0.2)
        self.turn(-60)
        self.stop(0.2)
        self.turn(30)        # back to heading

    # ── Room-by-room route ────────────────────────────────────────────────────

    def explore(self):
        time.sleep(3.0)   # let SLAM warm up
        self.get_logger().info('=== Phase 1: Sweep base / entrance ===')
        self.sweep()
        self.stop(0.5)

        # ── Corridor to room1 (east, ~1.5 m) ──────────────────────────────
        self.get_logger().info('=== Phase 2: Drive to Room 1 ===')
        self.forward(1.5)
        self.stop(0.3)
        self.sweep()
        self.stop(0.3)

        # ── Long corridor to room2 (further east, ~5.5 m more) ────────────
        self.get_logger().info('=== Phase 3: Drive to Room 2 ===')
        self.forward(5.5)
        self.stop(0.3)
        self.sweep()
        self.stop(0.3)

        # ── Turn north and drive up to room4 ──────────────────────────────
        self.get_logger().info('=== Phase 4: Head north → Room 4 ===')
        self.turn(90)
        self.stop(0.3)
        self.forward(4.0)
        self.stop(0.3)
        self.sweep()
        self.stop(0.3)

        # ── Drive west back to room3 ───────────────────────────────────────
        self.get_logger().info('=== Phase 5: Head west → Room 3 ===')
        self.turn(90)
        self.stop(0.3)
        self.forward(5.5)
        self.stop(0.3)
        self.sweep()
        self.stop(0.3)

        # ── Return south to base ───────────────────────────────────────────
        self.get_logger().info('=== Phase 6: Return south to base ===')
        self.turn(-90)
        self.stop(0.3)
        self.forward(4.0)
        self.stop(0.3)

        # Final sweep at base
        self.sweep()
        self.stop(1.0)
        self.get_logger().info('=== Exploration complete – saving map ===')


def main(args=None):
    rclpy.init(args=args)
    node = SlamExplorer()
    node.explore()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
