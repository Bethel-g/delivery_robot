#!/usr/bin/env python3
"""
slam_explorer.py — Autonomous SLAM Coverage Driver
===================================================
Drives the robot through all rooms in a systematic pattern so
slam_toolbox builds a complete, clean map.

Route (~4 minutes):
  Base → Room1 → Room2 → (north) Room4 → (west) Room3 → base

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

LINEAR_SPEED  = 0.22   # m/s
ANGULAR_SPEED = 0.70   # rad/s for directional turns
SPIN_SPEED    = 0.90   # rad/s for 360 scans (faster)


class SlamExplorer(Node):

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
        self._odom_ready = False

    def _odom_cb(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._odom_ready = True

    def _tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    # ── Primitives ────────────────────────────────────────────────────────────

    def drive(self, metres):
        """Drive forward using odometry distance feedback."""
        while not self._odom_ready:
            self._tick()

        start_x, start_y = self._x, self._y
        target = abs(metres)
        sign = 1.0 if metres >= 0 else -1.0

        while True:
            self._tick()
            if math.hypot(self._x - start_x, self._y - start_y) >= target:
                break
            self._pub.publish(_vel(sign * LINEAR_SPEED, 0.0))

        self._pub.publish(STOP)
        time.sleep(0.2)

    def turn(self, degrees):
        """
        Time-based turn — avoids the yaw-wrapping bug for large angles.
        Works reliably for any angle in both directions.
        """
        radians  = abs(math.radians(degrees))
        duration = radians / ANGULAR_SPEED
        sign     = 1.0 if degrees >= 0 else -1.0
        end      = time.time() + duration
        while time.time() < end:
            self._pub.publish(_vel(0.0, sign * ANGULAR_SPEED))
            time.sleep(0.05)
        self._pub.publish(STOP)
        time.sleep(0.2)

    def spin_360(self):
        """Full 360 deg spin — time-based at SPIN_SPEED."""
        self.get_logger().info('    → 360° scan')
        duration = (2 * math.pi) / SPIN_SPEED   # ~7 s
        end = time.time() + duration
        while time.time() < end:
            self._pub.publish(_vel(0.0, SPIN_SPEED))
            time.sleep(0.05)
        self._pub.publish(STOP)
        time.sleep(0.4)

    def pause(self, seconds=0.5):
        end = time.time() + seconds
        while time.time() < end:
            self._pub.publish(STOP)
            self._tick()

    def room_sweep(self, depth=1.2, width=0.9):
        """
        Drive a rectangle around the inside of a room.
        Restores original heading on exit.
        """
        self.drive(depth)
        self.turn(90)
        self.drive(width)
        self.turn(90)
        self.drive(depth)
        self.turn(90)
        self.drive(width)
        self.turn(90)

    # ── Exploration route ─────────────────────────────────────────────────────

    def explore(self):
        self.get_logger().info('Waiting for odometry...')
        while not self._odom_ready:
            self._tick()
            time.sleep(0.05)

        time.sleep(3.0)   # let slam_toolbox initialise

        self.get_logger().info('╔══════════════════════════════╗')
        self.get_logger().info('║  SLAM Explorer — 8 phases    ║')
        self.get_logger().info('║  ~4 minutes total            ║')
        self.get_logger().info('╚══════════════════════════════╝')

        # 1. Base
        self.get_logger().info('[1/8] Base — initial scan')
        self.spin_360()
        self.pause(0.5)

        # 2. Room 1
        self.get_logger().info('[2/8] Driving east to Room 1')
        self.drive(1.8)
        self.spin_360()
        self.room_sweep(1.1, 0.9)
        self.pause(0.4)

        # 3. Room 2
        self.get_logger().info('[3/8] East corridor to Room 2')
        self.drive(4.6)
        self.spin_360()
        self.room_sweep(1.1, 0.9)
        self.pause(0.4)

        # 4. Turn north — Room 4
        self.get_logger().info('[4/8] Turn north — driving to Room 4')
        self.turn(90)
        self.drive(2.0)
        self.spin_360()          # corridor midpoint
        self.drive(2.0)
        self.spin_360()
        self.room_sweep(1.1, 0.9)
        self.pause(0.4)

        # 5. Drive west — Room 3
        self.get_logger().info('[5/8] Drive west to Room 3')
        self.turn(90)
        self.drive(2.5)
        self.spin_360()          # top corridor midpoint
        self.drive(2.5)
        self.spin_360()
        self.room_sweep(1.1, 0.9)
        self.pause(0.4)

        # 6. Head south — left corridor
        self.get_logger().info('[6/8] Head south — left corridor')
        self.turn(-90)
        self.drive(1.8)
        self.spin_360()
        self.drive(1.8)
        self.pause(0.4)

        # 7. Loop-closure pass: full corridor east then west
        self.get_logger().info('[7/8] Loop-closure: corridor east then west')
        self.turn(-90)
        self.drive(4.6)
        self.turn(180)
        self.drive(4.6)
        self.pause(0.4)

        # 8. Return to base
        self.get_logger().info('[8/8] Returning to base — final scan')
        self.turn(90)
        self.drive(1.8)
        self.turn(180)
        self.spin_360()
        self.pause(1.0)

        self.get_logger().info('=== Exploration complete! Now save the map: ===')
        self.get_logger().info(
            'ros2 run nav2_map_server map_saver_cli '
            '-f ~/delivery_ws/src/delivery_robot/maps/office_map '
            '--ros-args -p save_map_timeout:=10.0'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SlamExplorer()
    try:
        node.explore()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node._pub.publish(STOP)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
