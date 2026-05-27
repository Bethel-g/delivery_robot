#!/usr/bin/env python3
"""
slam_explorer.py — Autonomous SLAM Coverage Driver (LIDAR Wall Follower)
=========================================================================
Uses live /scan data to follow the right-hand wall, naturally exploring
all accessible rooms without hitting walls.

Right-hand rule guarantees full coverage: the robot hugs the right wall,
turns right whenever a gap appears, and turns left whenever blocked.

Usage (after slam_launch.py is running):
  ros2 run delivery_robot slam_explorer
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def _vel(linear=0.0, angular=0.0):
    t = Twist()
    t.linear.x = linear
    t.angular.z = angular
    return t


STOP = _vel()

# ── Tuning ────────────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.18   # m/s — slow for clean LIDAR sweeps
TURN_SPEED       = 0.55   # rad/s
TARGET_WALL_DIST = 0.55   # desired distance from right wall (m)
FRONT_WARN_DIST  = 0.65   # slow/turn when this close ahead
FRONT_STOP_DIST  = 0.45   # hard stop/turn threshold
EXPLORE_SECONDS  = 260    # total mapping time (~4.5 min)


class SlamExplorer(Node):

    def __init__(self):
        super().__init__('slam_explorer')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, scan_qos)
        self._scan = None

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    # ── LIDAR helpers ─────────────────────────────────────────────────────────

    def _sector(self, scan: LaserScan, idx_start: int, idx_end: int) -> float:
        """
        Return the minimum valid range in a sector of the scan array.
        Indices wrap around (modulo len).

        LIDAR layout for this robot (360 samples, angle_min=-pi):
          index   0 = -180 deg (directly behind)
          index  90 = -90  deg (right)
          index 180 =   0  deg (forward)
          index 270 = +90  deg (left)
        """
        n = len(scan.ranges)
        vals = []
        for i in range(idx_start, idx_end):
            r = scan.ranges[i % n]
            if math.isfinite(r) and r > 0.12:
                vals.append(r)
        return min(vals) if vals else float('inf')

    # ── Wall-follower logic ───────────────────────────────────────────────────

    def _compute_cmd(self, scan: LaserScan) -> Twist:
        """
        Right-hand wall follower with four rules (priority order):
          1. Front blocked        → turn left in place
          2. Front-right corner   → gentle left steer
          3. Right gap / opening  → turn right into new room
          4. Normal wall tracking → proportional correction
        """
        front        = self._sector(scan, 168, 193)   # ±12° forward
        front_right  = self._sector(scan, 128, 168)   # 12–52° right of forward
        right        = self._sector(scan,  78, 103)   # ±12° right
        front_left   = self._sector(scan, 193, 233)   # 12–52° left of forward

        # Rule 1 — obstacle directly ahead
        if front < FRONT_WARN_DIST:
            speed  = 0.0 if front < FRONT_STOP_DIST else FORWARD_SPEED * 0.3
            # Bias turn direction toward the more open side
            turn   = TURN_SPEED * (1.2 if front_left >= front_right else 0.8)
            return _vel(speed, turn)

        # Rule 2 — wall closing in from front-right (approaching corner)
        if front_right < TARGET_WALL_DIST - 0.05:
            return _vel(FORWARD_SPEED * 0.6, TURN_SPEED * 0.5)

        # Rule 3 — opening on the right (doorway or new room)
        if right > TARGET_WALL_DIST + 0.35:
            return _vel(FORWARD_SPEED * 0.8, -TURN_SPEED * 0.7)

        # Rule 4 — track the right wall with a proportional controller
        error   = right - TARGET_WALL_DIST          # positive = too far
        angular = -0.9 * error                      # turn right if too far
        angular = max(-TURN_SPEED, min(TURN_SPEED, angular))
        return _vel(FORWARD_SPEED, angular)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def explore(self):
        self.get_logger().info('Waiting for LIDAR...')
        while self._scan is None:
            self._tick()
            time.sleep(0.05)

        time.sleep(3.0)   # let slam_toolbox initialise

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  SLAM Explorer — LIDAR wall follower     ║')
        self.get_logger().info(f'║  Mapping for {EXPLORE_SECONDS} s — watch RViz2 map  ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')

        start = time.time()
        last_log = 0

        while True:
            self._tick()
            elapsed = time.time() - start

            if elapsed >= EXPLORE_SECONDS:
                break

            # Progress log every 30 s
            if int(elapsed) // 30 > last_log:
                last_log = int(elapsed) // 30
                remaining = EXPLORE_SECONDS - elapsed
                self.get_logger().info(
                    f'  Mapping... {elapsed:.0f}s elapsed, {remaining:.0f}s remaining')

            if self._scan is not None:
                cmd = self._compute_cmd(self._scan)
                self._pub.publish(cmd)

            time.sleep(0.05)

        self._pub.publish(STOP)

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
        node.get_logger().info('Interrupted by user.')
    finally:
        node._pub.publish(STOP)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
