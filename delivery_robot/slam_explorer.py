#!/usr/bin/env python3
"""
slam_explorer.py — Autonomous SLAM Coverage Driver (LIDAR Wall Follower)
=========================================================================
Uses live /scan data to follow the right-hand wall, naturally exploring
all accessible rooms without hitting walls or getting stuck on obstacles.

Obstacle handling:
  - Reactive: LIDAR-based wall follower avoids static and moving obstacles
  - Stuck detection: odometry checks every 1.5 s; triggers escape if no movement
  - Escape: back up, turn toward most open direction, resume

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
from nav_msgs.msg import Odometry


def _vel(linear=0.0, angular=0.0):
    t = Twist()
    t.linear.x = linear
    t.angular.z = angular
    return t


STOP = _vel()

# ── Tuning ────────────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.18   # m/s
TURN_SPEED       = 0.55   # rad/s
TARGET_WALL_DIST = 0.55   # desired distance from right wall (m)
FRONT_WARN_DIST  = 0.65   # begin turning when this close ahead
FRONT_STOP_DIST  = 0.45   # hard stop threshold

STUCK_TIMEOUT    = 1.5    # seconds without movement → stuck
STUCK_MOVE_THR   = 0.04   # metres — minimum movement to not be "stuck"
ESCAPE_BACK_T    = 1.4    # seconds of reverse during escape
ESCAPE_TURN_T    = 1.6    # seconds of turn during escape

EXPLORE_SECONDS  = 260    # total mapping time (~4.5 min)


class SlamExplorer(Node):

    def __init__(self):
        super().__init__('slam_explorer')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos)

        self._scan = None
        self._x = 0.0
        self._y = 0.0

        # Stuck detection state
        self._last_check_time = time.time()
        self._last_check_x    = 0.0
        self._last_check_y    = 0.0
        self._escape_count    = 0

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def _tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    # ── LIDAR helpers ─────────────────────────────────────────────────────────

    def _sector_min(self, scan: LaserScan, idx_start: int, idx_end: int) -> float:
        """
        Minimum valid range in an index slice of the scan.

        LIDAR layout (360 samples, angle_min = -pi):
          index   0 = -180 deg (behind)
          index  90 = - 90 deg (right)
          index 180 =    0 deg (forward)
          index 270 = + 90 deg (left)
        """
        n = len(scan.ranges)
        vals = [
            scan.ranges[i % n]
            for i in range(idx_start, idx_end)
            if math.isfinite(scan.ranges[i % n]) and scan.ranges[i % n] > 0.12
        ]
        return min(vals) if vals else float('inf')

    def _most_open_direction(self, scan: LaserScan) -> float:
        """Return angular velocity that steers toward the most open direction."""
        left  = self._sector_min(scan, 220, 270)
        right = self._sector_min(scan,  90, 140)
        # positive angular = turn left (CCW), negative = turn right (CW)
        return TURN_SPEED if left >= right else -TURN_SPEED

    # ── Stuck detection ───────────────────────────────────────────────────────

    def _is_stuck(self) -> bool:
        now = time.time()
        if now - self._last_check_time < STUCK_TIMEOUT:
            return False
        dist = math.hypot(self._x - self._last_check_x,
                          self._y - self._last_check_y)
        self._last_check_time = now
        self._last_check_x    = self._x
        self._last_check_y    = self._y
        return dist < STUCK_MOVE_THR

    def _reset_stuck_timer(self):
        self._last_check_time = time.time()
        self._last_check_x    = self._x
        self._last_check_y    = self._y

    # ── Escape maneuver ───────────────────────────────────────────────────────

    def _escape(self):
        """
        Back up then turn toward the most open direction.
        Called whenever stuck detection fires.
        """
        self._escape_count += 1
        self.get_logger().warn(
            f'STUCK detected (escape #{self._escape_count}) — executing escape maneuver')

        # Phase 1: reverse away from obstacle
        end = time.time() + ESCAPE_BACK_T
        while time.time() < end:
            self._tick()
            self._pub.publish(_vel(-FORWARD_SPEED, 0.0))
            time.sleep(0.05)

        # Phase 2: turn toward most open space
        turn_dir = self._most_open_direction(self._scan) if self._scan else TURN_SPEED
        end = time.time() + ESCAPE_TURN_T
        while time.time() < end:
            self._tick()
            self._pub.publish(_vel(0.0, turn_dir))
            time.sleep(0.05)

        self._pub.publish(STOP)
        self._reset_stuck_timer()
        self.get_logger().info('Escape complete — resuming wall following')

    # ── Wall-follower logic ───────────────────────────────────────────────────

    def _compute_cmd(self, scan: LaserScan) -> Twist:
        """
        Right-hand wall follower — priority rules:

          1. Completely boxed in (front + both sides)  → back up immediately
          2. Front blocked                              → turn toward open side
          3. Front-right corner closing in              → gentle left steer
          4. Right gap / doorway                        → enter it (turn right)
          5. Normal                                     → proportional wall track
        """
        front       = self._sector_min(scan, 168, 193)   # ±12° forward
        front_right = self._sector_min(scan, 128, 168)   # 12–52° right-forward
        front_left  = self._sector_min(scan, 193, 233)   # 12–52° left-forward
        right       = self._sector_min(scan,  78, 103)   # ±12° right
        left        = self._sector_min(scan, 258, 283)   # ±12° left
        rear        = self._sector_min(scan,   0,  30)   # ±15° behind

        # Rule 1 — boxed in: front AND both sides close → back up
        if front < FRONT_STOP_DIST and right < 0.40 and left < 0.40:
            self.get_logger().warn('Boxed in — backing up')
            return _vel(-FORWARD_SPEED, 0.0)

        # Rule 2 — front blocked: turn toward more open side
        if front < FRONT_WARN_DIST:
            speed    = 0.0 if front < FRONT_STOP_DIST else FORWARD_SPEED * 0.25
            turn_dir = TURN_SPEED if front_left >= front_right else -TURN_SPEED
            strength = 1.3 if front < FRONT_STOP_DIST else 1.0
            return _vel(speed, turn_dir * strength)

        # Rule 3 — corner ahead on right side → steer left slightly
        if front_right < TARGET_WALL_DIST - 0.05:
            return _vel(FORWARD_SPEED * 0.6, TURN_SPEED * 0.45)

        # Rule 4 — opening on the right (doorway, new room) → enter it
        if right > TARGET_WALL_DIST + 0.35:
            return _vel(FORWARD_SPEED * 0.8, -TURN_SPEED * 0.65)

        # Rule 5 — proportional right-wall tracking
        error   = right - TARGET_WALL_DIST       # positive = too far from wall
        angular = -0.9 * error                   # negative = steer right
        angular = max(-TURN_SPEED, min(TURN_SPEED, angular))
        return _vel(FORWARD_SPEED, angular)

    # ── Main exploration loop ─────────────────────────────────────────────────

    def explore(self):
        self.get_logger().info('Waiting for LIDAR scan...')
        while self._scan is None:
            self._tick()
            time.sleep(0.05)

        time.sleep(3.0)   # let slam_toolbox warm up

        self._reset_stuck_timer()
        start    = time.time()
        last_log = 0

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  SLAM Explorer — LIDAR wall follower     ║')
        self.get_logger().info(f'║  Mapping for {EXPLORE_SECONDS}s  (escape logic ON)  ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')

        while True:
            self._tick()
            elapsed = time.time() - start

            if elapsed >= EXPLORE_SECONDS:
                break

            # Progress every 30 s
            bucket = int(elapsed) // 30
            if bucket > last_log:
                last_log = bucket
                self.get_logger().info(
                    f'  {elapsed:.0f}s elapsed — {EXPLORE_SECONDS - elapsed:.0f}s remaining'
                    f'  (escapes so far: {self._escape_count})')

            if self._scan is None:
                continue

            # Stuck check — fires every STUCK_TIMEOUT seconds
            if self._is_stuck():
                self._escape()
                continue

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
