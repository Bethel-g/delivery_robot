#!/usr/bin/env python3
"""
metrics_logger.py — Navigation Performance Metrics Logger
==========================================================
Records per-mission performance data for comparing Algorithm 1 (DWB)
vs Algorithm 2 (MPPI).  Subscribes to /odom and /delivery_status,
then writes a CSV row after every completed or aborted mission.

Metrics captured:
  - duration_s      : wall-clock mission time (seconds)
  - path_length_m   : integrated odometry path (metres)
  - avg_velocity_ms : mean forward speed while moving (m/s)
  - recoveries      : number of Nav2 recovery behaviours triggered
  - success         : True/False

Run alongside the navigation stack:
  ros2 run delivery_robot metrics_logger --ros-args -p algorithm:=dwa
  ros2 run delivery_robot metrics_logger --ros-args -p algorithm:=mppi

At session end (Ctrl-C) a comparison table is printed to the terminal.
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MetricsLogger(Node):

    FIELDS = [
        'timestamp', 'algorithm', 'mission',
        'duration_s', 'path_length_m', 'avg_velocity_ms',
        'recoveries', 'success',
    ]

    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('algorithm', 'dwa')
        self.declare_parameter(
            'log_file', os.path.expanduser('~/delivery_metrics.csv'))

        self._algo = self.get_parameter('algorithm').value
        self._log = self.get_parameter('log_file').value

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, odom_qos)
        self._status_sub = self.create_subscription(
            String, '/delivery_status', self._status_cb, 10)

        # Odometry tracking
        self._prev_x: float | None = None
        self._prev_y: float | None = None
        self._path_m = 0.0
        self._speeds: list[float] = []

        # Mission state
        self._start_time: float | None = None
        self._mission_label = ''
        self._recoveries = 0

        # Persistent records for end-of-session summary
        self._records: list[dict] = []

        self._ensure_header()
        self.get_logger().info(
            f'MetricsLogger ready  algorithm={self._algo}  log={self._log}')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        if self._prev_x is not None and self._start_time is not None:
            self._path_m += math.hypot(x - self._prev_x, y - self._prev_y)

        speed = math.hypot(vx, vy)
        if self._start_time is not None and speed > 0.01:
            self._speeds.append(speed)

        self._prev_x, self._prev_y = x, y

    def _status_cb(self, msg: String) -> None:
        status = msg.data

        if status.startswith('mission_start:'):
            self._mission_label = status.split(':', 1)[1]
            self._start_time = time.time()
            self._path_m = 0.0
            self._speeds = []
            self._recoveries = 0
            self._prev_x = None
            self._prev_y = None
            self.get_logger().info(
                f'[METRICS] mission started: {self._mission_label}')

        elif 'recovery' in status or 'timeout' in status:
            self._recoveries += 1

        elif status in ('mission_complete', 'mission_aborted'):
            self._finish(success=(status == 'mission_complete'))

    # ── Recording ─────────────────────────────────────────────────────────

    def _finish(self, success: bool) -> None:
        if self._start_time is None:
            return

        duration = time.time() - self._start_time
        avg_vel = (sum(self._speeds) / len(self._speeds)
                   if self._speeds else 0.0)

        rec = {
            'timestamp':      datetime.now().isoformat(timespec='seconds'),
            'algorithm':      self._algo,
            'mission':        self._mission_label,
            'duration_s':     round(duration, 2),
            'path_length_m':  round(self._path_m, 3),
            'avg_velocity_ms': round(avg_vel, 4),
            'recoveries':     self._recoveries,
            'success':        success,
        }
        self._records.append(rec)
        self._append_csv(rec)

        self.get_logger().info(
            f'\n╔══ METRICS ═══════════════════════════════════\n'
            f'║  Algorithm   : {self._algo}\n'
            f'║  Mission     : {self._mission_label}\n'
            f'║  Duration    : {duration:.2f} s\n'
            f'║  Path length : {self._path_m:.3f} m\n'
            f'║  Avg speed   : {avg_vel:.4f} m/s\n'
            f'║  Recoveries  : {self._recoveries}\n'
            f'║  Success     : {success}\n'
            f'╚══════════════════════════════════════════════'
        )
        self._start_time = None

    # ── CSV helpers ───────────────────────────────────────────────────────

    def _ensure_header(self) -> None:
        if not os.path.exists(self._log):
            with open(self._log, 'w', newline='') as f:
                csv.DictWriter(f, fieldnames=self.FIELDS).writeheader()

    def _append_csv(self, rec: dict) -> None:
        with open(self._log, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=self.FIELDS).writerow(rec)
        self.get_logger().info(f'[METRICS] appended to {self._log}')

    # ── Session summary ───────────────────────────────────────────────────

    def print_summary(self) -> None:
        if not self._records:
            return
        self.get_logger().info(
            '\n╔══ SESSION SUMMARY ═══════════════════════════════════════════════════╗')
        self.get_logger().info(
            f'  {"Algo":<6} {"Mission":<30} {"Time(s)":>8} '
            f'{"Path(m)":>8} {"AvgSpd":>8} {"Rec":>4} {"OK":>4}')
        self.get_logger().info('  ' + '─' * 72)
        for r in self._records:
            ok = 'Y' if r['success'] else 'N'
            self.get_logger().info(
                f"  {r['algorithm']:<6} {r['mission']:<30} "
                f"{r['duration_s']:>8.1f} {r['path_length_m']:>8.2f} "
                f"{r['avg_velocity_ms']:>8.4f} {r['recoveries']:>4} {ok:>4}")
        self.get_logger().info(
            '╚══════════════════════════════════════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
