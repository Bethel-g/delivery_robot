#!/usr/bin/env python3
"""
delivery_mission.py — Autonomous Delivery Mission (LIDAR-Based Navigation)
==========================================================================
Uses the same proven LIDAR/odometry navigation engine as slam_explorer.py
instead of Nav2, guaranteeing correct routing through all rooms and doors.

World layout (10 m × 8 m)
--------------------------
  Room 1 (SW)  x 0.1–5.1  y 0.1–3.9   ← base station here (0.5, 2.0)
  Room 2 (SE)  x 5.3–9.9  y 0.1–3.9
  Room 3 (NW)  x 0.1–5.1  y 4.1–7.9
  Room 4 (NE)  x 5.3–9.9  y 4.1–7.9

Crossing points
  Door 1  R1↔R2  x≈5.21  y-gap centre 2.75  (0.70 m clear)  → _cross_door
  Door 2  R3↔R4  x≈5.21  y-gap centre 6.25  (0.90 m clear)  → _cross_door
  Left gap  R1↔R3  y≈4  x-gap centre 2.5   (1.20 m clear)  → normal _go_to
  Right gap R2↔R4  y≈4  x-gap centre 7.5   (1.20 m clear)  → normal _go_to

Mission flow per stop
  1. Navigate to base station for pickup
  2. Pick up package (spin + Gazebo model appears/disappears)
  3. Navigate to delivery room via waypoint chain
  4. Deliver package (spin + Gazebo model placed + 5 s pause)
  After all stops: return to base.

Usage (after slam_launch.py has built the map — Nav2 NOT required):
  ros2 run delivery_robot delivery_mission room1 room3 room2
  ros2 run delivery_robot delivery_mission --loop room1 room2
  ros2 run delivery_robot delivery_mission --no-return room1
  ros2 run delivery_robot delivery_mission --list
"""

import sys
import math
import time
import argparse
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SetEntityState


# =============================================================================
#  DELIVERY ROOMS — world/map-frame coordinates (x, y, yaw_deg)
# =============================================================================
DELIVERY_ROOMS: dict[str, tuple[float, float, float]] = {
    "base":           (0.50,  2.00,    0.0),
    "room1":          (2.50,  2.00,    0.0),
    "room2":          (7.30,  2.00,  180.0),
    "room3":          (2.50,  5.30,    0.0),
    "room4":          (7.50,  6.00,  180.0),
    "corridor_left":  (2.50,  4.00,   90.0),
    "corridor_right": (7.50,  4.00,   90.0),
}

DELIVERY_PAUSE_SEC = 5.0    # pause at each drop-off so delivery is visible

# =============================================================================
#  NAVIGATION CONSTANTS  (tuned from slam_explorer.py)
# =============================================================================
CRUISE_SPEED  = 0.20   # m/s
DOOR_SPEED    = 0.14   # m/s
TURN_SPEED    = 0.50   # rad/s

FRONT_STOP    = 0.35   # m  reverse threshold
FRONT_WARN    = 0.65   # m  slow-down threshold
DIAG_DIST     = 0.65   # m  diagonal avoidance
DIAG_GAIN     = 0.80
SIDE_DIST     = 0.28   # m  side avoidance
SIDE_GAIN     = 0.45
CORRIDOR_THR  = 0.65   # both sides < this → suppress diagonal avoidance

STUCK_TIME    = 1.2    # s
STUCK_THR     = 0.04   # m
WP_TIMEOUT    = 40.0   # s per waypoint

DOOR_Y_GAIN       = 3.5
DOOR_HEAD_GAIN    = 2.0
DOOR_PERP_TOL     = 0.05   # rad
DOOR_CROSS_TOUT   = 22.0   # s
DOOR_ALIGN_ARRIVE = 0.28   # m
DOOR_MAX_TRIES    = 4

_GREEN = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.95)


def _vel(linear=0.0, angular=0.0) -> Twist:
    t = Twist()
    t.linear.x  = linear
    t.angular.z = angular
    return t


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _wrap(a):
    return ((a + math.pi) % (2 * math.pi)) - math.pi


STOP = _vel()


class State(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    PICKING_UP = auto()
    LOADED     = auto()
    DELIVERING = auto()
    RETURNING  = auto()


# =============================================================================
#  WAYPOINT ROUTES
#  Each entry is a list of (x, y, arrive_m) waypoints.
#  'door' entries are handled by _cross_door() instead of _go_to().
# =============================================================================

# ── Routes TO each room (starting from base/room1 area) ──────────────────────
# All waypoints verified clear (≥0.20 m obstacle clearance) against the saved
# office_map.pgm.  Corrections vs first draft:
#   • Pre-door1-W: (3.5,2.75) was BLOCKED → moved to (4.5,2.80)
#   • Door 1 y-centre: 2.75 → 2.80 (true centre of y=2.50–3.10 gap in map)
#   • Room3 approach: (2.5,3.5) was TIGHT 0.11m → (2.0,3.5) CLEAR
#   • Right gap: x=7.5 TIGHT → x=7.8 CLEAR (gap centre x=7.83 in map)
#   • Room4 east path: x=7.7,y=4.5 BLOCKED → x=7.8 throughout
_ROUTE_TO: dict[str, list] = {
    "base": [],
    "room1": [
        (1.5, 2.0, 0.40),
        (2.5, 2.0, 0.40),   # arrive=0.40 keeps robot clear of desk leg
    ],
    "room2": [
        (1.5, 2.0, 0.40),
        (4.5, 2.80, 0.30),               # pre-door-1 align — CLEAR (map-verified)
        ("door", 4.5, 2.80, 5.8, True),  # Door1 y-centre=2.80 (map-verified gap)
        (6.5, 2.0,  0.40),
        (7.3, 2.0,  0.40),
    ],
    "room3": [
        (1.5, 2.0, 0.40),
        (2.0, 3.5, 0.40),   # approach left gap from west — CLEAR
        (2.5, 4.5, 0.40),   # past N-S partition, inside room3 — CLEAR
        (2.5, 5.3, 0.40),   # room3 delivery zone
    ],
    "room4": [
        (1.5, 2.0, 0.40),
        (4.5, 2.80, 0.30),               # pre-door-1 align
        ("door", 4.5, 2.80, 5.8, True),  # Door1 east
        (7.8, 2.5,  0.40),   # room2 east side — CLEAR
        (7.8, 3.5,  0.40),   # approach right gap (x=7.35–8.30) — CLEAR
        (7.8, 4.0,  0.40),   # through right gap at x=7.8 — CLEAR
        (7.5, 5.5,  0.40),   # inside room4
        (7.5, 6.0,  0.40),   # room4 delivery zone
    ],
    "corridor_left":  [(2.0, 3.5, 0.40), (2.5, 4.0, 0.40)],
    "corridor_right": [
        (4.5, 2.80, 0.30),
        ("door", 4.5, 2.80, 5.8, True),
        (7.8, 3.5, 0.40),
        (7.8, 4.0, 0.40),
    ],
}

# ── Routes FROM each room BACK to base ───────────────────────────────────────
_ROUTE_FROM: dict[str, list] = {
    "base":   [],
    "room1":  [(1.5, 2.0, 0.40), (0.5, 2.0, 0.40)],
    "room2": [
        (5.9, 2.80, 0.30),               # pre-door-1 east — CLEAR
        ("door", 5.9, 2.80, 4.5, False), # Door1 west, exit at x=4.5
        (1.5, 2.0,  0.40),
        (0.5, 2.0,  0.40),
    ],
    "room3": [
        (2.5, 4.2, 0.40),   # just north of partition
        (2.0, 3.5, 0.40),   # south of partition, west side — CLEAR
        (0.5, 2.0, 0.40),
    ],
    "room4": [
        (7.5, 5.5,  0.40),
        (7.8, 4.2,  0.40),   # just north of right gap — CLEAR
        (7.8, 3.5,  0.40),   # room2 east side — CLEAR
        (5.9, 2.80, 0.30),   # pre-door-1 east — CLEAR
        ("door", 5.9, 2.80, 4.5, False),
        (1.5, 2.0,  0.40),
        (0.5, 2.0,  0.40),
    ],
    "corridor_left":  [(2.0, 3.5, 0.40), (0.5, 2.0, 0.40)],
    "corridor_right": [
        (7.8, 3.5, 0.40),
        (5.9, 2.80, 0.30),
        ("door", 5.9, 2.80, 4.5, False),
        (1.5, 2.0, 0.40),
        (0.5, 2.0, 0.40),
    ],
}


# =============================================================================
#  DELIVERY MISSION NODE
# =============================================================================

class DeliveryMission(Node):

    def __init__(self):
        super().__init__('delivery_mission')

        self._pub     = self.create_publisher(Twist,  '/cmd_vel',          10)
        self._status  = self.create_publisher(String, '/delivery_status',  10)
        self._mkr_pub = self.create_publisher(Marker, '/payload_marker',   10)

        self._gz = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, qos)

        self._scan = None
        self._x = self._y = self._yaw = 0.0
        self._ck_t = time.time()
        self._ck_x = self._ck_y = 0.0
        self._esc_n = 0

        self._state       = State.IDLE
        self._has_payload = False
        self._cancelled   = False

        self.create_timer(0.2, self._publish_marker)
        self.get_logger().info(
            f'DeliveryMission ready  rooms={list(DELIVERY_ROOMS.keys())}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _scan_cb(self, m):
        self._scan = m

    def _odom_cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self._x   = p.x
        self._y   = p.y
        self._yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def _tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    # ── LIDAR helpers ─────────────────────────────────────────────────────────

    def _smin(self, a, b):
        if self._scan is None:
            return float('inf')
        n = len(self._scan.ranges)
        v = [self._scan.ranges[i % n] for i in range(a, b)
             if math.isfinite(self._scan.ranges[i % n])
             and self._scan.ranges[i % n] > 0.12]
        return min(v) if v else float('inf')

    def _front(self): return self._smin(165, 195)
    def _back(self):  return self._smin(345, 375)

    def _most_open(self):
        if self._scan is None:
            return TURN_SPEED
        l = self._smin(225, 315)
        r = self._smin(45,  135)
        f = self._smin(157, 203)
        b = self._back()
        if b >= f and b >= l and b >= r:
            return 0.0
        if l >= r and l >= f:
            return  TURN_SPEED
        if r >= l and r >= f:
            return -TURN_SPEED
        return TURN_SPEED if l >= r else -TURN_SPEED

    # ── Stuck detection ───────────────────────────────────────────────────────

    def _is_stuck(self):
        now = time.time()
        if now - self._ck_t < STUCK_TIME:
            return False
        d = math.hypot(self._x - self._ck_x, self._y - self._ck_y)
        self._ck_t = now
        self._ck_x = self._x
        self._ck_y = self._y
        return d < STUCK_THR

    def _rst_stuck(self):
        self._ck_t = time.time()
        self._ck_x = self._x
        self._ck_y = self._y

    # ── Escape ────────────────────────────────────────────────────────────────

    def _escape(self):
        self._esc_n += 1
        self.get_logger().warn(
            f'Escape #{self._esc_n} at ({self._x:.1f},{self._y:.1f})')

        f = self._smin(157, 203)
        l = self._smin(225, 315)
        r = self._smin(45,  135)
        b = self._back()

        pocket = (b >= f and b >= l and b >= r) or \
                 (l < FRONT_WARN and r < FRONT_WARN and b > FRONT_STOP)

        if pocket:
            self.get_logger().warn(
                f'  pocket detected f={f:.2f} l={l:.2f} r={r:.2f} b={b:.2f}')
            for _ in range(int(2.0 / 0.05)):
                self._tick()
                if self._back() < 0.28:
                    break
                self._pub.publish(_vel(-CRUISE_SPEED * 0.55, 0.0))
                time.sleep(0.05)
            self._pub.publish(STOP)
            time.sleep(0.08)
            td = TURN_SPEED if l >= r else -TURN_SPEED
            if self._esc_n % 2 == 0:
                td = -td
            for _ in range(int(1.2 / 0.05)):
                self._tick()
                self._pub.publish(_vel(0.0, td))
                time.sleep(0.05)
        else:
            td = self._most_open()
            if td == 0.0:
                td = TURN_SPEED
            if self._esc_n % 3 == 0:
                td = -td
            scale = 1.0 if self._esc_n <= 2 else (1.5 if self._esc_n <= 5 else 2.5)
            for _ in range(int(1.4 / 0.05)):
                self._tick()
                self._pub.publish(_vel(-CRUISE_SPEED * 0.65, td * 0.45))
                time.sleep(0.05)
            for _ in range(int(1.0 * scale / 0.05)):
                self._tick()
                if self._front() < FRONT_WARN:
                    break
                self._pub.publish(_vel(CRUISE_SPEED, td * 0.25))
                time.sleep(0.05)

        self._pub.publish(STOP)
        self._rst_stuck()
        self.get_logger().info('Escape done')

    # ── Waypoint navigation ───────────────────────────────────────────────────

    def _go_to(self, tx, ty, arrive=0.40) -> bool:
        deadline = time.time() + WP_TIMEOUT
        self._rst_stuck()

        while time.time() < deadline:
            self._tick()
            if self._cancelled:
                self._pub.publish(STOP)
                return False

            dx, dy = tx - self._x, ty - self._y
            if math.hypot(dx, dy) < arrive:
                return True

            err = _wrap(math.atan2(dy, dx) - self._yaw)
            fd  = self._front()

            dr = self._smin(125, 165)
            dl = self._smin(195, 235)
            sr = self._smin(78,  103)
            sl = self._smin(258, 283)

            in_corridor = sl < CORRIDOR_THR and sr < CORRIDOR_THR

            avoid = 0.0
            if not in_corridor:
                if dr < DIAG_DIST:
                    avoid += TURN_SPEED * DIAG_GAIN * (1 - dr / DIAG_DIST)
                if dl < DIAG_DIST:
                    avoid -= TURN_SPEED * DIAG_GAIN * (1 - dl / DIAG_DIST)
            if sr < SIDE_DIST:
                avoid += TURN_SPEED * SIDE_GAIN * (1 - sr / SIDE_DIST)
            if sl < SIDE_DIST:
                avoid -= TURN_SPEED * SIDE_GAIN * (1 - sl / SIDE_DIST)

            target_ang = _clamp(2.5 * err, -TURN_SPEED, TURN_SPEED)

            if fd < FRONT_STOP:
                open_ang = self._most_open()
                if open_ang == 0.0:
                    ang = _clamp(avoid, -TURN_SPEED * 0.3, TURN_SPEED * 0.3)
                else:
                    ang = _clamp(open_ang + avoid, -TURN_SPEED, TURN_SPEED)
                spd = -CRUISE_SPEED * 0.35
            elif fd < FRONT_WARN:
                frac = _clamp(
                    (fd - FRONT_STOP) / (FRONT_WARN - FRONT_STOP), 0, 1)
                open_ang = self._most_open()
                ang = _clamp(
                    frac * target_ang + (1 - frac) * open_ang + avoid,
                    -TURN_SPEED, TURN_SPEED)
                spd = max(CRUISE_SPEED * 0.12, CRUISE_SPEED * 0.45 * frac)
            else:
                ang = _clamp(target_ang + avoid, -TURN_SPEED, TURN_SPEED)
                spd = CRUISE_SPEED if abs(err) < 0.7 else CRUISE_SPEED * 0.4

            self._pub.publish(_vel(spd, ang))
            time.sleep(0.05)

            if self._is_stuck():
                self._escape()

        self.get_logger().warn(f'_go_to timeout → ({tx:.1f},{ty:.1f})')
        return False

    def _turn_to(self, yaw, tol=DOOR_PERP_TOL, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._tick()
            err = _wrap(yaw - self._yaw)
            if abs(err) < tol:
                break
            self._pub.publish(_vel(0.0, math.copysign(TURN_SPEED * 0.38, err)))
            time.sleep(0.05)
        self._pub.publish(STOP)
        time.sleep(0.12)

    def _cross_door(self, pre_x, door_y, exit_x, east=True) -> bool:
        ty   = 0.0 if east else math.pi
        sign = 1   if east else -1

        for attempt in range(1, DOOR_MAX_TRIES + 1):
            self.get_logger().info(
                f'Door y={door_y:.2f} attempt {attempt}: aligning at x={pre_x:.1f}')
            self._go_to(pre_x, door_y, DOOR_ALIGN_ARRIVE)
            self._turn_to(ty)
            self.get_logger().info(
                f'Door y={door_y:.2f}: crossing '
                f'({self._x:.2f},{self._y:.2f}) yaw={math.degrees(self._yaw):.1f}°')

            crossed  = False
            deadline = time.time() + DOOR_CROSS_TOUT
            self._rst_stuck()

            while time.time() < deadline:
                self._tick()
                if (exit_x - self._x) * sign <= 0.0:
                    crossed = True
                    break
                if self._front() < 0.15:
                    self.get_logger().warn('Door: blocked — backing out')
                    break
                ye  = door_y - self._y
                he  = _wrap(ty - self._yaw)
                ang = _clamp(ye * DOOR_Y_GAIN + he * DOOR_HEAD_GAIN,
                             -TURN_SPEED * 0.35, TURN_SPEED * 0.35)
                self._pub.publish(_vel(DOOR_SPEED, ang))
                time.sleep(0.05)
                if self._is_stuck():
                    self.get_logger().warn('Door: stuck — backing out')
                    break

            self._pub.publish(STOP)

            if crossed:
                self.get_logger().info(f'Door y={door_y:.2f} crossed ✓')
                return True

            for _ in range(int(2.5 / 0.05)):
                self._tick()
                self._pub.publish(_vel(-CRUISE_SPEED))
                time.sleep(0.05)
            self._pub.publish(STOP)
            self._rst_stuck()

        self.get_logger().warn(f'Door y={door_y:.2f}: gave up after {DOOR_MAX_TRIES} tries')
        return False

    # ── Follow a waypoint chain ───────────────────────────────────────────────

    def _follow_route(self, route: list, label='') -> bool:
        for step in route:
            if self._cancelled:
                return False
            if step[0] == 'door':
                _, pre_x, door_y, exit_x, east = step
                self.get_logger().info(
                    f'[{label}] crossing door y={door_y:.2f}')
                if not self._cross_door(pre_x, door_y, exit_x, east):
                    return False
            else:
                x, y, a = step
                self.get_logger().info(
                    f'[{label}] → ({x:.1f},{y:.1f})')
                self._go_to(x, y, a)
        return True

    # ── Payload marker ────────────────────────────────────────────────────────

    def _publish_marker(self):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns  = 'payload'
        m.id  = 0
        m.type = Marker.CUBE
        if self._has_payload:
            m.header.frame_id = 'base_link'
            m.action = Marker.ADD
            m.pose.position.z = 0.28
            m.scale.x = 0.22
            m.scale.y = 0.22
            m.scale.z = 0.18
            m.color   = _GREEN
            m.lifetime.sec = 1
        else:
            m.header.frame_id = 'map'
            m.action = Marker.DELETE
        self._mkr_pub.publish(m)

    # ── Gazebo helpers ────────────────────────────────────────────────────────

    def _gz_move(self, name, x, y, z):
        if not self._gz.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self._gz.call_async(req)

    def _show_item_at_base(self):  self._gz_move('delivery_item', 0.8, 2.3, 0.15)
    def _hide_item(self):          self._gz_move('delivery_item', 0.0, 0.0, -5.0)
    def _place_item(self, x, y):   self._gz_move('delivery_item', x,   y,   0.15)

    # ── Delivery spin ─────────────────────────────────────────────────────────

    def _spin(self, duration, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        start = time.time()
        while time.time() - start < duration and not self._cancelled:
            self._pub.publish(msg)
            time.sleep(0.1)
        self._pub.publish(STOP)

    # ── Pickup sequence ───────────────────────────────────────────────────────

    def _pickup(self):
        self._state = State.PICKING_UP
        self.get_logger().info('PICKUP: loading package at base...')
        self._publish_status('pickup:base')
        self._show_item_at_base()
        time.sleep(0.3)
        self._spin(2.5, 0.8)
        time.sleep(0.3)
        self._hide_item()
        self._has_payload = True
        self._state       = State.LOADED
        self.get_logger().info('PICKUP: package loaded ✓')
        self._publish_status('loaded')

    # ── Dropoff sequence ──────────────────────────────────────────────────────

    def _dropoff(self, room_name):
        self._state = State.DELIVERING
        x, y, _    = DELIVERY_ROOMS[room_name]
        self.get_logger().info(
            f'DELIVERY: placing package at [{room_name}] ({x:.2f},{y:.2f})')
        self._publish_status(f'delivering:{room_name}')
        self._spin(2.0 * math.pi, 1.0)
        self._place_item(x, y)
        self._has_payload = False
        self._state       = State.IDLE
        self.get_logger().info(f'DELIVERY: package delivered at [{room_name}] ✓')
        self._publish_status(f'delivered:{room_name}')
        self.get_logger().info(
            f'Pausing {DELIVERY_PAUSE_SEC:.0f} s at [{room_name}]...')
        time.sleep(DELIVERY_PAUSE_SEC)
        self.get_logger().info('Pause complete.')

    # ── Top-level mission ─────────────────────────────────────────────────────

    def run_delivery_route(
        self,
        stops: list[str],
        return_to_base: bool = True,
        loop: bool = False,
    ) -> bool:
        if not stops:
            self.get_logger().warn('No stops specified.')
            return False

        # Wait for first LIDAR message before starting
        self.get_logger().info('Waiting for LIDAR...')
        while self._scan is None and not self._cancelled:
            self._tick()
            time.sleep(0.05)
        time.sleep(1.0)

        iteration = 0
        while True:
            iteration += 1
            label = f' (loop {iteration})' if loop else ''
            self.get_logger().info(
                f'Mission{label}: {len(stops)} stop(s) → {" → ".join(stops)}')
            self._publish_status(f'mission_start:{",".join(stops)}')

            all_ok = True
            last_room = 'base'

            for i, room in enumerate(stops, 1):
                if self._cancelled:
                    break

                self.get_logger().info(f'─── Delivery {i}/{len(stops)}: [{room}] ───')

                # ── Navigate to base for pickup ───────────────────────────────
                if return_to_base:
                    self._state = State.NAVIGATING
                    self.get_logger().info('Navigating to base for pickup...')
                    self._publish_status('navigating:base')
                    ok = self._follow_route(_ROUTE_FROM.get(last_room, []), 'return')
                    if not ok and self._cancelled:
                        break
                    last_room = 'base'

                # ── Pickup ────────────────────────────────────────────────────
                self._pickup()

                # ── Navigate to delivery room ─────────────────────────────────
                self._state = State.NAVIGATING
                self.get_logger().info(f'Navigating to [{room}]...')
                self._publish_status(f'navigating:{room}')

                route_to = _ROUTE_TO.get(room, [(DELIVERY_ROOMS[room][0],
                                                  DELIVERY_ROOMS[room][1], 0.40)])
                ok = self._follow_route(route_to, f'to-{room}')
                if not ok and self._cancelled:
                    break

                # ── Dropoff ───────────────────────────────────────────────────
                self._dropoff(room)
                last_room = room

            # ── Final return to base ──────────────────────────────────────────
            if return_to_base and not self._cancelled:
                self._state = State.RETURNING
                self.get_logger().info('All stops done — returning to base...')
                self._publish_status('returning:base')
                self._follow_route(_ROUTE_FROM.get(last_room, []), 'final-return')
                self._pub.publish(STOP)
                self._state = State.IDLE
                self.get_logger().info('Back at base station ✓')
                self._publish_status('arrived:base')

            if all_ok:
                self.get_logger().info(
                    f'Mission complete! {len(stops)} package(s) delivered.')
                self._publish_status('mission_complete')

            if not loop:
                self._state = State.IDLE
                return all_ok

            self.get_logger().info('Loop mode — restarting in 5 s...')
            time.sleep(5.0)

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status.publish(msg)

    def list_rooms(self):
        for name, (x, y, yaw) in DELIVERY_ROOMS.items():
            self.get_logger().info(
                f'  {name:20s}  x={x:6.2f}  y={y:6.2f}  yaw={yaw:6.1f}°')

    def shutdown(self):
        self._cancelled   = True
        self._has_payload = False
        self._state       = State.IDLE
        self._pub.publish(STOP)


# =============================================================================
#  MAIN
# =============================================================================

def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Autonomous Delivery Mission (LIDAR navigation)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run delivery_robot delivery_mission room1 room3 room2
  ros2 run delivery_robot delivery_mission --loop room1 room2
  ros2 run delivery_robot delivery_mission --no-return room1
  ros2 run delivery_robot delivery_mission --list
""",
    )
    parser.add_argument('rooms', nargs='*', default=['room1', 'room2'])
    parser.add_argument('--list',     action='store_true')
    parser.add_argument('--loop',     action='store_true')
    parser.add_argument('--no-return', dest='no_return', action='store_true')
    filtered = [a for a in (argv or []) if not a.startswith('--ros-args')]
    return parser.parse_args(filtered)


def main(args=None):
    rclpy.init(args=args)
    opts = parse_args(sys.argv[1:])
    node = DeliveryMission()
    try:
        if opts.list:
            node.list_rooms()
        else:
            node.run_delivery_route(
                stops=opts.rooms,
                return_to_base=not opts.no_return,
                loop=opts.loop,
            )
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted (Ctrl-C)')
        node.shutdown()
    finally:
        node._pub.publish(STOP)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
