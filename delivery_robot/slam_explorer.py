#!/usr/bin/env python3
"""
slam_explorer.py — Full-Map SLAM Explorer (< 6 min, returns to start)
======================================================================
Visits all 4 rooms in one loop and builds a complete map.

Design principles
-----------------
1. Generic waypoints  → proportional heading + front emergency-stop only.
   Side/diagonal avoidance is NOT used — it fights the heading controller
   near door frames and prevents the robot from entering rooms.

2. Narrow N-S doorways → dedicated _cross_door() that:
     a. Uses _go_to() to reach a pre-door alignment point
        (far enough from the frame that no avoidance fires).
     b. Turns to face exactly perpendicular (east or west).
     c. Drives straight through at reduced speed with ONLY a small
        Y-centering correction. No obstacle avoidance at all —
        the door is OPEN at the centre Y and the robot just passes.

3. Wide E-W gaps (1.20 m) → normal waypoint navigation.

World layout  10 m × 8 m
-------------------------
  Room 1 (SW)  x 0.1–5.1  y 0.1–3.9   start (0.5, 2.0)
  Room 2 (SE)  x 5.3–9.9  y 0.1–3.9
  Room 3 (NW)  x 0.1–5.1  y 4.1–7.9
  Room 4 (NE)  x 5.3–9.9  y 4.1–7.9

Door / gap positions (measured from world file)
-----------------------------------------------
  Door 1  R1↔R2  x≈5.21  y gap 2.40–3.10  centre y=2.75  (0.70 m clear)
  Door 2  R3↔R4  x≈5.21  y gap 5.80–6.70  centre y=6.25  (0.90 m clear)
  Left gap   R1↔R3  y≈4  x gap 1.9–3.1   centre x=2.5   (1.20 m clear)
  Right gap  R2↔R4  y≈4  x gap 6.9–8.1   centre x=7.5   (1.20 m clear)
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

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _wrap(a):
    return ((a + math.pi) % (2 * math.pi)) - math.pi


STOP = _vel()

# ── Speeds ────────────────────────────────────────────────────────────────────
CRUISE_SPEED  = 0.20   # m/s  open-space cruise
DOOR_SPEED    = 0.14   # m/s  door-crossing speed (slow + controlled)
TURN_SPEED    = 0.50   # rad/s

# ── Obstacle avoidance (used ONLY in _go_to, NOT during door crossing) ────────
FRONT_STOP    = 0.35   # m  reverse threshold (never fully stop — back up instead)
FRONT_WARN    = 0.65   # m  begin slowing + avoidance blending
DIAG_DIST     = 0.65   # m  diagonal avoidance trigger
DIAG_GAIN     = 0.80   # diagonal push gain
SIDE_DIST     = 0.28   # m  side avoidance trigger (0.10 m edge clearance)
SIDE_GAIN     = 0.45   # side push gain
# When BOTH lateral sides are within CORRIDOR_THR simultaneously (table-wall pocket,
# tight passage), diagonal avoidance is suppressed to prevent oscillation.
CORRIDOR_THR  = 0.65   # m  bilateral clearance threshold for corridor detection

# ── Navigation ────────────────────────────────────────────────────────────────
STUCK_TIME    = 1.2    # s  (reduced from 2.0 — faster escape trigger)
STUCK_THR     = 0.04   # m
WP_TIMEOUT    = 35.0   # s per waypoint

# ── Door crossing ─────────────────────────────────────────────────────────────
DOOR_Y_GAIN       = 3.5    # Y centering gain during straight crossing
DOOR_HEAD_GAIN    = 2.0    # heading correction gain during crossing
DOOR_PERP_TOL     = 0.05   # rad  (~3°) heading tolerance before crossing
DOOR_CROSS_TOUT   = 22.0   # s  crossing timeout
DOOR_ALIGN_ARRIVE = 0.28   # m  arrive distance for pre-door alignment
DOOR_MAX_TRIES    = 4

# =============================================================================
#  Exploration path
# =============================================================================
# Format: (x, y, arrive_m)  — all normal waypoints use 0.40 m arrive distance.

# ── Room 1 (SW) ──────────────────────────────────────────────────────────────
R1 = [
    (1.5, 1.5, 0.40),
    (3.2, 1.2, 0.40),   # south side  (cabinet at x=3.55–4.05 is clear to the E)
    (4.5, 1.5, 0.40),
    (3.5, 2.75, 0.40),  # pre-door-1 alignment: on door-centre Y, far from wall
]
# Door 1: Room 1 → Room 2  (east)
DOOR1 = dict(pre_x=3.5, door_y=2.75, exit_x=5.8, east=True)

# ── Room 2 (SE) ──────────────────────────────────────────────────────────────
# round_table at (8.5, 3.0) r=0.5 → route: south side → east column → north row
R2 = [
    # desk_r2a top=y1.35, desk_r2b top=y1.35 → y=1.8 gives 0.27 m clearance ✓
    # round_table at (8.5, 3.0) r=0.5 → east edge x=9.0, south edge y=2.5
    # Route goes WEST of the table (not east) to avoid the narrow table-wall pocket.
    (6.5, 1.8, 0.40),   # SW desk area
    (9.3, 1.5, 0.40),   # SE corner — south of table (table S edge y=2.5, safe)
    (9.3, 2.0, 0.40),   # east wall, below table — stays ≥0.5 m south of table
    (7.5, 2.0, 0.40),   # cross west of table (table W edge x=8.0, 0.5 m clear)
    (7.5, 3.9, 0.40),   # north along west side of table → X-align with right gap
]
# Right E-W gap: Room 2 → Room 4
R2_GAP = [
    (7.5, 3.8, 0.12),   # tight X-align (gap 1.2 m wide — forgiving)
    (7.5, 4.3, 0.40),
]

# ── Room 4 (NE) ──────────────────────────────────────────────────────────────
R4 = [
    (9.2, 5.0, 0.40),
    (8.7, 7.0, 0.40),   # N area  (server_rack x=9.2–9.8; was 9.0 → 0.02m clear, now 0.32m ✓)
    (7.5, 7.0, 0.40),   # NW  (printer x=6.15–6.85; was 7.0,7.2 INSIDE printer, now 0.52m ✓)
    (6.2, 6.25, 0.40),  # pre-door-2 alignment: on door-centre Y, inside R4
]
# Door 2: Room 4 → Room 3  (west)
DOOR2 = dict(pre_x=6.2, door_y=6.25, exit_x=4.3, east=False)

# ── Room 3 (NW) ──────────────────────────────────────────────────────────────
# conf_table x=1.5–3.5, y=6.0–7.0  →  go NE first, then west along y=7.5
R3 = [
    # conf_table x=1.5–3.5  y=6.0–7.0   shelf x=0.3–0.7  y=4.2–5.8
    # Old west-wall route (x=1.0) clipped the shelf (0.12 m clear < 0.15 m min).
    # New route stays EAST of the table throughout, then crosses below it:
    (4.5, 7.5, 0.40),   # NE — above table (y=7.5>7.0) east of table (x=4.5>3.5) ✓
    (4.5, 5.5, 0.40),   # SE of table — south of table (y=5.5<6.0) east (x=4.5>3.5) ✓
    (2.0, 5.5, 0.40),   # W below table — south of table ✓, shelf gap=1.12 m ✓
    (1.5, 5.0, 0.40),   # near shelf — shelf gap=0.62 m ✓
    (2.5, 4.8, 0.40),   # X-align with left gap
]
# Left E-W gap: Room 3 → Room 1
R3_GAP = [
    (2.5, 4.2, 0.12),   # tight X-align
    (2.5, 3.8, 0.40),
]

# ── Room 1 final + return to base ────────────────────────────────────────────
R1_END = [
    (2.5, 2.5, 0.40),
    (2.5, 1.5, 0.40),
    (0.5, 2.0, 0.40),   # BASE STATION
]


class SlamExplorer(Node):

    def __init__(self):
        super().__init__('slam_explorer')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)

        self._scan = None
        self._x = self._y = self._yaw = 0.0
        self._ck_t = time.time()
        self._ck_x = self._ck_y = 0.0
        self._esc_n = 0

    def _scan_cb(self, m): self._scan = m
    def _odom_cb(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self._x = p.x; self._y = p.y
        self._yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    def _tick(self): rclpy.spin_once(self, timeout_sec=0.05)

    # ── LIDAR ─────────────────────────────────────────────────────────────────

    def _smin(self, a, b):
        if self._scan is None: return float('inf')
        n = len(self._scan.ranges)
        v = [self._scan.ranges[i%n] for i in range(a,b)
             if math.isfinite(self._scan.ranges[i%n]) and self._scan.ranges[i%n]>0.12]
        return min(v) if v else float('inf')

    def _front(self): return self._smin(165, 195)   # ±15°
    def _back(self):  return self._smin(345, 375)   # ±15° behind (i%n wraps)

    def _most_open(self):
        """Return angular velocity toward most open quadrant.
        Returns 0.0 when back is the most open direction (caller reverses straight)."""
        if self._scan is None: return TURN_SPEED
        l = self._smin(225, 315)   # left quadrant
        r = self._smin(45,  135)   # right quadrant
        f = self._smin(157, 203)   # front
        b = self._back()           # back (wraps via i%n)
        # Back wins only when it strictly beats all other sectors
        if b >= f and b >= l and b >= r:
            return 0.0             # signal: reverse straight — pocket/corridor
        if l >= r and l >= f:
            return  TURN_SPEED
        if r >= l and r >= f:
            return -TURN_SPEED
        return TURN_SPEED if l >= r else -TURN_SPEED

    # ── Stuck detection ───────────────────────────────────────────────────────

    def _is_stuck(self):
        now = time.time()
        if now - self._ck_t < STUCK_TIME: return False
        d = math.hypot(self._x-self._ck_x, self._y-self._ck_y)
        self._ck_t=now; self._ck_x=self._x; self._ck_y=self._y
        return d < STUCK_THR

    def _rst_stuck(self):
        self._ck_t=time.time(); self._ck_x=self._x; self._ck_y=self._y

    # ── Open-space escape ─────────────────────────────────────────────────────

    def _escape(self):
        """
        Escape covering every stuck scenario:

        1. Pocket / corridor  (table-wall sandwich, tight passage)
           Detected when: back is most-open OR both lateral sides are close
           simultaneously.  Action: reverse straight out, then turn away from
           the tighter side.

        2. Standard one-wall stuck
           Detected when: back is not the winning direction.
           Action: reverse + turn toward most-open quadrant (escalating on
           repeated attempts; flips every 3rd to break oscillation).
        """
        self._esc_n += 1
        self.get_logger().warn(
            f'Escape #{self._esc_n} at ({self._x:.1f},{self._y:.1f})')

        f = self._smin(157, 203)
        l = self._smin(225, 315)
        r = self._smin(45,  135)
        b = self._back()

        # ── Scenario 1: pocket / bilateral corridor ───────────────────────────
        # Back is the most-open direction, OR both lateral sides are tight
        # (diag pushes from table and wall cancel → oscillation → stuck).
        pocket = (b >= f and b >= l and b >= r) or \
                 (l < FRONT_WARN and r < FRONT_WARN and b > FRONT_STOP)

        if pocket:
            self.get_logger().warn(
                f'  pocket/corridor detected '
                f'f={f:.2f} l={l:.2f} r={r:.2f} b={b:.2f} — backing out')

            # Phase 1: reverse straight until back clearance drops or 2 s elapse
            for _ in range(int(2.0 / 0.05)):
                self._tick()
                if self._back() < 0.28:
                    break
                self._pub.publish(_vel(-CRUISE_SPEED * 0.55, 0.0))
                time.sleep(0.05)
            self._pub.publish(STOP)
            time.sleep(0.08)

            # Phase 2: turn away from the tighter lateral side
            td = TURN_SPEED if l >= r else -TURN_SPEED
            if self._esc_n % 2 == 0:
                td = -td   # alternate turn direction every other pocket escape
            for _ in range(int(1.2 / 0.05)):
                self._tick()
                self._pub.publish(_vel(0.0, td))
                time.sleep(0.05)

        else:
            # ── Scenario 2: standard one-wall stuck ──────────────────────────
            td = self._most_open()
            if td == 0.0:
                td = TURN_SPEED    # back won but pocket check was False — use left
            if self._esc_n % 3 == 0:
                td = -td           # flip every 3rd to break left-biased loops
            scale = 1.0 if self._esc_n <= 2 else (1.5 if self._esc_n <= 5 else 2.5)

            # Phase 1: reverse while turning (clears wall without jamming)
            for _ in range(int(1.4 / 0.05)):
                self._tick()
                self._pub.publish(_vel(-CRUISE_SPEED * 0.65, td * 0.45))
                time.sleep(0.05)

            # Phase 2: forward sweep to exit blocked zone
            steps = int(1.0 * scale / 0.05)
            for _ in range(steps):
                self._tick()
                if self._front() < FRONT_WARN:
                    break
                self._pub.publish(_vel(CRUISE_SPEED, td * 0.25))
                time.sleep(0.05)

        self._pub.publish(STOP)
        self._rst_stuck()
        self.get_logger().info('Escape done — resuming')

    # ── General waypoint navigation ───────────────────────────────────────────

    def _go_to(self, tx, ty, arrive=0.40):
        """
        Navigate to (tx, ty).  Never fully stops for obstacles:
          • fd < FRONT_STOP  → back up + steer toward open space (no zero-vel stop)
          • fd < FRONT_WARN  → slow down, blend heading with open-space direction
          • fd clear         → full speed, proportional heading + avoidance pushes
        Avoidance is ALWAYS applied, not gated on fd > FRONT_WARN.
        """
        deadline = time.time() + WP_TIMEOUT
        self._rst_stuck()

        while time.time() < deadline:
            self._tick()
            dx, dy = tx - self._x, ty - self._y
            if math.hypot(dx, dy) < arrive:
                return True

            err = _wrap(math.atan2(dy, dx) - self._yaw)
            fd  = self._front()

            # Read all avoidance sectors (always, not just when fd is clear)
            dr = self._smin(125, 165)   # 15°–55° right
            dl = self._smin(195, 235)   # 15°–55° left
            sr = self._smin(78,  103)   # side right
            sl = self._smin(258, 283)   # side left

            # ── Narrow-corridor detection ─────────────────────────────────────
            # When BOTH lateral sides are simultaneously within CORRIDOR_THR the
            # robot is in a tight passage (table-wall pocket, tight doorway…).
            # Opposite diagonal pushes cancel or amplify each other causing
            # oscillation, so suppress diagonal avoidance in that case.
            in_corridor = sl < CORRIDOR_THR and sr < CORRIDOR_THR

            # Avoidance pushes (additive to heading angular)
            avoid = 0.0
            if not in_corridor:
                if dr < DIAG_DIST: avoid += TURN_SPEED * DIAG_GAIN * (1 - dr/DIAG_DIST)
                if dl < DIAG_DIST: avoid -= TURN_SPEED * DIAG_GAIN * (1 - dl/DIAG_DIST)
            if sr < SIDE_DIST: avoid += TURN_SPEED * SIDE_GAIN * (1 - sr/SIDE_DIST)
            if sl < SIDE_DIST: avoid -= TURN_SPEED * SIDE_GAIN * (1 - sl/SIDE_DIST)

            target_ang = _clamp(2.5 * err, -TURN_SPEED, TURN_SPEED)

            if fd < FRONT_STOP:
                # Too close — reverse while steering toward most-open direction.
                # Never publish zero linear: that jams the robot against the wall.
                open_ang = self._most_open()
                if open_ang == 0.0:
                    # Back is the most-open direction (pocket scenario) —
                    # reverse straight rather than spinning into the obstacle.
                    ang = _clamp(avoid, -TURN_SPEED * 0.3, TURN_SPEED * 0.3)
                else:
                    ang = _clamp(open_ang + avoid, -TURN_SPEED, TURN_SPEED)
                spd = -CRUISE_SPEED * 0.35   # slow reverse
            elif fd < FRONT_WARN:
                # Closing in — smoothly blend: heading (far) ↔ open-space (near)
                frac = _clamp((fd - FRONT_STOP) / (FRONT_WARN - FRONT_STOP), 0, 1)
                open_ang = self._most_open()
                ang = _clamp(frac * target_ang + (1 - frac) * open_ang + avoid,
                             -TURN_SPEED, TURN_SPEED)
                # Keep a minimum 12 % cruise so the robot never freezes
                spd = max(CRUISE_SPEED * 0.12, CRUISE_SPEED * 0.45 * frac)
            else:
                # Open path — heading control + avoidance
                ang = _clamp(target_ang + avoid, -TURN_SPEED, TURN_SPEED)
                spd = CRUISE_SPEED if abs(err) < 0.7 else CRUISE_SPEED * 0.4

            self._pub.publish(_vel(spd, ang))
            time.sleep(0.05)
            if self._is_stuck():
                self._escape()

        self.get_logger().warn(f'Timeout ({tx:.1f},{ty:.1f})')
        return False

    def _run(self, seg, label=''):
        for i,(x,y,a) in enumerate(seg,1):
            self.get_logger().info(f'  [{label}] WP {i}/{len(seg)} ({x:.1f},{y:.1f})')
            self._go_to(x, y, a)

    # ── Perpendicular turn ────────────────────────────────────────────────────

    def _turn_to(self, yaw, tol=DOOR_PERP_TOL, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._tick()
            err = _wrap(yaw - self._yaw)
            if abs(err) < tol: break
            self._pub.publish(_vel(0.0, math.copysign(TURN_SPEED*0.38, err)))
            time.sleep(0.05)
        self._pub.publish(STOP); time.sleep(0.12)

    # ── Narrow doorway crossing ───────────────────────────────────────────────

    def _cross_door(self, pre_x, door_y, exit_x, east=True):
        """
        Cross a narrow N-S doorway.

        pre_x   : X position of the pre-door alignment point (far from wall).
        door_y  : Y coordinate of the door centre.
        exit_x  : X target on the far side of the door.
        east    : True = cross heading east, False = heading west.

        Method
        ------
        1. _go_to(pre_x, door_y) — approach the door on the centre-Y line
           from a safe distance where no avoidance fires.
        2. _turn_to() — face exactly perpendicular (east/west).
        3. Drive straight at DOOR_SPEED with ONLY Y-centering and heading
           corrections.  Zero obstacle avoidance — the door is open at
           door_y and the robot will simply pass through.
        4. On failure (front < 0.15 m or timeout) back out and retry.
        """
        ty   = 0.0 if east else math.pi
        sign = 1   if east else -1

        for attempt in range(1, DOOR_MAX_TRIES+1):
            self.get_logger().info(
                f'Door y={door_y:.2f} attempt {attempt}: '
                f'aligning at x={pre_x:.1f}')

            # ── 1. Y-alignment ───────────────────────────────────────────────
            self._go_to(pre_x, door_y, DOOR_ALIGN_ARRIVE)

            # ── 2. Turn perpendicular ────────────────────────────────────────
            self._turn_to(ty)
            self.get_logger().info(
                f'Door y={door_y:.2f}: aligned ({self._x:.2f},{self._y:.2f}) '
                f'yaw={math.degrees(self._yaw):.1f}° — crossing')

            # ── 3. Straight crossing (NO avoidance) ──────────────────────────
            crossed  = False
            deadline = time.time() + DOOR_CROSS_TOUT
            self._rst_stuck()

            while time.time() < deadline:
                self._tick()
                if (exit_x - self._x) * sign <= 0.0:
                    crossed = True; break

                # Only stop if physically blocked (< 0.15 m — a real wall)
                if self._front() < 0.15:
                    self.get_logger().warn('Door: front blocked — backing out')
                    break

                # Y centering + heading correction (tiny angular only)
                ye  = door_y - self._y
                he  = _wrap(ty - self._yaw)
                ang = _clamp(ye*DOOR_Y_GAIN + he*DOOR_HEAD_GAIN,
                             -TURN_SPEED*0.35, TURN_SPEED*0.35)
                self._pub.publish(_vel(DOOR_SPEED, ang))
                time.sleep(0.05)

                if self._is_stuck():
                    self.get_logger().warn('Door: stuck — backing out')
                    break

            self._pub.publish(STOP)

            if crossed:
                self.get_logger().info(f'Door y={door_y:.2f} crossed ✓')
                return True

            # Back out straight before next attempt
            for _ in range(int(2.5/0.05)):
                self._tick(); self._pub.publish(_vel(-CRUISE_SPEED)); time.sleep(0.05)
            self._pub.publish(STOP); self._rst_stuck()

        self.get_logger().warn(f'Door y={door_y:.2f}: gave up after {DOOR_MAX_TRIES} tries')
        return False

    # ── Exploration ───────────────────────────────────────────────────────────

    def explore(self):
        self.get_logger().info('Waiting for LIDAR…')
        while self._scan is None:
            self._tick(); time.sleep(0.05)
        time.sleep(2.5)   # let slam_toolbox initialise

        t0 = time.time()
        self.get_logger().info('╔═══════════════════════════════════════════╗')
        self.get_logger().info('║  SLAM Explorer — 4-room full-map survey   ║')
        self.get_logger().info('╚═══════════════════════════════════════════╝')

        # Room 1
        self._run(R1, 'R1')

        # Door 1: R1 → R2
        self.get_logger().info('>>> Door 1  R1 → R2  (east) <<<')
        self._cross_door(**DOOR1)

        # Room 2
        self._run(R2, 'R2')

        # Right E-W gap: R2 → R4
        self.get_logger().info('>>> Right gap  R2 → R4 <<<')
        self._run(R2_GAP, 'R2gap')

        # Room 4
        self._run(R4, 'R4')

        # Door 2: R4 → R3
        self.get_logger().info('>>> Door 2  R4 → R3  (west) <<<')
        self._cross_door(**DOOR2)

        # Room 3
        self._run(R3, 'R3')

        # Left E-W gap: R3 → R1
        self.get_logger().info('>>> Left gap  R3 → R1 <<<')
        self._run(R3_GAP, 'R3gap')

        # Room 1 final + return to base
        self._run(R1_END, 'R1-end')

        elapsed = time.time() - t0
        self._pub.publish(STOP)
        self.get_logger().info(
            f'Exploration done in {elapsed:.0f} s  '
            f'({elapsed/60:.1f} min)  escapes={self._esc_n}')

        self.get_logger().info('╔═══════════════════════════════════════════╗')
        self.get_logger().info('║  Save map:                                ║')
        self.get_logger().info('║  ros2 run nav2_map_server map_saver_cli \\ ║')
        self.get_logger().info('║    -f ~/delivery_ws/src/delivery_robot/   ║')
        self.get_logger().info('║       maps/office_map                     ║')
        self.get_logger().info('║    --ros-args -p save_map_timeout:=10.0   ║')
        self.get_logger().info('╚═══════════════════════════════════════════╝')


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
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
