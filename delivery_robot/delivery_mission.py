#!/usr/bin/env python3
"""
delivery_mission.py — Autonomous Delivery Mission Node
=======================================================
Orchestrates a full pick-up → navigate → deliver → return cycle.

State machine:
  IDLE → PICKING_UP (at base) → LOADED → NAVIGATING → DELIVERING → RETURNING

Visual feedback:
  • /payload_marker  (visualization_msgs/Marker)   — green box above robot
  • /delivery_status (std_msgs/String)              — mission event strings
  • /gazebo/set_entity_state                        — moves delivery_item model

Usage (after launching navigation_launch.py):
  ros2 run delivery_robot delivery_mission room1 room3 room2
  ros2 run delivery_robot delivery_mission --list
  ros2 run delivery_robot delivery_mission --loop room1 room2
  ros2 run delivery_robot delivery_mission --algorithm mppi room1 room2
"""

import sys
import math
import time
import argparse
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SetEntityState


# =============================================================================
#  DELIVERY ROOMS — map-frame coordinates (x, y, yaw_deg)
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

NAV_TIMEOUT_SEC = 120.0

# Colour constants for payload marker
_GREEN  = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.95)
_GREY   = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4)


class State(Enum):
    IDLE        = auto()
    PICKING_UP  = auto()
    LOADED      = auto()
    NAVIGATING  = auto()
    DELIVERING  = auto()
    RETURNING   = auto()


def yaw_to_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    yaw_rad = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


class DeliveryMission(Node):
    """
    Autonomous delivery mission orchestrator.

    Adds on top of the original:
      • Delivery state machine with pickup / deliver phases
      • RViz marker (green box) that follows the robot when payload is loaded
      • Gazebo delivery_item model management via /gazebo/set_entity_state
      • Algorithm label for metrics correlation
    """

    def __init__(self):
        super().__init__('delivery_mission')

        self.declare_parameter('algorithm', 'dwa')
        self._algo = self.get_parameter('algorithm').value

        # ── Action client ─────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ────────────────────────────────────────────────────
        self._status_pub  = self.create_publisher(String, '/delivery_status', 10)
        self._vel_pub     = self.create_publisher(Twist,  '/cmd_vel', 10)
        self._marker_pub  = self.create_publisher(Marker, '/payload_marker', 10)

        # ── Gazebo state client (moves delivery_item in simulation) ───────
        self._gz_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        # ── Subscribers ───────────────────────────────────────────────────
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, odom_qos)
        self._odom_x = 0.0
        self._odom_y = 0.0

        # ── State ─────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._has_payload = False
        self._cancelled = False

        # Marker timer: refreshes the floating payload box at 5 Hz
        self._marker_timer = self.create_timer(0.2, self._publish_marker)

        self.get_logger().info(
            f'DeliveryMission ready  algorithm={self._algo}  '
            f'rooms={list(DELIVERY_ROOMS.keys())}')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'   pos=({self._odom_x:.2f},{self._odom_y:.2f})  '
            f'remaining={fb.distance_remaining:.2f} m  '
            f'elapsed={fb.navigation_time.sec}.{fb.navigation_time.nanosec//1_000_000:03d} s',
            throttle_duration_sec=3,
        )

    # ── Payload visual (RViz marker in base_link frame) ───────────────────

    def _publish_marker(self) -> None:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'payload'
        m.id = 0
        m.type = Marker.CUBE

        if self._has_payload:
            # Attached to robot: base_link frame, floats on top
            m.header.frame_id = 'base_link'
            m.action = Marker.ADD
            m.pose.position.z = 0.28
            m.scale.x = 0.22
            m.scale.y = 0.22
            m.scale.z = 0.18
            m.color = _GREEN
            m.lifetime.sec = 1          # auto-expires if not refreshed
        else:
            # Hide the marker
            m.header.frame_id = 'map'
            m.action = Marker.DELETE

        self._marker_pub.publish(m)

    # ── Gazebo delivery_item control ──────────────────────────────────────

    def _gz_move(self, name: str, x: float, y: float, z: float) -> None:
        """Teleport a Gazebo model to (x, y, z) in the world frame."""
        if not self._gz_client.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self._gz_client.call_async(req)

    def _show_delivery_item_at_base(self) -> None:
        """Respawn the package at the base station ready for pickup."""
        self._gz_move('delivery_item', 0.8, 2.3, 0.15)

    def _hide_delivery_item(self) -> None:
        """Move the Gazebo delivery_item model out of sight (picked up)."""
        self._gz_move('delivery_item', 0.0, 0.0, -5.0)

    def _place_delivery_item(self, x: float, y: float) -> None:
        """Place delivery_item at the delivery destination."""
        self._gz_move('delivery_item', x, y, 0.15)

    # ── Core navigation ───────────────────────────────────────────────────

    def _build_goal(self, x: float, y: float, yaw_deg: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        return goal

    def navigate_to_room(self, room_name: str) -> bool:
        if room_name not in DELIVERY_ROOMS:
            self.get_logger().error(f'Unknown room: "{room_name}"')
            return False

        x, y, yaw = DELIVERY_ROOMS[room_name]
        self.get_logger().info(
            f'Navigating to [{room_name}]  target=({x:.2f},{y:.2f},{yaw:.0f}°)')
        self._publish_status(f'navigating:{room_name}')

        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                'navigate_to_pose action server not available. '
                'Is navigation_launch.py running and fully started?')
            return False

        goal = self._build_goal(x, y, yaw)
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, future)

        handle: ClientGoalHandle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'Goal to [{room_name}] rejected')
            self._publish_status(f'rejected:{room_name}')
            return False

        result_future = handle.get_result_async()
        deadline = time.time() + NAV_TIMEOUT_SEC
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() > deadline:
                self.get_logger().warn(f'Timeout waiting for [{room_name}]')
                handle.cancel_goal_async()
                self._publish_status(f'timeout:{room_name}')
                return False
            if self._cancelled:
                handle.cancel_goal_async()
                return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arrived at [{room_name}]')
            self._publish_status(f'arrived:{room_name}')
            return True
        elif status == GoalStatus.STATUS_CANCELED:
            self._publish_status(f'cancelled:{room_name}')
            return False
        else:
            self.get_logger().error(
                f'Navigation to [{room_name}] FAILED (status={status})')
            self._publish_status(f'failed:{room_name}')
            return False

    # ── Mission phases ────────────────────────────────────────────────────

    def _pickup_sequence(self) -> None:
        """Simulate picking up the package at the base station."""
        self._state = State.PICKING_UP
        self.get_logger().info('PICKUP: loading package at base station...')
        self._publish_status('pickup:base')

        # Respawn the Gazebo package model at the base so it appears for pickup
        self._show_delivery_item_at_base()
        time.sleep(0.3)

        # Brief spin to "scan" the pickup area
        self._spin(duration=2.5, angular_z=0.8)
        time.sleep(0.5)

        # Package is now on the robot — hide the Gazebo ground model
        self._hide_delivery_item()

        self._has_payload = True
        self._state = State.LOADED
        self.get_logger().info('PICKUP: package loaded ✓')
        self._publish_status('loaded')

    def _dropoff_sequence(self, room_name: str) -> None:
        """Simulate delivering the package at a destination room."""
        self._state = State.DELIVERING
        x, y, _ = DELIVERY_ROOMS[room_name]
        self.get_logger().info(
            f'DELIVERY: placing package at [{room_name}] ({x:.2f},{y:.2f})')
        self._publish_status(f'delivering:{room_name}')

        # Delivery spin
        self._spin(duration=2.0 * math.pi, angular_z=1.0)

        # Place delivery_item at destination in Gazebo world
        self._place_delivery_item(x, y)

        self._has_payload = False
        self._state = State.IDLE
        self.get_logger().info(f'DELIVERY: package delivered at [{room_name}] ✓')
        self._publish_status(f'delivered:{room_name}')

    def _spin(self, duration: float, angular_z: float) -> None:
        msg = Twist()
        msg.angular.z = angular_z
        start = time.time()
        while time.time() - start < duration and not self._cancelled:
            self._vel_pub.publish(msg)
            time.sleep(0.1)
        self._vel_pub.publish(Twist())

    # ── Mission orchestration ─────────────────────────────────────────────

    def run_delivery_route(
        self,
        stops: list[str],
        return_to_base: bool = True,
        loop: bool = False,
    ) -> bool:
        """
        Execute a delivery route.

        For each stop:
          1. Navigate to base station for pickup
          2. Pick up package (spin + green marker)
          3. Navigate to delivery room
          4. Deliver package (spin + place Gazebo model)

        After all stops: return to base (if return_to_base=True).
        With --no-return, step 1 and the final return are skipped.
        """
        if not stops:
            self.get_logger().warn('No stops specified.')
            return False

        # ── Wait for Nav2 before doing anything ───────────────────────────────
        self.get_logger().info(
            'Waiting for navigate_to_pose server (up to 30 s)...\n'
            'Make sure navigation_launch.py is running!')
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'navigate_to_pose server not available after 30 s.\n'
                'Start the navigation stack first:\n'
                '  ros2 launch delivery_robot navigation_launch.py')
            return False
        self.get_logger().info('Nav2 ready — starting mission.')

        iteration = 0
        while True:
            iteration += 1
            label = f' (loop {iteration})' if loop else ''
            self.get_logger().info(
                f'Mission{label}: {len(stops)} stop(s) → {" → ".join(stops)}')
            self._publish_status(f'mission_start:{",".join(stops)}')

            all_ok = True

            for i, room in enumerate(stops, 1):
                if self._cancelled:
                    break

                self.get_logger().info(
                    f'─── Delivery {i}/{len(stops)}: [{room}] ───')

                # ── Step 1: navigate to base for pickup ───────────────────
                if return_to_base:
                    self._state = State.NAVIGATING
                    self.get_logger().info('Going to base station for pickup...')
                    ok = self.navigate_to_room('base')
                    if not ok:
                        self.get_logger().error(
                            'Could not reach base station — aborting mission')
                        all_ok = False
                        self._publish_status('mission_aborted')
                        break

                # ── Step 2: pick up ───────────────────────────────────────
                self._pickup_sequence()

                # ── Step 3: navigate to delivery room ─────────────────────
                self._state = State.NAVIGATING
                success = self.navigate_to_room(room)

                if not success:
                    all_ok = False
                    self.get_logger().warn(
                        f'Failed to reach [{room}] — dropping payload and returning to base')
                    self._has_payload = False
                    if return_to_base:
                        self._state = State.RETURNING
                        self.navigate_to_room('base')
                    self._publish_status('mission_aborted')
                    break

                # ── Step 4: deliver ───────────────────────────────────────
                self._dropoff_sequence(room)

            # ── Final return to base after all stops ──────────────────────
            if all_ok and return_to_base and not self._cancelled:
                self._state = State.RETURNING
                self.get_logger().info('All stops done — returning to base...')
                self._publish_status('returning:base')
                ok = self.navigate_to_room('base')
                self._state = State.IDLE
                if ok:
                    self.get_logger().info('Back at base station ✓')
                    self._publish_status('arrived:base')
                else:
                    self.get_logger().warn('Could not complete final return to base')

            if all_ok:
                self.get_logger().info(
                    f'Mission complete! {len(stops)} package(s) delivered.')
                self._publish_status('mission_complete')

            if not loop:
                self._state = State.IDLE
                return all_ok

            self.get_logger().info('Loop mode — restarting in 5 s...')
            time.sleep(5.0)

    # ── Utilities ─────────────────────────────────────────────────────────

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def list_rooms(self) -> None:
        for name, (x, y, yaw) in DELIVERY_ROOMS.items():
            self.get_logger().info(
                f'  {name:20s}  x={x:6.2f}  y={y:6.2f}  yaw={yaw:6.1f}°')

    def shutdown(self) -> None:
        self._cancelled = True
        self._has_payload = False
        self._state = State.IDLE


# =============================================================================
#  MAIN
# =============================================================================

def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Autonomous Delivery Mission Node',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run delivery_robot delivery_mission room1 room3 room2
  ros2 run delivery_robot delivery_mission --algorithm mppi room1 room2
  ros2 run delivery_robot delivery_mission --list
  ros2 run delivery_robot delivery_mission --loop room1 room2
  ros2 run delivery_robot delivery_mission --no-return room1
""",
    )
    parser.add_argument(
        'rooms', nargs='*', default=['room1', 'room2'],
        help='Ordered delivery rooms (default: room1 room2)',
    )
    parser.add_argument('--list',       action='store_true')
    parser.add_argument('--loop',       action='store_true')
    parser.add_argument('--no-return',  dest='no_return', action='store_true')
    parser.add_argument(
        '--algorithm', default='dwa', choices=['dwa', 'mppi'],
        help='Algorithm label for metrics (default: dwa)')

    filtered = [a for a in (argv or []) if not a.startswith('--ros-args')]
    return parser.parse_args(filtered)


def main(args=None):
    rclpy.init(args=args)

    cli_args = sys.argv[1:]
    opts = parse_args(cli_args)

    node = DeliveryMission()

    # Pass algorithm label so metrics_logger can correlate
    try:
        node.set_parameters([
            rclpy.parameter.Parameter(
                'algorithm',
                rclpy.parameter.Parameter.Type.STRING,
                opts.algorithm,
            )
        ])
    except Exception:
        pass
    node._algo = opts.algorithm

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
        node.get_logger().info('Mission interrupted by user (Ctrl-C)')
        node.shutdown()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
