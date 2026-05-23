#!/usr/bin/env python3
"""
delivery_mission.py — Autonomous Delivery Mission Node
=======================================================
Sends NavigateToPose action goals to Nav2, sequencing
multi-stop delivery routes with full feedback and error recovery.

Usage (after launching navigation_launch.py):
  ros2 run delivery_robot delivery_mission room1 room3 room2
  ros2 run delivery_robot delivery_mission --list
  ros2 run delivery_robot delivery_mission --loop room1 room2

Rooms defined in DELIVERY_ROOMS below — update x/y after SLAM mapping.
"""

import sys
import math
import time
import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# =============================================================================
#  DELIVERY ROOMS — map-frame coordinates (x, y, yaw_deg)
#  Update these values after your SLAM mapping run to match your map.
# =============================================================================
DELIVERY_ROOMS: dict[str, tuple[float, float, float]] = {
    #  name       x      y    yaw (degrees)
    "base":   (0.50,  2.00,    0.0),   # charging dock — always return here
    "room1":  (2.50,  2.00,    0.0),   # bottom-left room
    "room2":  (7.30,  2.00,  180.0),   # bottom-right room
    "room3":  (2.50,  5.30,    0.0),   # top-left room
    "room4":  (7.50,  6.00,  180.0),   # top-right room
    "corridor_left":  (2.50, 4.00, 90.0),   # left door junction
    "corridor_right": (7.50, 4.00, 90.0),   # right door junction
}

# Timeout (seconds) to wait for a goal to complete before giving up
NAV_TIMEOUT_SEC = 120.0


def yaw_to_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    """Convert yaw in degrees to quaternion (x, y, z, w)."""
    yaw_rad = math.radians(yaw_deg)
    return (
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    )


class DeliveryMission(Node):
    """
    ROS 2 node that sequences autonomous delivery routes using Nav2.

    Architecture:
      • NavigateToPose action client → Nav2 BT Navigator
      • Publishes mission status on /delivery_status (String)
      • Subscribes to /odom for real-time position logging
    """

    def __init__(self):
        super().__init__('delivery_mission')

        # ── Action client ─────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, '/delivery_status', 10)
        self._vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, odom_qos)
        self._current_x: float = 0.0
        self._current_y: float = 0.0

        # ── State ─────────────────────────────────────────────────────────
        self._mission_active = False
        self._cancelled = False

        self.get_logger().info('🤖  DeliveryMission node started')
        self.get_logger().info(
            f'   Known rooms: {list(DELIVERY_ROOMS.keys())}')

    # ─── Callbacks ────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y

    def _feedback_callback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        elapsed = fb.navigation_time.sec + fb.navigation_time.nanosec / 1e9
        # Log at most every 3 seconds (throttled)
        self.get_logger().info(
            f'   📍 pos=({self._current_x:.2f}, {self._current_y:.2f})  '
            f'remaining={dist:.2f} m  elapsed={elapsed:.1f} s',
            throttle_duration_sec=3,
        )

    # ─── Core navigation ─────────────────────────────────────────────────

    def _build_goal(self, x: float, y: float, yaw_deg: float) -> NavigateToPose.Goal:
        """Build a NavigateToPose goal from map coordinates."""
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        return goal

    def navigate_to_room(self, room_name: str) -> bool:
        """
        Send a navigation goal to the named room.
        Blocks until the goal is reached, failed, or timed out.
        Returns True on success, False on failure.
        """
        if room_name not in DELIVERY_ROOMS:
            self.get_logger().error(f'❌ Unknown room: "{room_name}"')
            self.get_logger().error(
                f'   Valid rooms: {list(DELIVERY_ROOMS.keys())}')
            return False

        x, y, yaw = DELIVERY_ROOMS[room_name]
        self.get_logger().info(
            f'🚀 Navigating to [{room_name}]  '
            f'target=({x:.2f}, {y:.2f}, {yaw:.0f}°)')
        self._publish_status(f'navigating:{room_name}')

        # Wait for Nav2 to be ready
        self.get_logger().info('   Waiting for navigate_to_pose action server...')
        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('   Action server not available!')
            return False

        # Send goal
        goal = self._build_goal(x, y, yaw)
        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle: ClientGoalHandle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'   Goal to [{room_name}] REJECTED by Nav2')
            self._publish_status(f'rejected:{room_name}')
            return False

        self.get_logger().info(f'   Goal accepted ✓ — robot is moving...')

        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        deadline = time.time() + NAV_TIMEOUT_SEC
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() > deadline:
                self.get_logger().warn(
                    f'   ⏰ Timeout waiting for [{room_name}] — cancelling goal')
                goal_handle.cancel_goal_async()
                self._publish_status(f'timeout:{room_name}')
                return False
            if self._cancelled:
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅  Arrived at [{room_name}]')
            self._publish_status(f'arrived:{room_name}')
            return True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'⚠️  Navigation to [{room_name}] was cancelled')
            self._publish_status(f'cancelled:{room_name}')
            return False
        else:
            self.get_logger().error(
                f'❌  Navigation to [{room_name}] FAILED  (status={status})')
            self._publish_status(f'failed:{room_name}')
            return False

    # ─── Mission orchestration ───────────────────────────────────────────

    def run_delivery_route(
        self,
        stops: list[str],
        return_to_base: bool = True,
        loop: bool = False,
    ) -> bool:
        """
        Execute a multi-stop delivery route.

        Args:
            stops:          Ordered list of room names to visit.
            return_to_base: Append 'base' to the route after all stops.
            loop:           Repeat the route indefinitely (Ctrl-C to stop).

        Returns:
            True if all stops succeeded, False if any failed.
        """
        if not stops:
            self.get_logger().warn('No delivery stops specified.')
            return False

        route = stops + (['base'] if return_to_base else [])
        self._mission_active = True

        iteration = 0
        while True:
            iteration += 1
            label = f'(loop {iteration})' if loop else ''
            self.get_logger().info(
                f'\n📋  Delivery route {label}: '
                f'{" → ".join(route)}\n'
                f'    {len(stops)} stop(s) + return-to-base'
            )
            self._publish_status(f'mission_start:{",".join(route)}')

            all_ok = True
            for i, room in enumerate(route, 1):
                self.get_logger().info(
                    f'\n── Stop {i}/{len(route)}: [{room}] ──────────────')
                success = self.navigate_to_room(room)

                if not success:
                    all_ok = False
                    self.get_logger().error(
                        f'Mission interrupted at stop [{room}].')
                    # Emergency return to base (unless we are already going there)
                    if room != 'base' and return_to_base:
                        self.get_logger().info(
                            '🆘  Attempting emergency return to base...')
                        self.navigate_to_room('base')
                    self._publish_status('mission_aborted')
                    break
                else:
                    # Brief pause at each delivery location
                    if room != 'base':
                        self.get_logger().info(
                            f'📦  Package delivered at [{room}]. '
                            f'Initiating drop-off sequence...')
                        self._perform_dropoff_dance()

            if all_ok:
                self.get_logger().info(
                    '\n🎉  Delivery mission complete! All packages delivered.\n')
                self._publish_status('mission_complete')

            if not loop:
                self._mission_active = False
                return all_ok

            # Loop mode: wait before repeating
            self.get_logger().info(
                '🔁  Loop mode active. Restarting route in 5 s...')
            time.sleep(5.0)

    def _perform_dropoff_dance(self) -> None:
        """Perform a 360-degree 'delivery complete' spin."""
        self.get_logger().info('🔄  Performing 360-degree delivery scan...')
        msg = Twist()
        msg.angular.z = 1.0  # rad/s
        
        # Spin for ~2*PI seconds
        duration = 2.0 * math.pi
        start_time = time.time()
        while time.time() - start_time < duration:
            if self._cancelled:
                break
            self._vel_pub.publish(msg)
            time.sleep(0.1)
            
        # Stop the robot
        self._vel_pub.publish(Twist())

    # ─── Utilities ────────────────────────────────────────────────────────

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def list_rooms(self) -> None:
        """Print all known delivery rooms and their coordinates."""
        self.get_logger().info('Known delivery rooms:')
        for name, (x, y, yaw) in DELIVERY_ROOMS.items():
            self.get_logger().info(
                f'  {name:20s}  x={x:6.2f}  y={y:6.2f}  yaw={yaw:6.1f}°')

    def shutdown(self) -> None:
        self._cancelled = True
        self._mission_active = False


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
  ros2 run delivery_robot delivery_mission --list
  ros2 run delivery_robot delivery_mission --loop room1 room2
  ros2 run delivery_robot delivery_mission --no-return room1
""",
    )
    parser.add_argument(
        'rooms', nargs='*', default=['room1', 'room2'],
        help='Ordered list of rooms to visit (default: room1 room2)',
    )
    parser.add_argument(
        '--list', action='store_true',
        help='List all known rooms and exit',
    )
    parser.add_argument(
        '--loop', action='store_true',
        help='Repeat the route indefinitely',
    )
    parser.add_argument(
        '--no-return', dest='no_return', action='store_true',
        help='Do not return to base after deliveries',
    )
    # Remove ROS arguments before parsing
    filtered = [a for a in (argv or []) if not a.startswith('--ros-args')]
    return parser.parse_args(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryMission()

    # Parse CLI arguments (strip ROS-specific ones)
    cli_args = sys.argv[1:]
    opts = parse_args(cli_args)

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
        node.get_logger().info('\n🛑  Mission interrupted by user (Ctrl-C)')
        node.shutdown()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # already shut down by signal handler


if __name__ == '__main__':
    main()
