#!/usr/bin/env python3
"""
dynamic_obstacles.py — Dynamic Obstacle Controller
====================================================
Moves two person-shaped Gazebo obstacle cylinders in real time using the
/gazebo/set_entity_state service, simulating office workers walking through
the building at a realistic medium walking pace.

Obstacle 1 (dynamic_box)  — south corridor: linear east-west walk at y≈2.0
Obstacle 2 (dynamic_box2) — north corridor: linear east-west walk at y≈5.0

Both paths are pre-validated to stay clear of all office furniture and walls.
The two obstacles operate in separate corridors and therefore never collide.

Run alongside the navigation stack:
  ros2 run delivery_robot dynamic_obstacles

Parameters (ros-args):
  obs1_speed  (float, default 0.25) — m/s for south-corridor person
  obs2_speed  (float, default 0.30) — m/s for north-corridor person
  update_rate (float, default 10)   — Hz at which entity state is updated
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String


class DynamicObstacleController(Node):

    # ── Obstacle 1: south corridor, crossing horizontally (Y-axis) ───────────
    OBS1_X     = 4.5
    OBS1_Z     = 0.8     # half-height of 1.6 m person cylinder
    OBS1_Y_MIN = 1.0
    OBS1_Y_MAX = 3.0

    # ── Obstacle 2: north corridor, furniture-safe linear path ────────────
    # y=5.0 keeps 0.75 m clear of conf_table_room3 (y≈6.5) and 0.55 m from
    # shelf_room3 (x≈0.5) when x >= 1.5
    OBS2_Y     = 5.0
    OBS2_Z     = 0.8
    OBS2_X_MIN = 1.5
    OBS2_X_MAX = 8.0

    def __init__(self):
        super().__init__('dynamic_obstacle_controller')

        # Medium walking-pace defaults (human walk ≈ 1.4 m/s; robots see ~0.25–0.3)
        self.declare_parameter('obs1_speed', 0.25)
        self.declare_parameter('obs2_speed', 0.30)
        self.declare_parameter('update_rate', 10.0)

        self._obs1_speed = self.get_parameter('obs1_speed').value
        self._obs2_speed = self.get_parameter('obs2_speed').value
        rate             = self.get_parameter('update_rate').value

        self._client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        self._status_pub = self.create_publisher(String, '/obstacle_status', 10)

        self._t  = 0.0
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'DynamicObstacleController ready — '
            f'obs1 south-corridor {self._obs1_speed:.2f} m/s | '
            f'obs2 north-corridor {self._obs2_speed:.2f} m/s | '
            f'rate {rate:.0f} Hz'
        )

    # ── Main update loop ──────────────────────────────────────────────────

    def _tick(self):
        self._t += self._dt

        if not self._client.service_is_ready():
            return

        import math
        # Person 1 — south corridor moving horizontally across Y
        y1, yaw1 = self._ping_pong(
            self._t, self.OBS1_Y_MIN, self.OBS1_Y_MAX, self._obs1_speed)
        self._move('dynamic_box', self.OBS1_X, y1, self.OBS1_Z, yaw1 + math.pi / 2.0)

        # Person 2 is disabled as per request
        # half_period2 = (self.OBS2_X_MAX - self.OBS2_X_MIN) / self._obs2_speed
        # x2, yaw2 = self._ping_pong(
        #     self._t + half_period2,
        #     self.OBS2_X_MIN, self.OBS2_X_MAX, self._obs2_speed)
        # self._move('dynamic_box2', x2, self.OBS2_Y, self.OBS2_Z, yaw2)
        x2 = 0.0

        # Publish status for monitoring / debugging
        msg = String()
        msg.data = (
            f'person1=({self.OBS1_X:.2f}, {y1:.1f}) '
            f'person2=({x2:.2f}, {self.OBS2_Y:.1f})'
        )
        self._status_pub.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _ping_pong(t: float, val_min: float, val_max: float, speed: float) -> tuple[float, float]:
        """Linear ping-pong oscillation between val_min and val_max at given speed.
        Returns (position, yaw)."""
        import math
        span   = val_max - val_min
        period = 2.0 * span / speed
        phase  = (t % period) / period   # normalised 0 → 1
        if phase < 0.5:
            return val_min + span * (phase * 2.0), 0.0
        else:
            return val_max - span * ((phase - 0.5) * 2.0), math.pi

    def _move(self, name: str, x: float, y: float, z: float, yaw: float) -> None:
        import math
        req = SetEntityState.Request()
        req.state.name                   = name
        req.state.pose.position.x        = float(x)
        req.state.pose.position.y        = float(y)
        req.state.pose.position.z        = float(z)
        req.state.pose.orientation.x     = 0.0
        req.state.pose.orientation.y     = 0.0
        req.state.pose.orientation.z     = math.sin(yaw / 2.0)
        req.state.pose.orientation.w     = math.cos(yaw / 2.0)
        req.state.reference_frame        = 'world'
        self._client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
