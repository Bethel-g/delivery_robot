#!/usr/bin/env python3
"""
dynamic_obstacles.py — Dynamic Obstacle Controller
====================================================
Moves two Gazebo obstacle boxes in real time using the
/gazebo/set_entity_state service, creating a dynamic environment
that forces the robot to react and re-plan.

Obstacle 1 (dynamic_box)  — orange: circular path around corridor centre
Obstacle 2 (dynamic_box2) — blue:   linear oscillation across the corridor

Run alongside the navigation stack:
  ros2 run delivery_robot dynamic_obstacles

Parameters (ros-args):
  obs1_speed  (float, default 0.4) — angular speed rad/s for circular path
  obs2_speed  (float, default 0.8) — linear speed m/s for back-and-forth path
  update_rate (float, default 10)  — Hz at which entity state is updated
"""

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String


class DynamicObstacleController(Node):

    # Obstacle 1: circular path centred in the middle corridor
    OBS1_CX = 5.0
    OBS1_CY = 4.0
    OBS1_RADIUS = 1.5   # metres

    # Obstacle 2: linear oscillation along y=4 corridor
    OBS2_Y = 4.0
    OBS2_X_MIN = 2.2
    OBS2_X_MAX = 7.8

    def __init__(self):
        super().__init__('dynamic_obstacle_controller')

        self.declare_parameter('obs1_speed', 0.4)
        self.declare_parameter('obs2_speed', 0.8)
        self.declare_parameter('update_rate', 10.0)

        self._obs1_speed = self.get_parameter('obs1_speed').value
        self._obs2_speed = self.get_parameter('obs2_speed').value
        rate = self.get_parameter('update_rate').value

        self._client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        self._status_pub = self.create_publisher(String, '/obstacle_status', 10)

        self._t = 0.0
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'DynamicObstacleController ready  '
            f'obs1_speed={self._obs1_speed:.2f} rad/s  '
            f'obs2_speed={self._obs2_speed:.2f} m/s  '
            f'rate={rate:.0f} Hz'
        )

    # ── Main update loop ──────────────────────────────────────────────────

    def _tick(self):
        self._t += self._dt

        if not self._client.service_is_ready():
            return

        # Obstacle 1: circular orbit
        angle = self._obs1_speed * self._t
        x1 = self.OBS1_CX + self.OBS1_RADIUS * math.cos(angle)
        y1 = self.OBS1_CY + self.OBS1_RADIUS * math.sin(angle)
        self._move('dynamic_box', x1, y1, 0.2)

        # Obstacle 2: linear ping-pong
        span = self.OBS2_X_MAX - self.OBS2_X_MIN
        period = 2.0 * span / self._obs2_speed
        phase = (self._t % period) / period   # 0→1
        if phase < 0.5:
            x2 = self.OBS2_X_MIN + span * (phase * 2.0)
        else:
            x2 = self.OBS2_X_MAX - span * ((phase - 0.5) * 2.0)
        self._move('dynamic_box2', x2, self.OBS2_Y, 0.2)

    # ── Gazebo helper ─────────────────────────────────────────────────────

    def _move(self, name: str, x: float, y: float, z: float) -> None:
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
