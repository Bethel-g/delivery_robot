#!/usr/bin/env python3
"""
test_delivery_mission.py — Unit Tests for DeliveryMission
==========================================================
Tests the yaw_to_quaternion conversion and DELIVERY_ROOMS validation
without requiring a running ROS 2 instance.
"""

import math
import pytest


# Import the pure functions we can test offline
from delivery_robot.delivery_mission import (
    yaw_to_quaternion,
    DELIVERY_ROOMS,
)


class TestYawToQuaternion:
    """Test quaternion conversion accuracy."""

    def test_zero_yaw(self):
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        assert abs(qx) < 1e-9
        assert abs(qy) < 1e-9
        assert abs(qz) < 1e-9
        assert abs(qw - 1.0) < 1e-9

    def test_180_deg(self):
        qx, qy, qz, qw = yaw_to_quaternion(180.0)
        assert abs(qx) < 1e-6
        assert abs(qy) < 1e-6
        assert abs(qz - 1.0) < 1e-6
        assert abs(qw) < 1e-6

    def test_90_deg(self):
        qx, qy, qz, qw = yaw_to_quaternion(90.0)
        expected_zw = math.sqrt(2) / 2
        assert abs(qz - expected_zw) < 1e-6
        assert abs(qw - expected_zw) < 1e-6

    def test_unit_quaternion(self):
        for yaw in [0, 45, 90, 135, 180, -90, -180]:
            qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            assert abs(norm - 1.0) < 1e-6, f'Non-unit quaternion at yaw={yaw}'

    def test_negative_yaw(self):
        qx, qy, qz, qw = yaw_to_quaternion(-90.0)
        expected_zw = math.sqrt(2) / 2
        assert abs(qz + expected_zw) < 1e-6
        assert abs(qw - expected_zw) < 1e-6


class TestDeliveryRooms:
    """Test DELIVERY_ROOMS configuration."""

    def test_base_exists(self):
        assert 'base' in DELIVERY_ROOMS, '"base" room must exist for return-to-base'

    def test_all_rooms_have_three_coords(self):
        for name, coords in DELIVERY_ROOMS.items():
            assert len(coords) == 3, f'Room {name} must have (x, y, yaw)'

    def test_room_coords_are_numeric(self):
        for name, (x, y, yaw) in DELIVERY_ROOMS.items():
            assert isinstance(x, (int, float)), f'{name}.x must be numeric'
            assert isinstance(y, (int, float)), f'{name}.y must be numeric'
            assert isinstance(yaw, (int, float)), f'{name}.yaw must be numeric'

    def test_yaw_in_degree_range(self):
        for name, (_, _, yaw) in DELIVERY_ROOMS.items():
            assert -360 <= yaw <= 360, (
                f'Room {name} yaw={yaw} should be in degrees (-360 to 360)'
            )

    def test_base_near_origin(self):
        x, y, _ = DELIVERY_ROOMS['base']
        assert abs(x) < 5.0, 'Base station should be near the origin'
        assert abs(y) < 5.0, 'Base station should be near the origin'

    def test_room_names_are_strings(self):
        for name in DELIVERY_ROOMS:
            assert isinstance(name, str)
            assert len(name) > 0

    def test_room_positions_within_world(self):
        """All delivery rooms should fit within the 10x8 office world."""
        for name, (x, y, _) in DELIVERY_ROOMS.items():
            assert 0 <= x <= 10, f'Room {name} x={x} outside world (0–10 m)'
            assert 0 <= y <= 8,  f'Room {name} y={y} outside world (0–8 m)'
