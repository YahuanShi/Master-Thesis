#!/usr/bin/env python3
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from kinematics import ChassisParams, compute_drive  # noqa: E402

PARAMS = ChassisParams()


# ---------- deadzone ----------

class TestDeadzone:
    def test_all_below_deadzone(self):
        pos, vel = compute_drive(0.005, 0.005, 0.005, PARAMS)
        np.testing.assert_array_equal(pos, 0.0)
        np.testing.assert_array_equal(vel, 0.0)

    def test_vx_above_deadzone(self):
        _, vel = compute_drive(1.0, 0.0, 0.0, PARAMS)
        assert np.all(vel != 0.0)

    def test_exact_deadzone_boundary_passes_through(self):
        """Deadzone is strict '<', so exactly PARAMS.deadzone is NOT filtered."""
        _, vel = compute_drive(PARAMS.deadzone, 0.0, 0.0, PARAMS)
        assert np.all(vel != 0.0)


# ---------- straight drive (crab with vy=0) ----------

class TestStraightDrive:
    def test_forward_wheels_straight(self):
        pos, vel = compute_drive(1.0, 0.0, 0.0, PARAMS)
        np.testing.assert_array_equal(pos, 0.0)

    def test_forward_velocity_positive(self):
        _, vel = compute_drive(1.0, 0.0, 0.0, PARAMS)
        assert np.all(vel > 0)

    def test_forward_all_wheels_equal(self):
        _, vel = compute_drive(1.0, 0.0, 0.0, PARAMS)
        np.testing.assert_allclose(vel, vel[0])

    def test_forward_velocity_scales_with_gain(self):
        _, vel = compute_drive(1.0, 0.0, 0.0, PARAMS)
        expected = 1.0 * PARAMS.drive_gain
        np.testing.assert_allclose(vel, expected)

    def test_backward_velocity_negative(self):
        _, vel = compute_drive(-1.0, 0.0, 0.0, PARAMS)
        assert np.all(vel < 0)

    def test_backward_wheels_straight(self):
        pos, _ = compute_drive(-1.0, 0.0, 0.0, PARAMS)
        np.testing.assert_array_equal(pos, 0.0)

    def test_zero_cmd_vel(self):
        pos, vel = compute_drive(0.0, 0.0, 0.0, PARAMS)
        np.testing.assert_array_equal(pos, 0.0)
        np.testing.assert_array_equal(vel, 0.0)


# ---------- pivot turn ----------

class TestPivotTurn:
    def test_pivot_steering_geometry(self):
        pos, _ = compute_drive(0.0, 0.0, 1.0, PARAMS)
        ang = math.atan(PARAMS.wheel_base / PARAMS.steering_track)
        np.testing.assert_allclose(pos, [-ang, ang, ang, -ang])

    def test_pivot_right_velocities(self):
        _, vel = compute_drive(0.0, 0.0, 1.0, PARAMS)
        g = PARAMS.drive_gain
        np.testing.assert_allclose(vel, [-g, g, -g, g])

    def test_pivot_left_reverses_vel(self):
        _, vel = compute_drive(0.0, 0.0, -1.0, PARAMS)
        g = PARAMS.drive_gain
        np.testing.assert_allclose(vel, [g, -g, g, -g])

    def test_pivot_left_same_steer_as_right(self):
        pos_r, _ = compute_drive(0.0, 0.0, 1.0, PARAMS)
        pos_l, _ = compute_drive(0.0, 0.0, -1.0, PARAMS)
        np.testing.assert_allclose(pos_r, pos_l)

    def test_pivot_vel_scales_with_wz(self):
        _, vel1 = compute_drive(0.0, 0.0, 0.5, PARAMS)
        _, vel2 = compute_drive(0.0, 0.0, 1.0, PARAMS)
        np.testing.assert_allclose(vel1 * 2, vel2)


# ---------- Ackermann ----------

class TestAckermann:
    def test_front_wheels_steer(self):
        pos, _ = compute_drive(1.0, 0.0, 0.5, PARAMS)
        assert pos[0] != 0.0
        assert pos[1] != 0.0

    def test_rear_wheels_straight(self):
        pos, _ = compute_drive(1.0, 0.0, 0.5, PARAMS)
        assert pos[2] == 0.0
        assert pos[3] == 0.0

    def test_inner_outer_angle_differ(self):
        pos, _ = compute_drive(1.0, 0.0, 0.5, PARAMS)
        assert abs(pos[0]) != pytest.approx(abs(pos[1]))

    def test_all_wheels_forward(self):
        _, vel = compute_drive(1.0, 0.0, 0.5, PARAMS)
        assert np.all(vel > 0)

    def test_left_right_velocity_differ(self):
        _, vel = compute_drive(1.0, 0.0, 0.5, PARAMS)
        assert vel[0] != pytest.approx(vel[1])

    def test_front_rear_same_side_equal(self):
        _, vel = compute_drive(1.0, 0.0, 0.5, PARAMS)
        assert vel[0] == pytest.approx(vel[2])
        assert vel[1] == pytest.approx(vel[3])


# ---------- crab walk ----------

class TestCrabWalk:
    def test_all_wheels_same_angle(self):
        pos, _ = compute_drive(1.0, 1.0, 0.0, PARAMS)
        np.testing.assert_allclose(pos, pos[0])

    def test_all_wheels_same_velocity(self):
        _, vel = compute_drive(1.0, 1.0, 0.0, PARAMS)
        np.testing.assert_allclose(vel, vel[0])

    def test_magnitude_correct(self):
        _, vel = compute_drive(3.0, 4.0, 0.0, PARAMS)
        expected = math.hypot(3.0, 4.0) * PARAMS.drive_gain
        np.testing.assert_allclose(vel, expected)

    def test_45_degree_crab(self):
        pos, _ = compute_drive(1.0, 1.0, 0.0, PARAMS)
        expected_angle = math.atan2(1.0, 1.0)
        np.testing.assert_allclose(pos, expected_angle, atol=1e-10)


# ---------- custom params ----------

class TestCustomParams:
    def test_different_gain(self):
        p = ChassisParams(drive_gain=5.0)
        _, vel = compute_drive(1.0, 0.0, 0.0, p)
        np.testing.assert_allclose(vel, 5.0)

    def test_different_deadzone(self):
        p = ChassisParams(deadzone=0.5)
        _, vel = compute_drive(0.4, 0.0, 0.0, p)
        np.testing.assert_array_equal(vel, 0.0)

    def test_steering_track_with_offset(self):
        p = ChassisParams(wheel_steering_y_offset=0.1)
        expected = p.wheel_separation - 2 * 0.1
        assert p.steering_track == pytest.approx(expected)
