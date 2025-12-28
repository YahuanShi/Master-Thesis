#!/usr/bin/env python3
import math
import os
import sys

import numpy as np

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

import tf_transformations
from aruco_localization import pose_msg_to_matrix, pose_to_matrix  # noqa: E402
from geometry_msgs.msg import Point, Pose, Quaternion


class TestPoseToMatrix:
    def test_identity(self):
        T = pose_to_matrix(0, 0, 0, 0, 0, 0)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-10)

    def test_translation_only(self):
        T = pose_to_matrix(1.0, 2.0, 3.0, 0, 0, 0)
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-10)

    def test_yaw_90(self):
        T = pose_to_matrix(0, 0, 0, 0, 0, math.pi / 2)
        expected_rot = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1],
        ])
        np.testing.assert_allclose(T[:3, :3], expected_rot, atol=1e-10)

    def test_combined(self):
        T = pose_to_matrix(5.0, -3.0, 1.0, 0, 0, math.pi)
        assert abs(T[0, 3] - 5.0) < 1e-10
        assert abs(T[1, 3] - (-3.0)) < 1e-10
        assert abs(T[2, 3] - 1.0) < 1e-10
        np.testing.assert_allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-10)

    def test_homogeneous_row(self):
        T = pose_to_matrix(1, 2, 3, 0.1, 0.2, 0.3)
        np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], atol=1e-10)


class TestPoseMsgToMatrix:
    def test_identity_quaternion(self):
        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        T = pose_msg_to_matrix(pose)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-10)

    def test_translation(self):
        pose = Pose()
        pose.position = Point(x=1.0, y=2.0, z=3.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        T = pose_msg_to_matrix(pose)
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0])

    def test_90deg_yaw(self):
        q = tf_transformations.quaternion_from_euler(0, 0, math.pi / 2)
        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=0.0)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        T = pose_msg_to_matrix(pose)
        np.testing.assert_allclose(T[:3, :3], np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1],
        ]), atol=1e-10)

    def test_consistency_with_pose_to_matrix(self):
        x, y, z = 2.0, -1.0, 0.5
        roll, pitch, yaw = 0.1, 0.2, 0.3
        T_euler = pose_to_matrix(x, y, z, roll, pitch, yaw)

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        T_quat = pose_msg_to_matrix(pose)

        np.testing.assert_allclose(T_euler, T_quat, atol=1e-10)


class TestCorrectionMath:
    """Test the correction matrix computation used in markers_callback."""

    def test_no_error_gives_identity_correction(self):
        T_known = pose_to_matrix(5.0, 3.0, 0.0, 0, 0, 0.5)
        T_observed = T_known.copy()
        T_correction = T_known @ np.linalg.inv(T_observed)
        np.testing.assert_allclose(T_correction, np.eye(4), atol=1e-10)

    def test_translation_error_correction(self):
        T_known = pose_to_matrix(5.0, 3.0, 0.0, 0, 0, 0)
        T_observed = pose_to_matrix(5.5, 3.2, 0.0, 0, 0, 0)
        T_correction = T_known @ np.linalg.inv(T_observed)
        np.testing.assert_allclose(T_correction[:3, 3], [-0.5, -0.2, 0.0], atol=1e-10)

    def test_correction_distance_threshold(self):
        max_dist = 2.0
        T_known = pose_to_matrix(0, 0, 0, 0, 0, 0)
        T_observed = pose_to_matrix(3.0, 0, 0, 0, 0, 0)
        T_correction = T_known @ np.linalg.inv(T_observed)
        dx, dy, dz = T_correction[0, 3], T_correction[1, 3], T_correction[2, 3]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        assert dist > max_dist

    def test_weighted_average_two_markers(self):
        corrections = [
            (0, pose_to_matrix(0.1, 0.0, 0.0, 0, 0, 0), 1.0),
            (1, pose_to_matrix(0.3, 0.0, 0.0, 0, 0, 0), 1.0),
        ]
        weights = [1.0 / (c[2] + 0.01) for c in corrections]
        w_sum = sum(weights)
        weights = [w / w_sum for w in weights]

        avg_trans = np.zeros(3)
        for (_, T_c, _), w in zip(corrections, weights, strict=False):
            avg_trans += w * T_c[:3, 3]

        assert 0.1 < avg_trans[0] < 0.3
