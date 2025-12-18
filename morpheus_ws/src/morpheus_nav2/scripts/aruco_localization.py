#!/usr/bin/env python3
"""ArUco-based localization correction.

Subscribes to /aruco_markers (detected markers in camera frame), looks up each
marker's known world pose from a YAML map, and publishes a geometry_msgs/PoseWithCovarianceStamped
correction on /aruco_pose that robot_localization's EKF can fuse — or, when
running without EKF integration, broadcasts a corrected map→odom transform.

The node operates in two modes controlled by the 'mode' parameter:
  - "ekf"  : publish PoseWithCovarianceStamped for EKF fusion (default)
  - "tf"   : directly broadcast a corrected map→odom static transform
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
)
from ros2_aruco_interfaces.msg import ArucoMarkers

import tf2_ros
import tf_transformations


def pose_to_matrix(x, y, z, roll, pitch, yaw):
    T = tf_transformations.euler_matrix(roll, pitch, yaw)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def pose_msg_to_matrix(pose):
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    T = tf_transformations.quaternion_matrix(q)
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T


class ArucoLocalization(Node):
    def __init__(self):
        super().__init__('aruco_localization')

        self.declare_parameter('mode', 'ekf')
        self.declare_parameter('correction_weight', 0.3)
        self.declare_parameter('max_correction_distance', 2.0)
        self.declare_parameter('max_correction_angle', 0.5)
        self.declare_parameter('marker_map', rclpy.Parameter.Type.STRING_ARRAY)

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.correction_weight = self.get_parameter('correction_weight').get_parameter_value().double_value
        self.max_dist = self.get_parameter('max_correction_distance').get_parameter_value().double_value
        self.max_angle = self.get_parameter('max_correction_angle').get_parameter_value().double_value

        self.marker_poses = self._load_marker_map()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.mode == 'tf':
            self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/aruco_pose', 10)

        self.create_subscription(
            ArucoMarkers, '/aruco_markers',
            self.markers_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'ArUco localization started — mode={self.mode}, '
            f'{len(self.marker_poses)} markers loaded')

    def _load_marker_map(self):
        markers = {}
        for marker_id_str in ['0', '1', '2', '3', '4', '5', '6']:
            prefix = f'marker_map.{marker_id_str}'
            try:
                self.declare_parameter(f'{prefix}.x', 0.0)
                self.declare_parameter(f'{prefix}.y', 0.0)
                self.declare_parameter(f'{prefix}.z', 0.0)
                self.declare_parameter(f'{prefix}.roll', 0.0)
                self.declare_parameter(f'{prefix}.pitch', 0.0)
                self.declare_parameter(f'{prefix}.yaw', 0.0)

                x = self.get_parameter(f'{prefix}.x').get_parameter_value().double_value
                y = self.get_parameter(f'{prefix}.y').get_parameter_value().double_value
                z = self.get_parameter(f'{prefix}.z').get_parameter_value().double_value
                roll = self.get_parameter(f'{prefix}.roll').get_parameter_value().double_value
                pitch = self.get_parameter(f'{prefix}.pitch').get_parameter_value().double_value
                yaw = self.get_parameter(f'{prefix}.yaw').get_parameter_value().double_value

                markers[int(marker_id_str)] = pose_to_matrix(x, y, z, roll, pitch, yaw)
                self.get_logger().info(
                    f'Marker {marker_id_str}: [{x:.1f}, {y:.1f}, {z:.1f}]')
            except rclpy.exceptions.ParameterAlreadyDeclaredException:
                pass
        return markers

    def markers_callback(self, msg: ArucoMarkers):
        if not msg.marker_ids:
            return

        camera_frame = msg.header.frame_id
        stamp = msg.header.stamp

        try:
            tf_cam_to_odom = self.tf_buffer.lookup_transform(
                'odom', camera_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            tf_odom_to_map = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        T_odom_cam = self._transform_to_matrix(tf_cam_to_odom.transform)
        T_map_odom = self._transform_to_matrix(tf_odom_to_map.transform)

        corrections = []

        for i, marker_id in enumerate(msg.marker_ids):
            mid = int(marker_id)
            if mid not in self.marker_poses:
                continue

            T_cam_marker = pose_msg_to_matrix(msg.poses[i])
            T_map_marker_observed = T_map_odom @ T_odom_cam @ T_cam_marker
            T_map_marker_known = self.marker_poses[mid]

            T_correction = T_map_marker_known @ np.linalg.inv(T_map_marker_observed)

            dx = T_correction[0, 3]
            dy = T_correction[1, 3]
            dz = T_correction[2, 3]
            dist = np.sqrt(dx*dx + dy*dy + dz*dz)

            _, _, dyaw = tf_transformations.euler_from_matrix(T_correction)

            if dist > self.max_dist or abs(dyaw) > self.max_angle:
                self.get_logger().debug(
                    f'Marker {mid} correction too large '
                    f'(dist={dist:.2f}, yaw={dyaw:.2f}), skipping')
                continue

            corrections.append((mid, T_correction, dist))

        if not corrections:
            return

        if len(corrections) == 1:
            _, T_corr, _ = corrections[0]
        else:
            weights = [1.0 / (c[2] + 0.01) for c in corrections]
            w_sum = sum(weights)
            weights = [w / w_sum for w in weights]

            avg_trans = np.zeros(3)
            avg_quat = np.zeros(4)
            for (_, T_c, _), w in zip(corrections, weights):
                avg_trans += w * T_c[:3, 3]
                q = tf_transformations.quaternion_from_matrix(T_c)
                if avg_quat @ q < 0:
                    q = -q
                avg_quat += w * q
            avg_quat /= np.linalg.norm(avg_quat)

            T_corr = tf_transformations.quaternion_matrix(avg_quat)
            T_corr[:3, 3] = avg_trans

        w = self.correction_weight
        blended_trans = w * T_corr[:3, 3]
        q_corr = tf_transformations.quaternion_from_matrix(T_corr)
        q_identity = np.array([0.0, 0.0, 0.0, 1.0])
        blended_quat = tf_transformations.quaternion_slerp(q_identity, q_corr, w)

        T_new_map_odom = tf_transformations.quaternion_matrix(blended_quat)
        T_new_map_odom[:3, 3] = blended_trans
        T_new_map_odom = T_new_map_odom @ T_map_odom

        if self.mode == 'tf':
            self._publish_tf_correction(T_new_map_odom, stamp)
        else:
            self._publish_ekf_correction(T_new_map_odom, stamp)

    def _publish_ekf_correction(self, T_map_odom, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        q = tf_transformations.quaternion_from_matrix(T_map_odom)
        msg.pose.pose.position.x = T_map_odom[0, 3]
        msg.pose.pose.position.y = T_map_odom[1, 3]
        msg.pose.pose.position.z = T_map_odom[2, 3]
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        cov = 0.05
        msg.pose.covariance[0] = cov
        msg.pose.covariance[7] = cov
        msg.pose.covariance[14] = 999.0
        msg.pose.covariance[21] = 999.0
        msg.pose.covariance[28] = 999.0
        msg.pose.covariance[35] = cov * 2.0

        self.pose_pub.publish(msg)
        self.get_logger().debug('Published ArUco EKF correction')

    def _publish_tf_correction(self, T_map_odom, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        q = tf_transformations.quaternion_from_matrix(T_map_odom)
        t.transform.translation.x = T_map_odom[0, 3]
        t.transform.translation.y = T_map_odom[1, 3]
        t.transform.translation.z = T_map_odom[2, 3]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug('Published ArUco TF correction')

    @staticmethod
    def _transform_to_matrix(transform):
        q = [transform.rotation.x, transform.rotation.y,
             transform.rotation.z, transform.rotation.w]
        T = tf_transformations.quaternion_matrix(q)
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z
        return T


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
