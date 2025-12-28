#!/usr/bin/env python3
"""Simple height-based ground segmentation for 3D LiDAR point clouds.

Subscribes to a PointCloud2 topic, splits points into ground and obstacle
clouds based on a height threshold in the sensor frame, and republishes
each on separate topics for costmap consumption.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N, 3) float32 xyz array from a PointCloud2 message."""
    x_off = y_off = z_off = None
    for f in msg.fields:
        if f.name == 'x':
            x_off = f.offset
        elif f.name == 'y':
            y_off = f.offset
        elif f.name == 'z':
            z_off = f.offset
    if x_off is None:
        return np.empty((0, 3), dtype=np.float32)

    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = np.ndarray(len(data), dtype=np.float32, buffer=data, offset=x_off, strides=(msg.point_step,))
    y = np.ndarray(len(data), dtype=np.float32, buffer=data, offset=y_off, strides=(msg.point_step,))
    z = np.ndarray(len(data), dtype=np.float32, buffer=data, offset=z_off, strides=(msg.point_step,))
    return np.column_stack((x, y, z))


def xyz_to_pointcloud2(points: np.ndarray, header) -> PointCloud2:
    """Build a minimal xyz-only PointCloud2 from an (N, 3) float32 array."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = points.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


class GroundSegmentation(Node):
    def __init__(self):
        super().__init__('ground_segmentation')

        self.declare_parameter('ground_height_min', -2.0)
        self.declare_parameter('ground_height_max', -0.15)
        self.declare_parameter('obstacle_height_max', 3.0)
        self.declare_parameter('input_topic', '/scan/points')
        self.declare_parameter('obstacle_topic', '/scan/points/obstacles')
        self.declare_parameter('ground_topic', '/scan/points/ground')

        self.ground_min = self.get_parameter('ground_height_min').value
        self.ground_max = self.get_parameter('ground_height_max').value
        self.obstacle_max = self.get_parameter('obstacle_height_max').value
        input_topic = self.get_parameter('input_topic').value

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, qos)
        self.obstacle_pub = self.create_publisher(
            PointCloud2, self.get_parameter('obstacle_topic').value, qos)
        self.ground_pub = self.create_publisher(
            PointCloud2, self.get_parameter('ground_topic').value, qos)

        self.get_logger().info(
            f'Ground segmentation: z in [{self.ground_min}, {self.ground_max}] = ground, '
            f'({self.ground_max}, {self.obstacle_max}] = obstacle')

    def _cloud_cb(self, msg: PointCloud2):
        pts = pointcloud2_to_xyz(msg)
        if len(pts) == 0:
            return

        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]

        z = pts[:, 2]
        ground_mask = (z >= self.ground_min) & (z <= self.ground_max)
        obstacle_mask = (z > self.ground_max) & (z <= self.obstacle_max)

        if obstacle_mask.any():
            self.obstacle_pub.publish(
                xyz_to_pointcloud2(pts[obstacle_mask], msg.header))

        if ground_mask.any():
            self.ground_pub.publish(
                xyz_to_pointcloud2(pts[ground_mask], msg.header))


def main():
    rclpy.init()
    node = GroundSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
