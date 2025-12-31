#!/usr/bin/env python3
"""Height-based ground segmentation with slope detection for 3D LiDAR point clouds.

Ground points are split into flat ground and steep slopes. Steep slope points
are republished as obstacles so the Nav2 costmap avoids them.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
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


def detect_steep_ground(pts: np.ndarray, cell_size: float, slope_threshold: float) -> np.ndarray:
    """Return boolean mask marking ground points that lie on steep slopes.

    Uses a grid-based z-gradient: if neighbouring cells differ in height by
    more than slope_threshold (meters) per cell_size (meters), the region is
    considered a slope and those points are marked True (obstacle).

    slope_threshold / cell_size = tan(angle), e.g. 0.10/0.25 = 0.4 → ~22°.
    """
    if len(pts) < 10:
        return np.zeros(len(pts), dtype=bool)

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    x_min, y_min = x.min(), y.min()

    xi = ((x - x_min) / cell_size).astype(np.int32)
    yi = ((y - y_min) / cell_size).astype(np.int32)
    nx, ny = int(xi.max()) + 1, int(yi.max()) + 1

    # Mean z per cell via scatter
    z_sum = np.zeros((nx, ny), dtype=np.float64)
    z_cnt = np.zeros((nx, ny), dtype=np.int32)
    np.add.at(z_sum, (xi, yi), z)
    np.add.at(z_cnt, (xi, yi), 1)
    with np.errstate(invalid='ignore'):
        z_mean = np.where(z_cnt > 0, z_sum / z_cnt, np.nan)

    # Mark cells where z-gradient to any axis-aligned neighbour exceeds threshold
    steep = np.zeros((nx, ny), dtype=bool)
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        sx, ex = max(dx, 0), nx + min(dx, 0)
        sy, ey = max(dy, 0), ny + min(dy, 0)
        nx2, ex2 = max(-dx, 0), nx + max(dx, 0)  # shifted source slice
        ny2, ey2 = max(-dy, 0), ny + max(dy, 0)

        a = z_mean[sx:ex, sy:ey]
        b = z_mean[nx2:ex2, ny2:ey2]
        diff = np.abs(a - b)
        steep_patch = (diff > slope_threshold) & ~np.isnan(a) & ~np.isnan(b)
        steep[sx:ex, sy:ey] |= steep_patch

    return steep[xi, yi]


class GroundSegmentation(Node):
    def __init__(self):
        super().__init__('ground_segmentation')

        self.declare_parameter('ground_height_min', -2.0)
        self.declare_parameter('ground_height_max', -0.15)
        self.declare_parameter('obstacle_height_max', 3.0)
        self.declare_parameter('slope_cell_size', 0.25)
        self.declare_parameter('slope_threshold', 0.15)   # ~31° at 0.25 m cell
        self.declare_parameter('input_topic', '/scan/points')
        self.declare_parameter('obstacle_topic', '/scan/points/obstacles')
        self.declare_parameter('ground_topic', '/scan/points/ground')

        self.ground_min = self.get_parameter('ground_height_min').value
        self.ground_max = self.get_parameter('ground_height_max').value
        self.obstacle_max = self.get_parameter('obstacle_height_max').value
        self.cell_size = self.get_parameter('slope_cell_size').value
        self.slope_thr = self.get_parameter('slope_threshold').value
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
            f'Ground segmentation: z∈[{self.ground_min}, {self.ground_max}]=ground, '
            f'z>{self.ground_max}=obstacle; slope>{self.slope_thr}m/{self.cell_size}m cell=obstacle')

    def _cloud_cb(self, msg: PointCloud2):
        pts = pointcloud2_to_xyz(msg)
        if len(pts) == 0:
            return

        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        z = pts[:, 2]

        ground_mask = (z >= self.ground_min) & (z <= self.ground_max)
        obstacle_mask = (z > self.ground_max) & (z <= self.obstacle_max)

        # detect steep ground and add to obstacles
        if ground_mask.any():
            ground_pts = pts[ground_mask]
            steep = detect_steep_ground(ground_pts, self.cell_size, self.slope_thr)
            flat_pts = ground_pts[~steep]
            slope_pts = ground_pts[steep]

            # merge steep-slope points with regular obstacles
            obs_pts = pts[obstacle_mask]
            all_obstacles = np.concatenate([obs_pts, slope_pts]) if len(slope_pts) else obs_pts

            if len(all_obstacles):
                self.obstacle_pub.publish(xyz_to_pointcloud2(all_obstacles, msg.header))
            if len(flat_pts):
                self.ground_pub.publish(xyz_to_pointcloud2(flat_pts, msg.header))
        else:
            if obstacle_mask.any():
                self.obstacle_pub.publish(xyz_to_pointcloud2(pts[obstacle_mask], msg.header))


def main():
    rclpy.init()
    node = GroundSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
