#!/usr/bin/env python3
import os
import sys

import numpy as np

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from ground_segmentation import pointcloud2_to_xyz, xyz_to_pointcloud2  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _make_pointcloud2(points: np.ndarray) -> PointCloud2:
    """Build a minimal PointCloud2 from an (N, 3) float32 array."""
    msg = PointCloud2()
    msg.header = Header()
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


class TestPointCloud2ToXyz:
    def test_basic_roundtrip(self):
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
        msg = _make_pointcloud2(pts)
        result = pointcloud2_to_xyz(msg)
        np.testing.assert_allclose(result, pts)

    def test_empty_cloud(self):
        msg = PointCloud2()
        msg.fields = []
        msg.point_step = 12
        msg.data = b''
        result = pointcloud2_to_xyz(msg)
        assert result.shape == (0, 3)

    def test_single_point(self):
        pts = np.array([[7.5, -3.2, 0.1]], dtype=np.float32)
        msg = _make_pointcloud2(pts)
        result = pointcloud2_to_xyz(msg)
        np.testing.assert_allclose(result, pts, atol=1e-6)

    def test_missing_fields_returns_empty(self):
        msg = PointCloud2()
        msg.fields = []
        msg.point_step = 12
        msg.data = b''
        result = pointcloud2_to_xyz(msg)
        assert result.shape == (0, 3)


class TestXyzToPointCloud2:
    def test_header_preserved(self):
        header = Header()
        header.frame_id = 'test_frame'
        pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        msg = xyz_to_pointcloud2(pts, header)
        assert msg.header.frame_id == 'test_frame'
        assert msg.width == 1
        assert msg.height == 1
        assert msg.point_step == 12
        assert msg.is_dense is True

    def test_field_layout(self):
        pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        msg = xyz_to_pointcloud2(pts, Header())
        names = [f.name for f in msg.fields]
        assert names == ['x', 'y', 'z']
        offsets = [f.offset for f in msg.fields]
        assert offsets == [0, 4, 8]

    def test_roundtrip(self):
        pts = np.random.randn(100, 3).astype(np.float32)
        msg = xyz_to_pointcloud2(pts, Header())
        recovered = pointcloud2_to_xyz(msg)
        np.testing.assert_allclose(recovered, pts)


class TestSegmentationLogic:
    """Test the height-based classification logic directly."""

    def test_ground_vs_obstacle_split(self):
        ground_min, ground_max, obstacle_max = -2.0, -0.15, 3.0

        pts = np.array([
            [0.0, 0.0, -1.0],   # ground
            [1.0, 0.0, 0.5],    # obstacle
            [2.0, 0.0, -0.15],  # ground (boundary)
            [3.0, 0.0, -0.14],  # obstacle (just above boundary)
            [4.0, 0.0, 5.0],    # above obstacle_max — neither
        ], dtype=np.float32)

        z = pts[:, 2]
        ground_mask = (z >= ground_min) & (z <= ground_max)
        obstacle_mask = (z > ground_max) & (z <= obstacle_max)

        assert ground_mask.tolist() == [True, False, True, False, False]
        assert obstacle_mask.tolist() == [False, True, False, True, False]

    def test_nan_filtering(self):
        pts = np.array([
            [1.0, 2.0, 3.0],
            [float('nan'), 0.0, 0.0],
            [0.0, float('nan'), 0.0],
            [0.0, 0.0, float('nan')],
        ], dtype=np.float32)

        valid = np.isfinite(pts).all(axis=1)
        assert valid.tolist() == [True, False, False, False]

    def test_all_below_ground_min(self):
        ground_min, ground_max = -2.0, -0.15
        pts = np.array([[0.0, 0.0, -5.0]], dtype=np.float32)
        z = pts[:, 2]
        ground_mask = (z >= ground_min) & (z <= ground_max)
        assert not ground_mask.any()
