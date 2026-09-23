"""Check ROS cloud conversion and the PCL geometry used by the starter."""

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from pcl_python_merlab import PointCloud

from ur_move_merlab.pc_processing_template import (
    cloud_from_msg, transform_cloud,
)


def test_padded_cloud_filters_nan_and_infinity_and_round_trips():
    fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 16)]
    ]
    msg = point_cloud2.create_cloud(Header(frame_id='camera'), fields, [
        (1.0, 2.0, 3.0, 0.0), (float('nan'), 0.0, 1.0, 0.0),
        (1.0, float('inf'), 0.0, 0.0), (-1.0, 0.0, 0.5, 0.0),
    ])
    xyz = cloud_from_msg(msg).xyz
    np.testing.assert_allclose(xyz, [[1.0, 2.0, 3.0], [-1.0, 0.0, 0.5]])
    round_trip = point_cloud2.create_cloud_xyz32(msg.header, xyz.astype(np.float32))
    np.testing.assert_allclose(cloud_from_msg(round_trip).xyz, xyz)


def test_transform_normalizes_quaternion_and_preserves_input():
    transform = TransformStamped()
    transform.transform.translation.x = 1.0
    # Scaled quaternion for a 90-degree rotation about Z.
    transform.transform.rotation.z = 2**0.5
    transform.transform.rotation.w = 2**0.5
    xyz = np.array([[1.0, 2.0, 3.0]])
    np.testing.assert_allclose(transform_cloud(PointCloud(xyz), transform).xyz, [[-1.0, 1.0, 3.0]])
    np.testing.assert_array_equal(xyz, [[1.0, 2.0, 3.0]])


def test_voxel_centroids_and_negative_coordinates():
    xyz = np.array([[-0.9, 0, 0], [-0.1, 0, 0], [0.1, 0, 0],
                    [0.3, 0, 0], [1.0, 0, 0]], dtype=float)
    np.testing.assert_allclose(
        PointCloud(xyz).voxel_grid(1.0).xyz, [[-0.5, 0, 0], [0.2, 0, 0], [1.0, 0, 0]],
    )


def test_empty_cloud_preserves_xyz_shape():
    empty = cloud_from_msg(point_cloud2.create_cloud_xyz32(Header(), []))
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    assert empty.xyz.shape == (0, 3)
    assert empty.voxel_grid(0.005).xyz.shape == (0, 3)
    assert transform_cloud(empty, transform).xyz.shape == (0, 3)


@pytest.mark.parametrize('size', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_voxel_size(size):
    with pytest.raises(ValueError):
        PointCloud(np.zeros((1, 3))).voxel_grid(size)


def test_pcl_plane_segmentation_and_extraction():
    x, y = np.meshgrid(np.linspace(-0.2, 0.2, 10), np.linspace(-0.2, 0.2, 10))
    plane = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.05)))
    objects = np.array([[0.0, 0.0, 0.2], [0.1, 0.1, 0.3]])
    cloud = PointCloud(np.vstack((plane, objects)))
    inliers, coefficients = cloud.segment_plane(0.001, 200)
    assert len(inliers) == 100
    normal = np.asarray(coefficients[:3])
    assert abs(normal[2]) > 0.99
    assert abs(np.asarray(coefficients) @ [0.0, 0.0, 0.05, 1.0]) < 0.001
    np.testing.assert_allclose(cloud.extract(inliers, negative=True).xyz, objects)


def test_pcl_crop_and_cluster_indices():
    first = np.array([[0, 0, 0], [0.01, 0, 0], [0.02, 0, 0]])
    second = first + [0.3, 0.3, 0.3]
    cloud = PointCloud(np.vstack((first, second, [[3, 3, 3]])))
    cropped = cloud.crop_box([-0.1, -0.1, -0.1], [1, 1, 1])
    clusters = cropped.euclidean_clusters(0.03, min_size=3, max_size=10)
    assert {frozenset(c) for c in clusters} == {frozenset([0, 1, 2]), frozenset([3, 4, 5])}
    assert len(cloud) == 7
    assert len(cropped) == 6
    with pytest.raises(IndexError):
        cropped.extract([6])


def test_pcl_normals_on_flat_surface():
    x, y = np.meshgrid(np.linspace(-0.05, 0.05, 7), np.linspace(-0.05, 0.05, 7))
    cloud = PointCloud(np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))))
    normals = cloud.estimate_normals(0.06, viewpoint=[0, 0, 1])
    assert normals.shape == (49, 3)
    np.testing.assert_allclose(normals, np.tile([0, 0, 1], (49, 1)), atol=1e-5)


def test_pcl_empty_algorithms_and_input_validation():
    empty = PointCloud()
    assert empty.segment_plane(0.01) == ([], [])
    assert empty.euclidean_clusters(0.01) == []
    assert empty.estimate_normals(0.01).shape == (0, 3)
    assert len(empty.extract([], negative=True)) == 0
    with pytest.raises(ValueError):
        PointCloud(np.array([[float('nan'), 0, 0]]))
    with pytest.raises(ValueError):
        PointCloud(np.zeros((3, 2)))
