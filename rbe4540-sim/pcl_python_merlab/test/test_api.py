"""Numerical and error-contract tests using the real compiled PCL module."""
import numpy as np
import pytest
from pcl_python_merlab import PointCloud, KdTree, load_pcd, load_ply


@pytest.fixture
def plane():
    x, y = np.meshgrid(np.linspace(-0.1, 0.1, 11), np.linspace(-0.1, 0.1, 11))
    return PointCloud(np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.2))))


def test_filters_remove_isolated_point_and_preserve_input(plane):
    cloud = plane.concatenate(PointCloud([[4, 4, 4]]))
    for result in [cloud.pass_through('z', 0.1, 0.3),
                   cloud.statistical_outlier_removal(10, 1),
                   cloud.radius_outlier_removal(0.04, 2)]:
        assert len(result) == len(plane)
        assert np.max(result.xyz[:, 2]) < 0.3
    assert len(cloud.pass_through('z', 0.1, 0.3, negative=True)) == 1
    assert len(cloud.radius_outlier_removal(0.04, 2, negative=True)) == 1
    assert len(cloud) == 122


def test_sampling_and_projection(plane):
    first = plane.random_sample(15, seed=123).xyz
    np.testing.assert_array_equal(first, plane.random_sample(15, seed=123).xyz)
    assert len(first) == 15
    for output in [plane.uniform_sampling(0.05), plane.approximate_voxel_grid(0.05)]:
        assert 0 < len(output) < len(plane)
        np.testing.assert_allclose(output.xyz[:, 2], 0.2, atol=1e-6)
    np.testing.assert_allclose(plane.project_plane([0, 0, 2, 0]).xyz[:, 2], 0, atol=1e-7)
    assert len(plane.random_sample(0)) == 0


def test_geometry_pca_and_bounding_box():
    rng = np.random.default_rng(18)
    xyz = rng.normal(size=(400, 3)) * [0.4, 0.15, 0.03] + [1, 2, 3]
    cloud = PointCloud(xyz)
    np.testing.assert_allclose(cloud.centroid(), xyz.mean(axis=0), atol=2e-6)
    np.testing.assert_allclose(cloud.covariance(), np.cov(xyz.T, bias=True), atol=2e-6)
    low, high = cloud.bounds()
    np.testing.assert_allclose(low, xyz.min(axis=0), atol=1e-6)
    np.testing.assert_allclose(high, xyz.max(axis=0), atol=1e-6)
    pca = cloud.pca()
    assert np.all(np.diff(pca['eigenvalues']) <= 0)
    np.testing.assert_allclose(pca['eigenvectors'].T @ pca['eigenvectors'], np.eye(3), atol=1e-6)
    np.testing.assert_allclose(np.sort(pca['eigenvalues']), np.linalg.eigvalsh(np.cov(xyz.T)), atol=2e-6)
    box = cloud.oriented_bounding_box()
    local = (xyz - box['position']) @ box['rotation']
    assert np.all(local >= box['minimum'] - 1e-5)
    assert np.all(local <= box['maximum'] + 1e-5)


def test_tree_distances_and_lifetime():
    tree = KdTree(PointCloud([[0, 0, 0], [0.1, 0, 0], [0.3, 0, 0]]))
    indices, distances = tree.nearest_k([0, 0, 0], k=9)
    assert indices == [0, 1, 2]
    np.testing.assert_allclose(distances, [0, 0.01, 0.09], atol=1e-7)
    assert tree.radius_search([0, 0, 0], 0.15)[0] == [0, 1]
    assert tree.radius_search([0, 0, 0], 0.15, max_neighbors=1)[0] == [0]
    assert KdTree(PointCloud()).nearest_k([0, 0, 0]) == ([], [])


@pytest.mark.parametrize('model', ['line', 'sphere', 'circle2d', 'circle3d', 'plane'])
def test_ransac_models(model):
    rng = np.random.default_rng(17)
    t = np.linspace(0, 2*np.pi, 100, endpoint=False)
    if model == 'line':
        xyz = np.column_stack((t, t*2, t*0))
    elif model == 'sphere':
        xyz = rng.normal(size=(100, 3))
        xyz /= np.linalg.norm(xyz, axis=1)[:, None]
    elif model in ('circle2d', 'circle3d'):
        xyz = np.column_stack((np.cos(t), np.sin(t), np.zeros_like(t)))
    else:
        xyz = rng.uniform(-1, 1, (100, 3)); xyz[:, 2] = 0.2
    indices, coeff = PointCloud(xyz).segment_model(model, 0.001)
    assert len(indices) == 100
    assert np.isfinite(coeff).all()
    if model == 'sphere':
        np.testing.assert_allclose(coeff[:3], [0, 0, 0], atol=1e-4)
        assert abs(coeff[3]-1) < 1e-4


def test_cylinder_segmentation():
    t, z = np.meshgrid(np.linspace(0, 2*np.pi, 40, endpoint=False), np.linspace(-0.2, 0.2, 8))
    xyz = np.column_stack((0.05*np.cos(t.ravel()), 0.05*np.sin(t.ravel()), z.ravel()))
    normals = np.column_stack((np.cos(t.ravel()), np.sin(t.ravel()), np.zeros(t.size)))
    indices, coeff = PointCloud(xyz).segment_cylinder(normals, 0.001, 0.04, 0.06)
    assert len(indices) == len(xyz)
    assert abs(coeff[6]-0.05) < 1e-4
    assert abs(coeff[5]) > 0.99


def test_normals_curvature_features_and_regions(plane):
    normals = plane.normals_with_curvature(0.06, viewpoint=[0, 0, 1])
    assert normals.shape == (121, 4)
    np.testing.assert_allclose(normals[:, :3], np.tile([0, 0, 1], (121, 1)), atol=1e-4)
    np.testing.assert_allclose(normals[:, 3], 0, atol=1e-4)
    features = plane.fpfh(normals, 0.09)
    assert features.shape == (121, 33)
    assert np.isfinite(features).all()
    np.testing.assert_allclose(features.sum(axis=1), 300, atol=1e-3)
    vfh = plane.vfh(normals, viewpoint=[0, 0, 1])
    assert vfh.shape == (308,) and np.isfinite(vfh).all() and vfh.sum() > 0
    curves = plane.principal_curvatures(normals, 0.06)
    assert curves.shape == (121, 5)
    np.testing.assert_allclose(curves[:, 3:], 0, atol=1e-4)
    clusters = plane.region_growing(normals, neighbors=10, min_size=10)
    assert len(clusters) == 1 and set(clusters[0]) == set(range(121))


@pytest.mark.parametrize('method', ['icp', 'generalized_icp'])
def test_registration_recovers_small_translation(method):
    xyz = np.random.default_rng(42).uniform(-0.15, 0.15, (200, 3))
    source = PointCloud(xyz)
    translation = np.array([0.003, -0.002, 0.001])
    result = getattr(source, method)(PointCloud(xyz + translation))
    assert result['converged']
    np.testing.assert_allclose(result['transformation'][:3, 3], translation, atol=2e-4)
    np.testing.assert_allclose(result['cloud'].xyz, xyz+translation, atol=2e-4)
    assert result['fitness'] < 1e-7


def test_ndt_identity_on_dense_cloud():
    xyz = np.random.default_rng(55).normal(0, 0.1, (2000, 3))
    cloud = PointCloud(xyz)
    result = cloud.ndt(cloud, resolution=0.1)
    assert result['converged']
    assert np.isfinite(result['transformation']).all()
    assert result['fitness'] < 1e-4


def test_surface_methods(plane):
    smoothed = plane.moving_least_squares(0.06)
    assert len(smoothed) > 0
    np.testing.assert_allclose(smoothed.xyz[:, 2], 0.2, atol=1e-5)
    for vertices, faces in [plane.convex_hull(dimension=2), plane.concave_hull(0.04, dimension=2)]:
        assert len(vertices) >= 4 and faces
        assert all(0 <= i < len(vertices) for face in faces for i in face)
        np.testing.assert_allclose(vertices.xyz[:, 2], 0.2, atol=1e-6)
    tetra = PointCloud([[0,0,0],[1,0,0],[0,1,0],[0,0,1]])
    vertices, faces = tetra.convex_hull()
    assert len(vertices) == 4 and len(faces) == 4


@pytest.mark.parametrize('extension,loader', [('pcd', load_pcd), ('ply', load_ply)])
@pytest.mark.parametrize('binary', [True, False])
def test_io_roundtrip(tmp_path, plane, extension, loader, binary):
    path = str(tmp_path / ('cloud.' + extension))
    getattr(plane, 'save_' + extension)(path, binary=binary)
    np.testing.assert_allclose(loader(path).xyz, plane.xyz, atol=1e-6)
    with pytest.raises(RuntimeError):
        loader(str(tmp_path / 'missing'))


@pytest.mark.parametrize('operation', [
    lambda c: c.pass_through('rgb', 0, 1),
    lambda c: c.statistical_outlier_removal(10000),
    lambda c: c.radius_outlier_removal(-1),
    lambda c: c.random_sample(-1),
    lambda c: c.uniform_sampling(0),
    lambda c: c.project_plane([0, 0, 0, 0]),
    lambda c: c.segment_model('unknown', 0.01),
    lambda c: c.segment_model('sphere', 0.01, probability=1),
    lambda c: c.fpfh(np.zeros((len(c), 3)), 0.05),
    lambda c: c.region_growing(np.ones((len(c), 3))),
    lambda c: c.moving_least_squares(0.1, 0),
    lambda c: c.convex_hull(4),
    lambda c: c.icp(PointCloud()),
    lambda c: c.extract([0, 0], negative=True),
    lambda c: c.generalized_icp(c, correspondence_randomness=10000),
])
def test_invalid_inputs_raise_python_exceptions(plane, operation):
    with pytest.raises(ValueError):
        operation(plane)


def test_empty_contracts():
    c = PointCloud()
    assert len(c.pass_through('x', 0, 1)) == 0
    assert len(c.radius_outlier_removal(0.1)) == 0
    assert len(c.statistical_outlier_removal()) == 0
    assert len(c.moving_least_squares(0.1)) == 0
    assert c.normals_with_curvature(0.1).shape == (0, 4)
    assert c.fpfh(np.zeros((0, 3)), 0.1).shape == (0, 33)
    with pytest.raises(ValueError):
        c.centroid()
