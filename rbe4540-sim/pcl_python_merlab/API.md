# Python API reference

```python
from pcl_python_merlab import PointCloud, KdTree, load_pcd, load_ply
```

All algorithms execute in PCL. This reference describes the course binding's
Python signatures, not PCL's C++ class interfaces.

## Conventions

- `PointCloud(xyz)` accepts a finite `(N,3)` array and copies it to float32 XYZ.
  `PointCloud()` creates an empty cloud. `len(cloud)` is its point count.
- `cloud.xyz` returns a new `(N,3)` float32 NumPy array. Editing it does not
  modify the original cloud; construct a new `PointCloud` from edited data.
- Methods do not modify their input clouds. Indices refer to the cloud on
  which the method ran. Recompute indices after filtering or reordering.
- Distances are meters when input XYZ is in meters. Angles are radians.
  Neighbor searches and registration fitness return **squared distances**.
- Invalid parameters raise Python exceptions. Most filters/searches accept
  empty clouds. Measurements, registration, hulls, VFH, and saves need points;
  detailed minimum sizes are noted below.
- Long computations run synchronously. Downsample before expensive methods and
  avoid running them on every camera frame in a ROS callback.

## Filters

| Method | PCL implementation / return |
| --- | --- |
| `voxel_grid(leaf_size)` | `VoxelGrid`: centroid of each occupied voxel; new cloud |
| `approximate_voxel_grid(leaf_size)` | `ApproximateVoxelGrid`: faster approximate filter; new cloud. Best with spatially ordered input; unordered input may retain more points. |
| `crop_box(minimum, maximum)` | `CropBox`: XYZ lower/upper vectors, inclusive axis-aligned bounds; new cloud |
| `pass_through(axis, minimum, maximum, negative=False)` | `PassThrough`: `axis` is `'x'`, `'y'`, or `'z'`; `negative=True` retains the complement |
| `statistical_outlier_removal(mean_k=20, stddev_multiplier=1.0, negative=False)` | `StatisticalOutlierRemoval`: remove points whose mean neighbor distance is too large; `1 <= mean_k < N`; negative selects outliers |
| `radius_outlier_removal(radius, min_neighbors=3, negative=False)` | `RadiusOutlierRemoval`: retain points with enough other neighbors inside radius; negative selects outliers |
| `random_sample(count, seed=0)` | `RandomSample`: select `0 <= count <= N` input points; reproducible with the same input/seed |
| `uniform_sampling(radius)` | `UniformSampling`: select an original point near each occupied voxel center; radius is PCL's sampling grid parameter |
| `project_plane(coefficients)` | `ProjectInliers`: project XYZ onto `[a,b,c,d]`, where `ax+by+cz+d=0` |
| `extract(indices, negative=False)` | `ExtractIndices`: retain selected points, or remove them when negative; indices must be unique and valid |

Sizes/radii and statistical multipliers must be finite and positive. Bounds
must be ordered and finite. All operations above return `PointCloud` objects.

## Geometry

| Method | Return |
| --- | --- |
| `transform(matrix)` | New cloud transformed by a finite homogeneous `(4,4)` matrix; PCL `transformPointCloud` |
| `concatenate(other)` | New cloud containing this cloud followed by `other` |
| `centroid()` | `(3,)` center from `compute3DCentroid`; at least one point |
| `covariance()` | `(3,3)` covariance normalized by **N**, from `computeCovarianceMatrixNormalized`; at least one point |
| `bounds()` | `(minimum, maximum)` XYZ vectors from `getMinMax3D`; at least one point |
| `pca()` | Dictionary with `centroid`, `eigenvalues` (descending), `eigenvectors` (axes as columns); PCL `PCA`, at least three points |
| `oriented_bounding_box()` | Dictionary with `minimum`, `maximum` in box coordinates, `position`, and `rotation`; PCL `MomentOfInertiaEstimation`, at least three points |

PCL PCA eigenvalues use sample covariance (**N−1** normalization), unlike
`covariance()`. Eigenvector signs and axes for repeated eigenvalues are
ambiguous. The oriented box follows the principal axes; it is not guaranteed
to be the minimum-volume box. Convert a box-coordinate row vector to cloud
coordinates with `point @ rotation.T + position`.

## Neighbor search

Construct `tree = KdTree(cloud)` once and reuse it. The tree retains its cloud,
so the original Python cloud variable can go out of scope.

| Method | Return |
| --- | --- |
| `tree.nearest_k(query, k=1)` | `(indices, squared_distances)` as lists, sorted nearest first; caps k at cloud size |
| `tree.radius_search(query, radius, max_neighbors=0)` | Same format; `0` means no count limit |

`query` is a finite XYZ vector. Both methods return `([], [])` for an empty
cloud. A query coinciding with an input point includes that point at distance 0.

## Model fitting and segmentation

```python
inliers, coefficients = cloud.segment_plane(distance_threshold, max_iterations=100)
inliers, coefficients = cloud.segment_model(
    model, distance_threshold, max_iterations=1000, probability=0.99,
    min_radius=0.0, max_radius=3.402823466e38)
inliers, coefficients = cloud.segment_cylinder(
    normals, distance_threshold, min_radius, max_radius,
    max_iterations=1000, normal_distance_weight=0.1)
```

These use `SACSegmentation` or `SACSegmentationFromNormals`, RANSAC, and
coefficient optimization. `segment_plane` preserves the original starter API.
`segment_model` supports the strings below. Returned indices and coefficients
are Python lists; an unsuccessful fit gives empty lists. Probability must be
strictly between 0 and 1. Radius limits apply to radius-bearing models.

| Model | Coefficients |
| --- | --- |
| `plane` | `[a,b,c,d]`, `ax+by+cz+d=0` |
| `line` | `[px,py,pz,dx,dy,dz]`, point and direction |
| `sphere` | `[cx,cy,cz,r]` |
| `circle2d` | `[cx,cy,r]`, circle in XY coordinates |
| `circle3d` | `[cx,cy,cz,r,nx,ny,nz]` |
| Cylinder method | `[px,py,pz,dx,dy,dz,r]`, axis point, axis direction, radius |

Cylinder normals must correspond to the input cloud and be finite/nonzero.
The normal-distance weight is in `[0,1]`; it blends normal disagreement and
geometric distance according to PCL's cylinder model. Fit thresholds and bounds
must be tuned to your data. Validate a fitted plane's orientation and height
before treating it as the table.

```python
clusters = cloud.euclidean_clusters(tolerance, min_size=30, max_size=25000)
clusters = cloud.region_growing(
    normals, neighbors=30, smoothness_threshold=0.05235988,
    curvature_threshold=1.0, min_size=30, max_size=25000)
```

Both return lists of index lists, suitable for `cloud.extract(cluster)`.
Euclidean clustering uses a PCL KdTree. Region growing uses neighbor searches,
normal smoothness and curvature; pass `(N,4)` normals including curvature.
Its default angle is 3 degrees expressed in radians. Empty input gives `[]`.
Require `1 <= min_size <= max_size`.

## Normals and descriptors

| Method | PCL implementation / return |
| --- | --- |
| `estimate_normals(radius, viewpoint=[0,0,0])` | `NormalEstimation`; `(N,3)` normal array, original API |
| `normals_with_curvature(radius, viewpoint=[0,0,0])` | `NormalEstimation`; `(N,4)` array `[nx,ny,nz,curvature]` |
| `fpfh(normals, radius)` | `FPFHEstimation`; `(N,33)` feature histograms |
| `vfh(normals, viewpoint=[0,0,0])` | `VFHEstimation`; one `(308,)` histogram for the cloud; at least two points |
| `principal_curvatures(normals, radius)` | `PrincipalCurvaturesEstimation`; `(N,5)` array `[direction_x,direction_y,direction_z,pc1,pc2]` |

Viewpoints are in cloud coordinates; normal estimation flips normals toward
the viewpoint. Use the camera position expressed in the same frame as the
cloud. Neighborhoods with insufficient support can produce NaN normals or
descriptors. Descriptor/cylinder inputs accept `(N,3)` or `(N,4)` normals and
normalize finite nonzero normal vectors. Region growing requires the fourth
curvature column. Remove invalid points and their normal rows together before
passing normals into another algorithm:

```python
normals = cloud.normals_with_curvature(radius=0.02, viewpoint=camera_position)
valid = np.isfinite(normals).all(axis=1)
cloud = cloud.extract(np.flatnonzero(valid).tolist())
normals = normals[valid]
features = cloud.fpfh(normals, radius=0.04)
```

Choose a descriptor radius larger than the normal-estimation radius. This
example assumes `numpy as np` and a known `camera_position` in cloud coordinates.
Normal curvature is PCL's local surface-variation estimate, not a radius.

## Registration

```python
result = source.icp(
    target, initial_guess=np.eye(4), max_iterations=50,
    max_correspondence_distance=0.05, transformation_epsilon=1e-8)
result = source.generalized_icp(
    target, initial_guess=np.eye(4), max_iterations=50,
    max_correspondence_distance=0.05, transformation_epsilon=1e-8,
    correspondence_randomness=20)
result = source.ndt(
    target, resolution, step_size=0.1, initial_guess=np.eye(4),
    max_iterations=50, transformation_epsilon=1e-6)
```

These call `IterativeClosestPoint`, `GeneralizedIterativeClosestPoint`, and
`NormalDistributionsTransform`. Each returns a dictionary:

- `cloud`: aligned source PointCloud.
- `transformation`: final `(4,4)` **source-to-target** transform, including the
  initial guess (do not apply that guess again).
- `converged`: PCL's convergence flag.
- `fitness`: PCL's mean squared nearest-target distance score.

These are local registration methods and benefit from a good initial guess.
Check convergence and alignment quality before using a result for motion.
ICP needs at least three points per cloud. GICP also needs at least
`correspondence_randomness` points in each cloud, with that parameter at least
three. NDT requires sufficient target samples per occupied voxel; at least six
target points overall is only an input check, not a guarantee of convergence.
Resolution controls target voxel size. Transformation epsilon has the semantics
of each PCL algorithm; it is not a shared physical-distance threshold.

## Surface processing

| Method | Return |
| --- | --- |
| `moving_least_squares(radius, polynomial_order=2)` | Smoothed XYZ PointCloud from `MovingLeastSquares`; order 1–5, no normal output |
| `convex_hull(dimension=3)` | `(vertices, polygons)` from `ConvexHull` |
| `concave_hull(alpha, dimension=3)` | `(vertices, polygons)` from `ConcaveHull` using alpha shapes |

Hull vertices are a PointCloud; polygons are lists of indices into those
vertices. Dimension must be 2 or 3 with at least `dimension+1` input points.
Use dimension 2 for planar data. Degenerate geometry may produce an empty hull.
The alpha parameter controls concave-hull detail. MLS may change the point
count when neighborhoods lack support; don't reuse old indices afterward.

## File I/O

```python
cloud = load_pcd('scan.pcd')
cloud = load_ply('scan.ply')
cloud.save_pcd('filtered.pcd', binary=True)
cloud.save_ply('filtered.ply', binary=True)
```

Paths are strings. Files must contain scalar float32 x/y/z fields. Loaders
remove nonfinite XYZ points and discard other fields, organization and metadata.
PLY loading returns points, not polygon connectivity. Saves write nonempty XYZ
clouds; `binary=False` selects ASCII. Existing destination files are overwritten.
Read/write failures raise exceptions. These functions use PCL I/O; no additional
Python file libraries are required.

`pcl_python_merlab.pcl_version` reports the linked PCL version, and `__version__`
reports the course binding version.
