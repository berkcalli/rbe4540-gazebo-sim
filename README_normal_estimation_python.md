# Surface normal estimation using Python bindings

Adapted from the [PCL normal-estimation tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/normal_estimation.html#normal-estimation).
This example supplies a synthetic plane to the tutorial's radius-based normal
estimator through `pcl_python_merlab`. C++ PCL computes the normals; Python
prepares the coordinates and inspects the results.

## Python example

Save as `normal_estimation.py`:

```python
import numpy as np
from pcl_python_merlab import PointCloud

# A 21-by-21 plane at z = 0.2 m, sampled every centimeter.
x, y = np.meshgrid(np.linspace(-0.1, 0.1, 21),
                   np.linspace(-0.1, 0.1, 21))
xyz = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.2)))
cloud = PointCloud(xyz.astype(np.float32))

# Orient normals toward a point above the plane, in the same frame.
viewpoint = [0.0, 0.0, 1.0]
normals = cloud.estimate_normals(radius=0.03, viewpoint=viewpoint)
print("Points:", len(cloud))
print("Normal array shape:", normals.shape)

# Some real scans contain points without adequate neighborhood support.
valid = np.isfinite(normals).all(axis=1)
valid &= np.linalg.norm(normals, axis=1) > 1e-8
usable_cloud = cloud.extract(np.flatnonzero(valid).tolist())
usable_normals = normals[valid]
if len(usable_cloud) == 0:
    raise RuntimeError("No usable normals; check point spacing and radius")
print("Usable normals:", len(usable_normals))
print("First five normals:\n", usable_normals[:5])

# An alternative binding returns curvature alongside each normal.
# This is a second estimation call, not a conversion of the previous array.
with_curvature = cloud.normals_with_curvature(
    radius=0.03, viewpoint=viewpoint)
print("Normal/curvature array shape:", with_curvature.shape)
print("First five curvature values:", with_curvature[:5, 3])

# Store matching XYZ and normal rows together. PCD saves are XYZ-only here.
np.savez("plane_normals.npz", xyz=usable_cloud.xyz, normals=usable_normals)
```

Run `python3 normal_estimation.py` in the sourced terminal. It writes
`plane_normals.npz` in the current directory, replacing an existing file.

## C++ to Python mapping

| Tutorial operation | Local binding equivalent |
| --- | --- |
| Set the input cloud | Call the method on `cloud` |
| Construct and set a search KdTree | Created internally by the binding |
| `setRadiusSearch(0.03)` | `radius=0.03` |
| `setViewPoint(...)` | `viewpoint=[x, y, z]` |
| Compute normal output | `estimate_normals(...)` returns `(N, 3)` |
| Read normal components and curvature | `normals_with_curvature(...)` returns `(N, 4)` |

Implementations are in [cloud.hpp](rbe4540-sim/pcl_python_merlab/src/cloud.hpp)
and [features.cpp](rbe4540-sim/pcl_python_merlab/src/features.cpp).
The fourth column is curvature, not a homogeneous coordinate or plane offset.
The local API exposes radius search, with no `k` argument, separate search
surface, or OpenMP estimator selection.

## Interpret the result

Expect 441 points, a `(441, 3)` normal array, and a `(441, 4)` array with
curvature. For this plane and viewpoint, normals should be approximately
`[0, 0, 1]`, with curvature near zero. Small numerical differences are normal.

PCL fits local neighborhoods and orients normals toward the viewpoint.
Its curvature estimate is the smallest covariance eigenvalue divided by the
sum of the eigenvalues; it is dimensionless, not a radius of curvature.
Too small a neighborhood can lack support; too large a neighborhood can blend
surfaces across edges. The `0.03` radius is 3 cm for meter-valued coordinates.

## Use a real cloud

Replace the synthetic input with `load_pcd("scan.pcd")`, importing `load_pcd`
from `pcl_python_merlab`. That loader discards nonfinite XYZ points and other
fields. For NumPy input, remove nonfinite coordinate rows before constructing
`PointCloud`. Tune the normal radius to your scan density.

The viewpoint defaults to `[0, 0, 0]`. Supply the camera position in the
cloud's coordinate frame; transforming a camera cloud does not automatically
update the viewpoint. A single viewpoint does not generally provide consistent
orientation for a scene assembled from multiple cameras.

Normal row `i` corresponds to input point `i`. Apply identical selections to
points and normals, as above. Recompute normals after voxel downsampling or
other operations that change the point geometry. If applying a rigid transform
to existing normals separately, use only its rotation, never its translation.

The cloud stores XYZ only. `save_pcd` will not include the normal array;
the example uses NumPy's NPZ format to preserve the paired arrays.

See the [API reference](rbe4540-sim/pcl_python_merlab/API.md),
[cylinder example](README_cylinder_segmentation_python.md), and
[merging guide](README_merging_point_clouds_python.md).
