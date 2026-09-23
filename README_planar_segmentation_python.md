# Planar segmentation using Python bindings to C++ PCL

This example adapts the [PCL plane segmentation tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/planar_segmentation.html#planar-segmentation)
to this project's `pcl_python_merlab` Python bindings. It creates 15 points on
`z = 1`, moves three off the plane, fits a plane with RANSAC, and prints the
coefficients and inlier points. NumPy prepares the input; the existing C++ PCL
implementation performs segmentation.


## Python example

Save the following as `planar_segmentation.py` in your workspace directory:

```python
import sys

import numpy as np
from pcl_python_merlab import PointCloud


def main():
    rng = np.random.default_rng(42)
    xyz = np.ones((15, 3), dtype=np.float32)
    xyz[:, :2] = rng.uniform(0.0, 1024.0, size=(15, 2))
    xyz[[0, 3, 6], 2] = [2.0, -2.0, 4.0]
    cloud = PointCloud(xyz)

    print(f"Input ({len(cloud)} points):")
    for point in xyz:
        print(" ", *point)

    inliers, coefficients = cloud.segment_plane(
        distance_threshold=0.01,
        max_iterations=50,
    )
    if not inliers:
        print("Plane fitting failed.", file=sys.stderr)
        return 1

    print("Plane [a, b, c, d]:", coefficients)
    print(f"Inlier count: {len(inliers)}")
    for index in inliers:
        print(f"  {index}:", *xyz[index])

    # Optional: use the returned indices to split the original cloud.
    plane_cloud = cloud.extract(inliers)
    outlier_cloud = cloud.extract(inliers, negative=True)
    print(f"Split: {len(plane_cloud)} plane points, "
          f"{len(outlier_cloud)} outlier points")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

Run it from the directory where you saved it:

```bash
python3 planar_segmentation.py
```

## How the binding replaces the C++ setup

The local implementation of `segment_plane` in
[cloud.hpp](rbe4540-sim/pcl_python_merlab/src/cloud.hpp) already constructs
`pcl::SACSegmentation<pcl::PointXYZ>` and configures it. The Python call uses
the following mapping:

| C++ operation | Python binding equivalent |
| --- | --- |
| Create and fill a `pcl::PointCloud<pcl::PointXYZ>` | `PointCloud(xyz)` copies an `(N, 3)` array to the C++ cloud |
| `setOptimizeCoefficients(true)` | Enabled inside `segment_plane` |
| `setModelType(pcl::SACMODEL_PLANE)` | Selected inside `segment_plane` |
| `setMethodType(pcl::SAC_RANSAC)` | Selected inside `segment_plane` |
| `setDistanceThreshold(0.01)` | `distance_threshold=0.01` |
| `setMaxIterations(50)` | `max_iterations=50` |
| `setInputCloud(cloud)` | The method acts on `cloud` |
| `segment(*inliers, *coefficients)` | Returns `(inliers, coefficients)` as Python lists |

The method's Python signature is registered in
[bindings.cpp](rbe4540-sim/pcl_python_merlab/src/bindings.cpp).
The local binding defaults to 100 iterations; this example explicitly uses
50, matching the iteration limit reported in the tutorial's sample output.
NumPy's seeded generator makes the input repeatable, but produces different
coordinates from C++ `rand()`.

## Interpret the result

Expect 12 inliers, with indices `1, 2, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14`.
The coefficients describe `a*x + b*y + c*z + d = 0`; for `z = 1`, they should
be approximately `[0, 0, 1, -1]`. Reversing all four signs describes the same
plane. Small floating-point differences are normal.

The last output line should be:

```text
Split: 12 plane points, 3 outlier points
```

The threshold uses the cloud's coordinate units. `PointCloud` stores no unit
or coordinate-frame metadata. When using camera data in meters, `0.01` means
one centimeter. The synthetic coordinates above reproduce the tutorial's
data-generation scale rather than the scale of a tabletop camera scene.

`extract` returns a new cloud without changing the original. Its indices must
refer to that original cloud; filtering or reordering before extraction can
make them invalid. The constructor copies input coordinates, so change `xyz`
before constructing `PointCloud`, or construct a new cloud after editing it.

For additional methods and input requirements, see the
[binding API reference](rbe4540-sim/pcl_python_merlab/API.md) and
[Python bindings guide](USING_CPP_BINDINGS_FROM_PYTHON.md).
