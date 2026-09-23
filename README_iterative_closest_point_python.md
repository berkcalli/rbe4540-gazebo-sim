# Iterative closest point using Python bindings

Adapted from the [PCL ICP tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/iterative_closest_point.html#iterative-closest-point).
Create five points, translate a copy by 0.7 along x, and estimate the transform
from the original to the copy using C++ PCL.


## Python example

Save as `iterative_closest_point.py`:

```python
import numpy as np
from pcl_python_merlab import PointCloud

rng = np.random.default_rng(42)
xyz = rng.uniform(0.0, 1024.0, size=(5, 3)).astype(np.float32)
source = PointCloud(xyz)
target_xyz = xyz.copy()
target_xyz[:, 0] += 0.7
target = PointCloud(target_xyz)
print("Source:\n", source.xyz)
print("Target:\n", target.xyz)

result = source.icp(
    target,
    max_iterations=50,
    max_correspondence_distance=1.0,
    transformation_epsilon=1e-8,
)
print("Converged:", result["converged"])
print("Fitness (squared coordinate units):", result["fitness"])
print("Source-to-target transform:\n", result["transformation"])
if not result["converged"]:
    raise RuntimeError("ICP did not converge")
print("Aligned source:\n", result["cloud"].xyz)
print("Largest coordinate error:",
      np.max(np.abs(result["cloud"].xyz - target.xyz)))
```

Run `python3 iterative_closest_point.py`. No input file is required.

## C++ to Python mapping

| C++ operation | Binding equivalent |
| --- | --- |
| Set source and target, then align | `source.icp(target, ...)` |
| `hasConverged()` | `result["converged"]` |
| `getFitnessScore()` | `result["fitness"]` |
| `getFinalTransformation()` | `result["transformation"]` |
| Aligned output cloud | `result["cloud"]` |

The implementation is in [registration.cpp](rbe4540-sim/pcl_python_merlab/src/registration.cpp).
The binding's default correspondence limit is `0.05`, smaller than the 0.7
translation. This example explicitly uses `1.0`. Each cloud needs at least
three points. The default initial guess is identity; an optional
`initial_guess` must be a finite homogeneous `(4, 4)` matrix.

## Interpret the result

Expect a matrix close to identity with translation `[0.7, 0, 0]`, convergence,
and a fitness near zero. NumPy generates different points from C++ `rand()`.
Float32 arithmetic at the deliberately large coordinate scale introduces
small errors, so do not expect exact equality.

The returned transformation maps source coordinates into target coordinates
and already includes any initial guess. The returned cloud is already aligned;
do not transform it a second time. Fitness is a mean squared nearest-target
distance. A convergence flag alone does not prove the alignment is correct;
inspect the score and geometry. ICP is a local method and can require a good
initial guess for real scans with larger rotations or partial overlap.

## Binding conventions

Processing returns new clouds. `cloud.xyz` returns an independent NumPy copy;
editing it does not change the C++ cloud. Input arrays must have finite XYZ
coordinates and shape `(N, 3)`. Distances use the input coordinate units.
The bindings retain XYZ only, discarding color, other fields, and metadata.
PCD loaders remove nonfinite points; saves require nonempty clouds and overwrite
existing destinations. Use a separate output directory when keeping old results.

See the [API reference](rbe4540-sim/pcl_python_merlab/API.md) and
[bindings guide](USING_CPP_BINDINGS_FROM_PYTHON.md) for further details.
