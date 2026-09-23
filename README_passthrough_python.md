# PassThrough filtering using Python bindings

Adapted from the [PCL PassThrough tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough).
This example prints five points, retains those in a z interval, and also shows
the complementary selection using the C++ PCL filter.

## Python example

Save as `passthrough.py`:

```python
import numpy as np
from pcl_python_merlab import PointCloud

rng = np.random.default_rng(42)
xyz = rng.uniform(-1.0, 1.0, size=(5, 3)).astype(np.float32)
# Deliberately include points below, within, and above the interval.
xyz[:, 2] = [-0.5, 0.25, 0.75, 1.5, -0.25]
cloud = PointCloud(xyz)
print("Input:\n", cloud.xyz)

filtered = cloud.pass_through(axis="z", minimum=0.0, maximum=1.0)
print("Kept:\n", filtered.xyz)
rejected = cloud.pass_through(
    axis="z", minimum=0.0, maximum=1.0, negative=True)
print("Complement:\n", rejected.xyz)
print(f"Counts: {len(cloud)} input, {len(filtered)} kept, "
      f"{len(rejected)} rejected")
```

Run `python3 passthrough.py`. No dataset download is needed.

## C++ to Python mapping

| C++ setting | Python argument |
| --- | --- |
| `setInputCloud` | Call the method on `cloud` |
| `setFilterFieldName` | `axis="z"` |
| `setFilterLimits` | `minimum=0.0, maximum=1.0` |
| `setNegative(true)` | `negative=True` |
| Filter output | Returned `PointCloud` |

The implementation is in [filters.cpp](rbe4540-sim/pcl_python_merlab/src/filters.cpp).
Only `x`, `y`, and `z` are supported; limits must be finite and ordered.
The normal selection includes the interval endpoints.

## Expected result and adaptation

Expect 2 kept points with z values `0.25` and `0.75`, and 3 rejected points.
The original source generates all coordinates in `[0, 1024)`, which can leave
no points in `[0, 1]` for a five-point input. This version deliberately assigns
z values across the interval to make both selections visible. It uses NumPy's
seeded generator for x and y, so coordinates differ from C++ `rand()`.

## Binding conventions

Processing returns new clouds. `cloud.xyz` returns an independent NumPy copy;
editing it does not change the C++ cloud. Input arrays must have finite XYZ
coordinates and shape `(N, 3)`. Distances use the input coordinate units.
The bindings retain XYZ only, discarding color, other fields, and metadata.
PCD loaders remove nonfinite points; saves require nonempty clouds and overwrite
existing destinations. Use a separate output directory when keeping old results.

See the [API reference](rbe4540-sim/pcl_python_merlab/API.md) and
[bindings guide](USING_CPP_BINDINGS_FROM_PYTHON.md) for further details.
