# Major and minor principal axes of a point cloud

Use the C++ PCL bindings to get two unit direction vectors in the cloud's
coordinate frame:

- **Major axis:** direction of greatest spread.
- **Minor axis:** direction of least spread (the third principal axis in 3D).

If needed, follow the [binding setup instructions](USING_CPP_BINDINGS_FROM_PYTHON.md)
first, then source your workspace's `install/setup.bash`.

## Python example

```python
from pcl_python_merlab import load_pcd

cloud = load_pcd("object.pcd")
if len(cloud) < 3:
    raise ValueError("PCA requires at least three points")

axes = cloud.pca()["eigenvectors"]
major_axis = axes[:, 0]
minor_axis = axes[:, 2]

print("Major axis [x, y, z]:", major_axis)
print("Minor axis [x, y, z]:", minor_axis)
```

Run `python3 point_cloud_pca.py`. If you already have a `PointCloud` named
`cloud`, start at the `axes = ...` line.

Both outputs are NumPy vectors with shape `(3,)`. The binding orders axes
from greatest to least variance and stores them as **columns**. The middle
axis is `axes[:, 1]`; for a flat object, this is the shorter in-plane axis,
while `axes[:, 2]` is perpendicular to the plane.

For a cloud elongated along x and thinnest along z, example output is:

```text
Major axis [x, y, z]: [1. 0. 0.]
Minor axis [x, y, z]: [0. 0. 1.]
```

Either vector may have the opposite sign and still describe the same axis.
These are directions, not positions or object dimensions. Isolate the desired
object before PCA; symmetric or collinear clouds can have ambiguous axes.

See the [API reference](rbe4540-sim/pcl_python_merlab/API.md) for more details.
