# Cylinder segmentation using Python bindings

Adapted from the [PCL cylinder segmentation tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/cylinder_segmentation.html#cylinder-segmentation).
The example crops depth, estimates normals, removes a plane, and fits a cylinder
using `pcl_python_merlab`.

## Build and run environment

For ROS 2 Jazzy and a workspace at `~/rbe4540`:

```bash
sudo apt update
sudo apt install -y libpcl-dev pybind11-dev python3-dev python3-numpy
source /opt/ros/jazzy/setup.bash
cd ~/rbe4540
colcon build --symlink-install --packages-select pcl_python_merlab
source install/setup.bash
```

Source both setup files in each new terminal and use Python compatible with
the compiled module. No simulator or running ROS node is needed.

## Download the input

Save the [tutorial dataset](https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd)
in the directory where you will run the script:

```bash
curl -fL https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd -o table_scene_mug_stereo_textured.pcd
```

## Differences required by the available bindings

The tutorial uses 50-neighbor normal estimation and a normal-aware plane model.
The local API exposes radius-based normals and ordinary plane RANSAC instead.
This version uses `radius=0.02` and `segment_plane`; consequently it is an
adaptation of the workflow, not a numerically identical translation. Tune the
normal radius for your point density. Cylinder fitting still uses normals,
RANSAC, and coefficient optimization in C++ PCL.

## Python example

Save as `cylinder_segmentation.py`:

```python
import numpy as np
from pcl_python_merlab import load_pcd

cloud = load_pcd("table_scene_mug_stereo_textured.pcd")
print("Loaded finite XYZ points:", len(cloud))
cloud = cloud.pass_through(axis="z", minimum=0.0, maximum=1.5)
if len(cloud) < 3:
    raise RuntimeError("Too few points after depth filtering")

normals = cloud.estimate_normals(radius=0.02, viewpoint=[0.0, 0.0, 0.0])
valid = np.isfinite(normals).all(axis=1)
valid &= np.linalg.norm(normals, axis=1) > 1e-8
# Apply the same selection to points and normals to preserve row alignment.
cloud = cloud.extract(np.flatnonzero(valid).tolist())
normals = normals[valid]
print("Points with usable normals:", len(cloud))

plane_indices, plane_coefficients = cloud.segment_plane(
    distance_threshold=0.03, max_iterations=100)
if not plane_indices:
    raise RuntimeError("No plane found; inspect the input and threshold")
plane = cloud.extract(plane_indices)
plane.save_pcd("table_scene_mug_stereo_textured_plane.pcd", binary=False)
print("Plane coefficients:", plane_coefficients)
print("Plane points:", len(plane))

keep = np.ones(len(cloud), dtype=bool)
keep[plane_indices] = False
remaining = cloud.extract(np.flatnonzero(keep).tolist())
remaining_normals = normals[keep]
if len(remaining) < 3:
    raise RuntimeError("Too few points remain for cylinder fitting")

indices, coefficients = remaining.segment_cylinder(
    normals=remaining_normals,
    distance_threshold=0.05,
    min_radius=0.0,
    max_radius=0.1,
    max_iterations=10000,
    normal_distance_weight=0.1,
)
if not indices:
    raise RuntimeError("No cylinder found; inspect normals and fit parameters")
cylinder = remaining.extract(indices)
cylinder.save_pcd("table_scene_mug_stereo_textured_cylinder.pcd", binary=False)
print("Cylinder [px, py, pz, dx, dy, dz, radius]:", coefficients)
print("Cylinder points:", len(cylinder))
```

Run `python3 cylinder_segmentation.py` in the dataset directory. Normal
estimation and fitting can take time; the calls execute synchronously.

## C++ to Python mapping

| C++ operation | Binding equivalent |
| --- | --- |
| PassThrough on z | `pass_through("z", 0.0, 1.5)` |
| NormalEstimation with `setKSearch(50)` | Adapted to `estimate_normals(radius=0.02)` |
| SACMODEL_NORMAL_PLANE | Adapted to `segment_plane(...)` without normal weighting |
| ExtractIndices for points and normals | `extract(...)` plus the identical NumPy row selection |
| SACMODEL_CYLINDER with input normals | `segment_cylinder(normals=..., ...)` |
| Radius bounds and normal weighting | `min_radius`, `max_radius`, `normal_distance_weight` |

Cylinder and normal handling are implemented in
[features.cpp](rbe4540-sim/pcl_python_merlab/src/features.cpp) and
[cloud.hpp](rbe4540-sim/pcl_python_merlab/src/cloud.hpp).

## Interpret the result

Successful execution creates separate plane and cylinder files. Cylinder
coefficients contain a point on the axis, its direction, and the radius.
The axis point is not necessarily the object's center and its direction may
have either sign. Normal weighting blends geometric distance and normal
agreement; the cylinder threshold is not purely a point-to-surface cutoff.

Keep each normal row paired with its point whenever selecting points. The
origin viewpoint assumes the dataset's camera is at the coordinate origin;
for transformed camera clouds, supply the camera position in that frame.
The loader removes nonfinite coordinates before the script prints its input
count, so it can differ from the tutorial's raw count. Invalid normals are
also removed here. Do not expect the tutorial's exact segmentation or radius.

## Binding conventions

Processing returns new clouds. `cloud.xyz` returns an independent NumPy copy;
editing it does not change the C++ cloud. Input arrays must have finite XYZ
coordinates and shape `(N, 3)`. Distances use the input coordinate units.
The bindings retain XYZ only, discarding color, other fields, and metadata.
PCD loaders remove nonfinite points; saves require nonempty clouds and overwrite
existing destinations. Use a separate output directory when keeping old results.

See the [API reference](rbe4540-sim/pcl_python_merlab/API.md) and
[bindings guide](USING_CPP_BINDINGS_FROM_PYTHON.md) for further details.
