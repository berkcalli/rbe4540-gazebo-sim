# Merging two point clouds using Python bindings

Use `pcl_python_merlab` to transform both clouds into a common coordinate
frame, concatenate their points, and optionally downsample the result.
These operations use the project's existing C++ PCL bindings.

Concatenation appends coordinates. It does not estimate camera poses, align
surfaces, or remove duplicate points. If both inputs already share a frame
and units, `cloud_a.concatenate(cloud_b)` is sufficient to combine them.


## Python example: merge using a known transform

Save as `merging_point_clouds.py`. The example creates two small observations
of a plane, with one shared point. Coordinates are in meters.

```python
import numpy as np
from pcl_python_merlab import PointCloud

cloud_a = PointCloud(np.array([
    [0.025, 0.025, 0.205],
    [0.075, 0.025, 0.205],
    [0.125, 0.025, 0.205],
], dtype=np.float32))

# T_a_from_b maps B-frame coordinates into frame A:
# p_a = R_a_from_b @ p_b + t_a_from_b (column-vector convention).
T_a_from_b = np.array([
    [0.0, -1.0, 0.0,  0.20],
    [1.0,  0.0, 0.0, -0.10],
    [0.0,  0.0, 1.0,  0.05],
    [0.0,  0.0, 0.0,  1.00],
], dtype=np.float32)

# For this demonstration only, construct B's measurements from known
# A-frame positions. A real sensor would supply cloud_b directly.
b_positions_in_a = np.array([
    [0.125, 0.025, 0.205],
    [0.175, 0.025, 0.205],
    [0.225, 0.025, 0.205],
], dtype=np.float32)
rotation = T_a_from_b[:3, :3]
translation = T_a_from_b[:3, 3]
b_local = (b_positions_in_a - translation) @ rotation
cloud_b = PointCloud(b_local)

# Actual merging workflow: transform, append, then optionally downsample.
b_in_a = cloud_b.transform(T_a_from_b)
merged = cloud_a.concatenate(b_in_a)
downsampled = merged.voxel_grid(leaf_size=0.01)
print("Input sizes:", len(cloud_a), len(cloud_b))
print("Concatenated points:", len(merged))
print("After voxel filtering:", len(downsampled))
print("Merged XYZ in frame A:\n", downsampled.xyz)
downsampled.save_pcd("merged_cloud.pcd", binary=False)
```

Run `python3 merging_point_clouds.py`. Expect 6 concatenated points and 5
after voxel filtering, because the common point shares one voxel. It saves
`merged_cloud.pcd` in the current directory, overwriting an existing file.

The synthetic points are collinear, chosen to make the merge easy to inspect;
they are not suitable for determining a unique 3D registration transform.
The transform is supplied, not estimated.

## C++ to Python mapping

| C++ operation | Binding equivalent |
| --- | --- |
| `pcl::transformPointCloud` | `cloud_b.transform(T_a_from_b)` |
| Point-cloud addition | `cloud_a.concatenate(b_in_a)` |
| VoxelGrid with equal XYZ leaf sizes | `merged.voxel_grid(leaf_size=0.01)` |
| PCD writer | `downsampled.save_pcd(path, binary=False)` |

See [geometry.cpp](rbe4540-sim/pcl_python_merlab/src/geometry.cpp) for
concatenation and [cloud.hpp](rbe4540-sim/pcl_python_merlab/src/cloud.hpp) for
transforms and voxel filtering. Each method returns a new cloud. Before
downsampling, the result contains all A points followed by all transformed B
points, so its length is `len(cloud_a) + len(cloud_b)`.

Voxel filtering replaces each occupied cell's points by their centroid. It
can combine distinct nearby samples as well as duplicates, and does not
correct calibration errors. Choose a leaf size consistent with the detail
you need. Do not reuse original point indices after this step.

## Merge two PCD files

The following reusable function uses the same workflow with real files.
Supply your measured B-to-A transform; an identity matrix is appropriate only
when both input files already use the same frame.

```python
from pcl_python_merlab import load_pcd


def merge_pcd_files(path_a, path_b, T_a_from_b, output_path, leaf_size=0.01):
    cloud_a = load_pcd(str(path_a))
    cloud_b = load_pcd(str(path_b))
    merged = cloud_a.concatenate(cloud_b.transform(T_a_from_b))
    if len(merged) == 0:
        raise ValueError("Both inputs are empty after loading")
    if leaf_size is not None:
        merged = merged.voxel_grid(leaf_size=leaf_size)
    merged.save_pcd(str(output_path), binary=False)
    return merged
```

Call `merge_pcd_files("a.pcd", "b.pcd", T_a_from_b, "merged.pcd")`
after defining your transform. Use `leaf_size=None` to keep every point.
Loaders retain finite XYZ coordinates only; colors, normals, organization,
and metadata are not preserved. Output paths overwrite existing files, so
use a destination distinct from both inputs.

## If the transform is unknown

Obtain camera calibration or use registration on sufficiently overlapping,
nondegenerate geometry. See the [ICP example](README_iterative_closest_point_python.md).
For `result = cloud_b.icp(cloud_a, ...)`, the source is B and the target is A;
`result["transformation"]` therefore maps B to A. After checking convergence,
fitness, and geometric alignment, merge with
`cloud_a.concatenate(result["cloud"])`. That cloud is already transformed;
do not apply the returned matrix to it again.

ICP is a local method and may need an initial pose estimate. Appending two
misaligned clouds simply leaves misaligned surfaces in the output.

## Two cameras in this ROS workspace

The [point-cloud environment](README_for_point_cloud_processing.md) provides
`/camera/points` and `/camera2/points`. The existing
[starter](rbe4540-sim/ur_move_merlab/ur_move_merlab/pc_processing_template.py)
processes one camera; a two-camera node needs a subscription for each topic.
Pair measurements from similar times, convert each message to a cloud, and
use TF at each message's timestamp to transform both to `base_link` or another
chosen common frame before concatenating them. Use the actual message
`frame_id`; do not infer a transform from the topic name.

The example matrix above is synthetic and is not camera calibration for the
simulator. The binding carries no frame IDs, units, or timestamps, so preserve
those separately and assign the common frame when publishing the result.
Time differences can produce duplicate-looking moving objects even with
correct calibration. For normal estimation on merged multi-view data, see
the [normal-estimation guide](README_normal_estimation_python.md).
