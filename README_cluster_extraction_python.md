# Euclidean cluster extraction using Python bindings

Adapted from the [PCL cluster extraction tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html#cluster-extraction).
Load a scene, downsample, repeatedly remove planes, then save each remaining
Euclidean cluster. All processing uses the local C++ PCL bindings.


## Python example

Save as `cluster_extraction.py`:

```python
from pcl_python_merlab import load_pcd

cloud = load_pcd("table_scene_lms400.pcd")
print("Input points:", len(cloud))
remaining = cloud.voxel_grid(leaf_size=0.01)
initial_count = len(remaining)
print("Downsampled points:", initial_count)

while len(remaining) > 0.3 * initial_count:
    inliers, coefficients = remaining.segment_plane(
        distance_threshold=0.02, max_iterations=100)
    if not inliers:
        print("No further plane found")
        break
    plane = remaining.extract(inliers)
    print("Removing plane points:", len(plane))
    remaining = remaining.extract(inliers, negative=True)

clusters = remaining.euclidean_clusters(
    tolerance=0.02, min_size=100, max_size=25000)
print("Clusters:", len(clusters))
for number, indices in enumerate(clusters):
    cluster = remaining.extract(indices)
    path = f"cloud_cluster_{number:04d}.pcd"
    cluster.save_pcd(path, binary=False)
    print(path, len(cluster))
```

Run `python3 cluster_extraction.py` in the directory containing the dataset.

## C++ to Python mapping

| C++ component | Binding equivalent |
| --- | --- |
| VoxelGrid with equal leaf sizes | `voxel_grid(leaf_size=0.01)` |
| Optimized plane RANSAC | `segment_plane(...)` |
| ExtractIndices | `extract(inliers, negative=True)` removes the plane |
| EuclideanClusterExtraction settings | `tolerance`, `min_size`, `max_size` |
| KdTree search | Created internally by `euclidean_clusters` |
| Vector of point-index groups | Python list of index lists |

These methods are implemented in [cloud.hpp](rbe4540-sim/pcl_python_merlab/src/cloud.hpp).
The binding constructs the search tree, so no Python `KdTree` is needed here.

## Interpret the result

The script prints shrinking point counts and writes one nonempty XYZ PCD per
accepted cluster. Exact counts depend on the data and PCL version; a cluster
number is not a persistent object identity. No files are written if no
clusters meet the size limits.

The 30% condition uses the initial **downsampled** count, not the raw input
count. This follows the tutorial's plane-removal heuristic; it is not a test
that all remaining points belong to objects. On other scenes it may remove
object surfaces too. Validate plane orientation and height when identifying
a table in the robot workspace.

Cluster indices refer to `remaining` after plane removal. Extract from that
same cloud without filtering or reordering it. For meter-valued coordinates,
`tolerance=0.02` connects nearby points at a 2 cm scale. Tune the tolerance
and size limits for your sensor density; too small a tolerance splits objects,
while too large a tolerance can connect separate objects.

## Binding conventions

Processing returns new clouds. `cloud.xyz` returns an independent NumPy copy;
editing it does not change the C++ cloud. Input arrays must have finite XYZ
coordinates and shape `(N, 3)`. Distances use the input coordinate units.
The bindings retain XYZ only, discarding color, other fields, and metadata.
PCD loaders remove nonfinite points; saves require nonempty clouds and overwrite
existing destinations. Use a separate output directory when keeping old results.

See the [API reference](rbe4540-sim/pcl_python_merlab/API.md) and
[bindings guide](USING_CPP_BINDINGS_FROM_PYTHON.md) for further details.
