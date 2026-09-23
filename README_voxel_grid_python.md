# Voxel-grid downsampling using Python bindings

Adapted from the [PCL VoxelGrid tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html#voxelgrid).
The example loads a scan, downsamples it with 0.01-unit voxels, and saves it.
Filtering runs in C++ PCL through `pcl_python_merlab`.


## Python example

Save as `voxel_grid.py`:

```python
from pcl_python_merlab import load_pcd

cloud = load_pcd("table_scene_lms400.pcd")
print("Input points:", len(cloud))
filtered = cloud.voxel_grid(leaf_size=0.01)
print("Output points:", len(filtered))
if len(filtered) == 0:
    raise RuntimeError("No points to save; check the input scan")
filtered.save_pcd("table_scene_lms400_downsampled.pcd", binary=False)
```

Run `python3 voxel_grid.py` in the sourced terminal.

## C++ to Python mapping

| C++ operation | Binding call |
| --- | --- |
| PCD reader | `load_pcd(path)` |
| VoxelGrid input and equal XYZ leaf sizes | `cloud.voxel_grid(leaf_size=0.01)` |
| PCD writer with ASCII output | `filtered.save_pcd(path, binary=False)` |

The implementation in [cloud.hpp](rbe4540-sim/pcl_python_merlab/src/cloud.hpp)
uses `pcl::VoxelGrid<PointXYZ>`. The tutorial uses `PCLPointCloud2` and preserves
additional fields; this binding produces XYZ-only output. Each occupied voxel
contributes its points' centroid, which is not necessarily the voxel center.
The binding accepts one positive leaf size for all three axes.

For coordinates in meters, `0.01` is 1 cm. A larger leaf size generally yields
fewer points and less detail. Inspect the printed counts and output file;
counts may differ with input cleaning and library version.

## Binding conventions

Processing returns new clouds. `cloud.xyz` returns an independent NumPy copy;
editing it does not change the C++ cloud. Input arrays must have finite XYZ
coordinates and shape `(N, 3)`. Distances use the input coordinate units.
The bindings retain XYZ only, discarding color, other fields, and metadata.
PCD loaders remove nonfinite points; saves require nonempty clouds and overwrite
existing destinations. Use a separate output directory when keeping old results.

See the [API reference](rbe4540-sim/pcl_python_merlab/API.md) and
[bindings guide](USING_CPP_BINDINGS_FROM_PYTHON.md) for further details.
