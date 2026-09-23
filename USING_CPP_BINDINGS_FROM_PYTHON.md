# Using C++ point-cloud functions from Python

This guide explains how to use this project's `pcl_python_merlab` module.
It provides some example of how to use Python bindings for C++ algorithms in the Point Cloud Library (PCL). 


## 2. Understand what crosses the Python/C++ boundary

```python
import numpy as np
from pcl_python_merlab import PointCloud

xyz = np.array([
    [0.10, 0.00, 0.20],
    [0.11, 0.00, 0.20],
    [0.30, 0.10, 0.25],
], dtype=np.float32)

cloud = PointCloud(xyz)
downsampled = cloud.voxel_grid(leaf_size=0.02)
print("Input points:", len(cloud))
print("Filtered XYZ:\n", downsampled.xyz)

# directly editing the points in the cloud

edited_xyz = cloud.xyz
edited_xyz[:, 2] += 0.10
moved_cloud = PointCloud(edited_xyz)

