## I assume that you can already run the `simple_pick_place_demo`. Start by installing the prerequisites.

sudo apt update
sudo apt install -y libpcl-dev pybind11-dev python3-dev python3-numpy ros-jazzy-sensor-msgs-py ros-jazzy-tf2-ros-py

## 2. Build in your workspace directory
colcon build --symlink-install
source install/setup.bash

## 3. Run the point-cloud environment.
ros2 launch ur_move_merlab pc_processing_env.launch.py

## 4. Run the sample code below. This is just a template for your to build on
ros2 run ur_move_merlab pc_processing_template

## 5. You can run rviz to visualize your point clouds.
ros2 run ros2 run rviz2 rviz2

## Two fixed RGB-D cameras

The point-cloud environment spawns cameras on opposite sides of the table.
Both look down at 45 degrees toward the manipulation area.

| Camera | World position (m) | Topics | Point-cloud TF frame |
| --- | --- | --- | --- |
| First | `(0.45, -0.75, 1.50)` | `/camera/image`, `/camera/depth_image`, `/camera/camera_info`, `/camera/points` | `pc_camera_link` |
| Second | `(0.45, 0.75, 1.50)` | `/camera2/image`, `/camera2/depth_image`, `/camera2/camera_info`, `/camera2/points` | `pc_camera2_link` |

Image topics use `pc_camera_optical_frame` and `pc_camera2_optical_frame`.
Both cameras have TF transforms to `world` and, through the robot, `base_link`.
In RViz, add both point-cloud topics with fixed frame `base_link` and Best Effort
reliability to compare the views.

The starter uses the first camera by default. To launch it using the second:

```bash
ros2 launch ur_move_merlab pc_processing_env.launch.py point_cloud_topic:=/camera2/points
```

Do not also run a separate starter while the launch's starter is enabled.
For two-camera fusion, subscribe to both topics, transform each cloud into the
same frame using its timestamp, and associate measurements taken at similar
times before combining them. The supplied starter still processes one camera.
