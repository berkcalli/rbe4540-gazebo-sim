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
