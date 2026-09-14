# Pick-and-Place Simulation Support

This package contains only the Gazebo and MoveIt resources required by the
`ur_move_merlab` fixed pick-and-place demo:

- UR5e and Robotiq gripper description and controllers;
- table, target marker, and grasp object models;
- an RGB-D camera publishing color, depth, and point-cloud data;
- a palm-mounted RGB camera pointing out between the gripper fingers;
- the Gazebo world and internal launch files used by the student demo.

Students should start the complete exercise through the public launch file:

```bash
ros2 launch ur_move_merlab simple_pick_place_demo.launch.py
```

The files in this package are supporting resources and do not provide a
separate assignment workflow.

The palm camera publishes 640 x 480 RGB images at 30 Hz on
`/palm_camera/image`, with calibration on `/palm_camera/camera_info`.
Both topics are bridged to ROS automatically. Its 90-degree horizontal field
of view supports viewing the colored object markers at close range.
The image and calibration frame is `palm_camera_optical_frame` (+Z forward,
+X right, +Y down), attached to the robot's TF tree and honoring `tf_prefix`.
The lens is centered 115 mm along the gripper base's +Z axis, 17 mm forward
of its original position. The camera has visual and inertial geometry but no
collision geometry, so it cannot collide with the object or gripper fingers. The fixed
RGB-D camera continues to use its existing `/camera/*` topics.
