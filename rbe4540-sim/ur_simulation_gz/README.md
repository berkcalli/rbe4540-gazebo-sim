# Pick-and-Place Simulation Support

This package contains only the Gazebo and MoveIt resources required by the
`ur_move_merlab` fixed pick-and-place demo:

- UR5e and Robotiq gripper description and controllers;
- table, target marker, and grasp object models;
- an RGB-D camera publishing color, depth, and point-cloud data;
- the Gazebo world and internal launch files used by the student demo.

Students should start the complete exercise through the public launch file:

```bash
ros2 launch ur_move_merlab simple_pick_place_demo.launch.py
```

The files in this package are supporting resources and do not provide a
separate assignment workflow.
