# Simple Simulated Pick and Place

## Motion interface

`ur_move_simple_interface` exposes Cartesian motion and joint trajectories to task nodes.
It starts without commanding any motion. To launch the scene, MoveIt, and this
interface together:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rbe4540/install/setup.bash
ros2 launch ur_move_merlab ur_move_simple_interface.launch.py
```

Use `gazebo_gui:=false` for a headless simulation, or `start_simulation:=false`
when the scene and MoveIt are already running. The pick-and-place demo already starts this same interface; use its running
services rather than starting a second instance.

The service `/cartesian_ref` uses
`common_interfaces_merlab/srv/SendPose`:

```text
geometry_msgs/Pose pose
---
bool success
```

The requested pose is the pose of **tool0 in base_link**, with position in meters
and orientation as a quaternion. `tool0` is the robot's tool mounting frame,
not the center between the finger pads. Nonzero quaternions are normalized.
The node computes a Cartesian path from the latest joint state and executes it
through MoveIt's `ExecuteTrajectory` action. `success: true` means execution
finished successfully; it does not mean only that the request was accepted.

The same node also exposes `/move_traj_single_point`
(`SendJointTrajectoryPoint`) and `/move_traj_multi_point` (`SendJointTrajectory`).
All three services share a motion lock and cancel timed-out execution.
Existing `joints`, `controller_name`, `joint_trajectory_timeout_sec`, and
`cartesian_*` parameters remain supported. Cartesian retries require a complete
path, and `cartesian_settle_delay_sec` applies after successful motion.

A task node can call it asynchronously:

```python
from common_interfaces_merlab.srv import SendPose

client = node.create_client(SendPose, '/cartesian_ref')
client.wait_for_service(timeout_sec=5.0)
request = SendPose.Request()
request.pose = target_pose  # geometry_msgs.msg.Pose, expressed in base_link
future = client.call_async(request)
# Keep spinning the task node; inspect future.result().success when complete.
```

Only complete Cartesian paths are executed. Invalid poses, missing or stale
joint states, incomplete paths, unavailable MoveIt services, execution failures,
and overlapping requests return `success: false`, with the reason in the node
log. Execution timeouts request cancellation. The interface remains busy until
the previous action ends, including when its acceptance or cancellation is late.
Tasks should wait for each response before requesting the next motion.

Defaults and timeouts are in `config/ur_move_simple_interface.yaml`; use
`config_file:=/absolute/path/to/config.yaml` to load another configuration.
Velocity and acceleration scaling default to 0.2. Collision checking is enabled
by default through `cartesian_avoid_collisions` against the **MoveIt planning scene**; Gazebo objects are not automatically
inserted into that scene. The fixed demo keeps its explicit `cartesian_avoid_collisions: false` override.
Cartesian motion can fail even for a reachable endpoint
if a complete Cartesian path from the current configuration is unavailable.

To run the interface alone without a launch file:

```bash
ros2 run ur_move_merlab ur_move_simple_interface --ros-args -p use_sim_time:=true
```

## End-effector velocity services

`/set_ee_velocity` and `/set_base_velocity` use
`common_interfaces_merlab/srv/SendTwist`. Both command the velocity of the
`tool0` origin: the first expresses the linear and angular components in the
moving tool axes, and the second in the fixed `base_link` axes. Angular
velocity rotates the tool about its own origin in both cases.

```text
geometry_msgs/Twist twist
---
bool success
string message
```

Linear components are in m/s and angular components in rad/s. The simulation
launch starts MoveIt Servo, which converts these commands to joint trajectories.
The interface selects Servo's Twist mode automatically.

For example, request 0.05 m/s along the tool's x axis:

```bash
ros2 service call /set_ee_velocity common_interfaces_merlab/srv/SendTwist \
  "{twist: {linear: {x: 0.05}}}"
```

Use `/set_base_velocity` for the same motion along the base x axis. Send an
all-zero command to either service to stop velocity motion:

```bash
ros2 service call /set_base_velocity common_interfaces_merlab/srv/SendTwist "{}"
```

Commands expire after `velocity_command_timeout_sec` (default 0.5 seconds).
For continuous motion, refresh the service request at about 10 Hz from your
node; the interface streams the latest command to Servo at 50 Hz. A single
CLI request therefore produces a short motion. `success: true` acknowledges
the command, not completion or attainment of the requested speed. Servo may
reduce or halt motion near collisions, singularities, or joint limits.

The default limits on the linear and angular vector magnitudes are 0.2 m/s
and 0.5 rad/s. Settings are in `config/ur_move_simple_interface.yaml`; Servo's
own settings are in `ur_simulation_gz/config/servo.yaml`. Both limits should
be updated together if a larger speed range is needed. Velocity control
requires recent joint position and velocity feedback. Joint and Cartesian
position requests are rejected while velocity control is active or stopping.
A zero velocity request stops velocity control only; it does not cancel a
position trajectory.

The gripper SRDF excludes each finger pad from collision checks against its
same-side inner knuckle, as it already does for the attached inner finger.
These neighboring parts normally remain about 1 mm apart. Servo's self-collision
slowdown distance is 5 mm, below the open gripper's normal 7 mm opposing-knuckle
clearance; other self-collision pairs and environment collision checks remain
enabled. Restart the simulation after changing the SRDF or Servo configuration.

Rebuild the interface messages and affected packages before using these services:

```bash
colcon build --symlink-install --packages-select common_interfaces_merlab ur_simulation_gz ur_move_merlab
source install/setup.bash
ros2 launch ur_move_merlab ur_move_simple_interface.launch.py
```

## Simple run example

Edit `ur_move_merlab/simple_run.py`, starting with `run()`. It sends two
Cartesian commands through `/cartesian_ref`, waits for each motion to finish,
and stops if either fails. Positions are in meters in `base_link`, with `tool0`
pointing downward. The example poses require a complete Cartesian path from
the robot's starting configuration.

Build from the workspace root and start the simulation and motion interface:

```bash
colcon build --symlink-install --packages-select ur_move_merlab
source install/setup.bash
ros2 launch ur_move_merlab ur_move_simple_interface.launch.py
```

Once the simulation is ready, run the example node in a second terminal:

```bash
source ~/rbe4540/install/setup.bash
ros2 run ur_move_merlab simple_run
```

After the two position commands, the node sends tool-frame X velocities of
+0.03 m/s and -0.03 m/s for three seconds each, refreshing every 0.1 seconds,
then sends zero velocity and exits. Each leg nominally travels 9 cm; simulation
speed and Servo limiting can reduce the actual displacement.

## Fixed Pick and Place

The first assignment demonstrates the manipulation sequence without using
perception to choose a grasp. The robot uses fixed poses in `base_link` to:

1. open the gripper and move to an observation configuration;
2. approach and grasp the red block;
3. lift it and transfer it above the green target;
4. place it, retreat, and return home.

The RGB-D camera remains in the scene. The `fixed_pick_place` node subscribes
to `/camera/image` and `/camera/points`, caches their newest messages, and
reports when data first arrives. Sensor data does not alter the fixed motion.

### Build and run

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ur_move_merlab simple_pick_place_demo.launch.py
```

The launch file starts a self-contained Gazebo scene, MoveIt, the motion service
interface, and the assignment node. No external object-model dataset is needed.
The demo runs once automatically and the node remains alive to receive camera
data.

To trigger the sequence manually instead:

```bash
ros2 launch ur_move_merlab simple_pick_place_demo.launch.py auto_start:=false
ros2 service call /run_pick_place std_srvs/srv/Trigger "{}"
```

Restart the simulation before a second run so that the block is reset to its
fixed pick position.

Closing Gazebo also shuts down the launch's ROS nodes and clock bridge. Wait
for the launch to exit before starting it again. If repeated `Detected jump
back in time` warnings appear, check for an older launch still running in
another terminal and stop it with Ctrl+C; overlapping clock bridges can
deliver simulation timestamps out of order.

### Student-facing parameters

Motion poses, hover distances, home joints, and gripper positions are in
`config/simple_pick_place.yaml`. The scene geometry is intentionally matched to
those coordinates. The two sensor topic names can be remapped from the launch
command with `image_topic:=...` and `point_cloud_topic:=...`.

The simulated gripper's driving joint is limited to 1 N·m in
`robotiq_description/urdf/robotiq_2f_140.xacro`. This lets the motor stall against
the block while maintaining a closing command. The previous 1000 N·m limit
destabilized the finger linkage during contact and caused the block to slip.
Restart Gazebo after changing this limit; already-spawned models retain their
original physics settings.

The workspace intentionally contains no real-robot, alternate perception, or
additional manipulation demos.
