#!/usr/bin/env python3
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Felix Exner

from copy import deepcopy
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from common_interfaces_merlab.srv import SendJointTrajectory, SendJointTrajectoryPoint, SendPose
from common_interfaces_merlab.srv import SendTwist
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Twist, TwistStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, ServoCommandType
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class JTCClient(Node):
    """Expose joint and Cartesian motion services with shared execution ownership."""

    def __init__(self, **kwargs):
        super().__init__('jtc_client', **kwargs)
        defaults = {
            'reference_frame': 'base_link',
            'end_effector_link': 'tool0',
            'planning_group': 'ur_manipulator',
            'controller_name': 'joint_trajectory_controller',
            'joints': [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
            ],
            'max_step': 0.01,
            'velocity_scaling': 0.2,
            'acceleration_scaling': 0.2,
            'cartesian_avoid_collisions': True,
            'state_timeout_sec': 2.0,
            'service_timeout_sec': 5.0,
            'planning_timeout_sec': 10.0,
            'execution_timeout_sec': 60.0,
            'cancel_timeout_sec': 5.0,
            'joint_trajectory_timeout_sec': 90.0,
            'cartesian_planning_attempts': 3,
            'cartesian_retry_delay_sec': 0.25,
            'cartesian_settle_delay_sec': 0.15,
            'servo_node_name': 'servo_node',
            'velocity_command_timeout_sec': 0.5,
            'velocity_publish_period_sec': 0.02,
            'velocity_stop_settle_sec': 0.3,
            'max_linear_velocity': 0.2,
            'max_angular_velocity': 0.5,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.settings = {
            name: self.get_parameter(name).value for name in defaults
        }
        for name in (
            'max_step', 'state_timeout_sec', 'service_timeout_sec',
            'planning_timeout_sec', 'execution_timeout_sec', 'cancel_timeout_sec',
            'joint_trajectory_timeout_sec',
            'velocity_command_timeout_sec', 'velocity_publish_period_sec',
            'velocity_stop_settle_sec', 'max_linear_velocity', 'max_angular_velocity',
        ):
            value = self.settings[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            value = self.settings[name]
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1]')

        for name in ('cartesian_retry_delay_sec', 'cartesian_settle_delay_sec'):
            value = self.settings[name]
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if self.settings['cartesian_planning_attempts'] < 1:
            raise ValueError('cartesian_planning_attempts must be positive')
        self.joints = self.settings['joints']
        if not self.joints or len(set(self.joints)) != len(self.joints):
            raise ValueError('joints must be nonempty and unique')
        self.controller_name = self.settings['controller_name']
        self._motion_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._joint_state = None
        self._state_received_at = 0.0
        self._callbacks = ReentrantCallbackGroup()
        self._state_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10,
            callback_group=self._callbacks,
        )
        self._planner = self.create_client(
            GetCartesianPath, 'compute_cartesian_path',
            callback_group=self._callbacks,
        )
        self._executor = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory',
            callback_group=self._callbacks,
        )
        self._pose_service = self.create_service(
            SendPose, 'cartesian_ref', self.trajectoryExecutionCallback,
            callback_group=self._callbacks,
        )
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            self.controller_name + '/follow_joint_trajectory',
            callback_group=self._callbacks,
        )
        self.srv_single_point_traj = self.create_service(
            SendJointTrajectoryPoint, 'move_traj_single_point',
            self.singlePointTrajectoryCallback, callback_group=self._callbacks,
        )
        self.srv_multi_point_traj = self.create_service(
            SendJointTrajectory, 'move_traj_multi_point',
            self.multiPointTrajectoryCallback, callback_group=self._callbacks,
        )
        # Velocity services and their watchdog serialize updates; ROS responses
        # and joint states continue to run in the reentrant callback group.
        self._velocity_callbacks = MutuallyExclusiveCallbackGroup()
        self._velocity_active = False
        self._velocity_stopping_at = None
        self._velocity_updated_at = 0.0
        self._velocity_command = TwistStamped()
        servo_name = self.settings['servo_node_name'].rstrip('/')
        self._twist_publisher = self.create_publisher(
            TwistStamped, servo_name + '/delta_twist_cmds', 1,
        )
        self._servo_mode = self.create_client(
            ServoCommandType, servo_name + '/switch_command_type',
            callback_group=self._callbacks,
        )
        self._servo_pause = self.create_client(
            SetBool, servo_name + '/pause_servo', callback_group=self._callbacks,
        )
        self._ee_velocity_service = self.create_service(
            SendTwist, 'set_ee_velocity', self._set_ee_velocity,
            callback_group=self._velocity_callbacks,
        )
        self._base_velocity_service = self.create_service(
            SendTwist, 'set_base_velocity', self._set_base_velocity,
            callback_group=self._velocity_callbacks,
        )
        self._velocity_timer = self.create_timer(
            self.settings['velocity_publish_period_sec'], self._update_velocity,
            callback_group=self._velocity_callbacks,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            f'Cartesian pose service: {self.resolve_service_name("cartesian_ref")}; '
            f'frame={self.settings["reference_frame"]}, '
            f'tool={self.settings["end_effector_link"]}'
        )

    def _set_ee_velocity(self, request, response):
        return self._set_velocity(request, response, self.settings['end_effector_link'])

    def _set_base_velocity(self, request, response):
        return self._set_velocity(request, response, self.settings['reference_frame'])

    def _servo_request(self, client, request):
        timeout = self.settings['service_timeout_sec']
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Servo service unavailable: {client.srv_name}')
        result = self._wait(client.call_async(request), timeout)
        if result is None or not result.success:
            raise RuntimeError(f'Servo service failed: {client.srv_name}')

    def _velocity_state(self):
        with self._state_lock:
            state = self._joint_state
            if (state is None or time.monotonic() - self._state_received_at >
                    self.settings['state_timeout_sec'] or
                    len(state.velocity) != len(state.name)):
                raise ValueError('Velocity control requires recent joint positions and velocities')
            velocities = dict(zip(state.name, state.velocity))
            values = [velocities[joint] for joint in self.joints]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('Joint velocities must be finite')
            return values

    def _set_velocity(self, request, response, frame):
        response.success = False
        try:
            twist = request.twist
            linear = (twist.linear.x, twist.linear.y, twist.linear.z)
            angular = (twist.angular.x, twist.angular.y, twist.angular.z)
            if not all(math.isfinite(value) for value in linear + angular):
                raise ValueError('Velocity components must be finite')
            if (math.hypot(*linear) > self.settings['max_linear_velocity'] or
                    math.hypot(*angular) > self.settings['max_angular_velocity']):
                raise ValueError('Requested velocity exceeds the configured speed limits')
            if not any(linear + angular):
                if self._velocity_active:
                    self._begin_velocity_stop()
                response.success = True
                response.message = 'Velocity stop requested'
                return response
            if self._velocity_stopping_at is not None:
                raise RuntimeError('Velocity motion is stopping; retry after it stops')
            self._velocity_state()
            if not self._velocity_active:
                if not self._motion_lock.acquire(blocking=False):
                    raise RuntimeError('Motion is busy')
                try:
                    # Resume resets Servo smoothing to the current robot state.
                    self._servo_request(self._servo_pause, SetBool.Request(data=True))
                    self._servo_request(
                        self._servo_mode,
                        ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST),
                    )
                    self._servo_request(self._servo_pause, SetBool.Request(data=False))
                except Exception:
                    self._motion_lock.release()
                    raise
                self._velocity_active = True
            self._velocity_command.header.frame_id = frame
            self._velocity_command.twist = deepcopy(twist)
            self._velocity_updated_at = time.monotonic()
            self._publish_velocity()
            response.success = True
            response.message = f'Velocity accepted in {frame}; refresh before the command timeout'
        except Exception as error:
            response.message = str(error)
            self.get_logger().warn(f'Velocity request rejected: {error}')
        return response

    def _publish_velocity(self):
        self._velocity_command.header.stamp = self.get_clock().now().to_msg()
        self._twist_publisher.publish(self._velocity_command)

    def _begin_velocity_stop(self):
        if self._velocity_stopping_at is None:
            self._velocity_stopping_at = time.monotonic()
        self._velocity_command.twist = Twist()
        self._publish_velocity()

    def _update_velocity(self):
        if not self._velocity_active:
            return
        now = time.monotonic()
        try:
            velocities = self._velocity_state()
        except ValueError:
            velocities = None
        if (velocities is None or now - self._velocity_updated_at >=
                self.settings['velocity_command_timeout_sec']):
            self._begin_velocity_stop()
        self._publish_velocity()
        if self._velocity_stopping_at is None:
            return
        if (now - self._velocity_stopping_at < self.settings['velocity_stop_settle_sec'] or
                velocities is None or any(abs(value) > 0.01 for value in velocities)):
            return
        try:
            # Pause only after zero commands have stopped the arm. This prevents
            # Servo's holding trajectories from overwriting the next planned move.
            self._servo_request(self._servo_pause, SetBool.Request(data=True))
        except Exception as error:
            self.get_logger().error(f'Cannot pause Servo; keeping motion locked: {error}')
            return
        self._velocity_active = False
        self._velocity_stopping_at = None
        self._motion_lock.release()

    def _on_joint_state(self, message):
        if len(message.name) != len(message.position):
            return
        if not set(self.settings['joints']).issubset(message.name):
            return
        if not all(math.isfinite(value) for value in message.position):
            return
        with self._state_lock:
            self._joint_state = deepcopy(message)
            self._state_received_at = time.monotonic()

    @staticmethod
    def _normalized_pose(pose):
        pose = deepcopy(pose)
        p, q = pose.position, pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise ValueError('Pose must contain only finite numbers')
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if norm < 1e-9:
            raise ValueError('Pose orientation must be a nonzero quaternion')
        q.x, q.y, q.z, q.w = (q.x / norm, q.y / norm, q.z / norm, q.w / norm)
        return pose

    @staticmethod
    def _wait(future, timeout):
        """Let the other executor threads process ROS responses while waiting."""
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not future.done() and not ready.wait(timeout):
            raise TimeoutError('Timed out waiting for a ROS response')
        return future.result()

    def _planning_request(self, pose):
        with self._state_lock:
            if (self._joint_state is None or
                    time.monotonic() - self._state_received_at >
                    self.settings['state_timeout_sec']):
                raise ValueError('No recent, complete joint state is available')
            state = deepcopy(self._joint_state)
        # Only positions are needed, and velocities/efforts may be partial.
        state.velocity = []
        state.effort = []
        request = GetCartesianPath.Request()
        request.header.frame_id = self.settings['reference_frame']
        request.group_name = self.settings['planning_group']
        request.link_name = self.settings['end_effector_link']
        request.start_state.joint_state = state
        request.start_state.is_diff = True
        request.waypoints = [pose]
        request.max_step = self.settings['max_step']
        request.avoid_collisions = self.settings['cartesian_avoid_collisions']
        request.max_velocity_scaling_factor = self.settings['velocity_scaling']
        request.max_acceleration_scaling_factor = self.settings['acceleration_scaling']
        return request

    def _release_after_execution(self, _future):
        self._motion_lock.release()

    def _cancel_late_goal(self, future):
        """A goal accepted after a timeout must not run alongside a new request."""
        try:
            goal = future.result()
            if goal is None or not goal.accepted:
                self._motion_lock.release()
                return
            result = goal.get_result_async()
        except Exception as error:
            self.get_logger().error(f'Late execution goal failed: {error}')
            self._motion_lock.release()
            return
        result.add_done_callback(self._release_after_execution)
        try:
            goal.cancel_goal_async()
        except Exception as error:
            self.get_logger().error(f'Late goal cancellation failed: {error}')

    def _run_motion(self, operation):
        if not self._motion_lock.acquire(blocking=False):
            self.get_logger().warn('Motion request rejected: motion is busy')
            return False
        ownership = {'release': True}
        try:
            success = operation(ownership)
            if success:
                time.sleep(self.settings['cartesian_settle_delay_sec'])
            return success
        except Exception as error:
            self.get_logger().error(f'Motion request failed: {error}')
            return False
        finally:
            if ownership['release']:
                self._motion_lock.release()

    def _execute_goal(self, client, goal, execution_timeout, ownership):
        timeout = self.settings['service_timeout_sec']
        if not client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('Trajectory execution action is unavailable')
        goal_future = client.send_goal_async(goal)
        try:
            handle = self._wait(goal_future, timeout)
        except TimeoutError:
            ownership['release'] = False
            goal_future.add_done_callback(self._cancel_late_goal)
            raise
        if handle is None or not handle.accepted:
            raise RuntimeError('Trajectory execution was rejected')
        result_future = handle.get_result_async()
        try:
            return self._wait(result_future, execution_timeout)
        except TimeoutError:
            # The action still owns the robot until its result arrives, even
            # when cancellation is delayed or refused.
            ownership['release'] = False
            result_future.add_done_callback(self._release_after_execution)
            handle.cancel_goal_async()
            try:
                self._wait(result_future, self.settings['cancel_timeout_sec'])
            except TimeoutError:
                self.get_logger().error(
                    'Execution has not stopped; rejecting new motions until it ends'
                )
            raise TimeoutError('Execution timed out; cancellation requested')

    def trajectoryExecutionCallback(self, request, response):
        response.success = self._run_motion(
            lambda ownership: self._move_cartesian(request.pose, ownership)
        )
        return response

    def _move_cartesian(self, pose, ownership):
        pose = self._normalized_pose(pose)
        if not self._planner.wait_for_service(
                timeout_sec=self.settings['service_timeout_sec']):
            raise RuntimeError('Cartesian planning service is unavailable')
        attempts = self.settings['cartesian_planning_attempts']
        for attempt in range(attempts):
            plan = self._wait(
                self._planner.call_async(self._planning_request(pose)),
                self.settings['planning_timeout_sec'],
            )
            if plan is None or plan.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError('MoveIt Cartesian planning failed')
            if not math.isfinite(plan.fraction):
                raise RuntimeError('MoveIt returned a nonfinite path fraction')
            if plan.fraction >= 1.0 - 1e-6:
                break
            if attempt + 1 == attempts:
                raise RuntimeError(
                    f'Cartesian path is incomplete ({plan.fraction:.1%}); '
                    'no motion was executed'
                )
            time.sleep(self.settings['cartesian_retry_delay_sec'])
        if not plan.solution.joint_trajectory.points:
            raise RuntimeError('MoveIt returned an empty trajectory')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan.solution
        goal.controller_names = [self.controller_name]
        result = self._execute_goal(
            self._executor, goal, self.settings['execution_timeout_sec'], ownership,
        )
        success = (
            result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result.error_code.val == MoveItErrorCodes.SUCCESS
        )
        if success:
            self.get_logger().info('Cartesian motion completed')
        else:
            self.get_logger().error('Cartesian trajectory execution failed')
        return success

    def singlePointTrajectoryCallback(self, request, response):
        response.success = self.execute_setpoint(request.goal_point)
        return response

    def multiPointTrajectoryCallback(self, request, response):
        response.success = self.execute_multi_point_trajectory(request.goal_points)
        return response

    def execute_setpoint(self, trajectory_point):
        return self._send_joint_points([trajectory_point], tolerance=0.01)

    def execute_multi_point_trajectory(self, trajectory_points):
        return self._send_joint_points(trajectory_points.points, tolerance=0.02)

    def _send_joint_points(self, points, tolerance):
        if not points:
            self.get_logger().error('Received empty joint trajectory')
            return False
        for point in points:
            if (len(point.positions) != len(self.joints) or
                    not all(math.isfinite(value) for value in point.positions)):
                self.get_logger().error('Joint positions must be complete and finite')
                return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=self.joints, points=list(points))
        goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        goal.goal_tolerance = [
            JointTolerance(position=tolerance, velocity=tolerance, name=joint)
            for joint in self.joints
        ]
        return self.send_joint_trajectory_goal(goal)

    def send_joint_trajectory_goal(self, goal_to_execute, timeout_sec=None):
        if timeout_sec is None:
            timeout_sec = self.settings['joint_trajectory_timeout_sec']

        def execute(ownership):
            result = self._execute_goal(
                self._action_client, goal_to_execute, timeout_sec, ownership,
            )
            success = (
                result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
                and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            if success:
                self.get_logger().info('Joint trajectory completed')
            else:
                self.get_logger().error('Joint trajectory execution failed')
            return success

        return self._run_motion(execute)


def main(args=None):
    rclpy.init(args=args)
    node = JTCClient()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
