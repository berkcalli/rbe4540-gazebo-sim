"""Exercise the public service against a controlled MoveIt backend."""

from copy import deepcopy
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from common_interfaces_merlab.srv import SendJointTrajectory, SendJointTrajectoryPoint, SendPose
from common_interfaces_merlab.srv import SendTwist
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, ServoCommandType
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ur_move_merlab.ur_move_simple_interface import JTCClient


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), 'Condition did not become true'


def pose_request():
    request = SendPose.Request()
    request.pose.position.x = 0.4
    request.pose.position.z = 0.5
    request.pose.orientation.w = 2.0  # Normalized by the interface.
    return request


@pytest.fixture
def rig():
    context = Context()
    rclpy.init(context=context)
    interface = JTCClient(
        context=context, namespace='interface_test',
        parameter_overrides=[
            Parameter('service_timeout_sec', value=0.5),
            Parameter('execution_timeout_sec', value=1.0),
            Parameter('cancel_timeout_sec', value=0.2),
        ],
    )
    backend = Node('backend', context=context, namespace='interface_test')
    group = ReentrantCallbackGroup()
    state = SimpleNamespace(
        fraction=1.0, planning_success=True, empty=False, planning_delay=0.0,
        abort=False, reject=False, reject_cancel=False, goal_delay=0.0, execute_delay=0.0,
        servo_calls=[], twists=[], servo_mode_success=True, servo_pause_success=True,
        joint_velocities=[0.0] * 6,
        started=threading.Event(), canceled=threading.Event(), plans=[], goals=[], joint_goals=[],
    )

    def plan(request, response):
        state.plans.append(deepcopy(request))
        time.sleep(state.planning_delay)
        response.fraction = state.fraction
        response.error_code.val = (
            MoveItErrorCodes.SUCCESS if state.planning_success
            else MoveItErrorCodes.PLANNING_FAILED
        )
        response.solution.joint_trajectory.joint_names = interface.settings['joints']
        if not state.empty:
            response.solution.joint_trajectory.points = [
                JointTrajectoryPoint(positions=[0.0] * 6),
            ]
        return response

    def accept(_goal):
        time.sleep(state.goal_delay)
        return GoalResponse.REJECT if state.reject else GoalResponse.ACCEPT

    def execute(handle):
        is_joint = isinstance(handle.request, FollowJointTrajectory.Goal)
        result_type = FollowJointTrajectory.Result if is_joint else ExecuteTrajectory.Result
        (state.joint_goals if is_joint else state.goals).append(handle.request)
        state.started.set()
        deadline = time.monotonic() + state.execute_delay
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                state.canceled.set()
                result = result_type()
                if not is_joint:
                    result.error_code.val = MoveItErrorCodes.PREEMPTED
                return result
            time.sleep(0.01)
        result = result_type()
        if state.abort:
            handle.abort()
            if is_joint:
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            else:
                result.error_code.val = MoveItErrorCodes.CONTROL_FAILED
        else:
            handle.succeed()
            if is_joint:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            else:
                result.error_code.val = MoveItErrorCodes.SUCCESS
        return result

    backend.create_service(
        GetCartesianPath, 'compute_cartesian_path', plan, callback_group=group,
    )
    action = ActionServer(
        backend, ExecuteTrajectory, 'execute_trajectory', execute,
        goal_callback=accept,
        cancel_callback=lambda _: (
            CancelResponse.REJECT if state.reject_cancel else CancelResponse.ACCEPT
        ),
        callback_group=group,
    )
    joint_action = ActionServer(
        backend, FollowJointTrajectory,
        'joint_trajectory_controller/follow_joint_trajectory', execute,
        goal_callback=accept,
        cancel_callback=lambda _: (
            CancelResponse.REJECT if state.reject_cancel else CancelResponse.ACCEPT
        ),
        callback_group=group,
    )
    joint_client = backend.create_client(
        SendJointTrajectoryPoint, 'move_traj_single_point', callback_group=group,
    )
    multi_client = backend.create_client(
        SendJointTrajectory, 'move_traj_multi_point', callback_group=group,
    )
    def mode(request, response):
        state.servo_calls.append(('mode', request.command_type))
        response.success = state.servo_mode_success
        return response

    def pause(request, response):
        state.servo_calls.append(('pause', request.data))
        response.success = state.servo_pause_success
        return response

    backend.create_service(
        ServoCommandType, 'servo_node/switch_command_type', mode, callback_group=group,
    )
    backend.create_service(
        SetBool, 'servo_node/pause_servo', pause, callback_group=group,
    )
    backend.create_subscription(
        TwistStamped, 'servo_node/delta_twist_cmds',
        lambda message: state.twists.append(deepcopy(message)), 10, callback_group=group,
    )
    ee_velocity_client = backend.create_client(
        SendTwist, 'set_ee_velocity', callback_group=group,
    )
    base_velocity_client = backend.create_client(
        SendTwist, 'set_base_velocity', callback_group=group,
    )
    publisher = backend.create_publisher(JointState, 'joint_states', 10)
    timer = backend.create_timer(
        0.02, lambda: publisher.publish(JointState(
            name=interface.settings['joints'], position=[0.0] * 6,
            velocity=state.joint_velocities,
        )), callback_group=group,
    )
    client = backend.create_client(
        SendPose, 'cartesian_ref', callback_group=group,
    )
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    executor.add_node(interface)
    executor.add_node(backend)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    assert client.wait_for_service(timeout_sec=5)
    wait_until(lambda: interface._joint_state is not None)
    assert joint_client.wait_for_service(timeout_sec=5)
    assert multi_client.wait_for_service(timeout_sec=5)
    yield SimpleNamespace(
        node=interface, state=state, client=client, timer=timer,
        joint_client=joint_client, multi_client=multi_client,
        ee_velocity_client=ee_velocity_client, base_velocity_client=base_velocity_client,
    )
    if interface._velocity_active:
        state.servo_pause_success = True
        state.joint_velocities = [0.0] * 6
        interface._velocity_updated_at = 0.0
        wait_until(lambda: not interface._velocity_active)
    interface._velocity_timer.cancel()
    timer.cancel()
    executor.shutdown(timeout_sec=5)
    thread.join(timeout=5)
    action.destroy()
    joint_action.destroy()
    interface.destroy_node()
    backend.destroy_node()
    context.shutdown()


def call(rig, request=None):
    future = rig.client.call_async(request or pose_request())
    wait_until(future.done)
    return future.result()


def test_pose_is_planned_and_execution_completes(rig):
    rig.state.execute_delay = 0.15
    future = rig.client.call_async(pose_request())
    wait_until(rig.state.started.is_set)
    assert not future.done()
    wait_until(future.done)
    assert future.result().success
    plan = rig.state.plans[0]
    assert plan.header.frame_id == 'base_link'
    assert plan.link_name == 'tool0'
    assert plan.waypoints[0].orientation.w == 1.0
    assert plan.avoid_collisions
    assert plan.max_velocity_scaling_factor == 0.2
    assert rig.state.goals[0].controller_names == ['joint_trajectory_controller']


@pytest.mark.parametrize('mode', ['partial', 'planning_failure', 'empty'])
def test_bad_plans_are_never_executed(rig, mode):
    if mode == 'partial':
        rig.state.fraction = 0.95
    elif mode == 'planning_failure':
        rig.state.planning_success = False
    else:
        rig.state.empty = True
    assert not call(rig).success
    assert not rig.state.goals


@pytest.mark.parametrize('mode', ['zero_quaternion', 'nonfinite'])
def test_invalid_pose_is_rejected_before_planning(rig, mode):
    request = pose_request()
    if mode == 'zero_quaternion':
        request.pose.orientation.w = 0.0
    else:
        request.pose.position.x = float('nan')
    assert not call(rig, request).success
    assert not rig.state.plans


@pytest.mark.parametrize('mode', ['abort', 'reject'])
def test_execution_failure_returns_false(rig, mode):
    setattr(rig.state, mode, True)
    assert not call(rig).success
    assert not rig.node._motion_lock.locked()


def test_concurrent_request_is_rejected(rig):
    rig.state.execute_delay = 0.3
    first = rig.client.call_async(pose_request())
    wait_until(rig.state.started.is_set)
    assert not call(rig).success
    wait_until(first.done)
    assert first.result().success
    assert len(rig.state.goals) == 1


def test_execution_timeout_cancels_motion(rig):
    rig.node.settings['execution_timeout_sec'] = 0.1
    rig.state.execute_delay = 2.0
    assert not call(rig).success
    wait_until(rig.state.canceled.is_set)
    wait_until(lambda: not rig.node._motion_lock.locked())


def test_goal_accepted_after_timeout_is_canceled(rig):
    rig.node.settings['service_timeout_sec'] = 0.1
    rig.state.goal_delay = 0.3
    rig.state.execute_delay = 2.0
    assert not call(rig).success
    assert rig.node._motion_lock.locked()
    assert not call(rig).success
    wait_until(rig.state.canceled.is_set)
    wait_until(lambda: not rig.node._motion_lock.locked())


def test_stale_or_incomplete_state_is_rejected(rig):
    rig.timer.cancel()
    time.sleep(0.05)
    with rig.node._state_lock:
        rig.node._state_received_at = time.monotonic() - 10
    with pytest.raises(ValueError, match='joint state'):
        rig.node._planning_request(pose_request().pose)
    before = rig.node._joint_state
    rig.node._on_joint_state(JointState(name=['shoulder_pan_joint'], position=[0.0]))
    assert rig.node._joint_state is before


def test_planning_timeout_never_executes_late_plan(rig):
    rig.node.settings['planning_timeout_sec'] = 0.05
    rig.state.planning_delay = 0.2
    assert not call(rig).success
    time.sleep(0.25)
    assert not rig.state.goals
    assert not rig.node._motion_lock.locked()


def test_refused_cancellation_keeps_interface_busy_until_action_ends(rig):
    rig.node.settings['execution_timeout_sec'] = 0.1
    rig.node.settings['cancel_timeout_sec'] = 0.05
    rig.state.execute_delay = 0.5
    rig.state.reject_cancel = True
    assert not call(rig).success
    assert rig.node._motion_lock.locked()
    assert not call(rig).success
    assert len(rig.state.goals) == 1
    wait_until(lambda: not rig.node._motion_lock.locked())


def test_late_goal_rejection_releases_interface(rig):
    rig.node.settings['service_timeout_sec'] = 0.1
    rig.state.goal_delay = 0.3
    rig.state.reject = True
    assert not call(rig).success
    wait_until(lambda: not rig.node._motion_lock.locked())
    assert not rig.state.goals


def joint_request():
    request = SendJointTrajectoryPoint.Request()
    request.goal_point.positions = [0.1] * 6
    request.goal_point.time_from_start.sec = 2
    return request


def joint_call(rig, request=None):
    future = rig.joint_client.call_async(request or joint_request())
    wait_until(future.done)
    return future.result()


def test_single_joint_target_waits_for_success(rig):
    rig.state.execute_delay = 0.15
    future = rig.joint_client.call_async(joint_request())
    wait_until(rig.state.started.is_set)
    assert not future.done()
    wait_until(future.done)
    assert future.result().success
    goal = rig.state.joint_goals[0]
    assert goal.trajectory.joint_names == rig.node.joints
    assert list(goal.trajectory.points[0].positions) == [0.1] * 6
    assert goal.trajectory.points[0].time_from_start.sec == 2


def test_multiple_joint_targets_are_forwarded(rig):
    request = SendJointTrajectory.Request()
    first = joint_request().goal_point
    second = deepcopy(first)
    second.positions = [0.2] * 6
    second.time_from_start.sec = 4
    request.goal_points.points = [first, second]
    future = rig.multi_client.call_async(request)
    wait_until(future.done)
    assert future.result().success
    assert rig.state.joint_goals[0].trajectory.points == [first, second]


@pytest.mark.parametrize('joint_first', [False, True])
def test_joint_and_cartesian_requests_share_motion_lock(rig, joint_first):
    rig.state.execute_delay = 0.3
    first = (rig.joint_client.call_async(joint_request()) if joint_first
             else rig.client.call_async(pose_request()))
    wait_until(rig.state.started.is_set)
    assert not (call(rig) if joint_first else joint_call(rig)).success
    wait_until(first.done)
    assert first.result().success
    assert len(rig.state.goals) + len(rig.state.joint_goals) == 1


@pytest.mark.parametrize('mode', ['abort', 'reject'])
def test_joint_execution_failure_returns_false(rig, mode):
    setattr(rig.state, mode, True)
    assert not joint_call(rig).success
    assert not rig.node._motion_lock.locked()


def test_joint_execution_timeout_cancels_and_blocks_cartesian_requests(rig):
    rig.node.settings['joint_trajectory_timeout_sec'] = 0.1
    rig.node.settings['cancel_timeout_sec'] = 0.05
    rig.state.execute_delay = 0.6
    rig.state.reject_cancel = True
    assert not joint_call(rig).success
    assert rig.node._motion_lock.locked()
    assert not call(rig).success
    wait_until(lambda: not rig.node._motion_lock.locked())


def test_late_joint_goal_is_canceled(rig):
    rig.node.settings['service_timeout_sec'] = 0.1
    rig.state.goal_delay = 0.3
    rig.state.execute_delay = 2.0
    assert not joint_call(rig).success
    assert rig.node._motion_lock.locked()
    assert not call(rig).success
    wait_until(rig.state.canceled.is_set)
    wait_until(lambda: not rig.node._motion_lock.locked())


@pytest.mark.parametrize('positions', [[0.0], [float('nan')] * 6])
def test_invalid_joint_target_is_rejected(rig, positions):
    request = joint_request()
    request.goal_point.positions = positions
    assert not joint_call(rig, request).success
    assert not rig.state.joint_goals


def test_empty_joint_trajectory_is_rejected(rig):
    future = rig.multi_client.call_async(SendJointTrajectory.Request())
    wait_until(future.done)
    assert not future.result().success
    assert not rig.state.joint_goals


def test_cartesian_settings_reach_planner(rig):
    rig.node.settings.update(
        reference_frame='world', end_effector_link='custom_tool',
        planning_group='custom_group', velocity_scaling=0.3,
        acceleration_scaling=0.4, max_step=0.005,
        cartesian_avoid_collisions=False,
    )
    assert call(rig).success
    plan = rig.state.plans[0]
    assert plan.header.frame_id == 'world'
    assert plan.link_name == 'custom_tool'
    assert plan.group_name == 'custom_group'
    assert plan.max_velocity_scaling_factor == 0.3
    assert plan.max_acceleration_scaling_factor == 0.4
    assert plan.max_step == 0.005
    assert not plan.avoid_collisions


def velocity_call(rig, *, tool=False, x=0.05, angular_z=0.0):
    request = SendTwist.Request()
    request.twist.linear.x = x
    request.twist.angular.z = angular_z
    client = rig.ee_velocity_client if tool else rig.base_velocity_client
    assert client.wait_for_service(timeout_sec=2.0)
    future = client.call_async(request)
    wait_until(future.done)
    return future.result()


@pytest.mark.parametrize('tool, frame', [(False, 'base_link'), (True, 'tool0')])
def test_velocity_commands_select_axes_and_speed_units(rig, tool, frame):
    assert velocity_call(rig, tool=tool, angular_z=0.1).success
    wait_until(lambda: bool(rig.state.twists))
    command = rig.state.twists[-1]
    assert command.header.frame_id == frame
    assert command.twist.linear.x == 0.05
    assert command.twist.angular.z == 0.1
    assert command.header.stamp.sec > 0
    assert rig.state.servo_calls[:3] == [('pause', True), ('mode', 1), ('pause', False)]
    assert rig.node._motion_lock.locked()


def test_velocity_can_be_updated_in_another_frame(rig):
    assert velocity_call(rig).success
    assert velocity_call(rig, tool=True, x=-0.03).success
    wait_until(lambda: any(t.header.frame_id == 'tool0' for t in rig.state.twists))
    command = rig.state.twists[-1]
    assert command.twist.linear.x == -0.03
    assert len(rig.state.servo_calls) == 3


def test_zero_velocity_stops_then_allows_planned_motion(rig):
    assert velocity_call(rig).success
    assert velocity_call(rig, x=0.0).success
    assert rig.node._motion_lock.locked()
    assert not velocity_call(rig).success
    wait_until(lambda: not rig.node._motion_lock.locked())
    assert rig.state.twists[-1].twist.linear.x == 0.0
    assert rig.state.servo_calls[-1] == ('pause', True)
    assert call(rig).success


def test_velocity_watchdog_stops_without_another_request(rig):
    rig.node.settings['velocity_command_timeout_sec'] = 0.1
    assert velocity_call(rig).success
    wait_until(lambda: not rig.node._motion_lock.locked())
    assert rig.state.twists[-1].twist.linear.x == 0.0
    assert not rig.node._velocity_active


def test_velocity_keeps_ownership_until_joints_stop(rig):
    assert velocity_call(rig).success
    rig.state.joint_velocities = [0.1] * 6
    assert velocity_call(rig, x=0.0).success
    time.sleep(0.4)
    assert rig.node._motion_lock.locked()
    assert not call(rig).success
    rig.state.joint_velocities = [0.0] * 6
    wait_until(lambda: not rig.node._motion_lock.locked())


def test_velocity_stop_waits_for_servo_pause_acknowledgment(rig):
    assert velocity_call(rig).success
    rig.state.servo_pause_success = False
    assert velocity_call(rig, x=0.0).success
    time.sleep(0.4)
    assert rig.node._motion_lock.locked()
    assert not joint_call(rig).success
    rig.state.servo_pause_success = True
    wait_until(lambda: not rig.node._motion_lock.locked())


@pytest.mark.parametrize('joint', [False, True])
def test_velocity_and_planned_motion_exclude_each_other(rig, joint):
    rig.state.execute_delay = 0.4
    first = (rig.joint_client.call_async(joint_request()) if joint
             else rig.client.call_async(pose_request()))
    wait_until(rig.state.started.is_set)
    assert not velocity_call(rig).success
    assert not rig.state.twists
    wait_until(first.done)
    assert velocity_call(rig).success
    assert not (joint_call(rig) if joint else call(rig)).success


@pytest.mark.parametrize('x, angular_z', [(float('nan'), 0.0), (0.21, 0.0), (0.0, 0.51)])
def test_invalid_velocity_is_not_forwarded(rig, x, angular_z):
    assert not velocity_call(rig, x=x, angular_z=angular_z).success
    assert not rig.state.twists
    assert not rig.node._motion_lock.locked()


def test_servo_start_failure_does_not_claim_motion(rig):
    rig.state.servo_mode_success = False
    assert not velocity_call(rig).success
    assert not rig.state.twists
    assert not rig.node._motion_lock.locked()


def test_velocity_requires_joint_velocity_feedback(rig):
    rig.state.joint_velocities = []
    wait_until(lambda: len(rig.node._joint_state.velocity) == 0)
    assert not velocity_call(rig).success
    assert not rig.state.twists


def test_lost_joint_feedback_stops_velocity_and_keeps_motion_locked(rig):
    assert velocity_call(rig).success
    rig.state.joint_velocities = []
    wait_until(lambda: bool(rig.state.twists) and rig.state.twists[-1].twist.linear.x == 0)
    assert rig.node._motion_lock.locked()
    rig.state.joint_velocities = [0.0] * 6
    wait_until(lambda: not rig.node._motion_lock.locked())
