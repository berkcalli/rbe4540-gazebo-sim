"""Run a fixed-pose pick-and-place while passively receiving camera data."""

import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from common_interfaces_merlab.srv import SendJointTrajectoryPoint
from common_interfaces_merlab.srv import SendPose
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class FixedPickPlace(Node):
    """Execute a parameterized demo without using perception for motion."""

    def __init__(self):
        """Create motion clients, sensor inputs, and the run service."""
        super().__init__("fixed_pick_place")
        self.callback_group = ReentrantCallbackGroup()
        self.run_lock = threading.Lock()

        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("point_cloud_topic", "/camera/points")
        self.declare_parameter("cartesian_service", "/cartesian_ref")
        self.declare_parameter("joint_service", "/move_traj_single_point")
        self.declare_parameter(
            "gripper_command_topic",
            "/forward_position_controller_gripper/commands",
        )
        self.declare_parameter("run_service", "/run_pick_place")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("service_timeout_sec", 90.0)
        self.declare_parameter("motion_timeout_sec", 45.0)
        self.declare_parameter(
            "home_joint_positions",
            [-3.14, -1.67, -0.55, -1.39, 1.39, 0.0],
        )
        self.declare_parameter("home_move_time_sec", 5.0)
        self.declare_parameter("pick_x", 0.45)
        self.declare_parameter("pick_y", 0.0)
        self.declare_parameter("pick_z", 0.36)
        self.declare_parameter("place_x", 0.45)
        self.declare_parameter("place_y", -0.24)
        self.declare_parameter("place_z", 0.36)
        self.declare_parameter("hover_height", 0.18)
        self.declare_parameter("lift_height", 0.20)
        self.declare_parameter("tool_qx", 1.0)
        self.declare_parameter("tool_qy", 0.0)
        self.declare_parameter("tool_qz", 0.0)
        self.declare_parameter("tool_qw", 0.0)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_closed_position", 0.70)
        self.declare_parameter("gripper_wait_sec", 1.5)

        self.image_topic = self.get_parameter("image_topic").value
        self.point_cloud_topic = self.get_parameter("point_cloud_topic").value
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        self.motion_timeout_sec = float(
            self.get_parameter("motion_timeout_sec").value
        )
        self.home_joint_positions = list(
            self.get_parameter("home_joint_positions").value
        )
        self.home_move_time_sec = float(
            self.get_parameter("home_move_time_sec").value
        )
        self.pick_position = self._position_parameters("pick")
        self.place_position = self._position_parameters("place")
        self.hover_height = float(self.get_parameter("hover_height").value)
        self.lift_height = float(self.get_parameter("lift_height").value)
        self.tool_orientation = self._normalized_tool_orientation()
        self.gripper_open_position = float(
            self.get_parameter("gripper_open_position").value
        )
        self.gripper_closed_position = float(
            self.get_parameter("gripper_closed_position").value
        )
        self.gripper_wait_sec = float(
            self.get_parameter("gripper_wait_sec").value
        )
        self._validate_parameters()

        self.latest_image = None
        self.latest_point_cloud = None
        self.image_count = 0
        self.point_cloud_count = 0

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self._point_cloud_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        gripper_topic = self.get_parameter("gripper_command_topic").value
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            gripper_topic,
            10,
        )
        self.cartesian_client = self.create_client(
            SendPose,
            self.get_parameter("cartesian_service").value,
            callback_group=self.callback_group,
        )
        self.joint_client = self.create_client(
            SendJointTrajectoryPoint,
            self.get_parameter("joint_service").value,
            callback_group=self.callback_group,
        )
        self.run_service = self.create_service(
            Trigger,
            self.get_parameter("run_service").value,
            self._run_service_callback,
            callback_group=self.callback_group,
        )

        self.auto_start_timer = None
        if self.get_parameter("auto_start").value:
            start_delay_sec = max(
                0.1,
                float(self.get_parameter("start_delay_sec").value),
            )
            self.auto_start_timer = self.create_timer(
                start_delay_sec,
                self._auto_start_callback,
                callback_group=self.callback_group,
            )

        self.get_logger().info(
            "Fixed-pose mode is active: camera messages are cached but do not "
            "change the pick or place poses."
        )
        self.get_logger().info(
            f"Receiving Image on {self.image_topic} and PointCloud2 on "
            f"{self.point_cloud_topic}"
        )

    def _position_parameters(self, prefix):
        return tuple(
            float(self.get_parameter(f"{prefix}_{axis}").value)
            for axis in ("x", "y", "z")
        )

    def _normalized_tool_orientation(self):
        orientation = tuple(
            float(self.get_parameter(f"tool_q{axis}").value)
            for axis in ("x", "y", "z", "w")
        )
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm < 1e-9:
            raise ValueError("The tool orientation quaternion cannot be zero")
        return tuple(value / norm for value in orientation)

    def _validate_parameters(self):
        if len(self.home_joint_positions) != 6:
            raise ValueError(
                "home_joint_positions must contain six joint values"
            )
        if self.hover_height <= 0.0:
            raise ValueError("hover_height must be positive")
        if self.lift_height <= 0.0:
            raise ValueError("lift_height must be positive")

    def _image_callback(self, message):
        self.latest_image = message
        self.image_count += 1
        if self.image_count == 1:
            self.get_logger().info(
                f"Received first image: {message.width}x{message.height}, "
                f"encoding={message.encoding}"
            )

    def _point_cloud_callback(self, message):
        self.latest_point_cloud = message
        self.point_cloud_count += 1
        if self.point_cloud_count == 1:
            self.get_logger().info(
                "Received first point cloud: "
                f"frame={message.header.frame_id}, "
                f"size={message.width}x{message.height}"
            )

    def _auto_start_callback(self):
        self.auto_start_timer.cancel()
        success, message = self._try_run_demo()
        log = self.get_logger().info if success else self.get_logger().error
        log(message)

    def _run_service_callback(self, request, response):
        del request
        response.success, response.message = self._try_run_demo()
        return response

    def _try_run_demo(self):
        if not self.run_lock.acquire(blocking=False):
            return False, "A pick-and-place cycle is already running"

        try:
            if not self._wait_for_dependencies():
                return False, "Motion services did not become ready"
            if not self._execute_pick_place():
                return (
                    False,
                    "Pick-and-place stopped after a failed motion step",
                )
            return True, "Fixed pick-and-place cycle completed"
        finally:
            self.run_lock.release()

    def _wait_for_dependencies(self):
        dependencies = (
            (self.joint_client, "joint trajectory"),
            (self.cartesian_client, "Cartesian motion"),
        )
        for client, label in dependencies:
            self.get_logger().info(f"Waiting for the {label} service")
            if not client.wait_for_service(
                timeout_sec=self.service_timeout_sec
            ):
                self.get_logger().error(
                    f"Timed out waiting for the {label} service"
                )
                return False

        deadline = time.monotonic() + min(10.0, self.service_timeout_sec)
        while self.gripper_pub.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    "No gripper command subscriber detected; continuing anyway"
                )
                break
            time.sleep(0.1)
        return True

    def _execute_pick_place(self):
        self.get_logger().info(
            "Starting with fixed coordinates; received sensor counts are "
            f"images={self.image_count}, point_clouds={self.point_cloud_count}"
        )
        self._command_gripper(
            self.gripper_open_position,
            "Opening gripper",
        )
        self._wait(self.gripper_wait_sec)

        if not self._move_home():
            return False

        pick_pose = self._make_pose(self.pick_position)
        pick_hover = self._offset_pose(pick_pose, self.hover_height)
        lift_pose = self._offset_pose(pick_pose, self.lift_height)
        place_pose = self._make_pose(self.place_position)
        place_hover = self._offset_pose(place_pose, self.hover_height)

        motion_steps = (
            (pick_hover, "Moving to pre-grasp hover"),
            (pick_pose, "Descending to fixed grasp pose"),
        )
        for pose, label in motion_steps:
            if not self._move_cartesian(pose, label):
                return False

        self._command_gripper(
            self.gripper_closed_position,
            "Closing gripper",
        )
        self._wait(self.gripper_wait_sec)

        carry_steps = (
            (lift_pose, "Lifting object"),
            (place_hover, "Transferring above place target"),
            (place_pose, "Descending to fixed place pose"),
        )
        for pose, label in carry_steps:
            if not self._move_cartesian(pose, label):
                return False

        self._command_gripper(
            self.gripper_open_position,
            "Releasing object",
        )
        self._wait(self.gripper_wait_sec)

        if not self._move_cartesian(
            place_hover,
            "Retreating from place target",
        ):
            return False
        return self._move_home()

    def _move_home(self):
        request = SendJointTrajectoryPoint.Request()
        request.goal_point.positions = self.home_joint_positions
        request.goal_point.time_from_start = self._duration_message(
            self.home_move_time_sec
        )
        self.get_logger().info("Moving to the observation configuration")
        return self._call_motion_service(
            self.joint_client,
            request,
            "Observation configuration",
        )

    def _move_cartesian(self, pose, label):
        request = SendPose.Request()
        request.pose = pose
        self.get_logger().info(
            f"{label}: x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
            f"z={pose.position.z:.3f}"
        )
        return self._call_motion_service(self.cartesian_client, request, label)

    def _call_motion_service(self, client, request, label):
        future = client.call_async(request)
        deadline = time.monotonic() + self.motion_timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().error(f"{label} timed out")
                return False
            time.sleep(0.05)

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f"{label} failed")
            return False
        return True

    def _command_gripper(self, position, label):
        command = Float64MultiArray()
        command.data = [position]
        self.gripper_pub.publish(command)
        self.get_logger().info(f"{label}: joint target={position:.3f}")

    def _make_pose(self, position):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self.tool_orientation
        return pose

    @staticmethod
    def _offset_pose(source, z_offset):
        pose = Pose()
        pose.position.x = source.position.x
        pose.position.y = source.position.y
        pose.position.z = source.position.z + z_offset
        pose.orientation = source.orientation
        return pose

    @staticmethod
    def _duration_message(seconds):
        seconds = max(0.0, float(seconds))
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        return Duration(sec=whole_seconds, nanosec=nanoseconds)

    @staticmethod
    def _wait(seconds):
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.05)


def main(args=None):
    """Start the fixed-pose assignment node."""
    rclpy.init(args=args)
    node = FixedPickPlace()
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


if __name__ == "__main__":
    main()
