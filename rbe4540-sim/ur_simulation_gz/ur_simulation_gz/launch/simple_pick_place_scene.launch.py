"""Launch a self-contained table, object, RGB-D camera, and UR simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the simulation scene used by the first assignment."""
    package_share = FindPackageShare("ur_simulation_gz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "ur_sim_moveit_w_gripper.launch.py"]
            )
        ),
        launch_arguments={"gazebo_gui": gazebo_gui}.items(),
    )

    spawn_table = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            PathJoinSubstitution(
                [package_share, "models", "assignment_table", "model.sdf"]
            ),
            "-name",
            "assignment_table",
            "-x",
            "0.50",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    spawn_object = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            PathJoinSubstitution(
                [package_share, "models", "assignment_object", "model.sdf"]
            ),
            "-name",
            "assignment_object",
            "-x",
            "0.45",
            "-y",
            "0.0",
            "-z",
            "0.84",
        ],
    )

    spawn_camera = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            PathJoinSubstitution(
                [package_share, "urdf", "camera_rgbd.urdf"]
            ),
            "-name",
            "camera_sensor",
            "-x",
            "0.45",
            "-y",
            "0.0",
            "-z",
            "1.8",
        ],
    )

    camera_bridges = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/points@sensor_msgs/msg/PointCloud2"
                "[gz.msgs.PointCloudPacked",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.45",
                "--y",
                "0.0",
                "--z",
                "1.1",
                "--roll",
                "-1.57079632679",
                "--pitch",
                "1.57079632679",
                "--yaw",
                "-1.57079632679",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "camera_sensor/camera_link/camera",
            ],
            output="screen",
        ),
    ]

    scene_entities = TimerAction(
        period=2.0,
        actions=[spawn_table, spawn_camera],
    )
    task_object = TimerAction(
        period=3.0,
        actions=[spawn_object],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo with its graphical interface.",
            ),
            simulation,
            *camera_bridges,
            scene_entities,
            task_object,
        ]
    )
