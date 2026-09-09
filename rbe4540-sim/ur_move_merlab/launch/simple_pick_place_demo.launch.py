"""Launch the first-assignment pick-and-place demonstration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the complete simulation and fixed-pose task launch."""
    auto_start = LaunchConfiguration("auto_start")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    image_topic = LaunchConfiguration("image_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    task_node_delay = LaunchConfiguration("task_node_delay")

    task_config = PathJoinSubstitution(
        [
            FindPackageShare("ur_move_merlab"),
            "config",
            "simple_pick_place.yaml",
        ]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_simulation_gz"),
                    "launch",
                    "simple_pick_place_scene.launch.py",
                ]
            )
        ),
        launch_arguments={"gazebo_gui": gazebo_gui}.items(),
    )

    motion_interface = Node(
        package="ur_move_merlab",
        executable="ur_move_simple_interface",
        output="screen",
        parameters=[task_config, {"use_sim_time": True}],
    )

    pick_place_demo = TimerAction(
        period=task_node_delay,
        actions=[
            Node(
                package="ur_move_merlab",
                executable="pick_place_demo",
                name="pick_place_demo",
                output="screen",
                parameters=[
                    task_config,
                    {
                        "auto_start": ParameterValue(
                            auto_start,
                            value_type=bool,
                        ),
                        "image_topic": image_topic,
                        "point_cloud_topic": point_cloud_topic,
                        "start_delay_sec": ParameterValue(
                            start_delay_sec,
                            value_type=float,
                        ),
                        "use_sim_time": True,
                    },
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description=(
                    "Run one fixed pick-and-place cycle automatically."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo with its graphical interface.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image",
                description=(
                    "Image topic cached for later perception assignments."
                ),
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="/camera/points",
                description=(
                    "Point-cloud topic cached for later perception "
                    "assignments."
                ),
            ),
            DeclareLaunchArgument(
                "start_delay_sec",
                default_value="2.0",
                description="Delay before the task requests its first motion.",
            ),
            DeclareLaunchArgument(
                "task_node_delay",
                default_value="2.0",
                description="Delay before starting the assignment task node.",
            ),
            simulation,
            motion_interface,
            pick_place_demo,
        ]
    )
