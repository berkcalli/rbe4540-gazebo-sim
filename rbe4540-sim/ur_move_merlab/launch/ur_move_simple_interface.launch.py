"""Start the simulation and its joint and Cartesian service interface."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'start_simulation', default_value='true',
            description='Start the scene and MoveIt along with the interface.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ur_move_merlab'), 'config',
                'ur_move_simple_interface.yaml',
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ur_simulation_gz'), 'launch',
                'simple_pick_place_scene.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('start_simulation')),
            launch_arguments={
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            }.items(),
        ),
        Node(
            package='ur_move_merlab', executable='ur_move_simple_interface',
            parameters=[LaunchConfiguration('config_file'), {'use_sim_time': True}],
            output='screen',
        ),
    ])
