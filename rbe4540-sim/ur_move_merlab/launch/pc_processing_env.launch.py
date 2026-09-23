"""Launch the student point-cloud environment and optional starter node."""

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
            'start_template', default_value='true',
            description='Run the student perception node (no automatic motion).',
        ),
        DeclareLaunchArgument('point_cloud_topic', default_value='/camera/points'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ur_simulation_gz'), 'launch',
                'pc_processing_scene.launch.py',
            ])),
            launch_arguments={'gazebo_gui': LaunchConfiguration('gazebo_gui')}.items(),
        ),
        Node(
            package='ur_move_merlab', executable='ur_move_simple_interface',
            output='screen', parameters=[PathJoinSubstitution([
                FindPackageShare('ur_move_merlab'), 'config',
                'ur_move_simple_interface.yaml',
            ]), {'use_sim_time': True}],
        ),
        Node(
            package='ur_move_merlab', executable='pc_processing_template',
            output='screen', condition=IfCondition(LaunchConfiguration('start_template')),
            parameters=[{
                'use_sim_time': True,
                'point_cloud_topic': LaunchConfiguration('point_cloud_topic'),
            }],
        ),
    ])
