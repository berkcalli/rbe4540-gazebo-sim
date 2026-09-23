"""Two-object scene with opposing fixed RGB-D cameras for perception labs."""

from math import pi

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('ur_simulation_gz')
    # Gazebo cameras look along local +X. Both aim at (0.45, 0, 0.75)
    # from opposite sides of the table, looking down at 45 degrees.
    cameras = [
        ('pc_processing_camera', '/camera', 'pc_camera',
         (0.45, -0.75, 1.50), (0.0, pi / 4, pi / 2)),
        ('pc_processing_camera2', '/camera2', 'pc_camera2',
         (0.45, 0.75, 1.50), (0.0, pi / 4, -pi / 2)),
    ]

    def spawn(model, xyz, rpy=(0.0, 0.0, 0.0)):
        return Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-world', 'empty', '-name', model, '-file',
                PathJoinSubstitution([package, 'models', model, 'model.sdf']),
                '-x', str(xyz[0]), '-y', str(xyz[1]), '-z', str(xyz[2]),
                '-R', str(rpy[0]), '-P', str(rpy[1]), '-Y', str(rpy[2]),
            ],
        )

    def static_tf(parent, child, xyz, rpy):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            output='screen', arguments=[
                '--frame-id', parent, '--child-frame-id', child,
                '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                '--roll', str(rpy[0]), '--pitch', str(rpy[1]),
                '--yaw', str(rpy[2]),
            ],
        )

    camera_nodes = []
    camera_spawns = []
    for model, topic, frame_prefix, xyz, rpy in cameras:
        camera_spawns.append(spawn(model, xyz, rpy))
        camera_nodes.extend([
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                output='screen', arguments=[
                    f'{topic}/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'{topic}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'{topic}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
            ),
            # Harmonic XYZ points use +X forward, +Y left, +Z up even when
            # the RGB-D sensor labels them with its optical frame.
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                output='screen', arguments=[
                    f'{topic}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                ], parameters=[{'override_frame_id': f'{frame_prefix}_link'}],
            ),
            # The existing robot_state_publisher supplies world -> base_link.
            static_tf('world', f'{frame_prefix}_link', xyz, rpy),
            static_tf(f'{frame_prefix}_link', f'{frame_prefix}_optical_frame',
                      (0.0, 0.0, 0.0), (-pi / 2, 0.0, -pi / 2)),
        ])

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                package, 'launch', 'ur_sim_moveit_w_gripper.launch.py',
            ])),
            launch_arguments={'gazebo_gui': LaunchConfiguration('gazebo_gui')}.items(),
        ),
        *camera_nodes,
        TimerAction(period=2.0, actions=[
            spawn('assignment_table', (0.50, 0.0, 0.0)),
            *camera_spawns,
        ]),
        TimerAction(period=3.0, actions=[
            spawn('assignment_object', (0.45, 0.0, 0.84)),
            spawn('pc_processing_cylinder', (0.60, 0.16, 0.83)),
        ]),
    ])
