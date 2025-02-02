import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import math

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    world_file = os.path.join(pkg_share, 'worlds', 'realistic_rescue.sdf')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Gazebo world file'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ),
        # launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    spawn_robots = []
    num_robots = 3
    radius = 5

    for i in range(num_robots):
        robot_name = f"robot{i+1}"
        robot_urdf = f"robot{i+1}.urdf"
        robot_urdf_path = os.path.join(pkg_share, 'urdf', robot_urdf)

        angle = (2 * math.pi / num_robots) * i
        x_position = radius * math.cos(angle)
        y_position = radius * math.sin(angle)

        # Static transform from map to robot's odom
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_static_tf_publisher',
            arguments=[
                '--x', f'{x_position}',
                '--y', f'{y_position}',
                '--z', '0',
                '--yaw', f'{angle}',
                '--frame-id', 'map',
                '--child-frame-id', f'{robot_name}/odom'
            ]
        )

        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': open(robot_urdf_path).read(),
                'frame_prefix': f'{robot_name}/'
            }]
        )

        # Spawn Entity
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', robot_urdf_path,
                '-x', f"{x_position}",
                '-y', f"{y_position}",
                '-z', '0.1',
                '-robot_namespace', robot_name
            ],
            output='screen'
        )

        # Swarm Controller
        swarm_controller = Node(
            package='my_robot',
            executable='leader_follower',
            name='swarm_controller',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'role': 'leader' if i == 0 else 'follower',
                'leader_name': 'robot1',
                'formation_distance': 1.5,
                'formation_angle': 120.0 * i,
                'global_frame': 'map'
            }]
        )

        spawn_robots.extend([static_tf, robot_state_publisher, spawn_entity, swarm_controller])

    return LaunchDescription([
        declare_world_arg,
        gazebo_launch,
    ] + spawn_robots)