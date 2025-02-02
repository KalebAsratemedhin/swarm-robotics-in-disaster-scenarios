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

    # Declare world argument
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to the Gazebo world file'
    )

    # Start Gazebo with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )
    
    spawn_robots = []
    num_robots = 3
    radius = 5

    for i in range(num_robots):
        robot_name = f"robot{i + 1}"
        robot_urdf = f"robot{i + 1}.urdf"
        robot_urdf_path = os.path.join(pkg_share, 'urdf', robot_urdf)

        # Calculate position
        angle = (2 * math.pi / num_robots) * i
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        # Existing nodes
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{'robot_description': open(robot_urdf_path).read()}]
        )

        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', robot_name, '-file', robot_urdf_path, '-x', str(x), '-y', str(y), '-z', '0.1'],
            output='screen'
        )

        # Position Publisher Node
        position_publisher = Node(
            package='my_robot',
            executable='position_publisher',
            namespace=robot_name,
            output='screen'
        )

        # Swarm Controller Node
        swarm_controller = Node(
            package='my_robot',
            executable='collision_avoidance_flock',
            namespace=robot_name,
            output='screen'
        )

        spawn_robots.extend([
            robot_state_publisher,
            spawn_entity,
            position_publisher,
            swarm_controller
        ])

    return LaunchDescription([declare_world_arg, gazebo_launch] + spawn_robots)