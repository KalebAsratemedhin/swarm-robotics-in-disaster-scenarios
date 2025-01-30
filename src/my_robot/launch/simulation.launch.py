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

    # Nodes to spawn robots in a circular formation
    spawn_robots = []
    num_robots = 3  # Define the number of robots in the swarm
    radius = 5  # Define the radius of the circle

    for i in range(num_robots):
        robot_name = f"robot{i + 1}"
        robot_urdf = f"robot{i + 1}.urdf"
        robot_urdf_path = os.path.join(pkg_share, 'urdf', robot_urdf)

        # Calculate the robot's position on the circle (polar to Cartesian conversion)
        angle = (2 * math.pi / num_robots) * i  # Evenly distributed angle
        x_position = radius * math.cos(angle)
        y_position = radius * math.sin(angle)

        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'robot_description': open(robot_urdf_path).read()}]
        )

        # Spawn Entity at the calculated position
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', robot_name, '-file', robot_urdf_path, '-x', f"{x_position}", '-y', f"{y_position}", '-z', '0.1'],
            output='screen'
        )

        spawn_robots.extend([robot_state_publisher, spawn_entity])

    # swarm_controller = Node(
    #     package='my_robot',
    #     executable='obstacle_avoidance',  
    #     output='screen'
    # )

    swarm_mover = Node(
        package='my_robot',
        executable='swarm_controller',  
        output='screen'
    )

    return LaunchDescription([declare_world_arg, gazebo_launch,  swarm_mover] + spawn_robots)
