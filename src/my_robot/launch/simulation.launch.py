import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')

    # Start Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # List of robot names
    robot_names = ["robot1", "robot2", "robot3"]

    # Launch descriptions for each robot
    launch_descriptions = []
    for robot_name in robot_names:
        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_state_publisher',
            output='screen',
            namespace=robot_name,
            parameters=[{'robot_description': open(os.path.join(pkg_share, 'urdf', 'my_robot.xacro')).read()}],
            arguments=[os.path.join(pkg_share, 'urdf', 'my_robot.xacro'), 'robot_name:=' + robot_name]
        )

        # Spawn Entity
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', robot_name, '-topic', f'{robot_name}/robot_description', '-x', str(robot_names.index(robot_name) * 2.0), '-y', '0', '-z', '0.1'],
            output='screen'
        )

        launch_descriptions.extend([robot_state_publisher, spawn_entity])

    return LaunchDescription([gazebo_launch] + launch_descriptions)