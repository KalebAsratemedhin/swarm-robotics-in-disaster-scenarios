import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String
import os

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.spawn_robots()

    def spawn_robots(self):
        for i in range(self.robot_count):
            robot_namespace = f'robot{i+1}'
            robot_name = f'my_robot{i+1}'
            robot_urdf = f'$(find my_robot)/urdf/{robot_name}.urdf'

            # Create the spawn entity request
            request = SpawnEntity.Request()
            request.name = robot_name
            request.xml = robot_urdf
            request.robot_namespace = robot_namespace  # Set the namespace for each robot
            
            # Call the service to spawn the robot
            future = self.client.call_async(request)
            future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Successfully spawned {response.name}")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn robot: {e}")

def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()
    rclpy.spin(spawner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
