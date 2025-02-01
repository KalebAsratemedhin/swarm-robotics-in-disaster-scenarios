#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        # List of robot names
        self.robot_names = ["robot1", "robot2", "robot3", "robot4", "robot5", "robot6", "robot7"]
        
        # Create velocity publishers for each robot
        self.vel_publishers = {
            name: self.create_publisher(Twist, f'{name}/cmd_vel', 10)
            for name in self.robot_names
        }
        
        # Define the radius for the circular formation
        self.radius = 5.0  # Adjust this value as needed
        
        # Create a timer to send movement commands
        self.timer = self.create_timer(0.1, self.move_swarm)

    def move_swarm(self):
        # Loop through each robot and calculate its position
        num_robots = len(self.robot_names)
        for i, name in enumerate(self.robot_names):
            twist = Twist()

            # Calculate the angle for each robot (evenly spaced on the circle)
            angle = 2 * math.pi * i / num_robots

            # Alternate directions: 
            # Half of the robots will move clockwise, the other half counterclockwise
            if i % 2 == 0:
                # Counterclockwise (positive angular velocity)
                twist.angular.z = 0.5  # Counterclockwise rotation
            else:
                # Clockwise (negative angular velocity)
                twist.angular.z = -0.5  # Clockwise rotation

            # Move forward in the circular path
            twist.linear.x = 0.5  # Constant speed for movement along the circle

            # Publish the movement command to the robot
            self.vel_publishers[name].publish(twist)

            # Print the calculated velocities (optional, for debugging)
            self.get_logger().info(f"Publishing to {name}: {twist.linear.x}, {twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmController()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
