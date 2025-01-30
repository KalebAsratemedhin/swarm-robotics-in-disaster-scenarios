#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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
        
        # Create a timer to send movement commands
        self.timer = self.create_timer(0.1, self.move_swarm)

    def move_swarm(self):
        # Define the velocity command for swarm movement
        # print("Publishing movement command...")
        twist = Twist()
        twist.linear.x = 0.5  # Move forward
        twist.angular.z = 0.0  # No rotation
        
        # Publish the same command to all robots
        for name, pub in self.vel_publishers.items(): 
            # print(f"Publishing to {name}: {twist}") 
            pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmController()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
