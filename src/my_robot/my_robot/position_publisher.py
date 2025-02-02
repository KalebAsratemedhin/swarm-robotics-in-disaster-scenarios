import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from my_robot.msg import SwarmPosition

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.robot_name = self.get_namespace().strip('/')
        self.subscription = self.create_subscription(
            Odometry,
            'ground_truth',  # Using ground truth for accurate simulation position
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(
            SwarmPosition,
            '/swarm_positions',
            10)
        
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        swarm_msg = SwarmPosition()
        swarm_msg.robot_name = self.robot_name
        swarm_msg.position = Point(x=position.x, y=position.y, z=position.z)
        self.publisher.publish(swarm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()