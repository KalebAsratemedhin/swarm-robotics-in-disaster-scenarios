import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.safe_distance = 0.5  # meters

    def scan_callback(self, msg):
        twist = Twist()
        front_scan = msg.ranges[0:30] + msg.ranges[-30:]
        min_distance = min(front_scan)
        
        if min_distance < self.safe_distance:
            # Obstacle detected - rotate
            twist.angular.z = 0.5
            twist.linear.x = 0.0
        else:
            # Clear path - move forward
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

