import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from my_robot.msg import SwarmPosition
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.robot_name = self.get_namespace().strip('/')
        
        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.swarm_sub = self.create_subscription(SwarmPosition, '/swarm_positions', self.swarm_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Variables
        self.current_pos = Point()
        self.other_robots = {}
        self.SAFE_DISTANCE = 1.0
        self.MAX_LIN_VEL = 0.5
        self.MAX_ANG_VEL = 1.0

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position

    def swarm_callback(self, msg):
        if msg.robot_name != self.robot_name:
            self.other_robots[msg.robot_name] = msg.position

    def scan_callback(self, msg):
        # Lidar processing
        ranges = [r if not math.isinf(r) else msg.range_max for r in msg.ranges]
        min_dist = min(ranges)
        min_idx = ranges.index(min_dist)
        angle = msg.angle_min + msg.angle_increment * min_idx

        # Calculate repulsions
        lidar_repulsion = self.lidar_repulsion(angle, min_dist)
        robot_repulsion = self.robot_repulsion()

        # Combine vectors
        total_x = lidar_repulsion[0] + robot_repulsion[0]
        total_y = lidar_repulsion[1] + robot_repulsion[1]

        # Determine velocities
        if min_dist < self.SAFE_DISTANCE or robot_repulsion != (0, 0):
            desired_angle = math.atan2(total_y, total_x)
            angular_vel = self.MAX_ANG_VEL * desired_angle
            linear_vel = self.MAX_LIN_VEL * 0.5
        else:
            angular_vel = 0.0
            linear_vel = self.MAX_LIN_VEL

        # Publish cmd_vel
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def lidar_repulsion(self, angle, distance):
        if distance >= self.SAFE_DISTANCE:
            return (0.0, 0.0)
        strength = 1.0 / (distance + 0.1)
        return (
            strength * math.cos(angle + math.pi),
            strength * math.sin(angle + math.pi)
        )

    def robot_repulsion(self):
        total_x, total_y = 0.0, 0.0
        for name, pos in self.other_robots.items():
            dx = self.current_pos.x - pos.x
            dy = self.current_pos.y - pos.y
            dist = math.hypot(dx, dy)
            if dist < self.SAFE_DISTANCE:
                strength = 1.0 / (dist + 0.1)
                total_x += (dx / dist) * strength
                total_y += (dy / dist) * strength
        return (total_x, total_y)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()