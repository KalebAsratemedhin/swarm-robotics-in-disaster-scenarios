#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        self.robot_names = ["robot1", "robot2", "robot3"]
        
        # Velocity publishers for each robot
        self.vel_publishers = {
            name: self.create_publisher(Twist, f'{name}/cmd_vel', 10)
            for name in self.robot_names
        }
        
        # Subscribers for LaserScan data
        self.scan_subscribers = {}
        self.latest_scans = {name: None for name in self.robot_names}
        for name in self.robot_names:
            self.scan_subscribers[name] = self.create_subscription(
                LaserScan,
                f'{name}/scan',
                lambda msg, robot_name=name: self.scan_callback(msg, robot_name),
                10
            )
        
        # Subscribers for Odometry data
        self.odom_subscribers = {}
        self.robot_orientations = {name: None for name in self.robot_names}
        for name in self.robot_names:
            self.odom_subscribers[name] = self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, robot_name=name: self.odom_callback(msg, robot_name),
                10
            )
        
        # Calculate target angles for each robot (radians)
        num_robots = len(self.robot_names)
        self.robot_target_yaw = {
            name: (2 * math.pi * i) / num_robots
            for i, name in enumerate(self.robot_names)
        }
        
        # State management: 'aligning' or 'moving'
        self.robot_states = {name: 'aligning' for name in self.robot_names}
        
        # Timer to update movements
        self.timer = self.create_timer(0.1, self.move_swarm)

    def scan_callback(self, msg, robot_name):
        self.latest_scans[robot_name] = msg

    def odom_callback(self, msg, robot_name):
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = self.euler_from_quaternion(q)
        self.robot_orientations[robot_name] = yaw

    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (0.0, 0.0, yaw)

    def move_swarm(self):
        for name, pub in self.vel_publishers.items():
            twist = self.compute_velocity(name)
            pub.publish(twist)

    def compute_velocity(self, name):
        current_yaw = self.robot_orientations.get(name)
        if current_yaw is None:
            return Twist()  # No odometry data yet

        target_yaw = self.robot_target_yaw[name]
        state = self.robot_states[name]

        if state == 'aligning':
            # Calculate orientation error
            error = target_yaw - current_yaw
            error = (error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

            if abs(error) < 0.1:  # Threshold ~5.7 degrees
                self.robot_states[name] = 'moving'
                return Twist(linear.x=0.5, angular.z=0.0)
            else:
                # Turn towards the target yaw
                angular_z = 0.5 if error > 0 else -0.5
                return Twist(linear.x=0.0, angular.z=angular_z)
        else:
            # Check for obstacles in the front sector
            scan = self.latest_scans.get(name)
            if scan is None:
                return Twist(linear.x=0.5, angular.z=0.0)

            front_angle_min = -0.5236  # -30 degrees in radians
            front_angle_max = 0.5236   # +30 degrees
            obstacle_threshold = 0.5  # meters

            ranges = scan.ranges
            angle_min = scan.angle_min
            angle_increment = scan.angle_increment

            # Calculate indices for front sector
            start_idx = max(0, int((front_angle_min - angle_min) / angle_increment))
            end_idx = min(len(ranges)-1, int((front_angle_max - angle_min) / angle_increment))
            front_ranges = ranges[start_idx:end_idx+1]

            # Filter out invalid (zero) readings
            valid_ranges = [r for r in front_ranges if r > 0]

            if not valid_ranges:
                return Twist(linear.x=0.5, angular.z=0.0)

            min_distance = min(valid_ranges)
            if min_distance < obstacle_threshold:
                # Determine obstacle position and adjust angular velocity
                min_index_in_front = front_ranges.index(min(valid_ranges))
                min_global_index = start_idx + min_index_in_front
                obstacle_angle = angle_min + min_global_index * angle_increment

                if obstacle_angle < 0:
                    angular_z = 0.5  # Turn left
                else:
                    angular_z = -0.5  # Turn right
                return Twist(linear.x=0.2, angular.z=angular_z)
            else:
                return Twist(linear.x=0.5, angular.z=0.0)

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmController()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



