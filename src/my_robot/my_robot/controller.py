import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class LeaderController(Node):
    def __init__(self):
        super().__init__('leader_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.obstacle_detected = False

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < 1.0

    def control_loop(self):
        cmd = Twist()
        if self.obstacle_detected:
            cmd.angular.z = 0.5
            cmd.linear.x = 0.1
        else:
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

class FollowerController(Node):
    def __init__(self, self_name, leader_name):
        super().__init__('follower_controller')
        self.self_name = self_name
        self.leader_name = leader_name
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.model_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.model_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.leader_pose = None
        self.self_pose = None
        self.obstacle_detected = False

    def model_callback(self, msg):
        try:
            leader_idx = msg.name.index(self.leader_name)
            self.leader_pose = msg.pose[leader_idx]
            self_idx = msg.name.index(self.self_name)
            self.self_pose = msg.pose[self_idx]
        except ValueError:
            pass

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < 1.0

    def control_loop(self):
        if self.leader_pose is None or self.self_pose is None:
            return

        cmd = Twist()
        
        # Obstacle avoidance has priority
        if self.obstacle_detected:
            cmd.linear.x = 0.1
            cmd.angular.z = -0.5
        else:
            # Calculate desired position (1m behind leader)
            leader_pos = self.leader_pose.position
            leader_orient = self.leader_pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([
                leader_orient.x, leader_orient.y, 
                leader_orient.z, leader_orient.w
            ])
            
            desired_x = leader_pos.x - 1.0 * math.cos(yaw)
            desired_y = leader_pos.y - 1.0 * math.sin(yaw)
            
            # Current position
            current_x = self.self_pose.position.x
            current_y = self.self_pose.position.y
            current_orient = self.self_pose.orientation
            _, _, current_yaw = tf_transformations.euler_from_quaternion([
                current_orient.x, current_orient.y,
                current_orient.z, current_orient.w
            ])
            
            # Calculate errors
            dx = desired_x - current_x
            dy = desired_y - current_y
            distance_error = math.hypot(dx, dy)
            angle_error = math.atan2(dy, dx) - current_yaw
            
            # Normalize angle error
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            # Control parameters
            cmd.linear.x = 0.5 * distance_error
            cmd.angular.z = 1.0 * angle_error

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    # Get parameters from command line
    node = Node('temp_node')
    self_name = node.declare_parameter('self_name', 'robot1').value
    leader_name = node.declare_parameter('leader_name', 'robot1').value
    node.destroy_node()
    
    if self_name == leader_name:
        controller = LeaderController()
    else:
        controller = FollowerController(self_name, leader_name)
    
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import math

# class LeaderFollowerController(Node):
#     def __init__(self):
#         super().__init__('leader_follower_controller')

#         self.declare_parameter('robot_name', 'robot1')
#         self.declare_parameter('is_leader', False)

#         self.robot_name = self.get_parameter('robot_name').value
#         self.is_leader = self.get_parameter('is_leader').value

#         self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        
#         if not self.is_leader:
#             self.leader_odom_sub = self.create_subscription(
#                 Odometry, '/robot1/odom', self.leader_callback, 10)
        
#         self.timer = self.create_timer(0.1, self.control_loop)

#         self.leader_x = 0.0
#         self.leader_y = 0.0

#     def leader_callback(self, msg):
#         self.leader_x = msg.pose.pose.position.x
#         self.leader_y = msg.pose.pose.position.y

#     def control_loop(self):
#         cmd = Twist()

#         if self.is_leader:
#             # Move in a straight line
#             cmd.linear.x = 2.5  # Constant forward speed
#             cmd.angular.z = 0.0  # No rotation
#         else:
#             # Calculate distance from leader
#             my_x, my_y = self.get_current_position()
#             distance = math.sqrt((self.leader_x - my_x) ** 2 + (self.leader_y - my_y) ** 2)

#             desired_distance = 1.0  # Maintain 1m behind the leader
#             error = distance - desired_distance

#             # Proportional control for speed adjustment
#             cmd.linear.x = min(max(0.2 * error, 0.0), 0.5)

#         self.cmd_vel_pub.publish(cmd)

#     def get_current_position(self):
#         # Default assumption (modify to use odometry if needed)
#         return (0.0, 0.0)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LeaderFollowerController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


