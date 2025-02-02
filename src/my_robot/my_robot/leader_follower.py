#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('role', 'follower'),
                ('leader_name', 'robot1'),
                ('formation_distance', 1.5),
                ('formation_angle', 30.0),
                ('global_frame', 'map')
            ]
        )
        
        self.role = self.get_parameter('role').value
        self.robot_ns = self.get_namespace().strip('/')
        self.leader_ns = self.get_parameter('leader_name').value
        self.global_frame = self.get_parameter('global_frame').value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Leader components
        if self.role == 'leader':
            self.odom_sub = self.create_subscription(
                Odometry,
                f'odom',
                self.leader_odom_cb,
                10,
                qos_profile=rclpy.qos.qos_profile_sensor_data
            )
            self.goal_pub = self.create_publisher(
                PoseStamped,
                f'goal_pose',
                10
            )
            self.create_timer(1.0, self.publish_goal)
            
        # Follower components
        else:
            self.leader_odom_sub = self.create_subscription(
                Odometry,
                f'/{self.leader_ns}/odom',
                self.leader_odom_cb,
                10,
                qos_profile=rclpy.qos.qos_profile_sensor_data
            )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'cmd_vel',
            10
        )
        
        self.create_timer(0.1, self.update_control)
        self.get_logger().info(f"Initialized {self.robot_ns} as {self.role}")

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.global_frame
        t = self.get_clock().now().nanoseconds * 1e-9
        goal.pose.position.x = 5.0 * math.sin(t)
        goal.pose.position.y = 5.0 * math.cos(t)
        self.goal_pub.publish(goal)

    def leader_odom_cb(self, msg):
        self.leader_pose = msg.pose.pose
        self.leader_twist = msg.twist.twist

    def update_control(self):
        if self.role == 'follower' and hasattr(self, 'leader_pose'):
            try:
                # Get transform through global frame
                transform = self.tf_buffer.lookup_transform(
                    f'{self.robot_ns}/base_link',
                    f'{self.leader_ns}/base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                angle_rad = math.radians(self.get_parameter('formation_angle').value)
                desired_x = self.get_parameter('formation_distance').value * math.cos(angle_rad)
                desired_y = self.get_parameter('formation_distance').value * math.sin(angle_rad)

                error_x = transform.transform.translation.x - desired_x
                error_y = transform.transform.translation.y - desired_y
                
                cmd = Twist()
                cmd.linear.x = 0.5 * error_x + 0.1 * self.leader_twist.linear.x
                cmd.angular.z = 0.8 * error_y + 0.1 * self.leader_twist.angular.z
                
                self.cmd_vel_pub.publish(cmd)

            except TransformException as e:
                self.get_logger().warning(f"Transform error: {str(e)}", throttle_duration_sec=5)
            except Exception as e:
                self.get_logger().error(f"Control error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()