import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Use parameter for safe distance
        self.declare_parameter('safe_distance', 0.5)
        
        # Initialize with proper namespace handling
        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',  # Will resolve to <namespace>/scan
            self.scan_callback,
            10
        )
        
        self.sub_camera = self.create_subscription(
            Image,
            'camera/image_raw',  
            self.image_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Twist, 
            'cmd_vel',  
            10
        )
        
        self.bridge = CvBridge()
        self.red_detected = False
        self.last_scan_time = self.get_clock().now()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define red color ranges
            lower_red = np.array([0, 150, 150])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            
            lower_red = np.array([170, 150, 150])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)
            
            combined_mask = cv2.bitwise_or(mask1, mask2)
            self.red_detected = cv2.countNonZero(combined_mask) > 50
            
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {str(e)}")

    def scan_callback(self, msg):
        # Emergency stop if no recent scans
        if (self.get_clock().now() - self.last_scan_time).nanoseconds > 1e9:
            self.get_logger().warn("No recent scan data!")
            return

        twist = Twist()
        
        if self.red_detected:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            safe_distance = self.get_parameter('safe_distance').value
            ranges = [r if not np.isinf(r) else 100.0 for r in msg.ranges]
            
            # Check 90° front cone (45° each side)
            num_readings = len(ranges)
            front_region = ranges[num_readings//4 : 3*num_readings//4]
            min_distance = min(front_region)
            
            if min_distance < safe_distance:
                twist.angular.z = 0.8 if min(front_region[:len(front_region)//2]) < safe_distance else -0.8
                twist.linear.x = 0.1
            else:
                twist.linear.x = 0.5
                twist.angular.z = 0.0
        
        self.publisher.publish(twist)
        self.last_scan_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()