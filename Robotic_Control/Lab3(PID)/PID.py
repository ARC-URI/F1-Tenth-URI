import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import time


class PID_node(Node):
    def __init__(self):
        super().__init__('gap_follow_driver')

        # Declare parameters
        self.declare_parameter('speed_min', 1.0)
        self.declare_parameter('speed_max', 5.0)
        self.declare_parameter('bubble_radius', 0.55)  
        self.declare_parameter('preprocess_window', 5)  
        self.declare_parameter('best_point_weight', 0.83)  

        #parameters
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_max = self.get_parameter('speed_max').value
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preprocess_window = self.get_parameter('preprocess_window').value
        self.best_point_weight = self.get_parameter('best_point_weight').value

        self.left = 0.0
        self.right = 0.0
        self.scan = []

        self.previousTime = 0.0
        self.PrevError = 0.0

        #laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            1
        )

        # Publisher for drive commands
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            1
        )

        self.get_logger().info('Gap Follow Driver started - Racing mode engaged!')
        self.get_logger().info(f'Speed range: {self.speed_min} - {self.speed_max} m/s')

    '''
    def control_callback(self):
        current_time = time.time()
        dt = current_time - self.previousTime
        msg = AckermannDriveStamped()
        
        msg.speed = 0.1
        msg.steering_angle = 0.001 * (self.right - self.left) + 0.05 * ((self.right - self.left) - self.PrevError) #PD CONTROLLER needs to be tuned + (0.001 * self.integral)
        
        self.drive_pub.publish(msg)
        self.PrevError = self.right - self.left
        self.i += 1
    '''

    def lidar_callback(self, scan_msg):
        """Process lidar data and publish drive commands"""
        # Preprocess lidar
        self.scan = scan_msg.ranges
        self.left = min(scan_msg.ranges[180:330])
        self.right = min(scan_msg.ranges[750:900])
        #print(f"left: {self.left}, right: {self.right}")
        print(f"front: {self.scan[540]}")

        current_time = time.time()
        dt = current_time - self.previousTime
        msg = AckermannDriveStamped()
        
        msg.drive.speed = 1.0# - 1.5 * min(max(0.0, self.scan[540]), 1.0)
        msg.drive.steering_angle = 1 * (self.right - self.left) + 1.5 * ((self.right - self.left) - self.PrevError) #PD CONTROLLER needs to be tuned + (0.001 * self.integral)
        
        self.drive_pub.publish(msg)
        self.PrevError = self.right - self.left
   

    
def main(args=None):
    rclpy.init(args=args)
    driver = PID_node()

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    