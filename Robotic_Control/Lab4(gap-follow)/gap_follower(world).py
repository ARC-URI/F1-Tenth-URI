import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


class GapFollowDriver(Node):
    def __init__(self):
        super().__init__('gap_follow_driver')

        # Declare parameters
        self.declare_parameter('speed_min', 1.0)
        self.declare_parameter('speed_max', 3.0)
        self.declare_parameter('bubble_radius', 0.2)  
        self.declare_parameter('preprocess_window', 5)  
        self.declare_parameter('best_point_weight', 0.53)  

        #parameters
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_max = self.get_parameter('speed_max').value
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preprocess_window = self.get_parameter('preprocess_window').value
        self.best_point_weight = self.get_parameter('best_point_weight').value

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

    def preprocess_lidar(self, ranges):
        """Preprocess the LiDAR scan"""
        # Convert to numpy array and handle inf values
        proc_ranges = np.array(ranges)
        proc_ranges[np.isinf(proc_ranges)] = 15.0  # max range

        # Apply moving average filter to reduce noise
        window = self.preprocess_window
        if len(proc_ranges) > window:
            proc_ranges = np.convolve(proc_ranges, np.ones(window)/window, mode='same')

        return proc_ranges

    def find_max_gap(self, ranges):
        """Find the start and end indices of the maximum gap in free space"""
        # Threshold for considering space as free
        threshold = 0.1  # meters

        # Find all indices where distance is above threshold
        free_space = ranges > threshold

        # Find continuous gaps
        gaps = []
        start_idx = None

        for i in range(len(free_space)):
            if free_space[i] and start_idx is None:
                start_idx = i
            elif not free_space[i] and start_idx is not None:
                gaps.append((start_idx, i - 1, i - start_idx))
                start_idx = None

        # Handle case where gap extends to the end
        if start_idx is not None:
            gaps.append((start_idx, len(free_space) - 1, len(free_space) - start_idx))

        # If no gaps found, return middle of scan
        if not gaps:
            mid = len(ranges) // 2
            return mid, mid

        # Find the largest gap
        max_gap = max(gaps, key=lambda x: x[2])
        return max_gap[0], max_gap[1]

    def find_best_point(self, start_i, end_i, ranges):
        """Find the best point in the gap to aim for"""
        # Get the deepest (farthest) point in the gap
        gap_ranges = ranges[start_i:end_i + 1]

        if len(gap_ranges) == 0:
            return (start_i + end_i) // 2

        # Find the index of the maximum distance in the gap
        max_idx = np.argmax(gap_ranges)

        # Weight toward the deepest point but also consider gap center
        deepest_point_idx = start_i + max_idx
        gap_center = (start_i + end_i) // 2
        best_point = int(deepest_point_idx * self.best_point_weight +
                        gap_center * (1 - self.best_point_weight))

        return best_point

    def lidar_callback(self, scan_msg):
        """Process lidar data and publish drive commands"""
        # Preprocess lidar
        ranges = self.preprocess_lidar(scan_msg.ranges)

        # Find closest point to create a safety bubble
        min_dist = np.min(ranges)
        min_idx = np.argmin(ranges)

        # Create safety bubble around closest point
        if min_dist < 0.33:  # If obstacle is closer than 1.5m
            # Calculate bubble radius in terms of array indices
            angle_increment = scan_msg.angle_increment
            bubble_angle = np.arcsin(self.bubble_radius / min_dist) if min_dist > self.bubble_radius else np.pi/.5
            bubble_indices = int(bubble_angle / angle_increment)

            # Set bubble region to zero
            start_bubble = max(0, min_idx - bubble_indices)
            end_bubble = min(len(ranges) - 1, min_idx + bubble_indices)
            ranges[start_bubble:end_bubble + 1] = 0.0

        # Find the max gap
        start_i, end_i = self.find_max_gap(ranges)

        # Find the best point in the gap
        best_point = self.find_best_point(start_i, end_i, ranges)

        # Calculate steering angle
        # Convert index to angle
        angle = scan_msg.angle_min + best_point * scan_msg.angle_increment

        # Calculate speed based on steering angle
        # Slow down for sharp turns, speed up for straight paths
        abs_angle = abs(angle)

        if abs_angle < 0.07:  # Nearly straight
            speed = self.speed_max
        elif abs_angle < 0.3:  # Slight turn
            speed = self.speed_max * 0.8
        elif abs_angle < 0.7:  # Moderate turn
            speed = self.speed_max * 0.6
        else:  # Sharp turn
            speed = self.speed_min

        # Also adjust speed based on gap size
        gap_size = end_i - start_i
        if gap_size < 50:  # Narrow gap
            speed *= 0.7

        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed

        self.drive_pub.publish(drive_msg)
        print('speed', drive_msg)


def main(args=None):
    rclpy.init(args=args)
    driver = GapFollowDriver()

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
