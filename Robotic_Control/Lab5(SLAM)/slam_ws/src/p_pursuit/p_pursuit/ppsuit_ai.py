import csv
from math import atan2, cos, hypot, sin

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class pure_pursuit:
    def __init__(self):
        with open('centerline.csv', mode='r', encoding='utf-8') as file:
            reader = csv.reader(file)
            next(reader)
            self.path_points = [(float(row[0]), float(row[1])) for row in reader]

        self.K_dd = 1.0
        self.min_ld = 0.5
        self.max_ld = 2.0
        self.current_speed = 1.0
        self.wheelbase = 0.33
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0

    def pose_update(self, position, orientation):
        self.current_position = position
        self.current_orientation = orientation

    def set_speed(self, speed):
        self.current_speed = max(abs(speed), 0.0)

    def compute_lookahead_distance(self):
        return float(np.clip(self.K_dd * self.current_speed, self.min_ld, self.max_ld))

    def find_lookahead_point(self):
        lookahead_distance = self.compute_lookahead_distance()
        if not self.path_points:
            return None

        for point in self.path_points:
            dx = point[0] - self.current_position[0]
            dy = point[1] - self.current_position[1]
            if hypot(dx, dy) >= lookahead_distance:
                return point
        return self.path_points[-1]

    def compute_steering_angle(self):
        lookahead_point = self.find_lookahead_point()
        if lookahead_point is None:
            return None

        dx = lookahead_point[0] - self.current_position[0]
        dy = lookahead_point[1] - self.current_position[1]

        cos_heading = cos(self.current_orientation)
        sin_heading = sin(self.current_orientation)
        x_local = cos_heading * dx + sin_heading * dy
        y_local = -sin_heading * dx + cos_heading * dy

        alpha = atan2(y_local, x_local)
        lookahead_distance = self.compute_lookahead_distance()
        if lookahead_distance <= 1e-6:
            return 0.0

        return atan2(2 * self.wheelbase * sin(alpha), lookahead_distance)


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)
        self.pose_sub_alt = self.create_subscription(
            Odometry,
            '/autodrive/f1tenth_1/pose',
            self.pose_callback,
            10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.pure_pursuit_instance = pure_pursuit()

    def timer_callback(self):
        steering_angle = self.pure_pursuit_instance.compute_steering_angle()
        if steering_angle is None:
            return

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = 1.0
        self.drive_pub.publish(msg)
        self.get_logger().info(
            f'steering_angle={steering_angle:.3f} rad, lookahead={self.pure_pursuit_instance.compute_lookahead_distance():.3f} m'
        )

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        yaw = atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        self.pure_pursuit_instance.pose_update((x, y), yaw)
        self.pure_pursuit_instance.set_speed(abs(msg.twist.twist.linear.x))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    print('Subscriber node has been started. Listening to odometry and publishing /drive.')
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()