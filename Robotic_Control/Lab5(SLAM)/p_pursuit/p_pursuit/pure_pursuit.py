import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class pure_pursuit:
    def __init__(self):
        self.lookahead_distance = 1.0  # meters
        self.current_position = (0, 0)  # x, y coordinates
        self.current_orientation = 0.0  # radians
        self.path_points = [(0.6, -1)]  # List of (x, y) tuples representing the path

    def update_position(self, position, orientation):
        self.current_position = position
        self.current_orientation = orientation

    def set_path(self, path_points):
        self.path_points = path_points

    def find_lookahead_point(self):
        for point in self.path_points:
            distance = ((point[0] - self.current_position[0]) ** 2 + (point[1] - self.current_position[1]) ** 2) ** 0.5
            if distance >= self.lookahead_distance:
                return point
        return None

    def compute_steering_angle(self):
        lookahead_point = self.find_lookahead_point()
        if lookahead_point is None:
            return None

        dx = lookahead_point[0] - self.current_position[0]
        dy = lookahead_point[1] - self.current_position[1]
        angle_to_point = atan2(dy, dx)
        steering_angle = angle_to_point - self.current_orientation
        return steering_angle

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.ranges)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    print("Subscriber node has been started. Listening to /autodrive/f1tenth_1/lidar topic...")
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()