import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu   # if the IMU uses this type

from .emergency_brake import EmergencyBrake
from .config import ACTIVE_CONTROLLER


class VehicleState:
    """
    Stores the latest state of the vehicle.

    MainDriver updates this object from ROS callbacks.
    Controllers simply read from it.
    """

    def __init__(self):

        # Sensor messages
        self.scan = None
        self.position = None
        self.imu = None

        # Vehicle information
        self.speed = 0.0

class MainDriver(Node):

    def __init__(self):
        super().__init__("main_driver")

        self.get_logger().info("Main Driver Initialized")

        ####################################################
        # Vehicle State
        ####################################################

        self.state = VehicleState()

        self.previous_position = None
        self.previous_time = None

        ####################################################
        # Modules
        ####################################################

        self.emergency_brake = EmergencyBrake()
        self.controller = ACTIVE_CONTROLLER()

        ####################################################
        # Publisher
        ####################################################

        self.steering_pub = self.create_publisher(
            Float32,
            "/autodrive/f1tenth_1/steering_command",
            10
        )

        self.throttle_pub = self.create_publisher(
            Float32,
            "/autodrive/f1tenth_1/throttle_command",
            10
        )

        ####################################################
        # Subscribers
        ####################################################

        self.create_subscription(
            LaserScan,
            "/autodrive/f1tenth_1/lidar",
            self.lidar_callback,
            10
        )

        self.create_subscription(
            Point,
            "/autodrive/f1tenth_1/ips",
            self.ips_callback,
            10
        )

        self.create_subscription(
            Imu,
            "/autodrive/f1tenth_1/imu",
            self.imu_callback,
            10
        )

    ####################################################
    # Callbacks
    ####################################################

    def lidar_callback(self, scan_msg):

        self.state.scan = scan_msg

        # Run the robot every time a new LiDAR scan arrives
        self.run_controller()

    def ips_callback(self, point_msg):

        current_time = self.get_clock().now().nanoseconds / 1e9

        self.state.position = point_msg

        # First IPS reading: just store it
        if self.previous_position is None:
            self.previous_position = point_msg
            self.previous_time = current_time
            return

        # Compute change in position
        dx = point_msg.x - self.previous_position.x
        dy = point_msg.y - self.previous_position.y

        # Compute elapsed time
        dt = current_time - self.previous_time

        # Compute speed (m/s)
        if dt > 0.0:
            distance = math.sqrt(dx**2 + dy**2)
            self.state.speed = distance / dt

            # Optional: Debug print
            # print(f"Speed: {self.state.speed:.2f} m/s")

        # Store current values for the next update
        self.previous_position = point_msg
        self.previous_time = current_time

    def imu_callback(self, imu_msg):

        self.state.imu = imu_msg

    ####################################################
    # Main Control Loop
    ####################################################

    def run_controller(self):

        # Wait until we have a scan
        if self.state.scan is None:
            return

        if self.state.position is None:
            return

        if self.state.imu is None:
            return

        ##############################################
        # Safety First
        ##############################################

        if self.emergency_brake.evaluate(self.state):
            self.publish_brake()
            print("im braking")
            return

        ##############################################
        # Normal Driving
        ##############################################

        speed, steering = self.controller.drive(self.state)

        print("im dribing rn")

        self.publish_drive(speed, steering)

    ####################################################
    # Publishing
    ####################################################

    def publish_drive(self, speed, steering):

        throttle_msg = Float32()
        throttle_msg.data = speed

        steering_msg = Float32()
        steering_msg.data = steering

        self.throttle_pub.publish(throttle_msg)
        self.steering_pub.publish(steering_msg)

    def publish_brake(self):

        throttle_msg = Float32()
        throttle_msg.data = 0.0

        steering_msg = Float32()
        steering_msg.data = 0.0

        self.throttle_pub.publish(throttle_msg)
        self.steering_pub.publish(steering_msg)

####################################################
# Main
####################################################

def main(args=None):

    rclpy.init(args=args)

    node = MainDriver()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()