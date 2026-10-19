#!/usr/bin/env python3
"""
autodrive_adapter_node.py

Bridges the AutoDRIVE Simulator F1TENTH ROS 2 topics to a standard ROS 2
navigation/SLAM stack.

Responsibilities:
  1. Republish AutoDRIVE's LiDAR scan as a clean sensor_msgs/LaserScan on
     /scan (what slam_toolbox expects).
  2. Publish nav_msgs/Odometry on /odom (+ TF odom -> base_link) directly
     from AutoDRIVE's ground-truth IPS position and IMU orientation, with
     forward speed from the wheel encoders and yaw rate from the IMU in the
     twist (the same fields the real car's VESC odometry fills in). This
     is a simulator, and AutoDRIVE explicitly provides ground-truth pose
     for this reason -- using it sidesteps dead-reckoning drift entirely,
     and (unlike integrating commanded throttle/steering) it tracks the
     vehicle correctly no matter how it's actually being driven, including
     via the simulator's own native controls rather than this node's /drive
     topic.
  3. Subscribe to ackermann_msgs/AckermannDriveStamped on /drive, convert
     speed + steering_angle into AutoDRIVE's normalized throttle/steering
     command topics, for callers that do want to drive via ROS.

All topic names and vehicle geometry are parameterized -- see
config/topics.yaml. Edit that file, not this one, if your AutoDRIVE
install uses different topic names.
"""

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions

from collections import deque

from sensor_msgs.msg import LaserScan, Imu, JointState
from rclpy.time import Time
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point
from ackermann_msgs.msg import AckermannDriveStamped
import tf2_ros


class AutoDriveAdapterNode(Node):
    def __init__(self):
        super().__init__('autodrive_adapter_node')

        # ---------------- Parameters (see config/topics.yaml) ----------------
        self.declare_parameter('autodrive_lidar_topic', '/autodrive/f1tenth_1/lidar')
        self.declare_parameter('autodrive_imu_topic', '/autodrive/f1tenth_1/imu')
        self.declare_parameter('autodrive_ips_topic', '/autodrive/f1tenth_1/ips')
        self.declare_parameter('autodrive_left_encoder_topic', '/autodrive/f1tenth_1/left_encoder')
        self.declare_parameter('autodrive_right_encoder_topic', '/autodrive/f1tenth_1/right_encoder')

        self.declare_parameter('autodrive_throttle_topic', '/autodrive/f1tenth_1/throttle_command')
        self.declare_parameter('autodrive_steering_topic', '/autodrive/f1tenth_1/steering_command')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('laser_frame', 'laser')

        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_throttle', 1.0)
        self.declare_parameter('wheel_radius', 0.059)
        self.declare_parameter('speed_at_full_throttle', 3.7)
        self.declare_parameter('drive_timeout', 0.5)
        self.declare_parameter('publish_tf', True)

        gp = self.get_parameter
        self.lidar_topic = gp('autodrive_lidar_topic').value
        self.imu_topic = gp('autodrive_imu_topic').value
        self.ips_topic = gp('autodrive_ips_topic').value

        self.throttle_topic = gp('autodrive_throttle_topic').value
        self.steering_topic = gp('autodrive_steering_topic').value

        self.scan_topic = gp('scan_topic').value
        self.odom_topic = gp('odom_topic').value
        self.drive_topic = gp('drive_topic').value

        self.base_frame = gp('base_frame').value
        self.odom_frame = gp('odom_frame').value
        self.laser_frame = gp('laser_frame').value

        self.max_steering_angle = float(gp('max_steering_angle').value)
        self.max_throttle = float(gp('max_throttle').value)
        self.wheel_radius = float(gp('wheel_radius').value)
        self.speed_at_full_throttle = float(gp('speed_at_full_throttle').value)
        self.drive_timeout = float(gp('drive_timeout').value)
        self.last_drive_time = None     # s, when the last /drive command arrived
        self.publish_tf = bool(gp('publish_tf').value)

        # ---------------- State: latest ground-truth pose from AutoDRIVE ----------------
        self.origin = None              # (x0, y0) -- first IPS reading, so odom starts at 0,0
        self.latest_position = None     # geometry_msgs/Point, world frame
        self.latest_orientation = None  # geometry_msgs/Quaternion, from IMU
        self.yaw_rate = 0.0             # rad/s, from IMU
        self.speed = 0.0                # m/s forward, from wheel encoders
        self.left_angle = None          # rad, latest left encoder reading
        self.wheel_history = deque()    # (time in s, mean wheel angle in rad) over the last ~0.6 s

        # ---------------- QoS ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Reliable so both slam_toolbox (best-effort reader, compatible with
        # a reliable writer) and rviz2's default LaserScan/Map displays
        # (which request reliable) can both subscribe without a QoS mismatch.
        scan_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---------------- Publishers (normalized, standard ROS 2 topics) ----------------
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, scan_pub_qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # ---------------- Publishers (AutoDRIVE native actuation topics) ----------------
        self.throttle_pub = self.create_publisher(Float32, self.throttle_topic, 10)
        self.steering_pub = self.create_publisher(Float32, self.steering_topic, 10)

        # ---------------- Subscribers (AutoDRIVE native sensor topics) ----------------
        # NOTE: AutoDRIVE's LiDAR message is consumed here as a LaserScan directly.
        # If your installed bridge instead publishes a custom message type, you
        # will need to adjust the msg type import + callback accordingly -- run
        # `ros2 topic info <topic> -v` to confirm the type and adjust here.
        self.create_subscription(LaserScan, self.lidar_topic, self.on_lidar, sensor_qos)
        self.create_subscription(Point, self.ips_topic, self.on_ips, sensor_qos)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, sensor_qos)
        self.create_subscription(
            JointState, gp('autodrive_left_encoder_topic').value, self.on_left_encoder, sensor_qos)
        self.create_subscription(
            JointState, gp('autodrive_right_encoder_topic').value, self.on_right_encoder, sensor_qos)

        # ---------------- Subscriber: drive commands (standard Ackermann message) ----------------
        self.create_subscription(AckermannDriveStamped, self.drive_topic, self.on_drive_cmd, 10)

        # ---------------- TF broadcaster ----------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # The AutoDRIVE bridge keeps applying the last throttle it received, so
        # stop the car ourselves when /drive commands stop arriving.
        self.create_timer(0.1, self.drive_watchdog)

        # Publish odometry at a fixed rate from the latest ground-truth pose.
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)  # 50 Hz

        self.get_logger().info(
            f'AutoDRIVE adapter ready.\n'
            f'  LiDAR:   {self.lidar_topic} -> {self.scan_topic}\n'
            f'  Drive:   {self.drive_topic} -> throttle={self.throttle_topic}, '
            f'steering={self.steering_topic}\n'
            f'  Odom:    ground-truth IPS/IMU -> {self.odom_topic} '
            f'(+ TF {self.odom_frame} -> {self.base_frame})'
        )

    # -------------------------------------------------------------------
    # LiDAR passthrough / re-stamping
    # -------------------------------------------------------------------
    def on_lidar(self, msg: LaserScan):
        # Re-stamp with our laser frame so the TF tree is self-consistent,
        # then republish on the normalized /scan topic that slam_toolbox uses.
        msg.header.frame_id = self.laser_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)

    # -------------------------------------------------------------------
    # Ground-truth pose feed (IPS position + IMU orientation)
    # -------------------------------------------------------------------
    def on_ips(self, msg: Point):
        if self.origin is None:
            self.origin = (msg.x, msg.y)
        self.latest_position = msg

    def on_imu(self, msg: Imu):
        self.latest_orientation = msg.orientation
        self.yaw_rate = msg.angular_velocity.z

    # -------------------------------------------------------------------
    # Forward speed from the wheel encoders
    # -------------------------------------------------------------------
    def on_left_encoder(self, msg: JointState):
        self.left_angle = msg.position[0]

    def on_right_encoder(self, msg: JointState):
        # The bridge publishes left then right from the same simulator sample.
        if self.left_angle is None:
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        angle = 0.5 * (self.left_angle + msg.position[0])
        # Samples arrive slowly and unevenly spaced, so difference over a
        # window instead of consecutive samples.
        history = self.wheel_history
        history.append((t, angle))
        while len(history) > 2 and t - history[1][0] >= 0.6:
            history.popleft()
        t0, angle0 = history[0]
        if t - t0 > 1e-3:
            self.speed = self.wheel_radius * (angle - angle0) / (t - t0)

    # -------------------------------------------------------------------
    # Drive command -> AutoDRIVE throttle/steering commands
    # -------------------------------------------------------------------
    def on_drive_cmd(self, msg: AckermannDriveStamped):
        speed = msg.drive.speed                      # m/s, signed (forward/reverse)
        steering_angle = msg.drive.steering_angle      # rad, virtual bicycle-model angle

        # Clamp to physical steering limits.
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))

        # AutoDRIVE expects normalized commands in [-1, 1], and its throttle
        # acts as a speed setpoint: the car settles at about
        # speed_at_full_throttle * throttle within ~0.3 s.
        throttle_cmd = speed / self.speed_at_full_throttle
        throttle_cmd = max(-self.max_throttle, min(self.max_throttle, throttle_cmd))
        steering_cmd = steering_angle / self.max_steering_angle  # -> [-1, 1]

        self.throttle_pub.publish(Float32(data=float(throttle_cmd)))
        self.steering_pub.publish(Float32(data=float(steering_cmd)))
        self.last_drive_time = self.get_clock().now().nanoseconds * 1e-9

    def drive_watchdog(self):
        if self.last_drive_time is None:
            return
        if self.get_clock().now().nanoseconds * 1e-9 - self.last_drive_time > self.drive_timeout:
            self.stop_vehicle()
            self.last_drive_time = None

    def stop_vehicle(self):
        self.throttle_pub.publish(Float32(data=0.0))
        self.steering_pub.publish(Float32(data=0.0))

    # -------------------------------------------------------------------
    # Odometry from AutoDRIVE's ground-truth IPS position + IMU orientation
    # -------------------------------------------------------------------
    def publish_odometry(self):
        if self.latest_position is None or self.latest_orientation is None:
            return  # haven't received a full pose yet

        now = self.get_clock().now()
        x0, y0 = self.origin
        x = self.latest_position.x - x0
        y = self.latest_position.y - y0

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.latest_orientation
        odom_msg.twist.twist.linear.x = self.speed
        odom_msg.twist.twist.angular.z = self.yaw_rate

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = self.latest_orientation
            self.tf_broadcaster.sendTransform(t)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Handle Ctrl+C and termination ourselves so the stop command below still goes out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = AutoDriveAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Repeat briefly: a single message sent just before exit can be lost.
        for _ in range(5):
            node.stop_vehicle()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
