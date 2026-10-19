"""Drive an F1TENTH car with the policy trained in AI/reinforcement learning/arc_rl.py.

Uses the standard F1TENTH topics, so the same node runs on the physical car
and (through autodrive_adapter_node) in the AutoDRIVE simulator:

    /scan  sensor_msgs/LaserScan                  in
    /odom  nav_msgs/Odometry (twist.linear.x)     in
    /drive ackermann_msgs/AckermannDriveStamped   out

The weights file carries the observation/action contract it was trained with,
so nothing about the LiDAR layout or speed range is configured here.
"""

import json
import os
import signal
import time

import numpy as np

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class Policy:
    """The exported actor MLP and its contract. Needs only NumPy."""

    def __init__(self, path):
        data = np.load(path)
        self.contract = json.loads(str(data['contract']))
        n_layers = sum(1 for k in data.files if k.startswith('w'))
        self.layers = [(data[f'w{i}'], data[f'b{i}']) for i in range(n_layers)]

        c = self.contract
        self.ray_angles = np.linspace(-c['lidar_fov'] / 2, c['lidar_fov'] / 2, c['lidar_num_rays'])
        self.prev_action = np.zeros(2)

    def act(self, obs):
        x = obs
        for i, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if i < len(self.layers) - 1:
                x = np.tanh(x)
        return np.clip(x, -1.0, 1.0)

    def step(self, ranges, speed):
        """LiDAR ranges at `ray_angles` (m) and forward speed (m/s) -> steering angle (rad), speed (m/s)."""
        c = self.contract
        lidar = np.clip(ranges, 0.0, c['lidar_max_range']) / c['lidar_max_range']
        speed_obs = np.clip(speed / c['speed_norm'], -1.0, 1.0)
        obs = np.concatenate([lidar, [speed_obs], self.prev_action]).astype(np.float32)

        action = self.act(obs)
        self.prev_action = action
        steering_angle = float(action[0] * c['max_steer'])
        target_speed = float(c['min_speed'] + (action[1] + 1.0) / 2.0 * (c['max_speed'] - c['min_speed']))
        return steering_angle, target_speed


def sample_scan(ranges, angle_min, angle_increment, range_min, ray_angles, max_range, yaw_offset=0.0):
    """Range (m) along each of `ray_angles`, taken as the median of the scan beams around it.

    `ray_angles` are relative to straight ahead, positive to the left.
    `yaw_offset` is the scan angle that points straight ahead (0 for a
    forward-mounted LiDAR). Readings past the sensor's range (+inf) count as
    `max_range`; invalid beams (NaN, zero, below range_min) are ignored, and a
    ray with no valid beam at all returns `max_range`.
    """
    r = np.asarray(ranges, dtype=np.float64)
    n = len(r)
    r = np.where(np.isposinf(r), max_range, r)
    r = np.where(np.isfinite(r) & (r >= max(range_min, 1e-3)), r, np.nan)

    centers = np.round(((ray_angles + yaw_offset - angle_min) % (2 * np.pi)) / angle_increment).astype(int)
    ray_spacing = abs(ray_angles[1] - ray_angles[0])
    half_window = max(0, int(ray_spacing / angle_increment / 2))
    idx = centers[:, None] + np.arange(-half_window, half_window + 1)[None, :]

    full_circle = n * angle_increment > 2 * np.pi - 2 * angle_increment
    if full_circle:
        idx = idx % n
    in_scan = (idx >= 0) & (idx < n)
    window = np.where(in_scan, r[np.clip(idx, 0, n - 1)], np.nan)

    has_valid = np.isfinite(window).any(axis=1)
    out = np.full(len(ray_angles), max_range)
    out[has_valid] = np.nanmedian(window[has_valid], axis=1)
    return np.minimum(out, max_range)


class PolicyNode(Node):

    def __init__(self):
        super().__init__('rl_policy_node')

        self.declare_parameter('weights_path', '')
        self.declare_parameter('max_speed', 1.0)          # m/s cap on the policy's speed; raise once it drives well
        self.declare_parameter('lidar_yaw_offset', 0.0)   # rad; scan angle that points straight ahead
        self.declare_parameter('control_rate', 0.0)       # Hz, upper limit; 0 uses the rate the policy was trained at
        self.declare_parameter('sensor_timeout', 0.5)     # s without a scan or odometry before stopping
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')

        weights_path = self.get_parameter('weights_path').value or os.path.join(
            get_package_share_directory('rl_policy'), 'models', 'f1tenth_policy.npz')
        self.policy = Policy(weights_path)

        self.max_speed = self.get_parameter('max_speed').value
        self.lidar_yaw_offset = self.get_parameter('lidar_yaw_offset').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        rate = self.get_parameter('control_rate').value or self.policy.contract['control_rate']
        self.min_period = 0.9 / rate

        self.scan_time = None
        self.speed = 0.0
        self.odom_time = None
        self.decision_time = 0.0

        self.pub_drive = self.create_publisher(AckermannDriveStamped, self.get_parameter('drive_topic').value, 1)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'Loaded {weights_path}; control at up to {rate:.1f} Hz, speed capped at {self.max_speed:.2f} m/s. '
            f'Contract: {self.policy.contract}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg):
        self.speed = msg.twist.twist.linear.x
        self.odom_time = self.now()

    def stale_sensors(self):
        now = self.now()
        return [name for name, t in (('scan', self.scan_time), ('odometry', self.odom_time))
                if t is None or now - t > self.sensor_timeout]

    def watchdog(self):
        stale = self.stale_sensors()
        if stale:
            self.get_logger().warn(f'No recent {" or ".join(stale)}; stopping', throttle_duration_sec=2.0)
            self.policy.prev_action = np.zeros(2)
            self.publish_drive(0.0, 0.0)

    def scan_callback(self, scan):
        # One decision per scan, at most control_rate times a second: a fast
        # LiDAR is decimated to the trained rate, a slow one sets the pace.
        self.scan_time = self.now()
        if self.scan_time - self.decision_time < self.min_period or self.stale_sensors():
            return
        self.decision_time = self.scan_time

        ranges = sample_scan(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
            self.policy.ray_angles, self.policy.contract['lidar_max_range'], self.lidar_yaw_offset)
        steering_angle, speed = self.policy.step(ranges, self.speed)
        speed = min(speed, self.max_speed)
        self.publish_drive(steering_angle, speed)

        self.get_logger().info(
            f'lidar R/C/L {ranges[0]:.2f}/{ranges[len(ranges) // 2]:.2f}/{ranges[-1]:.2f} m, '
            f'speed {self.speed:.2f} m/s -> steer {steering_angle:+.2f} rad, speed {speed:.2f} m/s',
            throttle_duration_sec=1.0)

    def publish_drive(self, steering_angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.pub_drive.publish(msg)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Handle Ctrl+C and termination ourselves so the stop command below still goes out;
    # whatever executes /drive keeps applying the last command it received.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Repeat briefly: a single message sent just before exit can be lost.
        for _ in range(5):
            node.publish_drive(0.0, 0.0)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
