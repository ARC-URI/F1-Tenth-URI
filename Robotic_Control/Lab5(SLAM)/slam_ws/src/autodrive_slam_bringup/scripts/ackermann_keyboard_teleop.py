#!/usr/bin/env python3
"""
ackermann_keyboard_teleop.py

WASD-style keyboard teleoperation that publishes ackermann_msgs/AckermannDriveStamped
on /drive, matching the message type the autodrive_adapter_node expects.

Controls:
  w / s : increase / decrease speed (forward / reverse)
  a / d : steer left / right
  space : zero speed (brake)
  q     : zero steering (straighten wheels)
  x     : quit

Speed and steering are held (latched) until changed, similar to common
F1TENTH teleop conventions, rather than requiring continuous key-holding.
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


HELP = """
Ackermann Keyboard Teleop
-------------------------
  w/s : speed up (fwd) / speed up (rev)
  a/d : steer left / right
  space : zero speed
  q   : zero steering
  +/- : increase / decrease step sizes
  x   : quit
-------------------------
"""


class AckermannKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_keyboard_teleop')

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('speed_step', 0.2)        # m/s per keypress
        self.declare_parameter('steering_step', 0.05)     # rad per keypress
        self.declare_parameter('max_speed', 3.0)          # m/s
        self.declare_parameter('max_steering_angle', 0.4189)  # rad

        drive_topic = self.get_parameter('drive_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.steering_step = float(self.get_parameter('steering_step').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)

        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.speed = 0.0
        self.steering_angle = 0.0

        self.get_logger().info(HELP)

    def publish_drive(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.pub.publish(msg)

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = AckermannKeyboardTeleop()

    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == 'w':
                node.speed = node.clamp(node.speed + node.speed_step, node.max_speed)
            elif key == 's':
                node.speed = node.clamp(node.speed - node.speed_step, node.max_speed)
            elif key == 'a':
                node.steering_angle = node.clamp(
                    node.steering_angle + node.steering_step, node.max_steering_angle)
            elif key == 'd':
                node.steering_angle = node.clamp(
                    node.steering_angle - node.steering_step, node.max_steering_angle)
            elif key == ' ':
                node.speed = 0.0
            elif key == 'q':
                node.steering_angle = 0.0
            elif key == '+':
                node.speed_step *= 1.2
                node.steering_step *= 1.2
            elif key == '-':
                node.speed_step /= 1.2
                node.steering_step /= 1.2
            elif key == 'x' or key == '\x03':  # x or Ctrl-C
                break

            node.publish_drive()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.speed = 0.0
        node.steering_angle = 0.0
        node.publish_drive()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
