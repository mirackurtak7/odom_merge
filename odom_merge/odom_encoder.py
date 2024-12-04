#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from math import sin, cos
import time


class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        # Robot parametreleri
        self.declare_parameter('wheel_radius', 0.08255)  # Tekerlek yarıçapı
        self.declare_parameter('wheel_base', 0.3)  # Tekerlekler arası mesafe
        self.declare_parameter('ticks_per_revolution', 240)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').value

        # Encoder değerleri
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        # Robot pozisyon ve yönü
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Abonelikler
        self.left_encoder_sub = self.create_subscription(Int16, 'left_enc', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Int16, 'right_enc', self.right_encoder_callback, 10)

        # Odometry yayıncısı
        self.odom_publisher = self.create_publisher(Odometry, '/odom_encoder', 10)

        # Zamanlama
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20 Hz

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Encoder farklarından tekerleklerin kat ettiği mesafeler
        left_distance = (self.left_ticks - self.last_left_ticks) / self.ticks_per_revolution * (2 * 3.14159 * self.wheel_radius)
        right_distance = (self.right_ticks - self.last_right_ticks) / self.ticks_per_revolution * (2 * 3.14159 * self.wheel_radius)

        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks

        # Ortalama ve yön değişikliği
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # Yeni pozisyon
        self.x += distance * cos(self.theta + delta_theta / 2.0)
        self.y += distance * sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Odometry mesajı
        quaternion = quaternion_from_euler(0, 0, self.theta)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
