#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
import serial
import math
import time
import tf_transformations
import numpy as np

class FusedOdometryNode(Node):
    def __init__(self):
        super().__init__('fused_odometry_node')

        # Robot parameters
        self.wheel_radius = 0.0325  # meters
        self.wheel_base = 0.215     # meters
        self.ticks_per_rev = 1000   # Encoder ticks per wheel revolution
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # State variables
        self.x_enc = 0.0  # Encoder-based position
        self.y_enc = 0.0
        self.theta_enc = 0.0
        self.last_time = self.get_clock().now()
        self.last_enc_time = None
        self.last_amcl_time = None
        self.enc_timeout = 0.5  # seconds
        self.amcl_timeout = 0.5  # seconds

        # Serial port configuration
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            self.ser = None

        # Subscribers
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # Publishers
        self.encoder_odom_pub = self.create_publisher(Odometry, '/encoder_odom', 10)
        self.amcl_odom_pub = self.create_publisher(Odometry, '/amcl_odom', 10)

        # Timer to update odometry (20 Hz)
        self.timer = self.create_timer(0.05, self.update_odometry)

    def amcl_callback(self, msg):
        """Convert AMCL pose to odometry and publish."""
        try:
            # Create odometry message
            odom = Odometry()
            odom.header = msg.header
            odom.header.frame_id = 'odom'  # AMCL pose is in map frame, but we publish relative to odom
            odom.child_frame_id = 'base_link'
            odom.pose.pose = msg.pose.pose
            odom.pose.covariance = msg.pose.covariance
            # Velocity is not provided by AMCL, so leave twist as zero
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0
            odom.twist.covariance = [0.0] * 36  # No velocity covariance

            self.amcl_odom_pub.publish(odom)
            self.last_amcl_time = self.get_clock().now()

            # Log pose for debugging
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            _, _, theta = tf_transformations.euler_from_quaternion(quaternion)
            self.get_logger().debug(f'AMCL odom: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        except Exception as e:
            self.get_logger().error(f"Error processing AMCL pose: {e}")

    def update_encoder_odometry(self, dt):
        """Compute odometry from encoder data."""
        v = 0.0
        omega = 0.0
        try:
            if self.ser and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if ',' in line:
                    left_ticks, right_ticks = map(int, line.split(','))
                    left_distance = left_ticks * self.m_per_tick
                    right_distance = right_ticks * self.m_per_tick
                    v_left = left_distance / dt
                    v_right = right_distance / dt
                    v = (v_right + v_left) / 2.0
                    omega = (v_right - v_left) / self.wheel_base
                    self.x_enc += v * math.cos(self.theta_enc) * dt
                    self.y_enc += v * math.sin(self.theta_enc) * dt
                    self.theta_enc += omega * dt
                    self.theta_enc = math.atan2(math.sin(self.theta_enc), math.cos(self.theta_enc))
                    self.last_enc_time = self.get_clock().now()
                    self.get_logger().debug(f'Encoder odom: x={self.x_enc:.2f}, y={self.y_enc:.2f}, theta={self.theta_enc:.2f}')
        except Exception as e:
            self.get_logger().error(f"Error processing encoder data: {e}")
        return v, omega

    def update_odometry(self):
        """Update and publish encoder odometry."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # seconds

        # Update encoder odometry
        v_enc, omega_enc = self.update_encoder_odometry(dt)

        # Publish encoder odometry
        if self.last_enc_time:
            enc_odom = Odometry()
            enc_odom.header.stamp = current_time.to_msg()
            enc_odom.header.frame_id = 'odom'
            enc_odom.child_frame_id = 'base_link'
            enc_odom.pose.pose.position.x = self.x_enc
            enc_odom.pose.pose.position.y = self.y_enc
            enc_odom.pose.pose.orientation.z = math.sin(self.theta_enc / 2.0)
            enc_odom.pose.pose.orientation.w = math.cos(self.theta_enc / 2.0)
            enc_odom.twist.twist.linear.x = v_enc
            enc_odom.twist.twist.angular.z = omega_enc
            enc_odom.pose.covariance = [0.01] * 36  # Placeholder, tune based on encoder noise
            enc_odom.twist.covariance = [0.01] * 36
            self.encoder_odom_pub.publish(enc_odom)

        # Check source availability
        enc_available = self.last_enc_time and \
                        (self.get_clock().now() - self.last_enc_time).nanoseconds / 1e9 < self.enc_timeout
        amcl_available = self.last_amcl_time and \
                         (self.get_clock().now() - self.last_amcl_time).nanoseconds / 1e9 < self.amcl_timeout

        if not enc_available:
            self.get_logger().warn('Encoder data unavailable')
        if not amcl_available:
            self.get_logger().warn('AMCL data unavailable')

        self.last_time = current_time

    def __del__(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed")

def main(args=None):
    rclpy.init(args=args)
    node = FusedOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Fused Odometry Node')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()