#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToArduinoNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_arduino_node')
        
        # Robot parameters (match these with your Arduino code)
        self.wheel_radius = 0.0325  # meters
        self.wheel_base = 0.21     # meters
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Wait for serial connection to establish
            self.get_logger().info("Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise
        
        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info("Node initialized, listening to /cmd_vel topic")

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from Twist message
        linear_vel = msg.linear.x   # m/s
        angular_vel = msg.angular.z # rad/s

        # Calculate wheel velocities for differential drive
        vR = linear_vel + (angular_vel * self.wheel_base) / 2.0
        vL = linear_vel - (angular_vel * self.wheel_base) / 2.0

        # Log the calculated velocities
        self.get_logger().info(f"vL: {vL:.3f} m/s, vR: {vR:.3f} m/s")

        # Send velocities to Arduino
        self.send_to_arduino(vL, vR)

    def send_to_arduino(self, vL, vR):
        try:
            # Format: "left_vel,right_vel\n"
            command = f"{vL:.3f},{vR:.3f}\n"
            self.serial_port.write(command.encode())
            
            # Read response from Arduino (optional)
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode().strip()
                self.get_logger().info(f"Arduino response: {response}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduinoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()