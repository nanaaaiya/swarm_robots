#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from communication_msgs.msg import RobotOdom  # Adjust based on your package
from geometry_msgs.msg import Point, Quaternion
import tf_transformations  # For quaternion to Euler conversion

class OdomToRobotOdom(Node):
    def __init__(self):
        super().__init__('odom_to_robot_odom')
        
        # Declare and get robot_id parameter (default: 0)
        self.declare_parameter('robot_id', 0)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        
        # Subscriber to /odom topic
        self.odom_sub = self.create_subscription(
            Odometry,     
            '/odom',
            self.odom_callback,
            10)
        
        # Publisher for /robot_odom topic
        self.robot_odom_pub = self.create_publisher(
            RobotOdom,
            '/robot_odom',
            10)
        
        self.get_logger().info(f'Odom to RobotOdom Node started for robot_id: {self.robot_id}')

    def odom_callback(self, msg):
        # Create RobotOdom message
        robot_odom = RobotOdom()
        robot_odom.robot_id = self.robot_id
        robot_odom.x = msg.pose.pose.position.x
        robot_odom.y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw (theta)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        robot_odom.theta = euler[2]  # Yaw angle (rotation around Z-axis)
        
        # Publish RobotOdom message
        self.robot_odom_pub.publish(robot_odom)
        self.get_logger().debug(f'Published RobotOdom for robot_id: {self.robot_id}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToRobotOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()