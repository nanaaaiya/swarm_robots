#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from communication_msgs.msg import RobotOdom, RobotStatus, Robots, Robot
from threading import Lock

class StatusReceiver(Node):
    def __init__(self):
        super().__init__('status_receiver')
        self.robot_data = {}  # Dictionary to store robot data {robot_id: Robot}
        self.data_lock = Lock()
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            RobotOdom,
            '/robot_odom',
            self.odom_callback,
            10)
        self.status_sub = self.create_subscription(
            RobotStatus,
            '/status_topic',
            self.status_callback,
            10)
        
        # Publisher
        self.all_status_pub = self.create_publisher(
            Robots,
            '/all_robot_status',
            10)
        
        # Timer for periodic publishing (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.05, self.publish_all_status)
        
        self.get_logger().info('Status Receiver Node started')

    def odom_callback(self, msg):
        with self.data_lock:
            robot_id = msg.robot_id
            if robot_id not in self.robot_data:
                self.robot_data[robot_id] = Robot()
                self.robot_data[robot_id].robot_id = robot_id
                self.robot_data[robot_id].available = False  # Default until status received
            
            self.robot_data[robot_id].x = msg.x
            self.robot_data[robot_id].y = msg.y
            self.robot_data[robot_id].theta = msg.theta

    def status_callback(self, msg):
        with self.data_lock:
            robot_id = msg.robot_id
            if robot_id not in self.robot_data:
                self.robot_data[robot_id] = Robot()
                self.robot_data[robot_id].robot_id = robot_id
                self.robot_data[robot_id].x = 0.0
                self.robot_data[robot_id].y = 0.0
                self.robot_data[robot_id].theta = 0.0
            
            self.robot_data[robot_id].available = msg.available

    def publish_all_status(self):
        with self.data_lock:
            robots_msg = Robots()
            robots_msg.robot_status = list(self.robot_data.values())
        
        self.all_status_pub.publish(robots_msg)
        self.get_logger().debug('Published all robot statuses')

def main(args=None):
    rclpy.init(args=args)
    node = StatusReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()