#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
import tf_transformations
import math

class WaypointNavigatorNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_node')
        
        # Declare parameters
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('waypoints', [])  # List of [x, y] coordinates
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.waypoints = self.get_parameter('waypoints').get_parameter_value().double_array_value
        
        # Validate waypoints (should be pairs of x, y coordinates)
        if len(self.waypoints) % 2 != 0 or not self.waypoints:
            self.get_logger().error('Invalid waypoints parameter: must be a list of [x, y] coordinates')
            raise ValueError('Invalid waypoints parameter')
        
        # Convert waypoints to list of (x, y) tuples
        self.waypoint_list = [(self.waypoints[i], self.waypoints[i+1]) for i in range(0, len(self.waypoints), 2)]
        self.current_waypoint_index = 0
        self.get_logger().info(f'Loaded {len(self.waypoint_list)} waypoints for Robot {self.robot_id}: {self.waypoint_list}')
        
        # Action client for Nav2
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            raise RuntimeError('Nav2 action server not available')
        
        # Start navigation
        if self.waypoint_list:
            self.send_next_waypoint()
        else:
            self.get_logger().warn('No waypoints provided, node will shut down')
            rclpy.shutdown()

    def send_next_waypoint(self):
        """Send the next waypoint to Nav2."""
        if self.current_waypoint_index >= len(self.waypoint_list):
            self.get_logger().info(f'All waypoints completed for Robot {self.robot_id}')
            rclpy.shutdown()
            return
        
        x, y = self.waypoint_list[self.current_waypoint_index]
        
        # Compute orientation: face the next waypoint or default to 0.0
        theta = 0.0
        if self.current_waypoint_index < len(self.waypoint_list) - 1:
            next_x, next_y = self.waypoint_list[self.current_waypoint_index + 1]
            dx = next_x - x
            dy = next_y - y
            theta = math.atan2(dy, dx)
        
        # Create PoseStamped for the waypoint
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_pose.pose.orientation = self.angle_to_quaternion(theta)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f'Sending waypoint {self.current_waypoint_index + 1}/{len(self.waypoint_list)} '
                              f'({x:.2f}, {y:.2f}) to Nav2 for Robot {self.robot_id}')
        
        # Send goal to Nav2
        self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle Nav2 goal response."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Nav2 goal rejected for Robot {self.robot_id}')
                self.current_waypoint_index += 1
                self.send_next_waypoint()
                return
            self.get_logger().info(f'Nav2 goal accepted for Robot {self.robot_id}')
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f'Error processing Nav2 goal response: {e}')
            self.current_waypoint_index += 1
            self.send_next_waypoint()

    def feedback_callback(self, feedback_msg):
        """Log navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Nav2 feedback for Robot {self.robot_id}: '
                               f'distance remaining: {feedback.distance_remaining:.2f} m')

    def result_callback(self, future):
        """Handle Nav2 goal result."""
        try:
            result = future.result()
            if result.status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached for Robot {self.robot_id}')
                self.current_waypoint_index += 1
                self.send_next_waypoint()
            else:
                self.get_logger().error(f'Navigation failed for Robot {self.robot_id}, status: {result.status}')
                self.current_waypoint_index += 1
                self.send_next_waypoint()
        except Exception as e:
            self.get_logger().error(f'Error processing Nav2 result: {e}')
            self.current_waypoint_index += 1
            self.send_next_waypoint()

    def angle_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

def main():
    rclpy.init()
    node = WaypointNavigatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        node.get_logger().info(f'Shutting down Waypoint Navigator Node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()