#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from communication_msgs.msg import RobotStatus, TaskAllocationArray, TaskAllocation, Task
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
import tf_transformations
import math
import time

class NavigationProcessNode(Node):
    def __init__(self):
        super().__init__('navigation_process_node')
        
        # Declare parameter for robot_id
        self.declare_parameter('robot_id', 0)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        
        # Initialize storage
        self.tasks = {}  # {task_id: {pickup_x, pickup_y, dropoff_x, dropoff_y, available}}
        self.current_pose = None  # (x, y, theta)
        self.current_task_ids = []  # Track active task IDs
        self.current_waypoints = []  # List of PoseStamped for current tasks
        self.current_waypoint_index = 0  # Index of current waypoint
        
        # Subscribers
        self.alloc_sub = self.create_subscription(
            TaskAllocationArray,
            '/task_allocations',
            self.task_allocation_callback,
            10
        )
        self.task_sub = self.create_subscription(
            Task,
            '/task_locations',
            self.task_location_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.robot_status_pub = self.create_publisher(
            RobotStatus,
            '/status_topic',
            10
        )
        self.task_pub = self.create_publisher(
            Task,
            '/task_locations',
            10
        )
        
        # Action client for Nav2
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
        
        # Publish initial robot status (available)
        self.publish_robot_status(available=True)
        self.get_logger().info(f'Navigation Process Node started for Robot {self.robot_id}')
    
    def odom_callback(self, msg):
        """Store the robot's current position and orientation from odometry."""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            _, _, theta = tf_transformations.euler_from_quaternion(quaternion)
            self.current_pose = (x, y, theta)
            self.get_logger().debug(f'Robot {self.robot_id} odometry: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
            
            # Check if current waypoint is reached
            if self.current_waypoints and self.current_waypoint_index < len(self.current_waypoints):
                self.check_waypoint_reached()
        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {e}')

    def task_location_callback(self, msg):
        """Store task data from /task_locations."""
        self.tasks[msg.task_id] = {
            'pickup_x': msg.pickup_x,
            'pickup_y': msg.pickup_y,
            'dropoff_x': msg.dropoff_x,
            'dropoff_y': msg.dropoff_y,
            'available': msg.available
        }
        self.get_logger().debug(f'Task {msg.task_id} updated: pickup=({msg.pickup_x}, {msg.pickup_y}), '
                               f'dropoff=({msg.dropoff_x}, {msg.dropoff_y}), available={msg.available}')

    def task_allocation_callback(self, msg):
        """Process task allocations and send waypoints to Nav2."""
        for allocation in msg.tasks:
            if allocation.robot_id == self.robot_id and allocation.task_ids:
                self.get_logger().info(f'Received task allocation for Robot {self.robot_id}: Tasks {allocation.task_ids}')
                
                # Update robot status to unavailable
                self.publish_robot_status(available=False)
                
                # Store task IDs and create waypoints
                self.current_task_ids = allocation.task_ids
                self.current_waypoints = []
                self.current_waypoint_index = 0
                
                for i, task_id in enumerate(allocation.task_ids):
                    task = self.tasks.get(task_id)
                    if not task:
                        self.get_logger().warn(f'Task {task_id} not found in task storage')
                        continue
                    
                    # Compute orientation: face the next waypoint or use current theta
                    theta = self.current_pose[2] if self.current_pose else 0.0
                    if i < len(allocation.task_ids) - 1:  # Face next pickup if available
                        next_task = self.tasks.get(allocation.task_ids[i + 1])
                        if next_task:
                            dx = next_task['pickup_x'] - task['dropoff_x']
                            dy = next_task['pickup_y'] - task['dropoff_y']
                            theta = math.atan2(dy, dx)
                    
                    # Pickup waypoint
                    pickup_pose = PoseStamped()
                    pickup_pose.header.frame_id = 'map'
                    pickup_pose.header.stamp = self.get_clock().now().to_msg()
                    pickup_pose.pose.position = Point(x=task['pickup_x'], y=task['pickup_y'], z=0.0)
                    pickup_pose.pose.orientation = self.angle_to_quaternion(theta)
                    self.current_waypoints.append(pickup_pose)
                    
                    # Dropoff waypoint
                    dropoff_pose = PoseStamped()
                    dropoff_pose.header.frame_id = 'map'
                    dropoff_pose.header.stamp = self.get_clock().now().to_msg()
                    dropoff_pose.pose.position = Point(x=task['dropoff_x'], y=task['dropoff_y'], z=0.0)
                    dropoff_pose.pose.orientation = self.angle_to_quaternion(theta)
                    self.current_waypoints.append(dropoff_pose)
                
                if self.current_waypoints:
                    self.get_logger().info(f'Generated {len(self.current_waypoints)} waypoints for Robot {self.robot_id}')
                    # Start navigation with the first waypoint
                    self.send_next_waypoint()
                else:
                    self.get_logger().warn(f'No valid waypoints generated for Robot {self.robot_id}')
                    self.reset_navigation()

    def send_next_waypoint(self):
        """Send the next waypoint to Nav2."""
        if self.current_waypoint_index >= len(self.current_waypoints):
            self.get_logger().info(f'All waypoints completed for Robot {self.robot_id}')
            self.complete_tasks()
            return
        
        goal_pose = self.current_waypoints[self.current_waypoint_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f'Sending waypoint {self.current_waypoint_index + 1}/{len(self.current_waypoints)} '
                              f'to Nav2 for Robot {self.robot_id}')
        
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
                self.reset_navigation()
                return
            self.get_logger().info(f'Nav2 goal accepted for Robot {self.robot_id}')
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f'Error processing Nav2 goal response: {e}')
            self.reset_navigation()

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
                self.reset_navigation()
        except Exception as e:
            self.get_logger().error(f'Error processing Nav2 result: {e}')
            self.reset_navigation()

    def check_waypoint_reached(self):
        """Check if the current waypoint is reached based on odometry."""
        if not self.current_pose or self.current_waypoint_index >= len(self.current_waypoints):
            return
        
        waypoint = self.current_waypoints[self.current_waypoint_index]
        dx = waypoint.pose.position.x - self.current_pose[0]
        dy = waypoint.pose.position.y - self.current_pose[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # Threshold for considering waypoint reached (adjust as needed)
        if distance < 0.1:  # 10 cm
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached (odometry check) '
                                  f'for Robot {self.robot_id}')
            self.current_waypoint_index += 1
            self.send_next_waypoint()

    def complete_tasks(self):
        """Mark tasks as completed and reset robot status."""
        for task_id in self.current_task_ids:
            task = self.tasks.get(task_id)
            if task:
                task_msg = Task()
                task_msg.task_id = task_id
                task_msg.pickup_x = task['pickup_x']
                task_msg.pickup_y = task['pickup_y']
                task_msg.dropoff_x = task['dropoff_x']
                task_msg.dropoff_y = task['dropoff_y']
                task_msg.available = False
                try:
                    self.task_pub.publish(task_msg)
                    self.get_logger().info(f'Task {task_id} marked as completed for Robot {self.robot_id}')
                except Exception as e:
                    self.get_logger().error(f'Error publishing task update: {e}')
        
        # Update robot status to available
        self.publish_robot_status(available=True)
        self.reset_navigation()

    def reset_navigation(self):
        """Reset navigation state."""
        self.current_task_ids = []
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.get_logger().info(f'Navigation reset for Robot {self.robot_id}')

    def publish_robot_status(self, available):
        """Publish robot status on /status_topic."""
        robot_status = RobotStatus()
        robot_status.robot_id = self.robot_id
        robot_status.available = available
        try:
            self.robot_status_pub.publish(robot_status)
            self.get_logger().info(f'Robot {self.robot_id} status published: available={available}')
        except Exception as e:
            self.get_logger().error(f'Error publishing robot status: {e}')

    def angle_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

def main():
    rclpy.init()
    node = NavigationProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Navigation Process Node')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()