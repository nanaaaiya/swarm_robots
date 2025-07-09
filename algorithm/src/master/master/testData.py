import rclpy
from rclpy.node import Node
from communication_msgs.msg import Robot, Task

class TaskRobotPublisher(Node):
    def __init__(self):
        super().__init__('task_robot_publisher')
        self.task_pub = self.create_publisher(Task, '/task_locations', 10)
        
        self.timer = self.create_timer(2.0, self.publish_data)

    def publish_data(self):
        # Publish static task data
        tasks = [
            (0, 1.0, 1.0, 6.0, 6.0, True),
            (1, 3.0, 4.0, 7.0, 8.0, True),
            (2, 5.0, 2.0, 9.0, 3.0, True),
            (3, 4.0, 6.0, 2.0, 9.0, True),
            (4, 7.0, 5.0, 8.0, 8.0, True)
        ]
        
        for task_id, pickup_x, pickup_y, dropoff_x, dropoff_y, available in tasks:
            task_msg = Task()
            task_msg.task_id = task_id
            task_msg.pickup_x = pickup_x
            task_msg.pickup_y = pickup_y
            task_msg.dropoff_x = dropoff_x
            task_msg.dropoff_y = dropoff_y
            task_msg.available = available
            self.task_pub.publish(task_msg)
            self.get_logger().info(f'Published Task {task_id}: Pickup({pickup_x}, {pickup_y}) -> Dropoff({dropoff_x}, {dropoff_y})')


def main():
    rclpy.init()
    node = TaskRobotPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
