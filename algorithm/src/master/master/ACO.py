#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.distance import cdist
from communication_msgs.msg import Robots, Task, TaskAllocation
from threading import Lock

class ACOAllocationNode(Node):
    def __init__(self):
        super().__init__('aco_allocation_node')
        
        # Subscribers
        self.robot_sub = self.create_subscription(
            Robots,
            '/all_robot_status',
            self.robot_status_callback,
            10)
        self.task_sub = self.create_subscription(
            Task,
            '/task_locations',
            self.task_location_callback,
            10)
        
        # Publisher
        self.alloc_pub = self.create_publisher(
            TaskAllocation,
            '/task_allocation',
            10)
        
        # Storage for incoming data
        self.robots = {}  # {robot_id: (x, y, available)}
        self.tasks = {}   # {task_id: (pickup_x, pickup_y, dropoff_x, dropoff_y, available)}
        self.data_lock = Lock()  # Lock for thread-safe access to robots and tasks
        
        # ACO Parameters
        self.num_ants = 30
        self.num_iterations = 100
        self.alpha = 1.0
        self.beta = 2.0
        self.rho = 0.1
        self.q = 100
        
        # Timer to run allocation periodically
        self.timer = self.create_timer(5.0, self.run_allocation)
        self.get_logger().info('ACO Allocation Node started')

    def robot_status_callback(self, msg):
        with self.data_lock:
            for robot in msg.robot_status:
                self.robots[robot.robot_id] = (robot.x, robot.y, robot.available)
                self.get_logger().info(f'Robot {robot.robot_id} at ({robot.x}, {robot.y}), Available: {robot.available}')

    def task_location_callback(self, msg):
        with self.data_lock:
            self.tasks[msg.task_id] = (msg.pickup_x, msg.pickup_y, msg.dropoff_x, msg.dropoff_y, msg.available)
            self.get_logger().info(f'Task {msg.task_id} added, Available: {msg.available}')

    def run_allocation(self):
        with self.data_lock:
            available_robots = {rid: r for rid, r in self.robots.items() if r[2]}  # Filter available robots
            available_tasks = {tid: t for tid, t in self.tasks.items() if t[4]}    # Filter available tasks
        
        if not available_robots or not available_tasks:
            self.get_logger().warn('No available robots or tasks.')
            return
        
        robot_positions = np.array([(r[0], r[1]) for r in available_robots.values()])
        task_positions = np.array([(t[0], t[1]) for t in available_tasks.values()])
        dropoff_positions = np.array([(t[2], t[3]) for t in available_tasks.values()])
        
        # Compute distance matrices
        dist_task = cdist(robot_positions, task_positions)
        dist_dropoff = cdist(task_positions, dropoff_positions)
        dist_dropoff_task = cdist(dropoff_positions, task_positions)
        
        # Run ACO
        aco = ACO(len(available_robots), len(available_tasks), dist_task, dist_dropoff, dist_dropoff_task,
                  self.num_ants, self.num_iterations, self.alpha, self.beta, self.rho, self.q)
        task_order, assignment, total_cost, robot_distances = aco.run()
        
        # Publish allocations
        robot_tasks = {rid: [] for rid in available_robots.keys()}
        task_id_list = list(available_tasks.keys())
        for idx, task_idx in enumerate(task_order):
            robot_id = list(available_robots.keys())[assignment[task_idx]]
            robot_tasks[robot_id].append(task_id_list[task_idx])
        
        for robot_id, tasks in robot_tasks.items():
            if tasks:
                msg = TaskAllocation()
                msg.robot_id = robot_id
                msg.task_ids = tasks
                msg.total_distance = float(robot_distances[list(available_robots.keys()).index(robot_id)])
                self.alloc_pub.publish(msg)
                self.get_logger().info(f'Robot {robot_id}: Tasks {tasks}, Distance: {msg.total_distance:.2f}')

class ACO:
    def __init__(self, n_robots, m_tasks, dist_task, dist_dropoff, dist_dropoff_task,
                 num_ants, num_iterations, alpha, beta, rho, q):
        self.n_robots = n_robots
        self.m_tasks = m_tasks
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.q = q
        self.pheromone_assignment = np.ones((m_tasks, n_robots))
        self.pheromone_sequence = np.ones((m_tasks, m_tasks))
        self.heuristic_assignment = 1 / (dist_task + 1e-6)
        self.heuristic_sequence = 1 / (dist_dropoff_task + 1e-6)
        self.dist_task = dist_task
        self.dist_dropoff = dist_dropoff

    def run(self):
        best_cost = float('inf')
        best_assignment, best_task_order, best_robot_distances = None, None, None
        for _ in range(self.num_iterations):
            solutions = [self.construct_solution() for _ in range(self.num_ants)]
            for assignment, task_order, cost, robot_distances in solutions:
                if cost < best_cost:
                    best_cost, best_assignment, best_task_order, best_robot_distances = cost, assignment.copy(), task_order.copy(), robot_distances.copy()
            self.update_pheromones(solutions)
        return best_task_order, best_assignment, best_cost, best_robot_distances

    def construct_solution(self):
        assignment = np.zeros(self.m_tasks, dtype=int)
        task_order = np.arange(self.m_tasks)
        np.random.shuffle(task_order)
        for task in range(self.m_tasks):
            probabilities = (self.pheromone_assignment[task] ** self.alpha) * (self.heuristic_assignment[:, task] ** self.beta)
            probabilities /= probabilities.sum()
            assignment[task] = np.random.choice(self.n_robots, p=probabilities)
        return assignment, task_order, 0, {i: 0.0 for i in range(self.n_robots)}

    def update_pheromones(self, solutions):
        self.pheromone_assignment *= (1 - self.rho)
        self.pheromone_sequence *= (1 - self.rho)
        for assignment, task_order, cost, _ in solutions:
            pheromone_deposit = self.q / (cost + 1e-6)
            for task, robot in enumerate(assignment):
                self.pheromone_assignment[task, robot] += pheromone_deposit

def main():
    rclpy.init()
    node = ACOAllocationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()