import rclpy
from rclpy.node import Node
from scipy.spatial.distance import cdist
import numpy as np
from communication_msgs.msg import Task, Robot, TaskAllocation

class Particle:
    def __init__(self, num_tasks, num_robots):
        self.task_order = self.generate_valid_sequence(num_tasks)
        self.assignment = np.random.choice(num_robots, num_tasks)
        self.velocity = np.zeros(2 * num_tasks)
        self.best_task_order = self.task_order.copy()
        self.best_assignment = self.assignment.copy()
        self.best_cost = float('inf')

    def generate_valid_sequence(self, num_tasks):
        task_indices = np.arange(num_tasks)
        np.random.shuffle(task_indices)
        valid_sequence = np.concatenate([[2 * task, 2 * task + 1] for task in task_indices])
        return valid_sequence

    def evaluate(self, robot_positions, task_positions, dropoff_positions):
        total_cost = 0
        for i in range(len(robot_positions)):
            assigned_tasks = np.where(self.assignment == i)[0]
            if len(assigned_tasks) == 0:
                continue

            ordered_sequence = self.task_order[np.isin(self.task_order // 2, assigned_tasks)]
            cost = np.linalg.norm(robot_positions[i] - task_positions[ordered_sequence[0] // 2])
            
            for j in range(len(ordered_sequence) - 1):
                current_index = ordered_sequence[j]
                next_index = ordered_sequence[j + 1]
                
                if current_index % 2 == 0:  # Pickup point
                    next_location = dropoff_positions[current_index // 2] if next_index % 2 else task_positions[next_index // 2]
                else:  # Dropoff point
                    next_location = dropoff_positions[next_index // 2] if next_index % 2 else task_positions[next_index // 2]
                
                cost += np.linalg.norm(
                    (task_positions if current_index % 2 == 0 else dropoff_positions)[current_index // 2] - next_location
                )
            
            total_cost += cost

        return total_cost

    def update(self, global_best_task_order, global_best_assignment, w, c1, c2):
        inertia = w * self.velocity
        cognitive = c1 * np.random.rand() * (self.best_task_order - self.task_order)
        social = c2 * np.random.rand() * (global_best_task_order - self.task_order)
        self.velocity = inertia + cognitive + social

        swap_indices = np.argsort(self.velocity)[:2]
        self.task_order[swap_indices] = self.task_order[swap_indices[::-1]]
        
        if np.random.rand() < 0.1:
            self.assignment[np.random.randint(0, len(self.assignment))] = np.random.choice(len(global_best_assignment))
        
        self.repair_sequence()
        
    def repair_sequence(self):
        task_positions = {task: i for i, task in enumerate(self.task_order)}
        for task in range(len(self.task_order) // 2):
            pickup_idx = task_positions[2 * task]
            dropoff_idx = task_positions[2 * task + 1]
            if pickup_idx > dropoff_idx:
                self.task_order[pickup_idx], self.task_order[dropoff_idx] = self.task_order[dropoff_idx], self.task_order[pickup_idx]

class PSOTaskAllocator(Node):
    def __init__(self):
        super().__init__('pso_task_allocator')
        self.robot_sub = self.create_subscription(
            Robot, 'robot_status', self.robot_callback, 10)
        self.task_sub = self.create_subscription(
            Task, 'task_locations', self.task_callback, 10)
        self.alloc_pub = self.create_publisher(
            TaskAllocation, 'task_allocation', 10)
        
        # Storage for robot and task data
        self.robots = {}  # robot_id -> (x, y, available)
        self.tasks = {}   # task_id -> (pickup_x, pickup_y, dropoff_x, dropoff_y, available)
        
        # Timer to run PSO periodically
        self.timer = self.create_timer(5.0, self.run_pso)

    def robot_callback(self, msg):
        self.robots[msg.robot_id] = (msg.x, msg.y, msg.available)
        self.get_logger().info(f'Robot {msg.robot_id} updated: ({msg.x}, {msg.y}), available: {msg.available}')

    def task_callback(self, msg):
        self.tasks[msg.task_id] = (msg.pickup_x, msg.pickup_y, msg.dropoff_x, msg.dropoff_y, msg.available)
        self.get_logger().info(f'Task {msg.task_id} updated, available: {msg.available}')

    def run_pso(self):
        if not self.robots or not self.tasks:
            self.get_logger().warn('Waiting for robot and task data...')
            return
        
        # Filter available robots and tasks
        available_robots = {rid: r for rid, r in self.robots.items() if r[2]}
        available_tasks = {tid: t for tid, t in self.tasks.items() if t[4]}
        
        if not available_robots or not available_tasks:
            self.get_logger().warn('No available robots or tasks to allocate.')
            return

        # Prepare position arrays
        robot_positions = np.array([(r[0], r[1]) for r in available_robots.values()])
        task_positions = np.array([(t[0], t[1]) for t in available_tasks.values()])
        dropoff_positions = np.array([(t[2], t[3]) for t in available_tasks.values()])

        # Initialize PSO
        particles = [Particle(len(available_tasks), len(available_robots)) for _ in range(30)]
        global_best_particle = min(particles, key=lambda p: p.evaluate(robot_positions, task_positions, dropoff_positions))
        global_best_task_order = global_best_particle.task_order.copy()
        global_best_assignment = global_best_particle.assignment.copy()
        global_best_cost = global_best_particle.evaluate(robot_positions, task_positions, dropoff_positions)

        # Run PSO iterations
        for _ in range(100):
            for particle in particles:
                particle.update(global_best_task_order, global_best_assignment, 0.7, 1.5, 1.5)
                cost = particle.evaluate(robot_positions, task_positions, dropoff_positions)
                if cost < particle.best_cost:
                    particle.best_cost = cost
                    particle.best_task_order = particle.task_order.copy()
                    particle.best_assignment = particle.assignment.copy()
                if cost < global_best_cost:
                    global_best_cost = cost
                    global_best_task_order = particle.task_order.copy()
                    global_best_assignment = particle.assignment.copy()

        # Prepare and publish allocations
        robot_ids = list(available_robots.keys())
        task_ids = list(available_tasks.keys())
        robot_task_assignments = {i: [] for i in range(len(available_robots))}
        
        # Convert task indices to actual task IDs
        for task_idx, robot_idx in enumerate(global_best_assignment):
            actual_task_id = task_ids[task_idx]
            robot_task_assignments[robot_idx].append(actual_task_id)

        # Calculate distances and publish
        for robot_idx, task_list in robot_task_assignments.items():
            if task_list:
                msg = TaskAllocation()
                msg.robot_id = robot_ids[robot_idx]
                msg.task_ids = task_list
                
                # Calculate total distance for this robot
                total_distance = 0.0
                robot_pos = robot_positions[robot_idx]
                for i, task_id in enumerate(task_list):
                    task_idx = task_ids.index(task_id)
                    if i == 0:
                        total_distance += np.linalg.norm(robot_pos - task_positions[task_idx])
                    total_distance += np.linalg.norm(task_positions[task_idx] - dropoff_positions[task_idx])
                    if i < len(task_list) - 1:
                        next_task_idx = task_ids.index(task_list[i + 1])
                        total_distance += np.linalg.norm(dropoff_positions[task_idx] - task_positions[next_task_idx])
                
                msg.total_distance = float(total_distance)
                self.alloc_pub.publish(msg)
                self.get_logger().info(f'Allocated to Robot {msg.robot_id}: Tasks {msg.task_ids}, Distance: {msg.total_distance:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PSOTaskAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()