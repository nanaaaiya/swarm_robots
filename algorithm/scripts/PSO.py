import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

# --------------------------
# Problem Setup
# --------------------------

n_robots = 3
m_tasks = 5

np.random.seed(42)
robot_positions = np.random.rand(n_robots, 2) * 10
task_positions = np.random.rand(m_tasks, 2) * 10
dropoff_positions = np.random.rand(m_tasks, 2) * 10  

# Compute distance matrices
dist_robot_task = cdist(robot_positions, task_positions)
dist_task_dropoff = cdist(task_positions, dropoff_positions)
dist_task_task = cdist(task_positions, task_positions)
dist_dropoff_task = cdist(dropoff_positions, task_positions)
dist_dropoff_dropoff = cdist(dropoff_positions, dropoff_positions)

# --------------------------
# PSO Algorithm
# --------------------------

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

    def evaluate(self):
        total_cost = 0
        for i in range(n_robots):
            assigned_tasks = np.where(self.assignment == i)[0]  
            if len(assigned_tasks) == 0:
                continue
            
            ordered_sequence = self.task_order[np.isin(self.task_order // 2, assigned_tasks)]  

            cost = np.linalg.norm(robot_positions[i] - task_positions[ordered_sequence[0] // 2])  

            for j in range(len(ordered_sequence) - 1):
                current_index = ordered_sequence[j]
                next_index = ordered_sequence[j + 1]

                if current_index % 2 == 0:  
                    next_location = dropoff_positions[current_index // 2] if next_index % 2 else task_positions[next_index // 2]
                    cost += np.linalg.norm(task_positions[current_index // 2] - next_location)
                else:  
                    next_location = dropoff_positions[next_index // 2] if next_index % 2 else task_positions[next_index // 2]
                    cost += np.linalg.norm(dropoff_positions[current_index // 2] - next_location)

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
            self.assignment[np.random.randint(0, len(self.assignment))] = np.random.choice(n_robots)

        self.repair_sequence()  

        cost = self.evaluate()
        if cost < self.best_cost:
            self.best_cost = cost
            self.best_task_order = self.task_order.copy()
            self.best_assignment = self.assignment.copy()

    def repair_sequence(self):
        task_positions = {task: i for i, task in enumerate(self.task_order)}
        for task in range(m_tasks):
            pickup_idx = task_positions[2 * task]
            dropoff_idx = task_positions[2 * task + 1]
            if pickup_idx > dropoff_idx:
                self.task_order[pickup_idx], self.task_order[dropoff_idx] = self.task_order[dropoff_idx], self.task_order[pickup_idx]


def PSO(num_particles=30, max_iter=100, w=0.7, c1=1.5, c2=1.5):
    particles = [Particle(m_tasks, n_robots) for _ in range(num_particles)]

    global_best_particle = min(particles, key=lambda p: p.evaluate())
    global_best_task_order = global_best_particle.best_task_order.copy()
    global_best_assignment = global_best_particle.best_assignment.copy()
    global_best_cost = global_best_particle.best_cost

    for _ in range(max_iter):
        for particle in particles:
            particle.update(global_best_task_order, global_best_assignment, w, c1, c2)
            if particle.best_cost < global_best_cost:
                global_best_cost = particle.best_cost
                global_best_task_order = particle.best_task_order.copy()
                global_best_assignment = particle.best_assignment.copy()

    return global_best_task_order, global_best_assignment, global_best_cost


# Run PSO
best_task_order, best_assignment, best_total_cost = PSO()

# --------------------------
# Visualization
# --------------------------

colors = ['r', 'g', 'b', 'm', 'c']

plt.figure(figsize=(10, 8))

# Plot robots
for i in range(n_robots):
    plt.scatter(*robot_positions[i], c=colors[i], marker='s', s=200, label=f'Robot {i}')

# Plot tasks
for i in range(m_tasks):
    plt.scatter(*task_positions[i], c='orange', marker='o', s=100, edgecolor='black', label=f'Task {i}' if i == 0 else "")

# Plot drop-off locations
for i in range(m_tasks):
    plt.scatter(*dropoff_positions[i], c='purple', marker='D', s=100, edgecolor='black', label=f'Drop-off {i}' if i == 0 else "")

# Plot paths
for i in range(n_robots):
    assigned_tasks = np.where(best_assignment == i)[0]
    ordered_sequence = best_task_order[np.isin(best_task_order // 2, assigned_tasks)]
    
    if len(ordered_sequence) == 0:
        continue
    
    robot_path = [robot_positions[i]]  # Start from the robot's initial position
    
    for step in ordered_sequence:
        if step % 2 == 0:  # Pickup task
            robot_path.append(task_positions[step // 2])
        else:  # Drop-off task
            robot_path.append(dropoff_positions[step // 2])
    
    robot_path = np.array(robot_path)
    plt.plot(robot_path[:, 0], robot_path[:, 1], linestyle='-', color=colors[i], linewidth=2)

plt.legend()
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Robot Task Allocation and Execution Path")
plt.grid(True)
plt.show()

# --------------------------
# Print Results
# --------------------------

print("\nOptimized Task Assignment and Execution Order:")
for i in range(n_robots):
    assigned_tasks = np.where(best_assignment == i)[0]
    ordered_sequence = best_task_order[np.isin(best_task_order // 2, assigned_tasks)]
    
    if len(ordered_sequence) == 0:
        continue
    
    path_str = []
    for step in ordered_sequence:
        if step % 2 == 0:
            path_str.append(f"Pick Task {step // 2}")
        else:
            path_str.append(f"Drop Task {step // 2}")
    
    print(f"Robot {i}: {' → '.join(path_str)}")

print(f"\nTotal Optimized Travel Distance: {best_total_cost:.2f}")
