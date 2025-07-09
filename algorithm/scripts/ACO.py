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
dist_task = cdist(robot_positions, task_positions)        # Robot to task
dist_dropoff = cdist(task_positions, dropoff_positions)   # Task to drop-off
dist_task_dropoff = cdist(task_positions, dropoff_positions)  # Task to drop-off
dist_dropoff_task = cdist(dropoff_positions, task_positions)  # Drop-off to task

# --------------------------
# ACO Implementation
# --------------------------

class ACO:
    def __init__(self, num_ants=30, num_iterations=100, alpha=1.0, beta=2.0, rho=0.1, q=100):
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.alpha = alpha  # Pheromone importance
        self.beta = beta    # Heuristic importance
        self.rho = rho      # Evaporation rate
        self.q = q          # Pheromone deposit factor
        # Initialize pheromone matrices
        self.pheromone_assignment = np.ones((m_tasks, n_robots))  # Task-to-robot pheromone
        self.pheromone_sequence = np.ones((m_tasks, m_tasks))     # Task-to-task pheromone
        # Heuristic information (inverse distance, avoid division by zero)
        self.heuristic_assignment = 1 / (dist_task + 1e-6)
        self.heuristic_sequence = 1 / (dist_dropoff_task + 1e-6)

    def run(self):
        best_cost = float('inf')
        best_assignment = None
        best_task_order = None
        best_robot_distances = None

        for _ in range(self.num_iterations):
            solutions = [self.construct_solution() for _ in range(self.num_ants)]
            for assignment, task_order, cost, robot_distances in solutions:
                if cost < best_cost:
                    best_cost = cost
                    best_assignment = assignment.copy()
                    best_task_order = task_order.copy()
                    best_robot_distances = robot_distances.copy()
            self.update_pheromones(solutions)
        
        return best_task_order, best_assignment, best_cost, best_robot_distances

    def construct_solution(self):
        # Initialize assignment and task order
        assignment = np.zeros(m_tasks, dtype=int)
        task_order = np.arange(m_tasks)
        np.random.shuffle(task_order)  # Random initial order

        # Assign tasks to robots probabilistically
        for task in range(m_tasks):
            probabilities = (self.pheromone_assignment[task] ** self.alpha) * (self.heuristic_assignment[:, task] ** self.beta)
            probabilities /= probabilities.sum()  # Normalize to sum to 1
            assignment[task] = np.random.choice(n_robots, p=probabilities)

        # Group tasks by robot
        robot_tasks = {i: [] for i in range(n_robots)}
        for task in task_order:
            robot = assignment[task]
            robot_tasks[robot].append(task)

        # Compute total cost and distances
        total_cost = 0
        robot_distances = {i: 0 for i in range(n_robots)}
        for i in range(n_robots):
            tasks = robot_tasks[i]
            if not tasks:
                continue
            prev_position = robot_positions[i]
            for task in tasks:
                pickup_pos = task_positions[task]
                dropoff_pos = dropoff_positions[task]
                robot_distances[i] += np.linalg.norm(prev_position - pickup_pos)  # Robot to Pickup
                robot_distances[i] += np.linalg.norm(pickup_pos - dropoff_pos)   # Pickup to Drop-off
                prev_position = dropoff_pos
            total_cost += robot_distances[i]

        return assignment, task_order, total_cost, robot_distances

    def update_pheromones(self, solutions):
        # Evaporation
        self.pheromone_assignment *= (1 - self.rho)
        self.pheromone_sequence *= (1 - self.rho)

        # Deposit pheromones based on solution quality
        for assignment, task_order, cost, _ in solutions:
            pheromone_deposit = self.q / cost
            # Update assignment pheromones
            for task, robot in enumerate(assignment):
                self.pheromone_assignment[task, robot] += pheromone_deposit
            # Update sequence pheromones (only for tasks assigned to the same robot)
            for idx in range(len(task_order) - 1):
                task_a = task_order[idx]
                task_b = task_order[idx + 1]
                if assignment[task_a] == assignment[task_b]:  # Same robot
                    self.pheromone_sequence[task_a, task_b] += pheromone_deposit

# Run ACO
aco = ACO(num_ants=30, num_iterations=100, alpha=1.0, beta=2.0, rho=0.1, q=100)
aco_task_order, aco_assignment, aco_total_cost, aco_robot_distances = aco.run()

# --------------------------
# Results Output
# --------------------------

print("\n=== Ant Colony Optimization (ACO) ===")
for i in range(n_robots):
    assigned_tasks = np.where(aco_assignment == i)[0]
    ordered_tasks = aco_task_order[np.isin(aco_task_order, assigned_tasks)]
    if len(ordered_tasks) == 0:
        print(f"Robot {i}: No tasks assigned | Total Distance: 0.00")
    else:
        path_str = []
        for task in ordered_tasks:
            path_str.append(f"Pick Task {task}")
            path_str.append(f"Drop Task {task}")
        print(f"Robot {i}: {' → '.join(path_str)} | Total Distance: {aco_robot_distances[i]:.2f}")
print(f"Total Distance (ACO): {aco_total_cost:.2f}")

# --------------------------
# Visualization
# --------------------------
plt.figure(figsize=(10, 8))
colors = ['blue', 'green', 'purple']

# Plot robots, tasks, and drop-offs
for i, pos in enumerate(robot_positions):
    plt.scatter(*pos, color=colors[i], s=200, edgecolors='black', label=f'Robot {i}')
for j, pos in enumerate(task_positions):
    plt.scatter(*pos, color='red', marker='s', s=100, edgecolors='black', label='Task' if j == 0 else "")
for j, pos in enumerate(dropoff_positions):
    plt.scatter(*pos, color='orange', marker='D', s=100, edgecolors='black', label='Drop-off' if j == 0 else "")

# Plot ACO routes
aco_routes = {}
for i in range(n_robots):
    assigned_tasks = np.where(aco_assignment == i)[0]
    ordered_tasks = aco_task_order[np.isin(aco_task_order, assigned_tasks)]
    if len(ordered_tasks) > 0:
        route = []
        for task in ordered_tasks:
            route.append(task)           # Pickup
            route.append(task + m_tasks) # Drop-off
        aco_routes[i] = route

for i, route in aco_routes.items():
    if route:
        path = [robot_positions[i]]
        for idx in route:
            if idx < m_tasks:
                path.append(task_positions[idx])
            else:
                path.append(dropoff_positions[idx - m_tasks])
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, color=colors[i], linestyle='-', marker='o', lw=2, label=f"Robot {i} Route")

plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("ACO (Pickup → Drop-off)")
plt.legend()
plt.grid(True)
plt.show()