import numpy as np
import pulp as pl
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment

# --------------------------
# Problem Setup (Shared)
# --------------------------

n_robots = 5
m_tasks = 20

np.random.seed(42)
robot_positions = np.random.rand(n_robots, 2) * 10
task_positions = np.random.rand(m_tasks, 2) * 10
dropoff_positions = np.random.rand(m_tasks, 2) * 10

dist_task = cdist(robot_positions, task_positions)
dist_dropoff = cdist(task_positions, dropoff_positions)
dist_task_dropoff = cdist(task_positions, dropoff_positions)
dist_dropoff_task = cdist(dropoff_positions, task_positions)

# --------------------------
# Method 1: MILP with TSP
# --------------------------

milp_problem = pl.LpProblem("Task_Allocation_With_TSP", pl.LpMinimize)
x = [[pl.LpVariable(f"x_{i}_{j}", cat=pl.LpBinary) for j in range(m_tasks)] for i in range(n_robots)]
milp_problem += pl.lpSum((dist_task[i][j] + dist_dropoff[j][j]) * x[i][j] for i in range(n_robots) for j in range(m_tasks))
for j in range(m_tasks):
    milp_problem += pl.lpSum(x[i][j] for i in range(n_robots)) == 1
# max_tasks_per_robot = int(np.ceil(m_tasks / n_robots))
# for i in range(n_robots):
#     milp_problem += pl.lpSum(x[i][j] for j in range(m_tasks)) <= max_tasks_per_robot
#     milp_problem += pl.lpSum(x[i][j] for j in range(m_tasks)) >= 1

milp_problem.solve(pl.PULP_CBC_CMD(msg=False))

milp_assignments = {i: [] for i in range(n_robots)}
for i in range(n_robots):
    for j in range(m_tasks):
        if pl.value(x[i][j]) == 1:
            milp_assignments[i].append(j)

milp_routes = {}
milp_total_distances = {}
for i, tasks in milp_assignments.items():
    if not tasks:
        continue
    num_tasks = len(tasks)
    cost_matrix = np.zeros((num_tasks, num_tasks))
    for a, task_a in enumerate(tasks):
        for b, task_b in enumerate(tasks):
            if a != b:
                cost_matrix[a, b] = dist_dropoff_task[task_a, task_b]
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    task_order = [tasks[i] for i in col_ind]
    route = []
    for task in task_order:
        route.append(task)
        route.append(task + m_tasks)
    milp_routes[i] = route
    total_distance = 0
    prev_position = robot_positions[i]
    for idx in route:
        if idx < m_tasks:
            current_position = task_positions[idx]
        else:
            current_position = dropoff_positions[idx - m_tasks]
        total_distance += np.linalg.norm(prev_position - current_position)
        prev_position = current_position
    milp_total_distances[i] = total_distance

# --------------------------
# Method 2: PSO
# --------------------------

class Particle:
    def __init__(self, num_tasks, num_robots):
        self.task_order = np.arange(num_tasks)
        np.random.shuffle(self.task_order)
        self.assignment = np.random.choice(num_robots, num_tasks)
        self.velocity = np.zeros(num_tasks)
        self.best_task_order = self.task_order.copy()
        self.best_assignment = self.assignment.copy()
        self.best_cost = float('inf')
        self.robot_distances = {i: 0 for i in range(n_robots)}

    def evaluate(self):
        total_cost = 0
        robot_distances = {i: 0 for i in range(n_robots)}
        for i in range(n_robots):
            assigned_tasks = np.where(self.assignment == i)[0]
            if len(assigned_tasks) == 0:
                continue
            ordered_tasks = self.task_order[np.isin(self.task_order, assigned_tasks)]
            if len(ordered_tasks) == 0:
                continue
            prev_position = robot_positions[i]
            for task in ordered_tasks:
                pickup_pos = task_positions[task]
                dropoff_pos = dropoff_positions[task]
                robot_distances[i] += np.linalg.norm(prev_position - pickup_pos)
                robot_distances[i] += np.linalg.norm(pickup_pos - dropoff_pos)
                prev_position = dropoff_pos
            total_cost += robot_distances[i]
        self.robot_distances = robot_distances
        return total_cost

    def update(self, global_best_task_order, global_best_assignment, w, c1, c2):
        inertia = w * self.velocity
        cognitive = c1 * np.random.rand() * (self.best_task_order - self.task_order)
        social = c2 * np.random.rand() * (global_best_task_order - self.task_order)
        self.velocity = inertia + cognitive + social
        swap_indices = np.argsort(self.velocity)[:2]
        self.task_order[swap_indices] = self.task_order[swap_indices[::-1]]
        if np.random.rand() < 0.1:
            task_idx = np.random.randint(0, len(self.assignment))
            new_robot = np.random.choice(n_robots)
            self.assignment[task_idx] = new_robot
        cost = self.evaluate()
        if cost < self.best_cost:
            self.best_cost = cost
            self.best_task_order = self.task_order.copy()
            self.best_assignment = self.assignment.copy()
            self.robot_distances = self.robot_distances.copy()

def PSO(num_particles=30, max_iter=100, w=0.7, c1=1.5, c2=1.5):
    particles = [Particle(m_tasks, n_robots) for _ in range(num_particles)]
    global_best_particle = min(particles, key=lambda p: p.evaluate())
    global_best_task_order = global_best_particle.best_task_order.copy()
    global_best_assignment = global_best_particle.best_assignment.copy()
    global_best_cost = global_best_particle.best_cost
    global_best_robot_distances = global_best_particle.robot_distances.copy()
    for _ in range(max_iter):
        for particle in particles:
            particle.update(global_best_task_order, global_best_assignment, w, c1, c2)
            if particle.best_cost < global_best_cost:
                global_best_cost = particle.best_cost
                global_best_task_order = particle.best_task_order.copy()
                global_best_assignment = particle.best_assignment.copy()
                global_best_robot_distances = particle.robot_distances.copy()
    return global_best_task_order, global_best_assignment, global_best_cost, global_best_robot_distances

pso_task_order, pso_assignment, pso_total_cost, pso_robot_distances = PSO()

# --------------------------
# Method 3: ACO
# --------------------------

class ACO:
    def __init__(self, num_ants=30, num_iterations=100, alpha=1.0, beta=2.0, rho=0.1, q=100):
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.alpha = alpha  # Pheromone importance
        self.beta = beta    # Heuristic importance
        self.rho = rho      # Evaporation rate
        self.q = q          # Pheromone deposit factor
        self.pheromone_assignment = np.ones((m_tasks, n_robots))  # Task-to-robot pheromone
        self.pheromone_sequence = np.ones((m_tasks, m_tasks))     # Task-to-task pheromone
        self.heuristic_assignment = 1 / (dist_task + 1e-6)        # Avoid division by zero
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
        assignment = np.zeros(m_tasks, dtype=int)
        task_order = np.arange(m_tasks)
        np.random.shuffle(task_order)  # Initial random order

        # Assign tasks to robots
        for task in range(m_tasks):
            probabilities = (self.pheromone_assignment[task] ** self.alpha) * (self.heuristic_assignment[:, task] ** self.beta)
            probabilities /= probabilities.sum()
            assignment[task] = np.random.choice(n_robots, p=probabilities)

        # For each robot, order its assigned tasks
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
                robot_distances[i] += np.linalg.norm(prev_position - pickup_pos)
                robot_distances[i] += np.linalg.norm(pickup_pos - dropoff_pos)
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
            for task, robot in enumerate(assignment):
                self.pheromone_assignment[task, robot] += pheromone_deposit
            for idx in range(len(task_order) - 1):
                task_a = task_order[idx]
                task_b = task_order[idx + 1]
                if assignment[task_a] == assignment[task_b]:  # Only within same robot
                    self.pheromone_sequence[task_a, task_b] += pheromone_deposit

aco = ACO(num_ants=30, num_iterations=100)
aco_task_order, aco_assignment, aco_total_cost, aco_robot_distances = aco.run()

# --------------------------
# Results Output
# --------------------------

print("\n=== Method 1: MILP with TSP ===")
for i, route in milp_routes.items():
    task_sequence = []
    for idx in route:
        if idx < m_tasks:
            task_sequence.append(f"Pick Task {idx}")
        else:
            task_sequence.append(f"Drop Task {idx - m_tasks}")
    print(f"Robot {i}: {' → '.join(task_sequence)} | Total Distance: {milp_total_distances[i]:.2f}")
print(f"Total Distance (MILP): {sum(milp_total_distances.values()):.2f}")

print("\n=== Method 2: PSO ===")
for i in range(n_robots):
    assigned_tasks = np.where(pso_assignment == i)[0]
    ordered_tasks = pso_task_order[np.isin(pso_task_order, assigned_tasks)]
    if len(ordered_tasks) == 0:
        print(f"Robot {i}: No tasks assigned | Total Distance: 0.00")
    else:
        path_str = []
        for task in ordered_tasks:
            path_str.append(f"Pick Task {task}")
            path_str.append(f"Drop Task {task}")
        print(f"Robot {i}: {' → '.join(path_str)} | Total Distance: {pso_robot_distances[i]:.2f}")
print(f"Total Distance (PSO): {pso_total_cost:.2f}")

print("\n=== Method 3: ACO ===")
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
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))
colors = ['blue', 'green', 'purple', 'blue', 'green', 'purple', 'blue', 'green', 'purple', 'blue', 'green', 'purple', 'blue', 'green', 'purple', 'blue', 'green', 'purple', 'blue', 'green', 'purple']

def plot_solution(ax, routes, title):
    for i, pos in enumerate(robot_positions):
        ax.scatter(*pos, color=colors[i], s=200, edgecolors='black', label=f'Robot {i}')
    for j, pos in enumerate(task_positions):
        ax.scatter(*pos, color='red', marker='s', s=100, edgecolors='black', label='Task' if j == 0 else "")
    for j, pos in enumerate(dropoff_positions):
        ax.scatter(*pos, color='orange', marker='D', s=100, edgecolors='black', label='Drop-off' if j == 0 else "")
    for i, route in routes.items():
        if route:
            path = [robot_positions[i]]
            for idx in route:
                if idx < m_tasks:
                    path.append(task_positions[idx])
                else:
                    path.append(dropoff_positions[idx - m_tasks])
            path_x, path_y = zip(*path)
            ax.plot(path_x, path_y, color=colors[i], linestyle='-', marker='o', lw=2, label=f"Robot {i} Route")
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

plot_solution(ax1, milp_routes, "MILP with TSP (Pickup → Drop-off)")

pso_routes = {}
for i in range(n_robots):
    assigned_tasks = np.where(pso_assignment == i)[0]
    ordered_tasks = pso_task_order[np.isin(pso_task_order, assigned_tasks)]
    if len(ordered_tasks) > 0:
        route = []
        for task in ordered_tasks:
            route.append(task)
            route.append(task + m_tasks)
        pso_routes[i] = route
plot_solution(ax2, pso_routes, "PSO (Pickup → Drop-off)")

aco_routes = {}
for i in range(n_robots):
    assigned_tasks = np.where(aco_assignment == i)[0]
    ordered_tasks = aco_task_order[np.isin(aco_task_order, assigned_tasks)]
    if len(ordered_tasks) > 0:
        route = []
        for task in ordered_tasks:
            route.append(task)
            route.append(task + m_tasks)
        aco_routes[i] = route
plot_solution(ax3, aco_routes, "ACO (Pickup → Drop-off)")

plt.tight_layout()
plt.show()