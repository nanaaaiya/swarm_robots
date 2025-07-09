import numpy as np
import pulp as pl
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment

# Define number of robots, tasks, and drop-off locations
n_robots = 2
m_tasks = 6

# Generate positions for robots, tasks, and drop-off locations
np.random.seed(42)
robot_positions = np.random.rand(n_robots, 2) * 10  # Robots start in a 10x10 area
task_positions = np.random.rand(m_tasks, 2) * 10    # Task locations
dropoff_positions = np.random.rand(m_tasks, 2) * 10  # Each task has a specific drop-off point

# Compute distance matrices
dist_task = cdist(robot_positions, task_positions)  # Robot to task
dist_dropoff = cdist(task_positions, dropoff_positions)  # Task to drop-off
dist_matrix = np.zeros((2 * m_tasks, 2 * m_tasks))  # Full distance matrix for TSP

# Fill the distance matrix for tasks + drop-off
for i in range(m_tasks):
    for j in range(m_tasks):
        if i != j:
            dist_matrix[i, j] = np.linalg.norm(task_positions[i] - task_positions[j])  # Task-to-task
            dist_matrix[i + m_tasks, j] = np.linalg.norm(dropoff_positions[i] - task_positions[j])  # Drop-off-to-task
            dist_matrix[i, j + m_tasks] = np.linalg.norm(task_positions[i] - dropoff_positions[j])  # Task-to-drop-off
            dist_matrix[i + m_tasks, j + m_tasks] = np.linalg.norm(dropoff_positions[i] - dropoff_positions[j])  # Drop-off-to-drop-off

# Define MILP problem
problem = pl.LpProblem("Task_Allocation_With_TSP", pl.LpMinimize)

# Decision Variables: x[i][j] = 1 if robot i is assigned to task j
x = [[pl.LpVariable(f"x_{i}_{j}", cat=pl.LpBinary) for j in range(m_tasks)] for i in range(n_robots)]

# Objective Function: Minimize total travel distance
problem += pl.lpSum((dist_task[i][j] + dist_dropoff[j][j]) * x[i][j] for i in range(n_robots) for j in range(m_tasks))

# Constraint: Each task is assigned to exactly one robot
for j in range(m_tasks):
    problem += pl.lpSum(x[i][j] for i in range(n_robots)) == 1

# Constraint: Each robot can take at most k tasks (optional)
max_tasks_per_robot = int(np.ceil(m_tasks / n_robots))
for i in range(n_robots):
    problem += pl.lpSum(x[i][j] for j in range(m_tasks)) <= max_tasks_per_robot

# Solve MILP
problem.solve(pl.PULP_CBC_CMD(msg=False))

# Get assignment results
assignments = {i: [] for i in range(n_robots)}
for i in range(n_robots):
    for j in range(m_tasks):
        if pl.value(x[i][j]) == 1:
            assignments[i].append(j)

# -----------------
# Solve TSP for Each Robot to Find Optimal Task Order
# -----------------
ordered_routes = {}
total_distances = {}

for i, tasks in assignments.items():
    if not tasks:
        continue
    
    # Create distance matrix for assigned tasks
    num_tasks = len(tasks)
    cost_matrix = np.zeros((num_tasks * 2, num_tasks * 2))

    task_indices = tasks
    dropoff_indices = [j + m_tasks for j in tasks]

    all_indices = task_indices + dropoff_indices

    for idx_a, a in enumerate(all_indices):
        for idx_b, b in enumerate(all_indices):
            if idx_a != idx_b:
                cost_matrix[idx_a, idx_b] = dist_matrix[a, b]

    # Solve TSP using Hungarian Algorithm (approximation)
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    
    # Order tasks based on TSP solution
    route = [all_indices[i] for i in col_ind]
    ordered_routes[i] = route

    # Compute total travel distance for the route
    total_distance = 0
    prev_position = robot_positions[i]
    
    for idx in route:
        if idx < m_tasks:
            current_position = task_positions[idx]  # Task location
        else:
            current_position = dropoff_positions[idx - m_tasks]  # Drop-off location

        total_distance += np.linalg.norm(prev_position - current_position)
        prev_position = current_position
    
    total_distances[i] = total_distance

# -----------------
# Print Task Allocation and Distances
# -----------------
print("\nTask Allocation and Total Distance:")
for i, route in ordered_routes.items():
    task_sequence = []
    for idx in route:
        if idx < m_tasks:
            task_sequence.append(f"Task {idx}")
        else:
            task_sequence.append(f"Drop-off {idx - m_tasks}")
    
    print(f"Robot {i}: {' → '.join(task_sequence)} | Total Distance: {total_distances[i]:.2f}")

# -----------------
# Visualization
# -----------------
plt.figure(figsize=(8, 8))
colors = ['blue', 'green', 'purple']  # Different colors for each robot

# Plot robots
for i, pos in enumerate(robot_positions):
    plt.scatter(*pos, color=colors[i], s=200, edgecolors='black', label=f'Robot {i}')

# Plot tasks
for j, pos in enumerate(task_positions):
    plt.scatter(*pos, color='red', marker='s', s=100, edgecolors='black', label='Task' if j == 0 else "")

# Plot drop-off locations
for j, pos in enumerate(dropoff_positions):
    plt.scatter(*pos, color='orange', marker='D', s=100, edgecolors='black', label='Drop-off' if j == 0 else "")

# Draw TSP paths for each robot
for i, route in ordered_routes.items():
    if route:
        path = [robot_positions[i]]
        for idx in route:
            if idx < m_tasks:
                path.append(task_positions[idx])  # Task location
            else:
                path.append(dropoff_positions[idx - m_tasks])  # Drop-off location
        
        # Plot the path
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, color=colors[i], linestyle='-', marker='o', lw=2, label=f"Robot {i} Route")

        # Label task numbers in order visited
        for step, idx in enumerate(route):
            if idx < m_tasks:
                plt.text(task_positions[idx][0], task_positions[idx][1], str(step+1), fontsize=12, color='black', ha='center', va='center')
            else:
                plt.text(dropoff_positions[idx - m_tasks][0], dropoff_positions[idx - m_tasks][1], str(step+1), fontsize=12, color='black', ha='center', va='center')

# Labels and legend
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Optimized Task Execution Order (TSP)")
plt.legend()
plt.grid(True)
plt.show()
