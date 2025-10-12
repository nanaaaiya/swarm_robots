# compare_methods_two_seed_layers.py
# Uses two layers of seeds:
#  - INSTANCE_SEED: controls robot/task/dropoff generation (fixed problem instances)
#  - run_seed passed into run_aco/run_pso: controls the algorithm randomness per trial
#
# Outputs per method:
#  WAYPOINTS = { "tb1": [(x,y,0.0), ...], ... }
#  Total Distance: XX.XX
#
# Requires: numpy, scipy, pulp (for MILP), matplotlib (optional, for plotting)
# pip install numpy scipy pulp matplotlib

import numpy as np
import pulp as pl
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
import math

# --------------------------
# CONFIG: Two seed layers
# --------------------------
with open("seeds.txt", "r") as f:
    try:
        INSTANCE_SEED = int(f.read())
    except ValueError:
        print("Error: seeds.txt must contain valid integers")
        exit(1)
# INSTANCE_SEED = 1068761995  # <-- seed used to generate the fixed instance (changeable)
# Example run seeds for stochastic algorithms (you will run many of these per instance)
# RUN_SEED_PSO = 1068761995
# RUN_SEED_ACO = 1068761995

# --------------------------
# Problem Setup (Shared, generated with INSTANCE_SEED)
# --------------------------
n_robots = 3
m_tasks = 10

rng_inst = np.random.RandomState(INSTANCE_SEED)

robot_positions = np.array([(0.0, 0.0), (5.0, 0.0), (0.0, 4.0)])  # tb1  # tb2  # tb3
task_positions = rng_inst.rand(m_tasks, 2) * 5
dropoff_positions = rng_inst.rand(m_tasks, 2) * 5

# distance matrices used by all methods
dist_robot_to_task = cdist(robot_positions, task_positions)  # shape (n_robots, m_tasks)
dist_task_to_drop = np.linalg.norm(
    task_positions - dropoff_positions, axis=1
)  # len m_tasks
# distances between dropoff -> next pickup (for routing)
dist_drop_to_task = cdist(dropoff_positions, task_positions)
dist_task_to_task = cdist(task_positions, task_positions)

# helper: label robots "tb1", "tb2", ...
robot_labels = [f"tb{i+1}" for i in range(n_robots)]


def route_distance_from_waypoints(waypoints):
    """Compute sum of Euclidean distances along a list of (x,y,0.0) tuples."""
    if not waypoints:
        return 0.0
    coords = [(wp[0], wp[1]) for wp in waypoints]
    dist = 0.0
    prev = None
    for p in coords:
        if prev is None:
            prev = p
            continue
        dist += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return dist


# --------------------------
# UTIL: build WAYPOINTS dict from a per-robot route (list of indices pick/drop encoded)
# --------------------------
def build_waypoints_dict(routes):
    """
    routes: dict robot_index -> list of idx where idx < m_tasks => pickup idx,
            idx >= m_tasks => dropoff idx-m_tasks
    returns WAYPOINTS dict mapping robot label -> list of (x,y,0.0)
    """
    WAYPOINTS = {}
    for i in range(n_robots):
        path = []
        seq = routes.get(i, [])
        for idx in seq:
            if idx < m_tasks:
                x, y = task_positions[idx]
            else:
                x, y = dropoff_positions[idx - m_tasks]
            path.append((round(float(x), 3), round(float(y), 3), 0.0))
        WAYPOINTS[robot_labels[i]] = path
    return WAYPOINTS


# --------------------------
# MILP + per-robot TSP (deterministic given INSTANCE_SEED)
# --------------------------
def run_milp_assignment_and_tsp():
    # assignment variables: x[i][j] robot i assigned task j
    prob = pl.LpProblem("assign", pl.LpMinimize)
    x = [
        [pl.LpVariable(f"x_{i}_{j}", cat=pl.LpBinary) for j in range(m_tasks)]
        for i in range(n_robots)
    ]

    # Objective: robot -> pickup + pickup -> dropoff (approximate local cost)
    prob += pl.lpSum(
        (dist_robot_to_task[i, j] + dist_task_to_drop[j]) * x[i][j]
        for i in range(n_robots)
        for j in range(m_tasks)
    )

    # each task assigned exactly once
    for j in range(m_tasks):
        prob += pl.lpSum(x[i][j] for i in range(n_robots)) == 1

    # optional: balance lower/upper bounds can be added; omitted for simplicity
    prob.solve(pl.PULP_CBC_CMD(msg=False))
    # collect assignments
    assignments = {i: [] for i in range(n_robots)}
    for i in range(n_robots):
        for j in range(m_tasks):
            val = pl.value(x[i][j])
            if val is not None and round(val) == 1:
                assignments[i].append(j)

    # For each robot, create a route: start -> (pickup -> dropoff) sequences.
    routes = {}
    total_distances = {}
    for i in range(n_robots):
        tasks = assignments.get(i, [])
        if not tasks:
            routes[i] = []
            total_distances[i] = 0.0
            continue

        # Simple heuristic ordering: nearest-neighbor on pickup locations starting from robot start
        # then for each pickup we append its dropoff immediately.
        remaining = set(tasks)
        order = []
        cur_pos = robot_positions[i]
        while remaining:
            # pick nearest pickup
            nearest = min(
                remaining, key=lambda t: np.linalg.norm(cur_pos - task_positions[t])
            )
            order.append(nearest)
            # move to dropoff next (we append dropoff in the final route)
            cur_pos = dropoff_positions[nearest]
            remaining.remove(nearest)

        route = []
        for t in order:
            route.append(t)  # pickup index
            route.append(t + m_tasks)  # dropoff encoded as t + m_tasks
        routes[i] = route
        # compute distance starting from robot start
        waypoints = []
        prev = tuple(robot_positions[i])
        dist_sum = 0.0
        for idx in route:
            if idx < m_tasks:
                cur = tuple(task_positions[idx])
            else:
                cur = tuple(dropoff_positions[idx - m_tasks])
            dist_sum += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            prev = cur
        total_distances[i] = dist_sum

    milp_waypoints = build_waypoints_dict(routes)
    milp_total_distance = sum(total_distances.values())
    return milp_waypoints, milp_total_distance, routes


# --------------------------
# PSO (uses run_seed for algorithm randomness)
# --------------------------
def run_pso(run_seed, num_particles=30, max_iter=100, w=0.7, c1=1.5, c2=1.5):
    rng = np.random.RandomState(int(run_seed))

    class Particle:
        def __init__(self):
            # random initial permutation and assignment
            self.task_order = rng.permutation(m_tasks)
            self.assignment = rng.randint(0, n_robots, size=m_tasks)
            self.best_task_order = self.task_order.copy()
            self.best_assignment = self.assignment.copy()
            self.best_cost = float("inf")
            self.robot_dist = {i: 0.0 for i in range(n_robots)}

        def evaluate(self):
            total = 0.0
            robot_dist = {i: 0.0 for i in range(n_robots)}
            for i in range(n_robots):
                assigned_tasks = [t for t in self.task_order if self.assignment[t] == i]
                if not assigned_tasks:
                    continue
                prev = tuple(robot_positions[i])
                for t in assigned_tasks:
                    pick = tuple(task_positions[t])
                    drop = tuple(dropoff_positions[t])
                    robot_dist[i] += math.hypot(pick[0] - prev[0], pick[1] - prev[1])
                    robot_dist[i] += math.hypot(drop[0] - pick[0], drop[1] - pick[1])
                    prev = drop
                total += robot_dist[i]
            self.robot_dist = robot_dist
            return total

        def mutate(self):
            # small permutation mutation and occasional assignment change
            a, b = rng.randint(0, m_tasks), rng.randint(0, m_tasks)
            self.task_order[a], self.task_order[b] = (
                self.task_order[b],
                self.task_order[a],
            )
            if rng.rand() < 0.05:
                idx = rng.randint(0, m_tasks)
                self.assignment[idx] = rng.randint(0, n_robots)

    # init particles
    particles = [Particle() for _ in range(num_particles)]
    for p in particles:
        p.best_cost = p.evaluate()
        p.best_task_order = p.task_order.copy()
        p.best_assignment = p.assignment.copy()

    global_best = min(particles, key=lambda p: p.best_cost)
    global_best_cost = global_best.best_cost
    global_best_task_order = global_best.best_task_order.copy()
    global_best_assignment = global_best.best_assignment.copy()
    global_best_robot_dist = global_best.robot_dist.copy()

    # simple PSO-like loop (here implemented as particle mutation + local acceptance)
    for _ in range(max_iter):
        for p in particles:
            p.mutate()
            cost = p.evaluate()
            if cost < p.best_cost:
                p.best_cost = cost
                p.best_task_order = p.task_order.copy()
                p.best_assignment = p.assignment.copy()
            if cost < global_best_cost:
                global_best_cost = cost
                global_best_task_order = p.best_task_order.copy()
                global_best_assignment = p.best_assignment.copy()
                global_best_robot_dist = p.robot_dist.copy()

    # build final routes according to global_best
    routes = {}
    total_distances = {}
    for i in range(n_robots):
        assigned_tasks = [
            t for t in global_best_task_order if global_best_assignment[t] == i
        ]
        route = []
        prev = tuple(robot_positions[i])
        dist_sum = 0.0
        for t in assigned_tasks:
            route.append(t)
            route.append(t + m_tasks)
            pick = tuple(task_positions[t])
            drop = tuple(dropoff_positions[t])
            dist_sum += math.hypot(pick[0] - prev[0], pick[1] - prev[1])
            dist_sum += math.hypot(drop[0] - pick[0], drop[1] - pick[1])
            prev = drop
        routes[i] = route
        total_distances[i] = dist_sum

    waypoints = build_waypoints_dict(routes)
    total = sum(total_distances.values())
    return waypoints, total, routes


# --------------------------
# ACO (uses run_seed)
# --------------------------
def run_aco(
    run_seed, num_ants=30, num_iterations=100, alpha=1.0, beta=2.0, rho=0.1, q=100.0
):
    rng = np.random.RandomState(int(run_seed))

    # pheromones
    pher_assign = np.ones((m_tasks, n_robots))
    pher_seq = np.ones((m_tasks, m_tasks))
    heuristic_assign = 1.0 / (
        dist_robot_to_task + 1e-6
    )  # (m_tasks x n_robots transposed)
    # run ants
    best_cost = float("inf")
    best_routes = None

    for _ in range(num_iterations):
        solutions = []
        for _ant in range(num_ants):
            # assignment probabilistically: for each task, sample robot
            assignment = np.zeros(m_tasks, dtype=int)
            for t in range(m_tasks):
                # probabilities over robots
                weights = (pher_assign[t] ** alpha) * (heuristic_assign[:, t] ** beta)
                weights = weights / (weights.sum() + 1e-12)
                # sample using rng choice
                assignment[t] = rng.choice(n_robots, p=weights)

            # produce an order: random permutation then group by assignment
            order = rng.permutation(m_tasks)
            robot_tasks = {
                i: [t for t in order if assignment[t] == i] for i in range(n_robots)
            }

            # compute cost and route
            routes = {}
            total_cost = 0.0
            for i in range(n_robots):
                rt = []
                prev = tuple(robot_positions[i])
                dist_sum = 0.0
                for t in robot_tasks[i]:
                    rt.append(t)
                    rt.append(t + m_tasks)
                    pick = tuple(task_positions[t])
                    drop = tuple(dropoff_positions[t])
                    dist_sum += math.hypot(pick[0] - prev[0], pick[1] - prev[1])
                    dist_sum += math.hypot(drop[0] - pick[0], drop[1] - pick[1])
                    prev = drop
                routes[i] = rt
                total_cost += dist_sum

            solutions.append((assignment.copy(), order.copy(), total_cost, routes))
            # update best
            if total_cost < best_cost:
                best_cost = total_cost
                best_routes = routes.copy()

        # pheromone evaporation
        pher_assign *= 1.0 - rho
        pher_seq *= 1.0 - rho

        # pheromone deposit
        for assignment, order, cost, routes in solutions:
            deposit = q / (cost + 1e-9)
            for t, r in enumerate(assignment):
                pher_assign[t, r] += deposit
            # for sequences inside same robot, reward adjacent pairs
            for i in range(n_robots):
                seq = [x for x in order if assignment[x] == i]
                for k in range(len(seq) - 1):
                    a = seq[k]
                    b = seq[k + 1]
                    pher_seq[a, b] += deposit

    waypoints = build_waypoints_dict(best_routes)
    total = best_cost if best_routes is not None else float("inf")
    return waypoints, total, best_routes


# --------------------------
# Run all methods and print outputs in requested format
# --------------------------
if __name__ == "__main__":
    import pickle

    # MILP (deterministic for this instance)
    milp_waypoints, milp_total, milp_routes = run_milp_assignment_and_tsp()

    print("\n=== MILP + per-robot TSP ===")
    print("WAYPOINTS = {")
    for k, v in milp_waypoints.items():
        print(f'    "{k}": {v},')
    print("}")
    print("Total Distance: {:.2f}".format(milp_total))

    # PSO (seeded)
    pso_waypoints, pso_total, pso_routes = run_pso(INSTANCE_SEED)
    print("\n=== PSO ===")
    print("WAYPOINTS = {")
    for k, v in pso_waypoints.items():
        print(f'    "{k}": {v},')
    print("}")
    print("Total Distance: {:.2f}".format(pso_total))

    # ACO (seeded)
    aco_waypoints, aco_total, aco_routes = run_aco(INSTANCE_SEED)
    print("\n=== ACO ===")
    print("WAYPOINTS = {")
    for k, v in aco_waypoints.items():
        print(f'    "{k}": {v},')
    print("}")
    print("Total Distance: {:.2f}".format(aco_total))

    solutions = {
        "milp": {"waypoints": milp_waypoints, "distance": milp_total},
        "pso": {"waypoints": pso_waypoints, "distance": pso_total},
        "aco": {"waypoints": aco_waypoints, "distance": aco_total},
    }
    with open("/home/hehe/Documents/GitHub/swarm_robots/solutions.pkl", "wb") as f:
        pickle.dump(solutions, f)

    print("Done")