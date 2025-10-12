import numpy as np

n_robots = 3
m_tasks = 9

np.random.seed(1078408761)
# robot_positions = np.random.rand(n_robots, 2) * 10
task_positions = np.random.rand(m_tasks, 2) * 10
dropoff_positions = np.random.rand(m_tasks, 2) * 10


print("Task Positions:")
# for i, pos in enumerate(task_positions):
#     print(f"Task {i+1}: ({pos[0]:.2f}, {pos[1]:.2f})")
print(task_positions)
print("\nDrop-off Positions:")
print(dropoff_positions)
# for i, pos in enumerate(dropoff_positions):
#     print(f"Drop-off {i+1}: ({pos[0]:.2f}, {pos[1]:.2f})")  

