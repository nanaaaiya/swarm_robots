import subprocess
import pickle
import time

# Commands
cmds = [
    "/home/hehe/Documents/GitHub/swarm_robots/.venv/bin/python /home/hehe/Documents/GitHub/swarm_robots/algorithm/scripts/generate_seeds.py",
    "/home/hehe/Documents/GitHub/swarm_robots/.venv/bin/python /home/hehe/Documents/GitHub/swarm_robots/algorithm/scripts/maincom.py",
    "ros2 launch turtlebot3_multi_robot tb_customworld.launch.py",
    "ros2 launch distance_tracker distance_tracker.launch.py",
    "/home/hehe/Documents/GitHub/swarm_robots/.venv/bin/python /home/hehe/Documents/GitHub/swarm_robots/simulation_ws/src/follow_waypoints/follow_waypoints/follow_waypoints_multi.py",
]


def run_command(cmd):
    """Run a command and capture its output (blocking)."""
    process = subprocess.Popen(
        cmd, shell=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        print(f"Error running command:\n{stderr}")
    return stdout.strip()


import subprocess
import os
import signal
import sys
import time


def start_process(cmd):
    """
    Start a process in a new process group so we can terminate the whole group later.
    Returns the Popen object.
    """
    return subprocess.Popen(
        cmd,
        shell=True,
        # start_new_session=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def stop_process(proc, timeout=3):
    """
    Terminate the whole process group/session for proc.
    Attempts graceful shutdown then kills if needed.
    """
    if proc is None:
        return

    if proc.poll() is not None:
        # already exited
        return

    try:
        # Kill the whole process group on POSIX
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)  # polite
        except Exception:
            # fallback: kill the single process
            proc.terminate()

        # wait briefly for graceful exit
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # force kill
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                proc.kill()
            proc.wait()
    except Exception as e:
        print("Error stopping process:", e, file=sys.stderr)
        try:
            proc.kill()
        except Exception:
            pass


# Step 1: Run the first command (generate_seeds.py)
print("Running seed generator...")
output = run_command(cmds[0])
print("Command output:\n", output)

# Step 2: Extract the seeds
seeds = [line.strip() for line in output.split("\n") if line.strip().isdigit()]
if not seeds:
    print("\nNo seeds found in the output!")
    exit()

# Step 3: Process each seed
for seed in seeds:
    print(f"\n=== Running for Seed: {seed} ===")

    # Write seed to file
    with open("seeds.txt", "w") as f:
        f.write(seed)

    # Run maincom.py
    output = run_command(cmds[1])
    print("Output from maincom.py:\n", output)
    if "Done" not in output:
        print(f"Error: 'Done' not found in output for seed {seed}")
        continue

    print("Running waypoint follower...")
    with open("solutions.pkl", "rb") as f:
        solutions = pickle.load(f)

    for name, data in solutions.items():
        waypoints = data.get("waypoints")
        print(f"Solution: {name}")
        with open("current_solution.pkl", "wb") as f_sol:
            pickle.dump(waypoints, f_sol)

        # Loop to allow restart on 'q'
        while True:
            print("\nLaunching simulation...")
            sim_proc = start_process(cmds[2])
            user_input = (
                input(
                    "Press Enter when the simulation is ready, or type 'q' to restart it: "
                )
                .strip()
                .lower()
            )

            if user_input == "q":
                print("Restarting simulation...")
                stop_process(sim_proc)
                continue  # restart loop

            print("Simulation running.")
            break  # exit the restart loop

        # Start distance tracker
        dist_proc = start_process(cmds[3])
        input("Press Enter when the distance tracker is running...")
        print("Distance tracker running.")

        # Run waypoint follower (blocking)
        output = run_command(cmds[4])
        print("Output from waypoint follower:\n", output)

        # Stop background processes
        print("Stopping background processes...")
        stop_process(dist_proc)
        stop_process(sim_proc)

    input("Press Enter to continue to the next seed...")

print("\nAll seeds processed successfully.")
