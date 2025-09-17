import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

ROBOT_NAMES = ["tb1", "tb2", "tb3"]

# WAYPOINTS = {
#     "tb1": [(0.5, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 2.0, math.pi/2)],
#     "tb2": [(4.5, 0.0, math.pi), (3.0, -1.0, math.pi), (3.0, -2.0, -math.pi/2)],
#     # "tb3": [(0.0, 3.5, 0.0), (1.5, 3.5, 0.0), (1.5, 2.0, -math.pi/2)],
#     "tb3": [(0.0, 1.0, 0.0), (0.5, 1.0, 0.0), (0.5, 1.0, 0.0)],
# }

WAYPOINTS = {
    "tb1": [(0.5, 1.0, 0.0), (2.0, 2.0, 0.0), (0.0, 1.0, 0.0), (0.5, 1.0, 0.0), (0.5, 0.0, 0.0)],
    "tb2": [(4.5, 0.0, 0.0), (3.0, -1.0, 0.0), (3.0, -2.0, 0.0)],
    "tb3": [(2.0, 0.0, 0.0)],
}

INITIAL_POSES = {
    "tb1": (0.0, 0.0, 0.0),
    "tb2": (5.0, 0.0, math.pi),
    "tb3": (0.0, 4.0, 0.0),
}

def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w

def make_pose(x: float, y: float, yaw: float, frame: str = "map", stamp=None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    if stamp is not None:
        ps.header.stamp = stamp
    qx, qy, qz, qw = yaw_to_quat(yaw)
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps

def result_to_str(res) -> str:
    try:
        return res.name
    except Exception:
        if res == TaskResult.SUCCEEDED: return "SUCCEEDED"
        if res == TaskResult.CANCELED:  return "CANCELED"
        if res == TaskResult.FAILED:    return "FAILED"
        return str(res)

def main():
    rclpy.init()

    # One navigator per namespace; spin them together
    navs = {ns: BasicNavigator(namespace=ns) for ns in ROBOT_NAMES}
    executor = MultiThreadedExecutor(num_threads=8)  # pick a sensible number for your CPU
    for nav in navs.values():
        executor.add_node(nav)

    # Set initial poses before activating Nav2
    for ns, nav in navs.items():
        if ns in INITIAL_POSES:
            x, y, yaw = INITIAL_POSES[ns]
            nav.setInitialPose(make_pose(x, y, yaw, stamp=nav.get_clock().now().to_msg()))

    # Wait Nav2 up
    for ns, nav in navs.items():
        print(f"[{ns}] waiting for Nav2 to become active…")
        nav.waitUntilNav2Active()
        print(f"[{ns}] Nav2 is active.")

    # Send waypoint batches once
    totals = {}
    for ns, nav in navs.items():
        wps_xyz = WAYPOINTS.get(ns, [])
        totals[ns] = len(wps_xyz)
        if totals[ns] == 0:
            print(f"[{ns}] no waypoints configured, skipping.")
            continue
        stamp = nav.get_clock().now().to_msg()
        wps = [make_pose(x, y, yaw, stamp=stamp) for (x, y, yaw) in wps_xyz]
        from std_msgs.msg import Bool
        print(f"[{ns}] sending {len(wps)} waypoints…")
        # publish per-robot mission_start so the tracker knows mission begins for this robot
        try:
            start_pub = nav.create_publisher(Bool, f'/{ns}/mission_start', 10)
            # publish immediately
            start_pub.publish(Bool(data=True))
        except Exception as e:
            print(f"[{ns}] warning: could not publish mission_start: {e}")
        nav.followWaypoints(wps)

    # Monitor all robots concurrently
    done = {ns: (totals.get(ns, 0) == 0) for ns in ROBOT_NAMES}
    try:
        while not all(done.values()):
            executor.spin_once(timeout_sec=0.05)
            for ns, nav in navs.items():
                if done[ns]:
                    continue
                fb = nav.getFeedback()
                if fb is not None:
                    print(f"[{ns}] at waypoint {fb.current_waypoint + 1}/{totals[ns]}")
                if nav.isTaskComplete():
                    res = nav.getResult()
                    print(f"[{ns}] result: {result_to_str(res)}")
                    done[ns] = True
    except KeyboardInterrupt:
        print("\n[main] KeyboardInterrupt → canceling active tasks…")
        for ns, nav in navs.items():
            try:
                nav.cancelNavTask()
            except Exception:
                pass
    finally:
        for nav in navs.values():
            executor.remove_node(nav)
            nav.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
