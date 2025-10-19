import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

ROBOT_NAMES = ["tb1", "tb2", "tb3"]

WAYPOINTS = {
    "tb1": [(0.09, 1.124, 0.0), (1.64, 0.601, 0.0), (1.743, 1.27, 0.0), (4.749, 1.098, 0.0), (1.485, 1.71, 0.0), (4.234, 3.19, 0.0), (0.564, 1.416, 0.0), (3.629, 3.443, 0.0)],
    "tb2": [(4.796, 0.353, 0.0), (4.433, 4.014, 0.0), (3.959, 3.488, 0.0), (3.115, 2.173, 0.0), (4.214, 1.36, 0.0), (1.237, 3.299, 0.0), (4.111, 0.044, 0.0), (3.571, 1.82, 0.0)],
    "tb3": [(0.324, 3.971, 0.0), (0.32, 3.365, 0.0), (0.24, 2.585, 0.0), (0.953, 2.227, 0.0), (0.796, 2.714, 0.0), (1.009, 0.651, 0.0), (1.864, 2.1, 0.0), (2.992, 1.957, 0.0), (3.026, 3.086, 0.0), (2.575, 4.23, 0.0), (2.403, 4.518, 0.0), (4.881, 0.858, 0.0), (2.073, 2.916, 0.0), (4.418, 2.619, 0.0), (4.549, 4.995, 0.0), (1.391, 1.22, 0.0), (1.98, 2.784, 0.0), (4.185, 3.347, 0.0), (1.063, 3.72, 0.0), (2.62, 1.817, 0.0), (0.138, 2.401, 0.0), (0.411, 2.435, 0.0), (0.188, 4.279, 0.0), (3.317, 0.856, 0.0)],
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
            stop_pub = nav.create_publisher(Bool, f'/{ns}/mission_stop', 10)
            # publish start
            start_pub.publish(Bool(data=True))
        except Exception as e:
            print(f"[{ns}] warning: could not publish mission_start/stop publishers: {e}")
        nav.followWaypoints(wps)
        # store stop_pub somewhere? you can attach to nav object:
        nav._mission_stop_pub = stop_pub

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
                    try:
                        if hasattr(nav, '_mission_stop_pub') and nav._mission_stop_pub is not None:
                            nav._mission_stop_pub.publish(Bool(data=True))
                    except Exception as e:
                        print(f"[{ns}] warning: failed to publish mission_stop: {e}")
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
