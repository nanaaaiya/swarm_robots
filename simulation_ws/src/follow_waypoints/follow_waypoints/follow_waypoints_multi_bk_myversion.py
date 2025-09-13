import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import math
from rclpy.executors import MultiThreadedExecutor

#from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Namespaces must match your launch (tb1/tb2/tb3)
ROBOT_NAMES = ["tb1", "tb2", "tb3"]

# Per-robot waypoints in MAP frame (x, y, yaw [rad])
WAYPOINTS = {
    "tb1": [
        (0.5, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 2.0, math.pi / 2),
    ],
    "tb2": [
        (4.5, 0.0, math.pi),
        (3.0, -1.0, math.pi),
        (3.0, -2.0, -math.pi / 2),
    ],
    "tb3": [
        (0.0, 3.5, 0.0),
        (1.5, 3.5, 0.0),
        (1.5, 2.0, -math.pi / 2),
    ],
}

# (Optional) If you prefer to set initial pose via API instead of your launch, add values here
INITIAL_POSES = {
    "tb1": (0.0, 0.0, 0.0),
    "tb2": (5.0, 0.0, 3.1416),
    "tb3": (0.0, 4.0, 0.0),
}

def yaw_to_quat(yaw):
    # z-yaw quaternion
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w

def quat_to_yaw(x, y, z, w):
    # yaw from quaternion (Z)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def make_pose(x: float, y: float, yaw: float, frame: str = "map", stamp=None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    if stamp is not None:
        ps.header.stamp = stamp
    qx, qy, qz, qw = yaw_to_quat(yaw)
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.1
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps

def result_to_str(res) -> str:
    try:
        # Newer enums have .name; older may be int
        return res.name
    except Exception:
        if res == TaskResult.SUCCEEDED:
            return "SUCCEEDED"
        if res == TaskResult.CANCELED:
            return "CANCELED"
        if res == TaskResult.FAILED:
            return "FAILED"
        return str(res)


def main():
    rclpy.init()

    navigator = BasicNavigator()
    # Create one navigator per robot (scoped by namespace)
    navs = {ns: BasicNavigator(namespace=ns) for ns in ROBOT_NAMES}

    # Spin all navigator nodes together
    executor = MultiThreadedExecutor()
    for nav in navs.values():
        executor.add_node(nav)

    # Wait for navigation to fully activate
    for ns, nav in navs.items():
        print(f"[{ns}] waiting for Nav2 to become active…")
        nav.waitUntilNav2Active()
        print(f"[{ns}] Nav2 is active.")

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    inspection_route = [ 
        [2.0, -0.5],
        [2.0, 1.0],
        [0.0, 2.0]]  # simulation points


    # # Tell AMCL where we are in the MAP ----
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = -2.0   # << set near the robot's real start in your map
    # initial_pose.pose.position.y = -0.5
    # qx, qy, qz, qw = yaw_to_quat(0.0)     # facing +x
    # initial_pose.pose.orientation.x = qx
    # initial_pose.pose.orientation.y = qy
    # initial_pose.pose.orientation.z = qz
    # initial_pose.pose.orientation.w = qw
    # navigator.setInitialPose(initial_pose)

    for ns, nav in navs.items():
        if ns in INITIAL_POSES:
            x, y, yaw = INITIAL_POSES[ns]
            init = make_pose(x, y, yaw, stamp=nav.get_clock().now().to_msg())
            nav.setInitialPose(init)

    while rclpy.ok():

        # Send our route
        # inspection_points = []
        # inspection_pose = PoseStamped()
        # inspection_pose.header.frame_id = 'map'
        # inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        # inspection_pose.pose.orientation.z = 1.0
        # inspection_pose.pose.orientation.w = 0.0
        # for pt in inspection_route:
        #     inspection_pose.pose.position.x = pt[0]
        #     inspection_pose.pose.position.y = pt[1]
        #     inspection_points.append(deepcopy(inspection_pose))
        # nav_start = navigator.get_clock().now()
        # navigator.followWaypoints(inspection_points)

        # Build and send waypoint lists
        totals = {}
        for ns, nav in navs.items():
            wps_xyz = WAYPOINTS.get(ns, [])
            totals[ns] = len(wps_xyz)
            if totals[ns] == 0:
                print(f"[{ns}] no waypoints configured, skipping.")
                continue
            stamp = nav.get_clock().now().to_msg()
            wps = [make_pose(x, y, yaw, stamp=stamp) for (x, y, yaw) in wps_xyz]
            print(f"[{ns}] sending {len(wps)} waypoints…")
            nav.followWaypoints(wps)

        # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
        # Simply print the current waypoint ID for the demonstation
        # i = 0
        # while not navigator.isTaskComplete():
        #     i = i + 1
        #     feedback = navigator.getFeedback()
        #     if feedback and i % 5 == 0:
        #         print('Executing current waypoint: ' +
        #             str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        # result = navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Inspection of shelves complete! Returning to start...')
        # elif result == TaskResult.CANCELED:
        #     print('Inspection of shelving was canceled. Returning to start...')
        #     exit(1)
        # elif result == TaskResult.FAILED:
        #     print('Inspection of shelving failed! Returning to start...')

        # Monitor progress until all robots finish
        done = {ns: (totals.get(ns, 0) == 0) for ns in ROBOT_NAMES}
        while not all(done.values()):
            executor.spin_once(timeout_sec=0.05)  # progress all nodes
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

                # Let callbacks process
            #    rclpy.spin_once(nav, timeout_sec=0.05)

        # go back to start
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # navigator.goToPose(initial_pose)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
