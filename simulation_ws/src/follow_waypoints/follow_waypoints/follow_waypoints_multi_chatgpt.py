#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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


def yaw_to_quat(yaw: float):
    """Quaternion for yaw angle (roll=pitch=0)."""
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))  # x, y, z, w


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


def wait_for_localization(nav: BasicNavigator, ns: str, timeout: float = 20.0) -> bool:
    """
    Wait until AMCL publishes /<ns>/amcl_pose once.
    Works across namespaces because subscription is created on the node in that namespace.
    """
    got_first = {"msg": None}

    def _cb(msg: PoseWithCovarianceStamped):
        got_first["msg"] = msg

    sub = nav.create_subscription(PoseWithCovarianceStamped, "amcl_pose", _cb, 10)
    del sub  # keep reference alive via closure

    t0 = time.time()
    while time.time() - t0 < timeout:
        if got_first["msg"] is not None:
            p = got_first["msg"].pose.pose.position
            print(f"[{ns}] localized: x={p.x:.2f}, y={p.y:.2f}")
            return True
        rclpy.spin_once(nav, timeout_sec=0.1)

    print(f"[{ns}] WARNING: no amcl_pose within {timeout:.0f}s (will proceed anyway).")
    return False


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

    # Create one navigator per robot (scoped by namespace)
    navs = {ns: BasicNavigator(namespace=ns) for ns in ROBOT_NAMES}

    # Bring Nav2 stacks up
    for ns, nav in navs.items():
        print(f"[{ns}] waiting for Nav2 to become active…")
        nav.waitUntilNav2Active()
        print(f"[{ns}] Nav2 is active.")

    # (Optional) programmatic initial pose; comment out if your launch publishes /<ns>/initialpose
    for ns, nav in navs.items():
        if ns in INITIAL_POSES:
            x, y, yaw = INITIAL_POSES[ns]
            init = make_pose(x, y, yaw, stamp=nav.get_clock().now().to_msg())
            nav.setInitialPose(init)

    # Wait for AMCL to give at least one pose (avoids sending goals too early)
    for ns, nav in navs.items():
        wait_for_localization(nav, ns, timeout=20.0)

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

    # Monitor progress until all robots finish
    done = {ns: (totals.get(ns, 0) == 0) for ns in ROBOT_NAMES}
    while not all(done.values()):
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
            rclpy.spin_once(nav, timeout_sec=0.05)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
