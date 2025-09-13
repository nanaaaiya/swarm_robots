import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import math

#from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def yaw_to_quat(yaw):
    # z-yaw quaternion
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w

def quat_to_yaw(x, y, z, w):
    # yaw from quaternion (Z)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    inspection_route = [ 
        [2.0, -0.5],
        [2.0, 1.0],
        [0.0, 2.0]]  # simulation points


    # Tell AMCL where we are in the MAP ----
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -2.0   # << set near the robot's real start in your map
    initial_pose.pose.position.y = -0.5
    qx, qy, qz, qw = yaw_to_quat(0.0)     # facing +x
    initial_pose.pose.orientation.x = qx
    initial_pose.pose.orientation.y = qy
    initial_pose.pose.orientation.z = qz
    initial_pose.pose.orientation.w = qw
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    while rclpy.ok():

        # Send our route
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.orientation.z = 1.0
        inspection_pose.pose.orientation.w = 0.0
        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            inspection_points.append(deepcopy(inspection_pose))
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(inspection_points)

        # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
        # Simply print the current waypoint ID for the demonstation
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Inspection of shelves complete! Returning to start...')
        elif result == TaskResult.CANCELED:
            print('Inspection of shelving was canceled. Returning to start...')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Inspection of shelving failed! Returning to start...')

        # go back to start
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # navigator.goToPose(initial_pose)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
