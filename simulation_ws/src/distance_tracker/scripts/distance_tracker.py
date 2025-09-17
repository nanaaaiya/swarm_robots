#!/usr/bin/env python3
"""
ROS2 (rclpy) node to track cumulative distance and times for multiple robots.
Supports:
 - global mission start topic (std_msgs/Bool) via parameters start_on_topic / start_topic_name
 - per-robot mission start topics (std_msgs/Bool) at /<robot>/mission_start (enabled via use_per_robot_start_topic)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import csv
import os
from threading import Lock
from typing import Dict

def stamp_to_float_secs(stamp):
    """Convert builtin_interfaces/Time-like object to float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9

class RobotTracker:
    def __init__(self, name, topic, msg_type='odom', vel_thresh=0.05, node: Node = None):
        self.name = name
        self.topic = topic
        self.msg_type = msg_type
        self.vel_thresh = vel_thresh
        self.node = node

        self.last_pos = None
        self.last_time = None
        self.cumdist = 0.0
        self.start_time = None
        self.end_time = None
        self.moving_time = 0.0
        self.lock = Lock()

    def handle_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        t = stamp_to_float_secs(msg.header.stamp)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._update(pos.x, pos.y, t, speed)

    def handle_pose(self, msg: PoseStamped):
        pos = msg.pose.position
        t = stamp_to_float_secs(msg.header.stamp)
        self._update(pos.x, pos.y, t, linear_speed=None)

    def _update(self, x, y, t, linear_speed=None):
        with self.lock:
            # If start_time hasn't been set at all, use this sample to set it (old behavior)
            if self.start_time is None:
                self.start_time = t
                self.last_time = t
                self.last_pos = (x, y)
                return

            # If start_time is set but we haven't recorded a last_pos (e.g. per-robot start reset),
            # initialize last_pos/last_time from this first incoming sample and do not compute distance yet.
            if self.last_pos is None or self.last_time is None:
                self.last_pos = (x, y)
                self.last_time = t
                return

            # ignore out-of-order
            if t < self.last_time:
                return

            dx = x - self.last_pos[0]
            dy = y - self.last_pos[1]
            dist = math.hypot(dx, dy)
            dt = (t - self.last_time) if (self.last_time is not None) else 0.0

            self.cumdist += dist

            speed = linear_speed if linear_speed is not None else (dist / dt if dt > 0 else 0.0)
            if speed > self.vel_thresh and dt > 0:
                self.moving_time += dt

            self.last_pos = (x, y)
            self.last_time = t
            self.end_time = t


    def get_summary(self):
        with self.lock:
            if self.start_time is None:
                total_time = 0.0
            elif self.end_time is None:
                now_msg = self.node.get_clock().now().to_msg()
                now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
                total_time = now - self.start_time
            else:
                total_time = self.end_time - self.start_time
            return {
                'name': self.name,
                'cumdist': float(self.cumdist),
                'total_time': float(total_time),
                'moving_time': float(self.moving_time),
                'start_time': self.start_time,
                'end_time': self.end_time
            }

class TeamDistanceTrackerNode(Node):
    def __init__(self):
        super().__init__('team_distance_tracker')
        # declare parameters
        self.declare_parameter('robot_list', ['robot1','robot2','robot3'])
        self.declare_parameter('topic_prefix', '')
        self.declare_parameter('pose_topic_suffix', '/odom')
        self.declare_parameter('msg_type', 'odom')  # 'odom' or 'pose'
        self.declare_parameter('velocity_threshold', 0.05)
        self.declare_parameter('output_csv', '/tmp/robot_metrics.csv')
        self.declare_parameter('start_on_topic', False)
        self.declare_parameter('start_topic_name', '/mission_start')
        self.declare_parameter('stop_on_topic', False)
        self.declare_parameter('stop_topic_name', '/mission_stop')
        self.declare_parameter('write_interval', 5.0)

        # NEW: per-robot start topic support
        self.declare_parameter('use_per_robot_start_topic', True)
        self.declare_parameter('per_robot_start_topic_prefix', '/')  # results like: /<robot>/mission_start
        self.declare_parameter('per_robot_start_topic_suffix', 'mission_start')  # default suffix

        # get params (defensive read)
        pl_param = self.get_parameter('robot_list').get_parameter_value()
        if pl_param.type == 17:  # string_array
            robot_list = pl_param.string_array_value
        else:
            robot_list = self.get_parameter('robot_list').value
        if robot_list is None:
            robot_list = ['robot1','robot2','robot3']

        topic_prefix = self.get_parameter('topic_prefix').value
        pose_suffix = self.get_parameter('pose_topic_suffix').value
        msg_type = self.get_parameter('msg_type').value
        vel_thresh = self.get_parameter('velocity_threshold').value
        self.output_csv = self.get_parameter('output_csv').value
        self.start_on_topic = self.get_parameter('start_on_topic').value
        self.start_topic_name = self.get_parameter('start_topic_name').value
        self.stop_on_topic = self.get_parameter('stop_on_topic').value
        self.stop_topic_name = self.get_parameter('stop_topic_name').value
        write_interval = self.get_parameter('write_interval').value

        # new per-robot params
        self.use_per_robot_start_topic = self.get_parameter('use_per_robot_start_topic').value
        self.per_robot_start_topic_prefix = self.get_parameter('per_robot_start_topic_prefix').value
        self.per_robot_start_topic_suffix = self.get_parameter('per_robot_start_topic_suffix').value

        # trackers and subscriptions
        self.trackers: Dict[str, RobotTracker] = {}
        for r in robot_list:
            prefix = topic_prefix.rstrip('/')
            topic = f"{prefix}/{r}{pose_suffix}" if prefix != '' else f"/{r}{pose_suffix}"
            topic = topic.replace('//','/')
            self.get_logger().info(f"Tracking {r} on topic {topic} (msg={msg_type})")
            tracker = RobotTracker(r, topic, msg_type, vel_thresh, node=self)
            self.trackers[r] = tracker
            if msg_type == 'odom':
                self.create_subscription(Odometry, topic, tracker.handle_odom, 10)
            else:
                self.create_subscription(PoseStamped, topic, tracker.handle_pose, 10)

        # optional start/stop listeners (global)
        if self.start_on_topic:
            self.create_subscription(Bool, self.start_topic_name, self._start_cb, 10)
            self.get_logger().info(f"Listening for global mission start on {self.start_topic_name}")
        if self.stop_on_topic:
            self.create_subscription(Bool, self.stop_topic_name, self._stop_cb, 10)
            self.get_logger().info(f"Listening for global mission stop on {self.stop_topic_name}")

        # NEW: subscribe to per-robot start topics if enabled
        if self.use_per_robot_start_topic:
            for r in list(self.trackers.keys()):
                # build topic like: /<robot>/mission_start or prefix + robot + suffix
                prefix = self.per_robot_start_topic_prefix.rstrip('/')
                tname = f"{prefix}/{r}/{self.per_robot_start_topic_suffix}" if prefix != '' else f"/{r}/{self.per_robot_start_topic_suffix}"
                tname = tname.replace('//','/')
                # closure to capture robot name
                def make_cb(robot_name):
                    def cb(msg):
                        self._robot_start_cb(robot_name, msg)
                    return cb
                self.create_subscription(Bool, tname, make_cb(r), 10)
                self.get_logger().info(f"Listening for per-robot start on {tname}")

        # periodic writer timer
        self.writer_timer = self.create_timer(write_interval, self._periodic_write)

        self.get_logger().info("team_distance_tracker (ROS2) ready.")

    # global start callback (starts/resets all trackers)
    def _start_cb(self, msg: Bool):
        if msg.data:
            now_msg = self.get_clock().now().to_msg()
            now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-99
            for t in self.trackers.values():
                with t.lock:
                    t.start_time = now
                    t.last_time = None
                    t.last_pos = None
                    t.cumdist = 0.0
                    t.moving_time = 0.0
            self.get_logger().info(f"Global Mission START at {now:.3f}")

    # per-robot start callback
    def _robot_start_cb(self, robot_name: str, msg: Bool):
        if not msg.data:
            return
        if robot_name not in self.trackers:
            self.get_logger().warn(f"Received start for unknown robot '{robot_name}'")
            return
        now_msg = self.get_clock().now().to_msg()
        now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        t = self.trackers[robot_name]
        with t.lock:
            t.start_time = now
            t.last_time = None
            t.last_pos = None
            t.cumdist = 0.0
            t.moving_time = 0.0
        self.get_logger().info(f"Mission START for {robot_name} at {now:.3f}")

    def _stop_cb(self, msg: Bool):
        if msg.data:
            now_msg = self.get_clock().now().to_msg()
            now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
            for t in self.trackers.values():
                with t.lock:
                    t.end_time = now
            self.get_logger().info(f"Global Mission STOP at {now:.3f}")
            self._write_csv(final=True)

    def _periodic_write(self):
        self._write_csv(final=False)

    def _write_csv(self, final=False):
        header = ['robot','timestamp','cumdist_m','total_time_s','moving_time_s','start_time','end_time']
        write_header = not os.path.exists(self.output_csv)
        try:
            with open(self.output_csv, 'a', newline='') as f:
                writer = csv.writer(f)
                if write_header:
                    writer.writerow(header)
                ts = self.get_clock().now().nanoseconds * 1e-9
                for name, t in self.trackers.items():
                    s = t.get_summary()
                    writer.writerow([name, ts, s['cumdist'], s['total_time'], s['moving_time'], s['start_time'], s['end_time']])
            if final:
                self.get_logger().info(f"Final metrics written to {self.output_csv}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

    def on_shutdown(self):
        self.get_logger().info("Shutting down — writing final CSV and summary.")
        self._write_csv(final=True)
        total_team_dist = 0.0
        for name, t in self.trackers.items():
            s = t.get_summary()
            self.get_logger().info(f"{name} | dist={s['cumdist']:.3f} m | total_time={s['total_time']:.2f} s | moving={s['moving_time']:.2f} s")
            total_team_dist += s['cumdist']
        self.get_logger().info(f"Team total distance: {total_team_dist:.3f} m")

def main(args=None):
    rclpy.init(args=args)
    node = TeamDistanceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
