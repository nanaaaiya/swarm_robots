#!/usr/bin/env python3
"""
ROS2 (rclpy) distance tracker — writes final CSV in project folder with:
  robot, total distance (km), total time (s)
and a Total row summing the three robots.

Saves to:
  ~/Documents/GitHub/swarm_robots/simulation_ws/src/distance_tracker/robot_metrics.csv
unless you override the output_csv parameter.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import csv
import os
import threading
import time
from threading import Lock
from typing import Dict

def stamp_to_float_secs(stamp):
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
        self.cumdist = 0.0        # meters
        self.start_time = None    # seconds (float)
        self.end_time = None      # seconds (float) set only on stop
        self.moving_time = 0.0    # seconds
        self.stopped = False
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
            # initialize start_time on first sample if not set
            if self.start_time is None:
                self.start_time = t
                self.last_time = t
                self.last_pos = (x, y)
                return

            # if we have no last position (e.g., after explicit start reset), initialize it
            if self.last_pos is None or self.last_time is None:
                self.last_pos = (x, y)
                self.last_time = t
                return

            # ignore out-of-order messages
            if t < self.last_time:
                return

            dx = x - self.last_pos[0]
            dy = y - self.last_pos[1]
            dist = math.hypot(dx, dy)   # meters
            dt = (t - self.last_time) if (self.last_time is not None) else 0.0

            self.cumdist += dist

            speed = linear_speed if linear_speed is not None else (dist / dt if dt > 0 else 0.0)
            if speed > self.vel_thresh and dt > 0:
                self.moving_time += dt

            self.last_pos = (x, y)
            self.last_time = t
            # do NOT set end_time here (end_time only on explicit STOP)

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
                'cumdist_m': float(self.cumdist),
                'total_time_s': float(total_time),
                'moving_time_s': float(self.moving_time),
                'start_time': self.start_time,
                'end_time': self.end_time,
                'stopped': self.stopped
            }

class TeamDistanceTrackerNode(Node):
    def __init__(self):
        super().__init__('team_distance_tracker')

        # Default output path: project src/distance_tracker folder
        default_dir = os.path.expanduser('~/Documents/GitHub/swarm_robots/simulation_ws/src/distance_tracker')
        default_csv = os.path.join(default_dir, 'robot_metrics1.csv')

        # parameters
        self.declare_parameter('robot_list', ['tb1','tb2','tb3'])
        self.declare_parameter('topic_prefix', '')
        self.declare_parameter('pose_topic_suffix', '/odom')
        self.declare_parameter('msg_type', 'odom')
        self.declare_parameter('velocity_threshold', 0.05)
        self.declare_parameter('output_csv', default_csv)
        self.declare_parameter('start_on_topic', False)
        self.declare_parameter('start_topic_name', '/mission_start')
        self.declare_parameter('stop_on_topic', False)
        self.declare_parameter('stop_topic_name', '/mission_stop')
        self.declare_parameter('write_interval', 2.0)
        # per-robot topic config
        self.declare_parameter('use_per_robot_start_topic', True)
        self.declare_parameter('per_robot_start_topic_prefix', '/')
        self.declare_parameter('per_robot_start_topic_suffix', 'mission_start')
        self.declare_parameter('use_per_robot_stop_topic', True)
        self.declare_parameter('per_robot_stop_topic_suffix', 'mission_stop')
        # auto-exit behavior
        self.declare_parameter('auto_exit_when_all_stopped', True)

        self._shutdown_called = False

        # read params defensively
        pl_param = self.get_parameter('robot_list').get_parameter_value()
        if pl_param.type == 17:
            robot_list = pl_param.string_array_value
        else:
            robot_list = self.get_parameter('robot_list').value
        if robot_list is None:
            robot_list = ['tb1','tb2','tb3']

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

        self.use_per_robot_start_topic = self.get_parameter('use_per_robot_start_topic').value
        self.per_robot_start_topic_prefix = self.get_parameter('per_robot_start_topic_prefix').value
        self.per_robot_start_topic_suffix = self.get_parameter('per_robot_start_topic_suffix').value
        self.use_per_robot_stop_topic = self.get_parameter('use_per_robot_stop_topic').value
        self.per_robot_stop_topic_suffix = self.get_parameter('per_robot_stop_topic_suffix').value

        self.auto_exit_when_all_stopped = self.get_parameter('auto_exit_when_all_stopped').value

        # trackers & subscriptions
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

        # global start/stop
        if self.start_on_topic:
            self.create_subscription(Bool, self.start_topic_name, self._start_cb, 10)
            self.get_logger().info(f"Listening for global mission start on {self.start_topic_name}")
        if self.stop_on_topic:
            self.create_subscription(Bool, self.stop_topic_name, self._stop_cb, 10)
            self.get_logger().info(f"Listening for global mission stop on {self.stop_topic_name}")

        # per-robot start subscribers
        if self.use_per_robot_start_topic:
            for r in list(self.trackers.keys()):
                prefix = self.per_robot_start_topic_prefix.rstrip('/')
                tname = f"{prefix}/{r}/{self.per_robot_start_topic_suffix}" if prefix != '' else f"/{r}/{self.per_robot_start_topic_suffix}"
                tname = tname.replace('//','/')
                def make_cb(robot_name):
                    def cb(msg):
                        self._robot_start_cb(robot_name, msg)
                    return cb
                self.create_subscription(Bool, tname, make_cb(r), 10)
                self.get_logger().info(f"Listening for per-robot start on {tname}")

        # per-robot stop subscribers
        if self.use_per_robot_stop_topic:
            for r in list(self.trackers.keys()):
                prefix = self.per_robot_start_topic_prefix.rstrip('/')
                sname = f"{prefix}/{r}/{self.per_robot_stop_topic_suffix}" if prefix != '' else f"/{r}/{self.per_robot_stop_topic_suffix}"
                sname = sname.replace('//','/')
                def make_stop_cb(robot_name):
                    def cb(msg):
                        self._robot_stop_cb(robot_name, msg)
                    return cb
                self.create_subscription(Bool, sname, make_stop_cb(r), 10)
                self.get_logger().info(f"Listening for per-robot stop on {sname}")

        # periodic writer
        self.writer_timer = self.create_timer(write_interval, self._periodic_write)
        self.get_logger().info("team_distance_tracker (ROS2) ready.")

    # global start: reset all
    def _start_cb(self, msg: Bool):
        if msg.data:
            now_msg = self.get_clock().now().to_msg()
            now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
            for t in self.trackers.values():
                with t.lock:
                    t.start_time = now
                    t.last_time = None
                    t.last_pos = None
                    t.cumdist = 0.0
                    t.moving_time = 0.0
                    t.stopped = False
                    t.end_time = None
            self.get_logger().info(f"Global Mission START at {now:.3f}")

    # per-robot start
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
            t.stopped = False
            t.end_time = None
        self.get_logger().info(f"Mission START for {robot_name} at {now:.3f}")

    # per-robot stop
    def _robot_stop_cb(self, robot_name: str, msg: Bool):
        if not msg.data:
            return
        if robot_name not in self.trackers:
            self.get_logger().warn(f"Received stop for unknown robot '{robot_name}'")
            return
        now_msg = self.get_clock().now().to_msg()
        now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        t = self.trackers[robot_name]
        with t.lock:
            t.end_time = now
            t.stopped = True
        self.get_logger().info(f"Mission STOP for {robot_name} at {now:.3f}")

        if self.auto_exit_when_all_stopped and self._all_robots_stopped():
            self.get_logger().info("All robots reported STOP — shutting down tracker.")
            self._attempt_shutdown()

    # global stop
    def _stop_cb(self, msg: Bool):
        if msg.data:
            now_msg = self.get_clock().now().to_msg()
            now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
            for t in self.trackers.values():
                with t.lock:
                    t.end_time = now
                    t.stopped = True
            self.get_logger().info(f"Global Mission STOP at {now:.3f}")
            self._write_csv(final=True)
            if self.auto_exit_when_all_stopped:
                self._attempt_shutdown()

    def _all_robots_stopped(self):
        for t in self.trackers.values():
            if not t.stopped:
                return False
        return True

    def _attempt_shutdown(self):
        if self._shutdown_called:
            return
        self._shutdown_called = True
        # final write before shutdown
        self._write_csv(final=True)

        try:
            self.get_logger().info("Tracker initiating rclpy.shutdown() (scheduled)...")
        except Exception:
            pass

        def do_shutdown():
            # small pause so log messages flush
            time.sleep(0.12)
            try:
                rclpy.shutdown()
            except Exception:
                pass
            # final fallback
            time.sleep(0.2)
            try:
                os._exit(0)
            except Exception:
                pass

        t = threading.Thread(target=do_shutdown, daemon=True)
        t.start()

    def _periodic_write(self):
        self._write_csv(final=False)

    def _write_csv(self, final=False):
        # CSV columns: robot, total distance (m), total time (s)
        outpath = os.path.abspath(self.output_csv)
        outdir = os.path.dirname(outpath)
        try:
            os.makedirs(outdir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f"Could not create output directory '{outdir}': {e}")
            return

        # prepare rows
        rows = []
        total_dist_m = 0.0
        total_time_s = 0.0
        for name, tr in self.trackers.items():
            s = tr.get_summary()
            dist_m = s['cumdist_m']      # already in meters
            t_s = s['total_time_s']
            rows.append((name, dist_m, t_s))
            total_dist_m += dist_m
            total_time_s += t_s

        # write CSV (overwrite each run to keep only final table)
        try:
            with open(outpath, 'w', newline='') as f:
                writer = csv.writer(f)
                # header
                writer.writerow(['robot', 'total distance (m)', 'total time (s)'])
                # robot rows
                for rname, dm, ts in rows:
                    writer.writerow([rname, f"{dm:.3f}", f"{ts:.2f}"])
                # blank row then Total row
                writer.writerow([])
                writer.writerow(['Total', f"{total_dist_m:.3f}", f"{total_time_s:.2f}"])
                try:
                    f.flush()
                    os.fsync(f.fileno())
                except Exception:
                    pass
            # log and print file location so user notices before program exits
            msg = f"Final metrics written to {outpath}"
            try:
                self.get_logger().info(msg)
            except Exception:
                pass
            print(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV '{outpath}': {e}")

    def on_shutdown(self):
        if not self._shutdown_called:
            try:
                self.get_logger().info("Shutting down — writing final CSV and summary.")
            except Exception:
                pass
            self._write_csv(final=True)

        total_team_dist_m = 0.0
        for name, t in self.trackers.items():
            s = t.get_summary()
            try:
                self.get_logger().info(
                    f"{name} | dist={s['cumdist_m']:.3f} m | total_time={s['total_time_s']:.2f} s | stopped={s.get('stopped', False)}"
                )
            except Exception:
                pass
            total_team_dist_m += s['cumdist_m']
        try:
            self.get_logger().info(f"Team total distance: {total_team_dist_m:.3f} m")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TeamDistanceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not node._shutdown_called:
                node.on_shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
