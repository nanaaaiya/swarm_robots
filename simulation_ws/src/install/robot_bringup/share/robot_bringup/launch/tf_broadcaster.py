#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from rclpy.time import Time


class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)  # 10 Hz

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # Example: Simple translation and rotation
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(math.pi / 4)
        t.transform.rotation.w = math.cos(math.pi / 4)

        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
