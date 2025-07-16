#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
from collections import deque

from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time


class TransformCollector(Node):
    def __init__(self):
        super().__init__('camera_to_world_tf')

        # TF2 buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)

        self.get_logger().info(
            "TransformCollector started, listening for panda_camera_link → world transforms."
        )

    def get_camera_point(self, u: float, v: float, depth: float) -> PointStamped:
        """Back-project a pixel (u,v,depth) into a PointStamped in camera frame."""
        # Camera intrinsics (fx, fy, cx, cy)
        fx, fy = 554.3827, 554.3827
        cx, cy = 320.0, 240.0

        x_c = (u - cx) * depth / fx
        y_c = (v - cy) * depth / fy
        z_c = depth

        pt = PointStamped()
        pt.header.frame_id = "panda_camera_link"
        pt.point.x = x_c
        pt.point.y = y_c
        pt.point.z = z_c
        return pt

    def convert_to_base_coordinates(self, u: float, v: float, depth: float):
        """
        One-shot: back-project and transform a single point
        from camera frame → world frame using Buffer.transform().
        """
        pt_cam = self.get_camera_point(u, v, depth)

        # 2) Lookup static/dynamic transform at time zero for static frames
        try:
            pt_cam.header.stamp = self.get_clock().now().to_msg()

            static_tf = self.tf_buffer.can_transform(
                "panda_link0",
                pt_cam.header.frame_id,
                pt_cam.header.stamp,
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f'Static lookup failed: {e}')
            return None

        # 3) Apply static transform to camera point
        try:
            pt_base = do_transform_point(pt_cam, static_tf)
            return (pt_base.point.x,
                    pt_base.point.y,
                    pt_base.point.z)
        except Exception as e:
            self.get_logger().error(f'Transform application failed: {e}')
            return None



def main(args=None):
    rclpy.init(args=args)
    node = TransformCollector()

    # example usage after a short delay:
    import time; time.sleep(1.0)
    result = node.convert_to_base_coordinates(400, 300, 2.0)
    if result:
        node.get_logger().info(f"One-shot world point: {result}")

    avg = node.get_latest_world_point(400, 300, 2.0)
    if avg:
        node.get_logger().info(f"Averaged world point: {avg}")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
