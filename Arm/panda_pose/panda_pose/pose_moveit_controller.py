#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys

class PoseMoveitController(Node):
    def __init__(self):
        super().__init__('pose_moveit_controller')
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('PoseMoveitController ready.')

    def pose_callback(self, msg):
        self.get_logger().info(f"Received pose: {msg.pose.position}")
        self.move_group.set_pose_target(msg)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            self.get_logger().info("Motion succeeded.")
        else:
            self.get_logger().warn("Motion failed.")

def main(args=None):
    rclpy.init(args=args)
    node = PoseMoveitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
