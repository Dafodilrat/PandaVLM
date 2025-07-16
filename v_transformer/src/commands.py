#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclpyDuration

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, OrientationConstraint
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from action_msgs.msg import GoalStatus

from cordinate_converter import TransformCollector


class PandaCommander(Node):
    def __init__(self):
        super().__init__('panda_commander')
        cb_group = ReentrantCallbackGroup()

        # TF‐based converter
        self.camera_fns = TransformCollector()

        # ——— MoveGroup action client ———
        from rclpy.action import ActionClient
        self.move_client = ActionClient(self, MoveGroup, 'move_action', callback_group=cb_group)
        self.get_logger().info('Waiting for move_action server...')
        self.move_client.wait_for_server()
        self.get_logger().info('Connected to move_action')

        # ——— Gripper: publish JointTrajectory directly ———
        self.grip_pub = self.create_publisher(
            JointTrajectory,
            'gripper_trajectory_controller/joint_trajectory',
            10
        )
        self.get_logger().info('Publishing gripper commands to gripper_trajectory_controller/joint_trajectory')

        # ——— Planning scene service clients ———
        self.scene_get = self.create_client(GetPlanningScene, 'get_planning_scene', callback_group=cb_group)
        self.scene_apply = self.create_client(ApplyPlanningScene, 'apply_planning_scene', callback_group=cb_group)
        self.get_logger().info('Waiting for planning-scene services...')
        self.scene_get.wait_for_service()
        self.scene_apply.wait_for_service()
        self.get_logger().info('Connected to planning-scene services')


    # alias so execute_sequence can call self.move_to()
    def move_to(self, x: float, y: float, z: float) -> bool:
        return self.move_to_pose(x, y, z)


    def move_to_pose(self, x: float, y: float, z: float) -> bool:
        """Plan & execute end-effector move to (x,y,z)."""
        self.get_logger().info(f'Moving to ({x:.3f}, {y:.3f}, {z:.3f})')

        # build PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        # build MotionPlanRequest
        req = MotionPlanRequest()
        req.group_name = 'panda_arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        # orientation constraint on link8
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = 'panda_link8'
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0

        c = Constraints()
        c.orientation_constraints = [oc]
        req.goal_constraints = [c]
        req.start_state.is_diff = True

        # send goal
        from moveit_msgs.action import MoveGroup as MoveGroupAct
        goal = MoveGroupAct.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True

        gh = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh)
        handle = gh.result()
        if not handle.accepted:
            self.get_logger().error('Move goal rejected')
            return False

        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        status = rf.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Move succeeded')
            return True
        else:
            self.get_logger().error('Move failed')
            return False


    def _pub_gripper(self, positions):
        """Publish a simple JointTrajectory to the gripper controller."""
        traj = JointTrajectory()
        traj.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = RclpyDuration(sec=1)
        traj.points = [pt]

        self.grip_pub.publish(traj)
        self.get_logger().info(f'Published gripper positions: {positions}')
        return True


    def open_gripper(self) -> bool:
        return self._pub_gripper([0.04, 0.04])


    def close_gripper(self) -> bool:
        return self._pub_gripper([0.0, 0.0])


    def get_planning_scene(self, components: int):
        req = GetPlanningScene.Request()
        req.components.components = components
        fut = self.scene_get.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().scene


    def apply_planning_scene(self, scene_diff):
        req = ApplyPlanningScene.Request()
        req.scene = scene_diff
        fut = self.scene_apply.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().success


    def execute_sequence(self, data, depth_image):
        actions = data['actions']
        for action in actions:
            self.get_logger().info(f'Executing action: {action}')

            if action.startswith("move_to"):
                x_img, y_img = data["position"]
                z = float(depth_image[int(y_img)][int(x_img)])
                self.get_logger().info(f"Image coords ({x_img},{y_img},{z})")
                world = self.camera_fns.convert_to_base_coordinates(x_img, y_img, z)
                if not world:
                    self.get_logger().error("Failed to convert to world coords")
                    return
                x, y, z = world
                if not self.move_to(x, y, z):
                    self.get_logger().error("Aborting on failed move_to()")
                    return

            elif action.startswith("close_gripper"):
                self.close_gripper()

            elif action.startswith("open_gripper"):
                self.open_gripper()

            elif action.startswith("lift"):
                dz = float(action.split("(")[1].rstrip(")"))
                self.move_to_pose(x, y, z + dz)


def main(args=None):
    rclpy.init(args=args)
    node = PandaCommander()

    # quick smoke test
    node.move_to_pose(0.4, 0.0, 0.3)
    node.close_gripper()
    node.move_to_pose(0.4, 0.0, 0.5)
    node.open_gripper()

    node.get_logger().info('Initialization done, spinning...')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
