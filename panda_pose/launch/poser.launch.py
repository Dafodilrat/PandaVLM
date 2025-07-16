from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='panda_pose',
            executable='pose_moveit_controller',
            output='screen'
        )
    ])
