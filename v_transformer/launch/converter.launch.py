from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='panda_img',
            executable='camera_to_world_tf',
            name='camera_to_world_tf',
            output='screen',
            parameters=[]  # you can add params here if needed
        )
    ])