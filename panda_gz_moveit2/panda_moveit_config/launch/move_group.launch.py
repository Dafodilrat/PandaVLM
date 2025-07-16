#!/usr/bin/env -S ros2 launch

from os import path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
import xacro

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def launch_setup(context, *args, **kwargs):
    # Get resolved strings from LaunchConfigurations
    description_package = LaunchConfiguration("description_package").perform(context)
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin").perform(context)
    ros2_control_command_interface = LaunchConfiguration("ros2_control_command_interface").perform(context)
    initial_positions_file_path =  path.join(description_package,"config","initial_joint_positions.yaml")
    enable_ros2_control = ros2_control_plugin == "fake"

    xacro_file = path.join(
        get_package_share_directory(description_package),
        "urdf",
        "panda.urdf.xacro"
    )

    robot_description_config = xacro.process_file(xacro_file, mappings={
        "name": "panda",
        "prefix": "",
        "gripper": "true",
        "collision_arm": "true",
        "collision_gripper": "true",
        "safety_limits": "true",
        "safety_position_margin": "0.15",
        "safety_k_position": "100.0",
        "safety_k_velocity": "40.0",
        "ros2_control": "true",
        "ros2_control_plugin": ros2_control_plugin,
        "ros2_control_command_interface": ros2_control_command_interface,
        "gazebo_preserve_fixed_joint": "false",
    })

    robot_description = robot_description_config.toxml()

    srdf_path = path.join(get_package_share_directory("panda_moveit_config"),"srdf","panda.srdf")

    print(srdf_path,flush=True)

    # LaunchConfigurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_servo = LaunchConfiguration("enable_servo")
    enable_rviz = LaunchConfiguration("enable_rviz")
    log_level = LaunchConfiguration("log_level")

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda",
                             package_name="panda_moveit_config",
                             robot_description=robot_description)
        .robot_description_semantic(file_path = srdf_path,
                                    mappings={"name": "panda","prefix": ""})
        .trajectory_execution()
        .joint_limits()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    controller_filename = PythonExpression([
        "'controllers_' + str(",
        ros2_control_command_interface,
        ") + '.yaml'"
    ])

    # Controller YAML
    controller_parameters = PathJoinSubstitution([
        FindPackageShare("panda_moveit_config"),
        "config",
        controller_filename,
    ])

    # Servo parameters
    servo_params = {
        "moveit_servo": load_yaml("panda_moveit_config", "config/servo.yaml")
    }
    servo_params["moveit_servo"].update({"use_gazebo": use_sim_time})

    # RViz config
    rviz_config_path = path.join(
        get_package_share_directory("panda_moveit_config"), "rviz", "moveit.rviz"
    )

    # Nodes
    nodes = [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
                {'start_state': {'content': initial_positions_file_path}},
            ],
        ),
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            output="log",
            parameters=[
                moveit_config.to_dict(),
                servo_params,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(enable_servo),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", rviz_config_path],
            parameters=[
                {"robot_description": robot_description},
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(enable_rviz),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

    ]

    if enable_ros2_control:
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="log",
                arguments=["--ros-args", "--log-level", log_level.perform(context)],
                parameters=[
                    {"robot_description": robot_description},
                    controller_parameters,
                    {"use_sim_time": use_sim_time},
                ],
            )
        )

    return nodes

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_servo", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("ros2_control_plugin", default_value="gz"),
        DeclareLaunchArgument("ros2_control_command_interface", default_value="effort"),
        DeclareLaunchArgument("description_package", default_value="panda_description"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
