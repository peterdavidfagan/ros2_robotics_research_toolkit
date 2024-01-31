"""
A launch file for running the motion planning python api tutorial
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    moveit_config = (
            MoveItConfigsBuilder(robot_name="panda", package_name="franka_robotiq_moveit_config")
            .robot_description(file_path=get_package_share_directory("franka_robotiq_description") + "/urdf/robot.urdf.xacro",
                mappings={"robot_ip": "192.168.106.99", "robotiq_gripper": "false", "use_fake_hardware": "true"})
            .robot_description_semantic("config/panda.srdf.xacro", mappings={"robotiq_gripper": "false"})
            .trajectory_execution("config/moveit_controllers.yaml")
            .moveit_cpp(
                file_path=get_package_share_directory("panda_motion_planning_demos")
                + "/config/moveit_cpp.yaml"
            )
            .to_moveit_configs()
            )
    
    moveit_config_dict = moveit_config.to_dict()
    moveit_py_node = Node(
        name="moveit_py",
        package="panda_motion_planning_demos",
        executable="motion_planning.py",
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "info"],
        parameters=[
            moveit_config_dict, 
            {"use_sim_time": use_sim_time}
            ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": use_sim_time},
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            ],
    )


    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            rviz_node,
            moveit_py_node,
        ]
        )


