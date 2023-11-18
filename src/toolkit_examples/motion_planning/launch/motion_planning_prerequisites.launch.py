import os
from launch import LaunchDescription
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="lite6", package_name="moveit_resources_lite6_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic("config/lite6.srdf")
        .robot_description(file_path=get_package_share_directory("moveit_resources_lite6_description") 
            + "/urdf/lite6.urdf")
        .moveit_cpp(
            file_path=get_package_share_directory("lite6_motion_planning_demos")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    rviz_config_file = (
        get_package_share_directory("lite6_motion_planning_demos") + "/config/planning_scene.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link_base"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )


    # We can start a notebook from a launch file
    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            rviz_node,
        ]
        )

