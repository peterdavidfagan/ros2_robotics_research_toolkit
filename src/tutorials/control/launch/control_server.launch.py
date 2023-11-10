import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['xarm/joint_states']}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_lite6_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    # robot driver launch
    # xarm_api/launch/_robot_driver.launch.py
    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
        launch_arguments={
            'robot_ip': '192.168.1.156',
            'report_type': 'dev',
            'dof': '6',
            'hw_ns': 'xarm',
            'add_gripper': 'false',
            'prefix': '',
            'robot_type': 'lite',
        }.items(),
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path],
        output="both",
    )

    load_controllers = []
    for controller in [
        "lite6_traj_controller",
        "lite6_position_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # We can start a notebook from a launch file
    return LaunchDescription(
        [
            robot_driver,
            ros2_control_node,
            joint_state_publisher,
        ]
        + load_controllers
        )

