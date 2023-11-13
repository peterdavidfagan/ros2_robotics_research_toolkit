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
            MoveItConfigsBuilder(robot_name="panda", package_name="franka_robotiq_moveit_config")
            .robot_description(file_path=get_package_share_directory("franka_robotiq_description") + "/urdf/robot.urdf.xacro", 
                mappings={"robot_ip": "192.168.106.99", "robotiq_gripper": "false"})
            .robot_description_semantic("config/panda.srdf.xacro")
            .trajectory_execution("config/moveit_controllers.yaml")
            .to_moveit_configs()
            )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['franka/joint_states']}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path
            ],
        remappings=[('joint_states', 'franka/joint_states')],
        output="both",
    )

    load_controllers = []
    for controller in [
        'panda_jtc_controller',
        'panda_position_controller',
        'joint_state_broadcaster', 
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
            joint_state_publisher,
            ros2_control_node,
        ]
        + load_controllers
        )

