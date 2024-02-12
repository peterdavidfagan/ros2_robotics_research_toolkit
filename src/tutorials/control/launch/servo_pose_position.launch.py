import os
import launch
import launch_ros
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import load_python_launch_file_as_module
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # declare parameter for using robot ip
    robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.106.99",
        description="Robot IP",
    )

    # declare parameter for using gripper
    use_gripper = DeclareLaunchArgument(
        "use_gripper",
        default_value="true",
        description="Use gripper",
    )
    
    # declare parameter for using fake controller
    use_fake_hardware = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use fake hardware",
    )

    moveit_config = (
            MoveItConfigsBuilder(robot_name="panda", package_name="franka_robotiq_moveit_config")
            .robot_description(file_path=get_package_share_directory("franka_robotiq_description") + "/urdf/robot.urdf.xacro", 
                mappings={
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robotiq_gripper": LaunchConfiguration("use_gripper"),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                    })
            .robot_description_semantic("config/panda.srdf.xacro")
            .trajectory_execution("config/moveit_controllers.yaml")
            .to_moveit_configs()
            )

    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("panda_control_demos")
        .yaml("config/servo_pose_position.yaml")
        .to_dict()
    }

    # This filter parameter should be >1. Increase it for greater smoothing but slower motion.
    low_pass_filter_coeff = {"butterworth_filter_coeff": 1.5}

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    low_pass_filter_coeff,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
              parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            low_pass_filter_coeff,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    return launch.LaunchDescription(
        [
            robot_ip,
            use_gripper,
            use_fake_hardware,
            servo_node,
            container,
        ]
    )
