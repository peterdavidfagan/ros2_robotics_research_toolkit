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
    

    # if we are using fake hardware adjust joint state topic
    if not LaunchConfiguration("use_fake_hardware"):
        joint_state_topic = "franka/joint_states"
        ros2_controller_config = "ros2_controllers.yaml"
    else:
        joint_state_topic = "/mujoco_joint_states"
        ros2_controller_config = "ros2_controllers_mujoco.yaml"

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': [joint_state_topic]}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        ros2_controller_config,
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path
            ],
        remappings=[('joint_states', joint_state_topic)],
        output="both",
    )

    load_controllers = []
    for controller in [
        'panda_jtc_controller',
        'joint_state_broadcaster', 
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("panda_control_demos")
        .yaml("config/servo_pose_jtc.yaml")
        .to_dict()
    }

    # This filter parameter should be >1. Increase it for greater smoothing but slower motion.
    low_pass_filter_coeff = {"butterworth_filter_coeff": 1.5}
    
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
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
            moveit_config.joint_limits,
        ],
        output="screen",
    )


    return launch.LaunchDescription(
        [
            robot_ip,
            use_gripper,
            use_fake_hardware,
            joint_state_publisher,
            ros2_control_node,
            servo_node,
            container,
        ]
    )
