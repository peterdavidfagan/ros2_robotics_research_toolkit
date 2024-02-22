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
        default_value="true", # default to fake hardware (Important: that user is explicit with intention of launching real hardware!)
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

    ros2_controller_path = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        ros2_controller_config,
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controller_path,
            ],
        remappings=[('joint_states', joint_state_topic)],
        output="both",
    )

    load_controllers = []
    for controller in [
        'joint_state_broadcaster',
        'panda_jtc_controller',
        'robotiq_position_controller',
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            robot_ip, 
            use_gripper,
            use_fake_hardware,
            joint_state_publisher,
            ros2_control_node,
        ]
        + load_controllers
        )

