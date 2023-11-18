#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        input("Press Enter to execute trajectory")
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # read config file
     # read moveit config files
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
    ).to_dict()

    # instantiate MoveItPy instance and get planning component
    lite6 = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
    lite6_arm = lite6.get_planning_component("lite6")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state to current state
    lite6_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    lite6_arm.set_goal_state(configuration_name="home")

    # plan to goal
    plan_and_execute(lite6, lite6_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    # instantiate a RobotState instance using the current robot model
    robot_model = lite6.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    lite6_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    lite6_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(lite6, lite6_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    lite6_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    lite6_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")

    # plan to goal
    plan_and_execute(lite6, lite6_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    # set plan start state to current state
    lite6_arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "joint1": 1.0,
        "joint2": 0.0,
        "joint3": 0.0,
        "joint4": 1.0,
        "joint5": 0.0,
        "joint6": 0.0,
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=lite6.get_robot_model().get_joint_model_group("lite6"),
    )
    lite6_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(lite6, lite6_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 5 - Planning with Multiple Pipelines simultaneously
    ###########################################################################

    # set plan start state to current state
    lite6_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    lite6_arm.set_goal_state(configuration_name="home")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        lite6, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # plan to goal
    plan_and_execute(
        lite6,
        lite6_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )


if __name__ == "__main__":
    main()
