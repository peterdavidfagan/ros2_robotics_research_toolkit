#!/usr/bin/env python3
"""
A script that demonstrated blending a large number of plans.

Note: moveit_servo can also be used to achieve similar motions in realtime in free space (but servo doesn't explicitly plan taking collision geometry into account).
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
import sys
import threading
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, RobotTrajectory as RobotTrajectoryMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration


from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def quaternion_from_vector(v):
    """Calculate the orientation quaternion to point in the given direction."""
    x, y, z = v
    norm = math.sqrt(x**2 + y**2 + z**2)
    if norm == 0:
        return [0, 0, 0, 1]  # No rotation (identity quaternion)
    else:
        angle = math.acos(max(-1, min(1, z / norm)))
        axis = [y, -x, 0]
        axis_norm = math.sqrt(axis[0]**2 + axis[1]**2)
        if axis_norm != 0:
            axis = [axis[0] / axis_norm, axis[1] / axis_norm, 0]
        sin_half_angle = math.sin(angle / 2)
        cos_half_angle = math.cos(angle / 2)
        return [axis[0] * sin_half_angle, axis[1] * sin_half_angle, axis[2] * sin_half_angle, cos_half_angle]

def spiral_trajectory_with_camera(height, radius, n_points, object_position):
    """Generate waypoints along a spiral trajectory with the camera pointing towards the object."""
    waypoints = []
    
    for i in range(n_points):
        angle = i * 2 * math.pi / n_points
        x = radius * ((math.cos(angle) + 1.0) / 2.0) + 0.25
        y = radius * ((math.sin(angle) + 1.0) / 2.0)
        z = (height * (i / n_points)) + 0.15
        
        # Calculate the orientation to point the camera towards the object
        camera_direction = [object_position[0] - x, object_position[1] - y, object_position[2] - z]
        camera_direction = np.array(camera_direction) / np.linalg.norm(camera_direction)
        orientation = [-0.707, 0.0, 0.0, 0.707]
        #orientation = quaternion_from_vector(camera_direction)
        
        waypoints.append(Pose(position=[x, y, z], orientation=orientation))
        
    return waypoints

        

def main():

    # moveit setup
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    lite6 = MoveItPy(node_name="moveit_py")
    arm = lite6.get_planning_component("lite6")
    logger.info("MoveItPy instance created")


    # move to home position
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    plan_result = arm.plan()
    if plan_result:
        robot_trajectory = plan_result.trajectory
        lite6.execute(robot_trajectory, controllers=[])

    # generate waypoints
    waypoints = spiral_trajectory_with_camera(height=0.3, radius=0.15, n_points=10, object_position=[0.25, 0.25, 0.0])
    logger.info("Generated {} waypoints".format(len(waypoints)))
    logger.info("Waypoints: {}".format(waypoints))

    plan_failed = False
    points = []
    for idx, waypoint in enumerate(waypoints):
        if idx == 0:
            arm.set_start_state_to_current_state()
        else:
            robot_model = lite6.get_robot_model()
            robot_state = RobotState(robot_model)
            logger.info("Setting robot state {}".format(idx))
            joint_position_map = dict(zip(robot_trajectory_msg.joint_trajectory.joint_names, robot_trajectory_msg.joint_trajectory.points[-1].positions))
            robot_state.joint_positions = joint_position_map 
            arm.set_start_state(robot_state=robot_state)
        
        logger.info("Setting goal state {}".format(idx))
        pose_goal = PoseStamped()
        pose_goal.pose.position.x = waypoint.position[0]
        pose_goal.pose.position.y = waypoint.position[1]
        pose_goal.pose.position.z = waypoint.position[2]
        pose_goal.pose.orientation.x = waypoint.orientation[0]
        pose_goal.pose.orientation.y = waypoint.orientation[1]
        pose_goal.pose.orientation.z = waypoint.orientation[2]
        pose_goal.pose.orientation.w = waypoint.orientation[3]
        pose_goal.header.frame_id = "link_base"
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_eef")
        
        logger.info("Planning to waypoint {}".format(idx))

        plan_result = arm.plan()
        if plan_result:
            logger.info("Planned to waypoint {}".format(idx))
            robot_trajectory_msg = plan_result.trajectory.get_robot_trajectory_msg()
            logger.info("Got trajectory message")
            joint_names = robot_trajectory_msg.joint_trajectory.joint_names # TODO: get from arm directly outside loop
            points.extend(robot_trajectory_msg.joint_trajectory.points)
        else:
            logger.error("Failed to plan to waypoint {}".format(idx))
            logger.error("Failed to plan to waypoint {}".format(waypoint))
            plan_failed = True
            break
    
    if not plan_failed:
        final_trajectory_msg = RobotTrajectoryMsg()
        final_trajectory_msg.joint_trajectory.joint_names = joint_names
        final_trajectory_msg.joint_trajectory.points = points
        # ensure duration from start is monotonically increasing
        for idx, point in enumerate(final_trajectory_msg.joint_trajectory.points):
            point.time_from_start = Duration(seconds=idx*0.1)
        
        robot_model = lite6.get_robot_model()
        robot_state = RobotState(robot_model)
        final_trajectory = RobotTrajectory(robot_model)
        final_trajectory.set_robot_trajectory_msg(robot_state, final_trajectory_msg)

        # perform time optimal trajectory generation
        totg_success = final_trajectory.apply_totg_time_parameterization(
                velocity_scaling_factor=0.5,
                acceleration_scaling_factor=0.5,
                )

        # perform ruckig smoothing
        ruckig_success = final_trajectory.apply_ruckig_smoothing(
                velocity_scaling_factor=0.5,
                acceleration_scaling_factor=0.5,
                )
        
        if totg_success and ruckig_success:
            # execute trajectory
            lite6.execute(final_trajectory, controllers=[])

    # move to home position
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    plan_result = arm.plan()
    if plan_result:
        robot_trajectory = plan_result.trajectory
        lite6.execute(robot_trajectory, controllers=[])


if __name__ == "__main__":
    main()
