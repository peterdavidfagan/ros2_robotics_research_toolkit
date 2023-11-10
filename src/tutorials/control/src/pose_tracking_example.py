#!/usr/bin/env python3
"""A basic data collection script."""
import os
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

from moveit_msgs.srv import ServoCommandType
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import numpy as np

from trajectory import Spiral

class PoseTracker(Node):
    """A node for tracking multiple pose trajectories."""

    def __init__(self):
        super().__init__("pose_tracker")
        self.logger = self.get_logger()
        
        # initialize target poses list
        self.poses = []

        # initialize motion planning client
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

        self.robot = MoveItPy(config_dict=moveit_config)
        self.arm = self.robot.get_planning_component("lite6")

        # initialize servo service client
        self.servo_command_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        while not self.servo_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.servo_pause_client = self.create_client(SetBool, "/servo_node/pause_servo")
        while not self.servo_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        #initialize servo target pose publisher
        self.publisher = self.create_publisher(PoseStamped, "/servo_node/pose_target_cmds", 10)

    def _set_target_pose(self, pose):
        """Sets the target pose."""
        self.target_pose = pose

    def _get_current_pose(self):
        """Gets the current pose of the robot."""
        self.arm.set_start_state_to_current_state()
        robot_state = self.arm.get_start_state()
        pose = robot_state.get_pose("link_eef")
        return pose

    # TODO: properly implement this
    def _check_pose_threshold(self, pose):
        """Checks if the robot is within the threshold of the target pose."""
        # TODO: check orientation + read thresholds from config
        current_pose = self._get_current_pose()
        del_x = current_pose.position.x - pose.pose.position.x
        del_y = current_pose.position.y - pose.pose.position.y
        del_z = current_pose.position.z - pose.pose.position.z
        if np.linalg.norm([del_x, del_y, del_z]) < 0.1:
            return True

    def set_servo_command_type(self, command_type):
        """Sets the servo command type."""
        request = ServoCommandType.Request()
        request.command_type = command_type
        self.future = self.servo_command_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result():
            self.get_logger().info("Servo command type set to: %s" % command_type)
        else:
            self.get_logger().error("Failed to set servo command type: %s" % command_type)

    def return_to_start(self):
        """Returns the robot to the start position."""
        # pause servo    
        request = SetBool.Request()
        request.data = True
        future = self.servo_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.logger.info("servo paused")

        # plan to ready config
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")
        plan = self.arm.plan()
        if plan:
            robot_trajectory = plan.trajectory
            self.robot.execute(robot_trajectory, controllers=[])

        # restart servo
        request = SetBool.Request()
        request.data = False
        future = self.servo_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.logger.info("servo started")

    def generate_waypoints(self, trajectory, num_points = 100):
        """Generates waypoints from a trajectory."""

        # trajectory generator
        positions, orientations = trajectory.sample_points(num_points=num_points)

        self.poses = []
        for (position, orientation) in zip(positions, orientations):
            pose = PoseStamped()
            pose.header.frame_id = "link_base"
            pose.pose.position = position
            pose.pose.orientation = orientation
            self.poses.append(pose)


    def track_pose_targets(self):
        """Sets the target pose."""

        # check if poses have been set
        if len(self.poses) == 0:
            self.get_logger().error("No poses set.")
            return

        while True:
            # set target pose
            self._set_target_pose(self.poses[0])

            if self._check_pose_threshold(self.target_pose):
                self.get_logger().info("Reached target pose: %s" % self.target_pose)

                # pop last pose from list
                self.poses.pop(0)

                # check if last pose
                if len(self.poses) == 0:
                    self.get_logger().info("Finished tracking poses.")
                    return

                # set new target pose
                self._set_target_pose(self.poses[0])

            # publish target pose
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.target_pose)


if __name__=="__main__":
    rclpy.init()
    node = PoseTracker()
    
    # return to start position
    node.return_to_start()

    # set servo command type to pose                                                                                     
    node.set_servo_command_type(2)
    spiral = Spiral(radius=0.1, height=0.1)
    
    # track pose targets
    node.generate_waypoints(spiral)
    node.track_pose_targets()

    # shutdown
    node.destroy_node()
    rclpy.shutdown()                                                                                                    

