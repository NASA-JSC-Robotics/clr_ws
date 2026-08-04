#!/usr/bin/env python3
#
# Copyright (c) 2026, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
Behaviors for motion planning with a RoboPlan planning server and execution
via ROS 2 controllers.

Planning behaviors call the services offered by a RoboPlan planning server
node (see clr_behavior_demos/scripts/roboplan_planning_server.py) and write
the resulting trajectory_msgs/JointTrajectory to the blackboard.

Execution sends that trajectory directly to a joint trajectory controller's
FollowJointTrajectory action.
"""

from typing import Any

from py_trees.common import Status
from py_trees.ports import PortInformation

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from clr_behavior_demos_msgs.srv import PlanCartesianPath, PlanToJointState, PlanToPose
from imetro_behavior.ros_behaviors.action_client import RosActionClientBase
from imetro_behavior.ros_behaviors.service_client import RosServiceClientBase


# Error code names for control_msgs/FollowJointTrajectory results.
FOLLOW_JOINT_TRAJECTORY_ERROR_DICT = {
    FollowJointTrajectory.Result.SUCCESSFUL: "SUCCESSFUL",
    FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
    FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
}


class RoboplanPlanToJointState(RosServiceClientBase):
    """
    Uses a RoboPlan planning server to plan a free-space motion to a target
    joint configuration.
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, service_type=PlanToJointState, **kwargs)

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {
            "group_name": PortInformation(data_type=str, required=False),
            "joint_names": PortInformation(data_type=list[str], required=True),
            "joint_positions": PortInformation(data_type=list[float], required=True),
            "velocity_scaling": PortInformation(data_type=float, required=False),
            "acceleration_scaling": PortInformation(data_type=float, required=False),
        }

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"trajectory": PortInformation(data_type=JointTrajectory)}

    def create_request(self) -> PlanToJointState.Request:
        """Create the planning service request."""
        joint_names = self.get_input("joint_names")
        joint_positions = self.get_input("joint_positions")
        if len(joint_positions) != len(joint_names):
            raise RuntimeError("Joint names and joint positions must have the same length.")

        return PlanToJointState.Request(
            group_name=self.get_input("group_name", ""),
            joint_names=joint_names,
            joint_positions=joint_positions,
            velocity_scaling=self.get_input("velocity_scaling", 0.0),
            acceleration_scaling=self.get_input("acceleration_scaling", 0.0),
        )

    def process_response(self, response: PlanToJointState.Response) -> Status:
        """Process the planning service response."""
        if response.success:
            self.node.get_logger().info(f"Motion plan succeeded: {response.message}")
            self._set_output("trajectory", response.trajectory)
            return Status.SUCCESS
        else:
            self.node.get_logger().error(f"Motion plan failed: {response.message}")
            return Status.FAILURE


class RoboplanPlanToPose(RosServiceClientBase):
    """
    Uses a RoboPlan planning server to plan a free-space motion to a target
    end effector pose (IK to find a goal configuration, then planning).
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, service_type=PlanToPose, **kwargs)

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {
            "group_name": PortInformation(data_type=str, required=False),
            "target_pose": PortInformation(data_type=PoseStamped, required=True),
            "velocity_scaling": PortInformation(data_type=float, required=False),
            "acceleration_scaling": PortInformation(data_type=float, required=False),
            "constrain_gripper_top_down": PortInformation(data_type=bool, required=False),
        }

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"trajectory": PortInformation(data_type=JointTrajectory)}

    def create_request(self) -> PlanToPose.Request:
        """Create the planning service request."""
        return PlanToPose.Request(
            group_name=self.get_input("group_name", ""),
            target_pose=self.get_input("target_pose"),
            velocity_scaling=self.get_input("velocity_scaling", 0.0),
            acceleration_scaling=self.get_input("acceleration_scaling", 0.0),
            constrain_gripper_top_down=self.get_input("constrain_gripper_top_down", False),
        )

    def process_response(self, response: PlanToPose.Response) -> Status:
        """Process the planning service response."""
        if response.success:
            self.node.get_logger().info(f"Motion plan succeeded: {response.message}")
            self._set_output("trajectory", response.trajectory)
            return Status.SUCCESS
        else:
            self.node.get_logger().error(f"Motion plan failed: {response.message}")
            return Status.FAILURE


class RoboplanPlanCartesianPath(RosServiceClientBase):
    """
    Uses a RoboPlan planning server to plan a straight-line Cartesian motion
    from the current end effector pose to a target pose.
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, service_type=PlanCartesianPath, **kwargs)

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {
            "group_name": PortInformation(data_type=str, required=False),
            "target_pose": PortInformation(data_type=PoseStamped, required=True),
            "max_linear_speed": PortInformation(data_type=float, required=False),
            "max_angular_speed": PortInformation(data_type=float, required=False),
        }

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"trajectory": PortInformation(data_type=JointTrajectory)}

    def create_request(self) -> PlanCartesianPath.Request:
        """Create the planning service request."""
        return PlanCartesianPath.Request(
            group_name=self.get_input("group_name", ""),
            target_pose=self.get_input("target_pose"),
            max_linear_speed=self.get_input("max_linear_speed", 0.0),
            max_angular_speed=self.get_input("max_angular_speed", 0.0),
        )

    def process_response(self, response: PlanCartesianPath.Response) -> Status:
        """Process the planning service response."""
        if response.success:
            self.node.get_logger().info(f"Motion plan succeeded: {response.message}")
            self._set_output("trajectory", response.trajectory)
            return Status.SUCCESS
        else:
            self.node.get_logger().error(f"Motion plan failed: {response.message}")
            return Status.FAILURE


class ExecuteJointTrajectory(RosActionClientBase):
    """
    Executes a joint trajectory by sending it directly to a ROS 2 joint
    trajectory controller's FollowJointTrajectory action.
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, action_type=FollowJointTrajectory, **kwargs)

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {"trajectory": PortInformation(data_type=JointTrajectory, required=True)}

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {}

    def create_goal(self) -> FollowJointTrajectory.Goal:
        """Create a trajectory execution goal."""
        return FollowJointTrajectory.Goal(trajectory=self.get_input("trajectory"))

    def process_result(self, result: Any) -> Status:
        """Process the trajectory execution action result."""
        error_code = result.result.error_code
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.node.get_logger().info("Trajectory execution succeeded!")
            return Status.SUCCESS
        else:
            error_code_str = FOLLOW_JOINT_TRAJECTORY_ERROR_DICT.get(error_code, "UNKNOWN")
            self.node.get_logger().error(f"Trajectory execution failed with error code: {error_code_str}")
            self.node.get_logger().error(f"Message: {result.result.error_string}")
            return Status.FAILURE
