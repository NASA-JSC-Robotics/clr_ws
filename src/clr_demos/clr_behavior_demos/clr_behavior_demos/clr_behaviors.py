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

import numpy as np

from rclpy.node import Node
from rclpy.time import Time

from py_trees.common import Access, Status
from py_trees.ports import BehaviourWithPorts, PortInformation

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation


def _transform_stamped_to_matrix(t) -> np.ndarray:
    """Convert a geometry_msgs/TransformStamped to a 4x4 homogeneous matrix."""
    trans = t.transform.translation
    rot = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
    T[:3, 3] = [trans.x, trans.y, trans.z]
    return T


def _matrix_to_pose_stamped(T: np.ndarray, frame_id: str) -> PoseStamped:
    """Convert a 4x4 homogeneous matrix to a PoseStamped."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(T[0, 3])
    pose.pose.position.y = float(T[1, 3])
    pose.pose.position.z = float(T[2, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
    pose.pose.orientation.x = float(q[0])
    pose.pose.orientation.y = float(q[1])
    pose.pose.orientation.z = float(q[2])
    pose.pose.orientation.w = float(q[3])
    return pose


def _rotation_matrix_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Build a 4x4 homogeneous transform for a pure rotation about an axis."""
    R = np.eye(4)
    R[:3, :3] = Rotation.from_rotvec(angle * axis).as_matrix()
    return R


# NOTE: Once connors stuff is in we won't need these
def rotate_about_frame(
    T_world_hinge: np.ndarray,
    T_world_grasp: np.ndarray,
    angle: float,
    axis: np.ndarray,
    keep_start_orientation: bool,
) -> np.ndarray:
    """
    Rotate the grasp frame about the hinge frame's origin and chunk it up.

    Algorithm:
      1. Express grasp in hinge-local coordinates:
           T_hinge_grasp = inv(T_world_hinge) @ T_world_grasp
      2. Apply the rotation in the hinge frame:
           T_rotated_local = R(axis, angle) @ T_hinge_grasp
      3. Optionally preserve the original local orientation (only the
         position orbits; the end-effector keeps pointing the same way
         relative to the hinge).
      4. Transform back to world:
           T_world_rotated = T_world_hinge @ T_rotated_local

    Returns:
        4x4 homogeneous matrix of the rotated pose in world frame.
    """
    # Step 1: grasp pose in hinge-local coordinates
    T_hinge_grasp = np.linalg.inv(T_world_hinge) @ T_world_grasp

    # Step 2: rotate about the axis in the hinge frame
    R = _rotation_matrix_about_axis(axis, angle)
    T_rotated_local = R @ T_hinge_grasp

    # Step 3: preserve the original orientation if requested
    if keep_start_orientation:
        T_rotated_local[:3, :3] = T_hinge_grasp[:3, :3]

    # Step 4: back to world
    return T_world_hinge @ T_rotated_local


class ComputeCartesianArcWaypoints(BehaviourWithPorts):
    """ComputeCartesianArcWaypoints behavior.

    Computes end-effector poses tracing an arc around a hinge frame.

    Ports
    -----
    Inputs:
        hinge_frame            (str)   – TF frame at the hinge (e.g. "bench_lid")
        grasp_frame            (str)   – TF frame of the end-effector grasp point
        num_steps              (int)   – number of arc waypoints (default 8)
        rotation_per_step      (float) – radians per step (negative = opening)
        rotation_axis_xyz      (str)   – comma-separated axis in hinge frame,
                                         e.g. "0.0, 1.0, 0.0"
        keep_start_orientation (bool)  – if true, hold the EE orientation fixed
                                         relative to the hinge (default true)
    Outputs:
        waypoints (list[PoseStamped]) – arc poses in the world frame
    """

    INPUT_PORTS = {
        "hinge_frame": PortInformation(data_type=str, required=True),
        "grasp_frame": PortInformation(data_type=str, required=True),
        "num_steps": PortInformation(data_type=int, required=False),
        "rotation_per_step": PortInformation(data_type=float, required=True),
        "rotation_axis_xyz": PortInformation(data_type=str, required=False),
        "keep_start_orientation": PortInformation(data_type=bool, required=False),
    }

    OUTPUT_PORTS = {"waypoints": PortInformation(data_type=list)}

    def setup(self, **kwargs) -> None:
        """Get access to the ROS node and the shared TF buffer."""
        self.node = kwargs.get("node")
        if not isinstance(self.node, Node):
            raise KeyError(f"A valid ROS node is required to setup the '{self.qualified_name}' node.")

        self.blackboard_client.register_key(key="/ros/tf_buffer", access=Access.READ)
        self.tf_buffer = self.blackboard_client.get("/ros/tf_buffer")

    def update(self) -> Status:
        hinge_frame: str = self.get_input("hinge_frame")
        grasp_frame: str = self.get_input("grasp_frame")
        num_steps: int = self.get_input("num_steps", 8)
        rotation_per_step: float = self.get_input("rotation_per_step")
        axis_str: str = self.get_input("rotation_axis_xyz", "0.0, 1.0, 0.0")
        keep_orientation: bool = self.get_input("keep_start_orientation", True)

        axis = np.array([float(v) for v in axis_str.split(",")])
        axis = axis / np.linalg.norm(axis)  # ensure unit vector

        # Look up both frames in world at the latest available time.
        try:
            hinge_tf = self.tf_buffer.lookup_transform("world", hinge_frame, Time())
            grasp_tf = self.tf_buffer.lookup_transform("world", grasp_frame, Time())
        except Exception as ex:
            self.node.get_logger().error(f"TF lookup failed: {ex}")
            return Status.FAILURE

        T_world_hinge = _transform_stamped_to_matrix(hinge_tf)
        T_world_grasp = _transform_stamped_to_matrix(grasp_tf)

        # Compute each arc waypoint. Step indices start at 1 so the first
        # waypoint is already rotated away from the starting contact pose.
        waypoints: list[PoseStamped] = []
        for i in range(1, num_steps + 1):
            angle = i * rotation_per_step

            T_world_rotated = rotate_about_frame(T_world_hinge, T_world_grasp, angle, axis, keep_orientation)

            waypoints.append(_matrix_to_pose_stamped(T_world_rotated, "world"))

        self._set_output("waypoints", waypoints)
        self.node.get_logger().info(
            f"Computed {len(waypoints)} arc waypoints about '{hinge_frame}' "
            f"(step={rotation_per_step:.4f} rad, axis=[{axis_str}], "
            f"keep_orientation={keep_orientation})."
        )
        return Status.SUCCESS
