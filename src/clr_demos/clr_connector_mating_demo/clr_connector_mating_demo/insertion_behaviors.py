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

"""Behaviors for the compliant peg insertion."""

from typing import Any

import numpy as np
from geometry_msgs.msg import PoseStamped, WrenchStamped
from py_trees.common import Access, Status
from py_trees.ports import BehaviourWithPorts, PortInformation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

from imetro_behavior.helpers import set_ros_node


class WaitForFrameToReachPose(BehaviourWithPorts):
    """
    Waits (RUNNING) until a TF frame is within tolerance of a target pose, or
    until the timeout elapses, then reports how close the frame got. Useful for
    compliant motions, where the commanded pose is not necessarily reached.

    By default the behavior succeeds either way and simply outputs the residual
    error; set `fail_on_timeout` to make a missed target return FAILURE instead.

    Optionally it also watches a force/torque wrench topic and finishes when
    the measured force exceeds `contact_force_threshold`.
    The `contact` output port reports whether that happened.

    Constructor arguments:
        frame: The TF frame to track, e.g., the tool tip.
        position_tolerance: Allowed position error, in meters.
        orientation_tolerance: Allowed orientation error, in radians.
        timeout: Seconds to wait for the frame to arrive.
        settle_time: Seconds the frame must continuously stay within tolerance.
        fail_on_timeout: Whether missing the target within the timeout is a failure.
        wrench_topic: WrenchStamped topic to watch for contact (empty disables it).
        contact_force_threshold: Force magnitude, in newtons, that counts as contact.
        contact_force_axis: Sensor-frame direction whose force component is compared against
            the threshold, e.g., [0.0, 0.0, 1.0]; if not specified, uses the force norm.
    """

    def __init__(
        self,
        name: str,
        *,
        frame: str,
        position_tolerance: float = 0.005,
        orientation_tolerance: float = 0.05,
        timeout: float = 30.0,
        settle_time: float = 0.0,
        fail_on_timeout: bool = False,
        wrench_topic: str = "",
        contact_force_threshold: float = 0.0,
        contact_force_axis: list[float] | None = None,
        **kwargs: Any,
    ):
        super().__init__(name, **kwargs)
        self.frame = frame
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.timeout = timeout
        self.settle_time = settle_time
        self.fail_on_timeout = fail_on_timeout
        self.wrench_topic = wrench_topic
        self.contact_force_threshold = contact_force_threshold
        self.contact_force_axis = None
        if contact_force_axis is not None:
            axis = np.array(contact_force_axis, dtype=float)
            if axis.shape != (3,) or not np.any(axis):
                raise ValueError(f"'{name}': contact_force_axis must be a nonzero 3-vector.")
            self.contact_force_axis = axis / np.linalg.norm(axis)
        if self.contact_force_threshold > 0.0 and not self.wrench_topic:
            raise ValueError(f"'{name}': contact detection requires the 'wrench_topic' argument.")
        self.node: Node

    INPUT_PORTS = {"target_pose": PortInformation(data_type=PoseStamped, required=True)}

    OUTPUT_PORTS = {
        "position_error": PortInformation(data_type=float, required=False),
        "orientation_error": PortInformation(data_type=float, required=False),
        "contact": PortInformation(
            data_type=bool, required=False, description="Whether the contact force threshold was reached."
        ),
    }

    def setup(self, **kwargs):
        """Gets the ROS node and TF buffer, and subscribes to the wrench topic if contact detection is on."""
        set_ros_node(self, **kwargs)
        self.blackboard_client.register_key(key="/ros/tf_buffer", access=Access.READ)
        self.tf_buffer = self.blackboard_client.get("/ros/tf_buffer")
        self._latest_force = None
        if self.contact_force_threshold > 0.0:
            self.node.create_subscription(WrenchStamped, self.wrench_topic, self._on_wrench, qos_profile_sensor_data)

    def _on_wrench(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        force = np.array([f.x, f.y, f.z])
        if self.contact_force_axis is None:
            self._latest_force = float(np.linalg.norm(force))
        else:
            self._latest_force = abs(float(force @ self.contact_force_axis))

    def initialise(self) -> None:
        self._start_time = self.node.get_clock().now()
        self._within_since = None
        self._last_log_time = None
        self._set_output("contact", False)

    def _seconds_since(self, start) -> float:
        return (self.node.get_clock().now() - start).nanoseconds * 1.0e-9

    def update(self) -> Status:
        """Compares the tracked frame against the target pose."""
        target_pose = self.get_input("target_pose")
        try:
            tform = self.tf_buffer.lookup_transform(target_pose.header.frame_id, self.frame, Time())
        except Exception as e:  # noqa: BLE001 -- TF raises a variety of exceptions
            self.logger.warning(f"TF lookup failed: {e}")
            if self._seconds_since(self._start_time) > self.timeout:
                return Status.FAILURE
            return Status.RUNNING

        t = tform.transform.translation
        q = tform.transform.rotation
        target_position = np.array(
            [target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z]
        )
        position_error = float(np.linalg.norm(np.array([t.x, t.y, t.z]) - target_position))
        qt = target_pose.pose.orientation
        orientation_error = float(
            (R.from_quat([q.x, q.y, q.z, q.w]).inv() * R.from_quat([qt.x, qt.y, qt.z, qt.w])).magnitude()
        )
        self._set_output("position_error", position_error)
        self._set_output("orientation_error", orientation_error)

        if self.contact_force_threshold > 0.0 and self._latest_force is not None:
            if self._latest_force >= self.contact_force_threshold:
                self.logger.info(
                    f"'{self.frame}' made contact ({self._latest_force:.1f} N) "
                    f"{position_error * 1000.0:.1f} mm short of the target pose."
                )
                self._set_output("contact", True)
                return Status.SUCCESS

        within = position_error <= self.position_tolerance and orientation_error <= self.orientation_tolerance
        now = self.node.get_clock().now()
        if within:
            if self._within_since is None:
                self._within_since = now
            if self._seconds_since(self._within_since) >= self.settle_time:
                self.logger.info(
                    f"'{self.frame}' reached the target pose "
                    f"(position error {position_error * 1000.0:.1f} mm, "
                    f"orientation error {np.degrees(orientation_error):.1f} deg)."
                )
                return Status.SUCCESS
        else:
            self._within_since = None

        if self._last_log_time is None or self._seconds_since(self._last_log_time) >= 1.0:
            self._last_log_time = now
            self.logger.info(
                f"Waiting for '{self.frame}': position error {position_error * 1000.0:.1f} mm, "
                f"orientation error {np.degrees(orientation_error):.1f} deg."
            )

        if self._seconds_since(self._start_time) > self.timeout:
            message = (
                f"'{self.frame}' did not reach the target pose within {self.timeout} seconds "
                f"(position error {position_error * 1000.0:.1f} mm, "
                f"orientation error {np.degrees(orientation_error):.1f} deg)."
            )
            if self.fail_on_timeout:
                self.logger.error(message)
                return Status.FAILURE
            self.logger.warning(message)
            return Status.SUCCESS
        return Status.RUNNING
