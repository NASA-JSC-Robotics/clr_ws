#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
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
Cartesian servoing node for the ChonkUR using OInK.

  1. Drag the interactive marker to set a target pose
  2. A background control loop continuously runs OInK one step per tick
  3. The result is published directly as a joint command

Use the iMarker dropdown menu to start / stop / reset tracking.

Intended as an example _only_.
"""

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from interactive_markers import InteractiveMarkerServer, MenuHandler

from roboplan.core import CartesianConfiguration
from roboplan.filters import SE3LowPassFilter
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    Oink,
    PositionLimit,
    VelocityLimit,
)
from roboplan_ros.visualization import RoboplanIKMarker
from roboplan_ros.cpp import se3ToPose

from clr_roboplan_demos import (
    create_scene,
    get_robot_config,
    spin_executor,
    JointStateTracker,
)


class CartesianServoNode(Node):

    def __init__(self):
        super().__init__("cartesian_servo_node")

        # Setup a basic scene with just ChonkUR
        self.declare_parameter("robot", "chonkur")
        self._config = get_robot_config(self.get_parameter("robot").value)

        if not self._config.supports_streaming:
            raise RuntimeError(
                f"Robot config '{self._config.name}' does not support streaming. "
                f"Use a config with supports_streaming=True."
            )

        self.get_logger().info(f"Using robot config '{self._config.name}' " f"(group={self._config.joint_group})")

        self._scene, _, _ = create_scene()
        group_info = self._scene.getJointGroupInfo(self._config.joint_group)
        self._q_indices = group_info.q_indices
        self._joint_names = group_info.joint_names

        self._js = JointStateTracker(self._scene, "/joint_states", self.get_logger())
        self._js.wait_for_joint_state(self.get_logger())

        # Setup OinK
        self._dt = 0.05  # 20 Hz
        self._regularization = 1e-5
        self._streaming = False
        self._skip_next_command = False
        self._lock = threading.Lock()

        self._oink = Oink(self._scene, self._config.joint_group)
        num_variables = len(self._oink.v_indices)

        goal = CartesianConfiguration()
        goal.base_frame = self._config.base_link
        goal.tip_frame = self._config.tip_link

        self._frame_task = FrameTask(
            self._oink,
            self._scene,
            goal,
            FrameTaskOptions(
                position_cost=1.0,
                orientation_cost=0.1,
                task_gain=1.0,
                lm_damping=0.01,
            ),
        )

        q_home = np.array(self._scene.getCurrentJointPositions())
        config_task = ConfigurationTask(
            self._oink,
            q_home[self._oink.q_indices],
            np.full(num_variables, 0.05),
            ConfigurationTaskOptions(task_gain=1.0, lm_damping=0.0, priority=2),
        )
        self._tasks = [self._frame_task, config_task]

        position_limit = PositionLimit(self._oink, gain=1.0)
        v_max = np.hstack([self._scene.getJointInfo(n).limits.max_velocity for n in self._joint_names])
        velocity_limit = VelocityLimit(self._oink, self._dt, v_max)
        self._constraints = [position_limit, velocity_limit]
        self._barriers = []

        initial_pose = self._scene.forwardKinematics(q_home, self._config.tip_link, self._config.base_link)
        self._raw_target = initial_pose.copy()
        self._reference_filter = SE3LowPassFilter(tau=0.1)
        self._reference_filter.reset(initial_pose)

        self._delta_q = np.zeros(num_variables)
        self._delta_q_full = np.zeros(len(q_home))

        def store_target(target_pose, _):
            self._raw_target = target_pose.copy()
            return None

        self._ik_marker = RoboplanIKMarker(
            scene=self._scene,
            base_link=self._config.base_link,
            tip_link=self._config.tip_link,
            ik_solve_fn=store_target,
        )

        self._marker_node = Node("imarker_oink_node")
        self._ik_server = InteractiveMarkerServer(self._marker_node, "roboplan_oink_ik")
        self._ik_server.insert(
            self._ik_marker.construct_imarker(),
            feedback_callback=self._on_ik_feedback,
        )
        self._ik_server.applyChanges()

        menu = MenuHandler()
        menu.insert("Start", callback=self._on_start_menu)
        menu.insert("Stop", callback=self._on_stop_menu)
        menu.insert("Reset", callback=self._on_reset_menu)
        menu.apply(self._ik_server, "ik_target")
        self._ik_server.applyChanges()

        self._marker_executor = SingleThreadedExecutor()
        self._marker_executor.add_node(self._marker_node)
        self._marker_thread = threading.Thread(
            target=spin_executor,
            daemon=True,
            args=(self._marker_executor, self.get_logger()),
        )
        self._marker_thread.start()

        self._cmd_pub = self.create_publisher(
            JointTrajectory,
            self._config.controller_joint_trajectory_topic,
            10,
        )

        self.create_service(Trigger, "~/reset", self._on_reset_srv)

        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

        self._reset()
        self.get_logger().info("Ready. Drag the marker, then right-click > Start to begin servoing.")

    def _on_ik_feedback(self, feedback):
        self._ik_marker.set_seed_configuration(self._js.latest_positions)
        self._ik_marker.process_feedback(feedback)

    def _control_loop(self):
        while self._running:
            loop_start = time.time()

            if self._streaming:
                with self._lock:
                    q_current = np.array(self._scene.getCurrentJointPositions())
                    self._scene.forwardKinematics(q_current, self._config.tip_link)

                    filtered = self._reference_filter.update(self._raw_target, self._dt)
                    self._frame_task.setTargetFrameTransform(filtered)

                    try:
                        self._oink.solveIk(
                            self._scene,
                            self._tasks,
                            self._constraints,
                            self._barriers,
                            self._delta_q,
                            self._regularization,
                        )
                    except RuntimeError as e:
                        self._delta_q[:] = 0.0
                        self.get_logger().warn(
                            f"IK solver failed: {e}",
                            throttle_duration_sec=1.0,
                        )

                    self._delta_q_full[:] = 0.0
                    self._delta_q_full[self._oink.v_indices] = self._delta_q
                    q_current = self._scene.integrate(q_current, self._delta_q_full)

                    self._scene.setJointPositions(q_current)
                    self._scene.forwardKinematics(q_current, self._config.tip_link)
                    self._js.latest_positions = q_current

                if self._skip_next_command:
                    self._skip_next_command = False
                else:
                    self._publish_joint_command(q_current)

            elapsed = time.time() - loop_start
            time.sleep(max(0, self._dt - elapsed))

    def _publish_joint_command(self, q):
        msg = JointTrajectory()
        msg.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = q[self._q_indices].tolist()
        msg.points = [point]
        self._cmd_pub.publish(msg)

    def _start(self):
        if self._streaming:
            return
        q = self._js.sync_to_hardware()
        with self._lock:
            self._scene.setJointPositions(q)
            self._scene.forwardKinematics(q, self._config.tip_link)
            current_pose = self._scene.forwardKinematics(q, self._config.tip_link, self._config.base_link)
            self._reference_filter.reset(current_pose)
            self._js.latest_positions = q
        self._skip_next_command = True
        self._streaming = True

    def _stop(self):
        self._streaming = False

    def _reset(self):
        self._stop()

        if self._js.last_msg is None:
            raise RuntimeError("No joint states received, cannot reset.")

        q = self._js.sync_to_hardware()
        with self._lock:
            self._scene.setJointPositions(q)
            self._scene.forwardKinematics(q, self._config.tip_link)
            initial_pose = self._scene.forwardKinematics(q, self._config.tip_link, self._config.base_link)
            self._raw_target = initial_pose.copy()
            self._reference_filter.reset(initial_pose)

        self._ik_marker.set_seed_configuration(q)
        self._ik_server.setPose("ik_target", se3ToPose(initial_pose))
        self._ik_server.applyChanges()

    def _on_start_menu(self, _):
        try:
            self._start()
            self.get_logger().info("Streaming started.")
        except Exception as e:
            self.get_logger().error(f"Failed to start: {e}")

    def _on_stop_menu(self, _):
        self._stop()
        self.get_logger().info("Streaming stopped.")

    def _on_reset_menu(self, _):
        try:
            self._reset()
            self.get_logger().info("Reset to current hardware state.")
        except Exception as e:
            self.get_logger().error(f"Failed to reset: {e}")

    def _on_reset_srv(self, _, response):
        try:
            self._reset()
            response.success = True
            response.message = "Reset to current hardware state."
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self._running = False
        self._control_thread.join(timeout=1.0)
        self._js.shutdown()
        self._marker_executor.shutdown()
        self._marker_thread.join(timeout=0.25)
        self._marker_node.destroy_node()

        self._ik_marker = None
        self._frame_task = None
        self._tasks = None
        self._constraints = None
        self._barriers = None
        self._oink = None
        self._reference_filter = None
        self._scene = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CartesianServoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
