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
from scipy.spatial.transform import Rotation, Slerp

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from interactive_markers import InteractiveMarkerServer, MenuHandler

from roboplan.core import CartesianConfiguration
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
from roboplan_ros.cpp import (
    buildConversionMap,
    fromJointState,
    se3ToPose,
)

from clr_roboplan_demos import (
    create_scene,
    get_robot_config,
    run_node,
    spin_executor,
)
from clr_roboplan_demos.utils import JointStateSubscriber


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

        # Start paused so the robot doesn't move until the user is ready and the
        # iMarker has been initialized. Otherwise danger.
        self._paused = True

        # Optimal IK params
        self.declare_parameter("task_gain", 1.0)
        self.declare_parameter("lm_damping", 0.01)
        self.declare_parameter("regularization", 1e-5)
        self.declare_parameter("position_cost", 1.0)
        self.declare_parameter("orientation_cost", 1.0)
        self.declare_parameter("control_freq", 25.0)
        self.declare_parameter("command_duration_ms", 0)

        # Commanded linear and angular velocities, along with a maximum tracking
        # error as a very basic safety mechanism.
        self.declare_parameter("linear_velocity", 0.1)
        self.declare_parameter("angular_velocity", 0.1)
        self.declare_parameter("max_tracking_error", 0.1)

        control_freq = self.get_parameter("control_freq").value
        task_gain = self.get_parameter("task_gain").value
        lm_damping = self.get_parameter("lm_damping").value
        self._regularization = self.get_parameter("regularization").value
        position_cost = self.get_parameter("position_cost").value
        orientation_cost = self.get_parameter("orientation_cost").value
        self._command_duration_ms = self.get_parameter("command_duration_ms").value
        self._linear_velocity = self.get_parameter("linear_velocity").value
        self._angular_velocity = self.get_parameter("angular_velocity").value
        self._max_tracking_error = self.get_parameter("max_tracking_error").value

        # Control loop time step for Cartesian tracking
        self._dt = 1.0 / control_freq

        # Subscribe to joint states to keep the scene in sync with hardware. These
        # can bog down other CBs, so putting it out here keeps the rest of the node
        # responsive.
        self._js_subscriber = JointStateSubscriber("clr_joint_state_listener", "/joint_states")

        # Wait for joint states
        while self._js_subscriber.last_joint_state is None:
            self.get_logger().info("Waiting for joint positions...")
            time.sleep(1.0)

        # Once we have joint states we can build the conversion map
        self._conversion_map = buildConversionMap(
            self._scene, self._js_subscriber.last_joint_state
        )

        # Set up the solver
        self._oink = Oink(self._scene, self._config.joint_group)
        self._num_variables = len(self._oink.v_indices)

        # Frame task for end-effector tracking (priority 1)
        # High task_gain drives tight per-tick tracking — the reference filter
        # upstream handles smoothing, so the solver should chase the filtered
        # target as closely as possible each step for a straight Cartesian path.
        goal = CartesianConfiguration()
        goal.base_frame = self._config.base_link
        goal.tip_frame = self._config.tip_link

        task_options = FrameTaskOptions(
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            task_gain=task_gain,
            lm_damping=lm_damping,
        )
        self._frame_task = FrameTask(self._oink, self._scene, goal, task_options)

        # Configuration task to regularize toward starting pose (priority 2).
        # Projected into the nullspace of the frame task so it never sacrifices
        # end-effector tracking — only uses redundant degrees of freedom.
        q_home = np.array(self._scene.getCurrentJointPositions())
        joint_weights = np.full(self._num_variables, 0.05)
        config_task = ConfigurationTask(
            self._oink,
            q_home[self._oink.q_indices],
            joint_weights,
            ConfigurationTaskOptions(task_gain=1.0, lm_damping=0.0, priority=2),
        )

        self._tasks = [self._frame_task, config_task]

        # Constraints: joint position and velocity limits
        position_limit = PositionLimit(self._oink, gain=1.0)
        v_max = np.hstack(
            [
                self._scene.getJointInfo(name).limits.max_velocity
                for name in self._joint_names
            ]
        )
        velocity_limit = VelocityLimit(self._oink, self._dt, v_max)
        self._constraints = [position_limit, velocity_limit]

        # TODO: Add self collision / env collision barriers
        self._barriers = []

        # Thread-safe access to scene and target
        self._lock = threading.Lock()

        # Constant-velocity reference pose — steps toward _raw_target each tick
        q_full = self._scene.getCurrentJointPositions()
        initial_pose = self._scene.forwardKinematics(
            q_full, self._config.tip_link, self._config.base_link
        )
        self._raw_target = initial_pose.copy()
        self._reference_pose = initial_pose.copy()

        self._delta_q = np.zeros(self._num_variables)
        self._delta_q_full = np.zeros(len(q_full))

        # This is a little odd because we don't actually want to do the solving
        # while dragging the ik marker. Really we just want to know the target pose
        # so that OinK can do the work of computing joint commands in the _control
        # loop_. Still, the marker does some nice things for us so we include it here
        # and just save the target pose away.
        def store_target(target_pose, _):
            self._raw_target = target_pose.copy()
            return None

        self._ik_marker = RoboplanIKMarker(
            scene=self._scene,
            base_link=self._config.base_link,
            tip_link=self._config.tip_link,
            ik_solve_fn=store_target,
        )

        # Interactive marker server
        self._marker_node = Node("imarker_server_node")
        self._ik_server = InteractiveMarkerServer(self._marker_node, "roboplan_ik")
        self._ik_server.insert(
            self._ik_marker.construct_imarker(),
            feedback_callback=self._on_ik_feedback,
        )
        self._ik_server.applyChanges()

        # Add menu items to reset, start, and pause the marker streaming
        menu = MenuHandler()
        menu.insert("Start", callback=self._on_start_menu)
        menu.insert("Pause", callback=self._on_pause_menu)
        menu.insert("Reset", callback=self._on_reset_menu)
        menu.apply(self._ik_server, "ik_target")
        self._ik_server.applyChanges()

        self._marker_executor = SingleThreadedExecutor()
        self._marker_executor.add_node(self._marker_node)
        self._marker_thread = threading.Thread(
            target=spin_executor, daemon=True, args=(self._marker_executor,)
        )
        self._marker_thread.start()

        # Joint command publisher
        self._cmd_pub = self.create_publisher(JointTrajectory, self._config.controller_joint_trajectory_topic, 10)

        # Reset service
        self.create_service(Trigger, "~/reset", self._on_reset)

        # Start control loop
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

        # Start a background thread for safety checks
        self._safety_thread = threading.Thread(target=self._safety_loop, daemon=True)
        self._safety_thread.start()

        # Reset and notify
        self._reset()
        self.get_logger().info(
            "Ready. Drag the interactive marker, then right-click > Start to begin servoing."
        )

    def _on_ik_feedback(self, feedback):
        """Pass feedback through the marker. solve_fn stores the target;
        the control loop tracks it."""
        self._ik_marker.set_seed_configuration(self._latest_joint_positions)
        self._ik_marker.process_feedback(feedback)

    def _control_loop(self):
        """Continuously run one OInK step per tick while not paused

        Will update commands based on the physical pose of the robot.
        """
        while self._running:
            loop_start = time.time()

            if not self._paused:
                with self._lock:
                    # Update control step from the last known joint position
                    if self._js_subscriber.last_joint_state is not None:
                        joint_config = fromJointState(
                            self._js_subscriber.last_joint_state,
                            self._scene,
                            self._conversion_map,
                        )
                        q_current = joint_config.positions
                    else:
                        q_current = np.array(self._scene.getCurrentJointPositions())

                    self._scene.setJointPositions(q_current)
                    self._scene.forwardKinematics(q_current, self._config.tip_link)

                    # Compute the target reference pose
                    self._step_reference()
                    self._frame_task.setTargetFrameTransform(self._reference_pose)

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
                            f"IK solver failed: {e}", throttle_duration_sec=1.0
                        )

                    self._delta_q_full[:] = 0.0
                    self._delta_q_full[self._oink.v_indices] = self._delta_q
                    q_commanded = self._scene.integrate(q_current, self._delta_q_full)

                    # Update scene to commanded state for FK consistency
                    self._scene.setJointPositions(q_commanded)
                    self._scene.forwardKinematics(q_commanded, self._config.tip_link)
                    self._latest_joint_positions = q_current  # seed marker from hw

                self._publish_joint_command(q_commanded)

            elapsed = time.time() - loop_start
            time.sleep(max(0, self._dt - elapsed))

    def _step_reference(self):
        """Advance the reference_pose towards the raw_target at a constant linear/angular velocity."""
        # Translation distance is just euclidean norm
        t_curr = self._reference_pose[:3, 3]
        t_targ = self._raw_target[:3, 3]
        t_diff = t_targ - t_curr
        trans_dist = np.linalg.norm(t_diff)

        # Rotation with the axis-angle of relative rotation
        r_curr = Rotation.from_matrix(self._reference_pose[:3, :3])
        r_targ = Rotation.from_matrix(self._raw_target[:3, :3])
        rot_angle = (r_curr.inv() * r_targ).magnitude()

        # Compute the time each component needs to arrive based on velocities
        t_trans = trans_dist / self._linear_velocity if trans_dist > 1e-8 else 0.0
        t_rot = rot_angle / self._angular_velocity if rot_angle > 1e-6 else 0.0
        t_arrive = max(t_trans, t_rot)

        # If we'd arrive within one tick, snap to target
        if t_arrive <= self._dt:
            self._reference_pose = self._raw_target.copy()
            return

        # Scale the linear and rotational interpolations so that
        # they arrive at the same time
        alpha = self._dt / t_arrive

        # Linear interpolation of translation
        self._reference_pose[:3, 3] = t_curr + alpha * t_diff

        # Slerp for rotation interpolation
        if rot_angle > 1e-6:
            slerp = Slerp([0.0, 1.0], Rotation.concatenate([r_curr, r_targ]))
            self._reference_pose[:3, :3] = slerp(alpha).as_matrix()

    def _publish_joint_command(self, q):
        """Publish a single-point JointTrajectory to command the robot."""
        msg = JointTrajectory()
        msg.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = q[self._q_indices].tolist()
        point.time_from_start = rclpy.duration.Duration(
            nanoseconds=self._command_duration_ms * 1E6
        ).to_msg()
        msg.points = [point]
        self._cmd_pub.publish(msg)

    def _safety_loop(self):
        """Monitor tracking error and pause if the robot can't keep up.

        TODO: This is pretty band-aid-y but could be improved with collision checking, etc.
        """
        while self._running:
            if not self._paused and self._js_subscriber.last_joint_state is not None:
                try:
                    joint_config = fromJointState(
                        self._js_subscriber.last_joint_state,
                        self._scene,
                        self._conversion_map,
                    )
                    q_hw = joint_config.positions
                    actual_pose = self._scene.forwardKinematics(
                        q_hw, self._config.tip_link, self._config.base_link
                    )
                    tracking_error = np.linalg.norm(
                        actual_pose[:3, 3] - self._reference_pose[:3, 3]
                    )
                    if tracking_error > self._max_tracking_error:
                        self._paused = True
                        self.get_logger().error(
                            f"Tracking error {tracking_error:.3f}m exceeds limit "
                            f"{self._max_tracking_error:.3f}m — pausing. "
                            f"Reset and restart to continue."
                        )
                except Exception as e:
                    self.get_logger().warn(
                        f"Safety monitor thread crashed! {e}",
                        throttle_duration_sec=1.0,
                    )
            # Hardcoding to 10 hz for now
            time.sleep(0.1)

    def _on_start_menu(self, _):
        self._reset()
        self._paused = False
        self.get_logger().info("Servoing started.")

    def _on_pause_menu(self, _):
        self._paused = True
        self.get_logger().info("Servoing paused.")

    def _on_reset_menu(self, _):
        self._reset()
        self.get_logger().info("Reset marker to current hardware state.")

    def _on_reset(self, _, response):
        self._reset()
        response.success = True
        response.message = "Reset marker to current hardware state."
        self.get_logger().info(response.message)
        return response

    def _reset(self):
        """Reset the marker and control state to the current hardware state."""
        if self._js_subscriber.last_joint_state is None:
            raise RuntimeError("No joint states received, cannot reset to hw state.")

        # Pause it
        self._on_pause_menu(None)

        joint_config = fromJointState(
            self._js_subscriber.last_joint_state, self._scene, self._conversion_map
        )
        self._latest_joint_positions = joint_config.positions

        with self._lock:
            self._scene.setJointPositions(self._latest_joint_positions)
            self._scene.forwardKinematics(self._latest_joint_positions, self._config.tip_link)
            initial_pose = self._scene.forwardKinematics(
                self._latest_joint_positions, self._config.tip_link, self._config.base_link
            )
            self._raw_target = initial_pose.copy()
            self._reference_pose = initial_pose.copy()

        self._ik_marker.set_seed_configuration(self._latest_joint_positions)
        pose = se3ToPose(initial_pose)
        self._ik_server.setPose("ik_target", pose)
        self._ik_server.applyChanges()

    def destroy_node(self):
        # Stop the control loop first so it releases the lock
        self._running = False
        self._control_thread.join(timeout=1.0)
        self._safety_thread.join(timeout=1.0)

        # Shut down executors and join their threads
        self._js_subscriber.shutdown()
        self._marker_executor.shutdown()
        self._marker_thread.join(timeout=0.25)
        self._marker_node.destroy_node()

        # Manually remove self referenced nanobind objects before destruction.
        # https://nanobind.readthedocs.io/en/latest/refleaks.html
        self._ik_marker = None

        super().destroy_node()


if __name__ == "__main__":
    rclpy.init()
    run_node(CartesianServoNode())
