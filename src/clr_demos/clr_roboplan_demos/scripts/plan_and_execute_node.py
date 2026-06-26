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
Example node demonstrating a basic plan, preview, and move workflow:
  1. Set a target pose with an interactive marker (IK)
  2. Plan a trajectory to that pose
  3. Preview the trajectory with visualization
  4. Execute (publish to "hardware")

Intended as an example _only_. Consumers are expected to use this as a
reference, rather than hardened application code.
"""

import time
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from interactive_markers import InteractiveMarkerServer, MenuHandler
from std_srvs.srv import Trigger

from roboplan.core import (
    CartesianConfiguration,
    JointConfiguration,
    PathShortcuttingOptions,
    PathShortcutter,
)
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.rrt import RRT, RRTOptions
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan_ros.visualization import RoboplanVisualizer, RoboplanIKMarker, markerFromJointTrajectory
from roboplan_ros.cpp import se3ToPose, toJointTrajectory
from roboplan_ros_py.trajectory_publisher import TrajectoryPublisher

from clr_roboplan_demos import (
    BEST_EFFORT_QOS,
    create_scene,
    get_robot_config,
    spin_executor,
    JointStateTracker,
)


class PlanAndExecuteNode(Node):
    """
    Example node with a set-pose, plan, preview, execute workflow.

    Uses an interactive marker for IK-based target selection, RRT for
    planning, TOPP-RA for time parameterization, and wraps a joint
    trajectory controller's action interface for execution.

    No monitoring provided, this is example code only!
    """

    def __init__(self):
        super().__init__("plan_and_execute_node")

        self.declare_parameter("robot", "clr")
        self._config = get_robot_config(self.get_parameter("robot").value)
        self.get_logger().info(
            f"Using robot config '{self._config.name}' "
            f"(group={self._config.joint_group})"
        )

        # Scene setup
        self._scene, urdf_xml, _ = create_scene()

        self._joint_group = self._config.joint_group
        self._base_link = self._config.base_link
        self._tip_link = self._config.tip_link

        group_info = self._scene.getJointGroupInfo(self._joint_group)
        self._q_indices = group_info.q_indices

        # Joint state tracking
        self._js = JointStateTracker(
            self._scene, "/joint_states", self.get_logger()
        )
        self._js.wait_for_joint_state(self.get_logger())

        # Set the IK solver options
        ik_options = SimpleIkOptions()
        ik_options.group_name = self._joint_group
        ik_options.step_size = 0.25
        ik_options.check_collisions = True

        # Increases likelihood of finding an "optimal" solution
        ik_options.fast_return = False
        ik_options.max_iters = 500
        ik_options.max_time = 0.05

        # Setup an IK solver and function to pass to the interactive marker
        ik_solver = SimpleIk(self._scene, ik_options)
        q_indices = self._q_indices
        joint_group = self._joint_group
        base_link = self._base_link
        tip_link = self._tip_link
        scene = self._scene

        def ik_solve_fn(target_pose, seed):
            goal = CartesianConfiguration()
            goal.base_frame = base_link
            goal.tip_frame = tip_link
            goal.tform = target_pose
            seed_jc = JointConfiguration()
            seed_jc.positions = seed[q_indices]
            solution = JointConfiguration()
            if ik_solver.solveIk(goal, seed_jc, solution):
                return scene.toFullJointPositions(joint_group, solution.positions)
            return None

        self._ik_marker = RoboplanIKMarker(
            scene=self._scene,
            base_link=self._base_link,
            tip_link=self._tip_link,
            ik_solve_fn=ik_solve_fn,
        )

        # Set up planning utilities
        self._rrt_options = RRTOptions()
        self._rrt_options.group_name = self._joint_group
        self._rrt_options.max_connection_distance = 3.0
        self._rrt_options.collision_check_step_size = 0.05
        self._rrt_options.max_planning_time = 5.0
        self._rrt_options.rrt_connect = True
        self._rrt_options.max_nodes = 10000
        self._rrt_options.goal_biasing_probability = 0.05
        self._rrt_options.collision_check_use_bisection = True
        self._include_shortcutting = True
        self._max_shortcutting_iters = 250

        self._shortcutting_options = PathShortcuttingOptions(
            group_name=self._joint_group,
            max_step_size=self._rrt_options.collision_check_step_size,
            max_iters=self._max_shortcutting_iters,
        )

        self._rrt = RRT(self._scene, self._rrt_options)
        self._toppra = PathParameterizerTOPPRA(self._scene, self._joint_group)
        self._shortcutter = PathShortcutter(self._scene, self._shortcutting_options)
        self._traj_dt = 0.01

        # Configure elements for determining and previewing poses from the
        # iMarker
        self._marker_node = Node("imarker_server_node")
        self._ik_server = InteractiveMarkerServer(self._marker_node, "roboplan_ik")
        self._ik_server.insert(
            self._ik_marker.construct_imarker(),
            feedback_callback=self._on_ik_feedback,
        )
        self._ik_server.applyChanges()

        # Needs its own executor for responsiveness
        self._marker_executor = SingleThreadedExecutor()
        self._marker_executor.add_node(self._marker_node)
        self._marker_thread = threading.Thread(
            target=spin_executor, daemon=True, args=(self._marker_executor, self.get_logger())
        )
        self._marker_thread.start()

        # Add menu to the iMarker for service access
        menu = MenuHandler()
        menu.insert("Plan", callback=self._on_plan_menu)
        menu.insert("Preview", callback=self._on_preview_menu)
        menu.insert("Execute", callback=self._on_execute_menu)
        menu.insert("Reset", callback=self._on_reset_menu)
        menu.apply(self._ik_server, "ik_target")
        self._ik_server.applyChanges()

        # IK determined target pose in blue
        self._ik_visualizer = RoboplanVisualizer(
            scene=self._scene,
            group_name=self._joint_group,
            urdf_xml=urdf_xml,
            frame_id="world",
            ns="roboplan_ik",
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),
        )
        self._ik_marker_pub = self.create_publisher(
            MarkerArray, "roboplan_ik/markers", BEST_EFFORT_QOS
        )

        # Configure tools for previewing trajectories, the markers will be
        # published in green.
        self._traj_visualizer = RoboplanVisualizer(
            scene=self._scene,
            group_name=self._joint_group,
            urdf_xml=urdf_xml,
            frame_id="world",
            ns="roboplan_traj",
            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3),
        )
        self._traj_marker_pub = self.create_publisher(
            MarkerArray, "roboplan_trajectory/markers", BEST_EFFORT_QOS
        )
        self._player = TrajectoryPublisher(
            self._scene,
            self._traj_visualizer,
            self._traj_marker_pub,
            self._q_indices,
        )

        # Publish the planned end-effector path as a light green line upon planning.
        # Unlike the IK/preview markers (which are republished continuously), the path
        # is published exactly once per plan, so use a latched QoS.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._planned_path_color = ColorRGBA(r=0.5, g=1.0, b=0.5, a=1.0)
        self._planned_path_pub = self.create_publisher(Marker, "/roboplan_trajectory/path", latched_qos)

        # Setup an action client for trajectory execution
        self._execute_client = ActionClient(
            self, FollowJointTrajectory, self._config.controller_action
        )

        # Target pose and planned trajectories
        self._target_q = None
        self._planned_traj = None

        # Setup Trigger Services
        self.create_service(Trigger, "~/plan", self._on_plan)
        self.create_service(Trigger, "~/preview", self._on_preview)
        self.create_service(Trigger, "~/execute", self._on_execute)
        self.create_service(Trigger, "~/reset", self._on_reset)

        # Reset and notify
        self._reset()
        self.get_logger().info("Ready. Move the interactive marker to set a target.")
        self.get_logger().info("Call services: ~/plan, ~/preview, ~/execute, ~/reset")

    def _on_ik_feedback(self, feedback):
        self._ik_marker.set_seed_configuration(self._js.latest_positions)
        q = self._ik_marker.process_feedback(feedback)
        if q is None:
            self.get_logger().warning("IK failed to solve")
        else:
            self._target_q = q
            self._ik_marker_pub.publish(self._ik_visualizer.markers_from_configuration(q))

    def _plan(self):
        if self._js.last_msg is None:
            return False, "Have not yet received joint positions from hardware. Cannot plan."
        if self._target_q is None:
            return False, "No target set. Move the interactive marker first."

        q = self._js.sync_to_hardware()
        self._scene.setJointPositions(q)

        start = JointConfiguration()
        start.positions = q[self._q_indices]

        goal = JointConfiguration()
        goal.positions = self._target_q[self._q_indices]

        self.get_logger().info("Planning...")
        plan_start_time = time.time()

        try:
            start_time = time.time()
            path = self._rrt.plan(start, goal)
            self.get_logger().info(f"  Finished planning in {time.time() - start_time} seconds.")
        except RuntimeError as e:
            self.get_logger().error(str(e))
            path = None

        if path is None:
            return False, "Planning failed."

        if self._include_shortcutting:
            self.get_logger().info("Shortcutting...")
            start_time = time.time()
            path = self._shortcutter.shortcut(path)
            self.get_logger().info(f"  Finished shortcutting in {time.time() - start_time} seconds.")

        self.get_logger().info("Generating trajectory...")
        start_time = time.time()
        self._planned_traj = self._toppra.generate(
            path,
            TOPPRAOptions(
                self._traj_dt,
                mode=SplineFittingMode.Adaptive,
                max_adaptive_iterations=5,
            ),
        )
        self.get_logger().info(f"  Finished generating trajectory in {time.time() - start_time} seconds.")

        self.get_logger().info(f"Total planning time: {time.time() - plan_start_time} seconds.")

        # Visualize the planned end-effector trajectory.
        self._planned_path_pub.publish(
            markerFromJointTrajectory(
                self._scene,
                self._planned_traj,
                [self._tip_link],
                frame_id="world",
                ns="planned_trajectory",
                color=self._planned_path_color,
            )
        )

        return (
            True,
            f"Planned trajectory with {len(self._planned_traj.positions)} points",
        )

    def _preview(self):
        if self._planned_traj is None:
            return False, "No trajectory to preview. Plan first."

        self.get_logger().info("Previewing trajectory...")
        self._player.play(
            self._planned_traj,
            self._traj_dt,
            on_complete=lambda: self.get_logger().info("Preview complete."),
        )
        return True, "Playback started."

    def _execute(self):
        if self._planned_traj is None:
            return False, "No trajectory to execute. Plan first."

        if not self._execute_client.wait_for_server(timeout_sec=2.0):
            return False, "Action server not available."

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = toJointTrajectory(self._planned_traj)

        self.get_logger().info("Sending trajectory for execution...")
        future = self._execute_client.send_goal_async(goal, feedback_callback=self._execute_feedback)
        future.add_done_callback(self._execute_goal_response)

        return True, "Trajectory sent for execution."

    def _execute_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Trajectory execution rejected.")
            return

        self.get_logger().info("Trajectory accepted, executing...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._execute_result)

    def _execute_feedback(self, feedback_msg):
        pass

    def _execute_result(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory execution complete.")
        else:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.error_code}")

    def _reset(self):
        """Clears all plans and resets to a hardware state."""
        if self._js.last_msg is None:
            raise RuntimeError("No joint states received, cannot reset to hw state.")

        q = self._js.sync_to_hardware()

        self._ik_marker.set_seed_configuration(q)
        fk = self._scene.forwardKinematics(q, self._tip_link, self._base_link)
        pose = se3ToPose(fk)

        self._ik_server.setPose("ik_target", pose)
        self._ik_server.applyChanges()
        self._ik_marker_pub.publish(self._ik_visualizer.markers_from_configuration(q))

        self._target_q = None
        self._planned_traj = None
        self._traj_marker_pub.publish(self._traj_visualizer.clear_markers())
        delete_marker = Marker()
        delete_marker.header.frame_id = "world"
        delete_marker.action = Marker.DELETEALL
        self._planned_path_pub.publish(delete_marker)

    # Menu callbacks
    def _on_plan_menu(self, feedback):
        _, msg = self._plan()
        self.get_logger().info(msg)

    def _on_preview_menu(self, feedback):
        _, msg = self._preview()
        self.get_logger().info(msg)

    def _on_execute_menu(self, feedback):
        _, msg = self._execute()
        self.get_logger().info(msg)

    def _on_reset_menu(self, feedback):
        try:
            self._reset()
            self.get_logger().info("Reset node to current state.")
        except Exception as e:
            self.get_logger().error(f"Failed to reset the node: {e}")

    # Trigger service callbacks
    def _on_plan(self, request, response):
        response.success, response.message = self._plan()
        return response

    def _on_preview(self, request, response):
        response.success, response.message = self._preview()
        return response

    def _on_execute(self, request, response):
        response.success, response.message = self._execute()
        return response

    def _on_reset(self, request, response):
        try:
            self._reset()
            response.success = True
            response.message = "Reset node to current state."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to reset the node: {e}"
            self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self._player.stop()
        self._js.shutdown()
        self._marker_executor.shutdown()
        self._marker_thread.join(timeout=0.25)
        self._marker_node.destroy_node()

        # Explicitly deconstruct things
        self._ik_marker = None
        self._ik_visualizer = None
        self._traj_visualizer = None
        self._ik_solver = None
        self._rrt = None
        self._toppra = None
        self._shortcutter = None
        self._scene = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlanAndExecuteNode()
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
