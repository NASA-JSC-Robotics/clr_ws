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
RoboPlan-based motion planning server for CLR behavior demos.

This node:
  * Loads the planning scene URDF from the robot description topic
    (falling back to processing the xacro files directly).
  * Keeps the scene in sync with the /joint_states topic from the robot.
  * Offers planning services that return trajectory_msgs/JointTrajectory
    messages, which callers execute by commanding the ROS 2 controllers:
      - ~/plan_to_joint_state: free-space planning to a joint configuration
      - ~/plan_to_pose: IK + free-space planning to an end effector pose
      - ~/plan_cartesian_path: straight-line Cartesian motion to a pose
    Each service takes an optional joint group name, validated against the
    groups in the scene.
  * Offers Trigger services to re-preview (~/preview) and execute
    (~/execute) the last planned trajectory, plus ~/reset.
  * Retains the interactive marker workflow from plan_and_execute_node.py,
    with an extra menu entry for Cartesian planning to the marker pose.
"""

import time
import threading
from dataclasses import dataclass, field

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from interactive_markers import InteractiveMarkerServer, MenuHandler
from std_srvs.srv import Trigger

from clr_behavior_demos_msgs.srv import (
    PlanCartesianPath,
    PlanToJointState,
    PlanToPose,
    SetCollisions,
)

from roboplan.core import (
    CartesianConfiguration,
    CartesianPath,
    JointConfiguration,
    PathShortcuttingOptions,
    PathShortcutter,
)
from roboplan.cartesian_planning import (
    CartesianPathPlanner,
    CartesianPlannerOptions,
    CartesianSpeedMode,
)
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.rrt import (
    ConstraintProjector,
    ConstraintProjectorOptions,
    PoseConstraint,
    RRT,
    RRTOptions,
)
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan_ros.visualization import RoboplanVisualizer, RoboplanIKMarker, markerFromJointTrajectory
from roboplan_ros.cpp import (
    buildConversionMap,
    fromJointState,
    poseToSE3,
    se3ToPose,
    toJointTrajectory,
)
from roboplan_ros_py.trajectory_publisher import TrajectoryPublisher

from clr_roboplan_demos import (
    BEST_EFFORT_QOS,
    create_scene,
    get_robot_config,
    run_node,
    spin_executor,
)
from clr_roboplan_demos.utils import JointStateSubscriber


def get_robot_description(topic: str, timeout_sec: float):
    """
    Waits for a URDF on the (latched) robot description topic.

    Returns the URDF XML string, or None if nothing was received in time.
    """
    listener_node = Node("robot_description_listener")
    executor = SingleThreadedExecutor()
    executor.add_node(listener_node)

    received = {}
    latched_qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    listener_node.create_subscription(
        String,
        topic,
        lambda msg: received.setdefault("urdf", msg.data),
        latched_qos,
    )

    end_time = time.monotonic() + timeout_sec
    while ("urdf" not in received) and (time.monotonic() < end_time):
        executor.spin_once(timeout_sec=0.1)

    executor.shutdown()
    listener_node.destroy_node()
    return received.get("urdf")


@dataclass
class GroupPlanningContext:
    """Planning utilities instantiated for a single joint group."""

    name: str
    joint_names: list = field(default_factory=list)
    q_indices: object = None
    ik_solver: object = None
    rrt: object = None

    # Canned RRT option sets. The planner is reconfigured with one of these on
    # every request, depending on whether the top-down constraint is enabled.
    rrt_options: object = None
    constrained_rrt_options: object = None
    top_down_constraint: object = None
    constraint_projector: object = None
    shortcutter: object = None
    toppra: object = None
    visualizer: object = None
    player: object = None

    # Per-DOF position limits for the group, or None if they could not be
    # mapped one-to-one onto the configuration vector.
    min_position: object = None
    max_position: object = None


class RoboplanPlanningServer(Node):
    """
    Motion planning service server backed by RoboPlan.

    Uses RRT + TOPP-RA for free-space planning and a Cartesian path planner
    for straight-line motions. Planned trajectories are returned as ROS
    JointTrajectory messages for execution on the ROS 2 controllers, and the
    last planned trajectory can be previewed on demand.

    No monitoring provided, this is demo code only!
    """

    def __init__(self):
        super().__init__("roboplan_planning_server")

        self.declare_parameter("robot", "clr")
        self._config = get_robot_config(self.get_parameter("robot").value)
        self.get_logger().info(f"Using robot config '{self._config.name}' (group={self._config.joint_group})")

        # Joint groups to serve. Planning requests for other groups are
        # rejected. The first entry that matches the robot config's group is
        # used for the interactive marker workflow.
        self.declare_parameter("planning_groups", ["clr", "ur_manipulator", "chonkur_grasp", "rail", "lift"])

        # Scene loading parameters.
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("robot_description_timeout", 10.0)

        # Joints resting exactly on a position limit can trip strict limit
        # checks in the planners from numerical drift alone, so start
        # configurations are nudged this far inside the limits.
        self.declare_parameter("joint_limit_margin", 1.0e-4)

        # Roll and pitch bound, in degrees, for the top-down gripper
        # constraint offered by ~/plan_to_pose.
        self.declare_parameter("top_down_tilt_bound_degrees", 5.0)

        # Cartesian planner settings. In "time_optimal" mode the trajectory is
        # re-timed optimally against the joint limits; in "bounded" mode the
        # tool speed/acceleration caps below apply instead. Slow auxiliary
        # joints (e.g. the lift) make bounded mode crawl, since its global
        # slow-down retry re-times the whole motion around their tiny
        # acceleration budgets, so time-optimal is the default.
        self.declare_parameter("cartesian_speed_mode", "time_optimal")
        self.declare_parameter("max_linear_speed", 0.1)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("max_linear_acceleration", 0.5)
        self.declare_parameter("max_angular_acceleration", 2.5)
        self.declare_parameter("max_position_error", 0.02)
        self.declare_parameter("max_orientation_error", 0.2)

        # Scene setup. Prefer the URDF from the robot description topic so the
        # planning scene always matches what the robot is actually running,
        # and fall back to processing the xacro files directly.
        description_topic = self.get_parameter("robot_description_topic").value
        description_timeout = self.get_parameter("robot_description_timeout").value
        self.get_logger().info(f"Waiting for robot description on '{description_topic}'...")
        urdf_xml = get_robot_description(description_topic, description_timeout)
        if urdf_xml is None:
            self.get_logger().warning(
                f"No robot description received on '{description_topic}' after "
                f"{description_timeout} seconds. Falling back to xacro files."
            )
        self._scene, self._urdf_xml, _ = create_scene(urdf_xml=urdf_xml)

        self._joint_group = self._config.joint_group
        self._base_link = self._config.base_link
        self._tip_link = self._config.tip_link
        self._traj_dt = 0.01
        self._include_shortcutting = True
        self._max_shortcutting_iters = 250

        # Serializes access to the scene and planners across service calls and
        # the interactive marker menu callbacks.
        self._planning_lock = threading.RLock()

        # Subscribe to joint states to keep the scene in sync with hardware. These
        # can bog down other CBs, so putting it out here keeps the rest of the node
        # responsive.
        self._js_subscriber = JointStateSubscriber("clr_joint_state_listener", "/joint_states")

        # Wait for joint states
        while self._js_subscriber.last_joint_state is None:
            self.get_logger().info("Waiting for joint positions...")
            time.sleep(1.0)

        # Once we have joint states we can build the conversion map
        self._conversion_map = buildConversionMap(self._scene, self._js_subscriber.last_joint_state)

        # Configure tools for previewing trajectories, the markers will be
        # published in green. The visualizers and players are per joint group
        # and live in the group planning contexts.
        self._traj_marker_pub = self.create_publisher(MarkerArray, "roboplan_trajectory/markers", BEST_EFFORT_QOS)

        # Build the planning contexts for the configured groups up front, so
        # bad group names fail at startup rather than on first request.
        self._group_contexts = {}
        for group_name in self.get_parameter("planning_groups").value:
            self._build_group_context(group_name)
        if self._joint_group not in self._group_contexts:
            raise RuntimeError(
                f"The robot config's joint group '{self._joint_group}' must be in the "
                f"'planning_groups' parameter ({list(self._group_contexts)})."
            )
        self._default_ctx = self._group_contexts[self._joint_group]
        self._q_indices = self._default_ctx.q_indices

        # A separate IK solver instance for the interactive marker since its
        # feedback runs on its own thread.
        marker_ik_solver = SimpleIk(self._scene, self._make_ik_options(self._joint_group))

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
            if marker_ik_solver.solveIk(goal, seed_jc, solution):
                return scene.toFullJointPositions(joint_group, solution.positions)
            return None

        self._ik_marker = RoboplanIKMarker(
            scene=self._scene,
            base_link=self._base_link,
            tip_link=self._tip_link,
            ik_solve_fn=ik_solve_fn,
        )

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
        self._marker_thread = threading.Thread(target=spin_executor, daemon=True, args=(self._marker_executor,))
        self._marker_thread.start()

        # Add menu to the iMarker for service access
        menu = MenuHandler()
        menu.insert("Plan", callback=self._on_plan_menu)
        menu.insert("Plan Cartesian", callback=self._on_plan_cartesian_menu)
        menu.insert("Preview", callback=self._on_preview_menu)
        menu.insert("Execute", callback=self._on_execute_menu)
        menu.insert("Reset", callback=self._on_reset_menu)
        menu.apply(self._ik_server, "ik_target")
        self._ik_server.applyChanges()

        # IK determined target pose in blue
        self._ik_visualizer = RoboplanVisualizer(
            scene=self._scene,
            group_name=self._joint_group,
            urdf_xml=self._urdf_xml,
            frame_id="world",
            ns="roboplan_ik",
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),
        )
        self._ik_marker_pub = self.create_publisher(MarkerArray, "roboplan_ik/markers", BEST_EFFORT_QOS)

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
        self._execute_client = ActionClient(self, FollowJointTrajectory, self._config.controller_action)

        # Target pose and planned trajectories
        self._target_q = None
        self._target_marker_pose = None
        self._planned_traj = None
        self._planned_ctx = None

        # Setup planning services for behavior trees and other clients.
        self.create_service(PlanToJointState, "~/plan_to_joint_state", self._on_plan_to_joint_state)
        self.create_service(PlanToPose, "~/plan_to_pose", self._on_plan_to_pose)
        self.create_service(PlanCartesianPath, "~/plan_cartesian_path", self._on_plan_cartesian_path)
        self.create_service(SetCollisions, "~/set_collisions", self._on_set_collisions)

        # Setup Trigger services. The plan services target the interactive
        # marker pose; preview/execute act on the last planned trajectory
        # regardless of which service produced it.
        self.create_service(Trigger, "~/plan", self._on_plan)
        self.create_service(Trigger, "~/plan_cartesian", self._on_plan_cartesian)
        self.create_service(Trigger, "~/preview", self._on_preview)
        self.create_service(Trigger, "~/execute", self._on_execute)
        self.create_service(Trigger, "~/reset", self._on_reset)

        # Reset and notify
        self._reset()
        self.get_logger().info("Ready. Move the interactive marker or call the planning services.")
        self.get_logger().info(
            "Services: ~/plan_to_joint_state, ~/plan_to_pose, ~/plan_cartesian_path, "
            "~/plan, ~/plan_cartesian, ~/preview, ~/execute, ~/reset"
        )

    def _make_ik_options(self, group_name):
        """IK solver options shared by the services and the interactive marker."""
        ik_options = SimpleIkOptions()
        ik_options.group_name = group_name
        ik_options.step_size = 0.2
        ik_options.check_collisions = True

        # Increases likelihood of finding an "optimal" solution. The per-solve
        # budget is kept moderate since _solve_ik runs several scored attempts.
        ik_options.fast_return = False
        ik_options.max_iters = 500
        ik_options.max_restarts = 5
        ik_options.max_time = 0.1
        return ik_options

    def _get_group_context(self, group_name):
        """
        Returns (context, error_message) for one of the served joint groups.
        An empty group name maps to the server's default group.
        """
        group_name = group_name or self._joint_group
        ctx = self._group_contexts.get(group_name)
        if ctx is None:
            return None, (
                f"Joint group '{group_name}' is not served by this node. "
                f"Available groups: {list(self._group_contexts)} "
                "(see the 'planning_groups' parameter)."
            )
        return ctx, ""

    def _build_group_context(self, group_name):
        """
        Builds and caches the planning utilities for a joint group, validating
        the group against the scene. Raises RuntimeError for unknown groups.
        """
        with self._planning_lock:
            try:
                group_info = self._scene.getJointGroupInfo(group_name)
            except Exception as e:
                raise RuntimeError(f"Unknown joint group '{group_name}': {e}")

            # Collect per-DOF position limits, used to nudge configurations
            # off the exact joint limits. Multi-DOF joints (e.g. continuous
            # joints) do not map one-to-one onto the configuration vector, so
            # skip the nudge entirely for groups containing them.
            min_position, max_position = [], []
            for joint_name in group_info.joint_names:
                limits = self._scene.getJointInfo(joint_name).limits
                min_position.extend(limits.min_position)
                max_position.extend(limits.max_position)
            if len(min_position) != len(group_info.q_indices):
                self.get_logger().warning(f"Group '{group_name}' has multi-DOF joints; skipping joint limit nudges.")
                min_position = max_position = None

            rrt_options = RRTOptions()
            rrt_options.group_name = group_name
            rrt_options.max_connection_distance = 5.0
            rrt_options.collision_check_step_size = 0.05
            rrt_options.max_planning_time = 2.0
            rrt_options.fast_return = False
            rrt_options.rrt_connect = True
            rrt_options.max_nodes = 5000
            rrt_options.goal_biasing_probability = 0.1
            rrt_options.collision_check_use_bisection = True

            shortcutting_options = PathShortcuttingOptions(
                group_name=group_name,
                max_step_size=rrt_options.collision_check_step_size,
                max_iters=self._max_shortcutting_iters,
            )

            # Constrained planning walks the trees in small projected hops, so
            # it wants a very different tuning: many more nodes, short connections,
            # RRT* rewiring to collapse the zigzag the hops leave behind, and no
            # fast return so the whole time budget is spent optimizing.
            constraint_projection = ConstraintProjectorOptions(path_step_size=0.1)
            constrained_rrt_options = RRTOptions(
                group_name=group_name,
                max_nodes=10000,
                max_connection_distance=1.0,
                collision_check_step_size=rrt_options.collision_check_step_size,
                collision_check_use_bisection=True,
                max_planning_time=3.0,
                rrt_connect=True,
                rrt_star=True,
                rewire_distance=3.0,
                fast_return=False,
                constraint_projection=constraint_projection,
            )

            # The top-down constraint keeps the tip link's approach (z) axis
            # near straight down in the world: the region frame's half turn
            # about x points its z axis down, and only roll and pitch relative
            # to it are bounded. Position and yaw are left unconstrained.
            tilt_bound = np.deg2rad(self.get_parameter("top_down_tilt_bound_degrees").value)
            region_tform = np.eye(4)
            region_tform[:3, :3] = np.diag([1.0, -1.0, -1.0])
            top_down_constraint = PoseConstraint(
                self._scene,
                group_name,
                self._tip_link,
                lower_bounds=np.array([-np.inf, -np.inf, -np.inf, -tilt_bound, -tilt_bound, -np.pi]),
                upper_bounds=np.array([np.inf, np.inf, np.inf, tilt_bound, tilt_bound, np.pi]),
                tform=region_tform,
            )

            ctx = GroupPlanningContext(
                name=group_name,
                joint_names=list(group_info.joint_names),
                q_indices=group_info.q_indices,
                ik_solver=SimpleIk(self._scene, self._make_ik_options(group_name)),
                rrt=RRT(self._scene, rrt_options),
                rrt_options=rrt_options,
                constrained_rrt_options=constrained_rrt_options,
                top_down_constraint=top_down_constraint,
                constraint_projector=ConstraintProjector(
                    self._scene, group_name, [top_down_constraint], constraint_projection
                ),
                shortcutter=PathShortcutter(self._scene, shortcutting_options),
                toppra=PathParameterizerTOPPRA(self._scene, group_name),
                visualizer=RoboplanVisualizer(
                    scene=self._scene,
                    group_name=group_name,
                    urdf_xml=self._urdf_xml,
                    frame_id="world",
                    ns="roboplan_traj",
                    color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3),
                ),
                min_position=None if min_position is None else np.array(min_position),
                max_position=None if max_position is None else np.array(max_position),
            )
            ctx.player = TrajectoryPublisher(
                self._scene,
                ctx.visualizer,
                self._traj_marker_pub,
                ctx.q_indices,
            )
            self._group_contexts[group_name] = ctx
            self.get_logger().info(f"Created planning context for group '{group_name}'.")
            return ctx

    def _nudge_within_limits(self, ctx, q_full):
        """
        Returns a copy of the configuration with the group's joints pushed a
        small margin inside their position limits. Joints resting exactly on a
        limit otherwise trip the planners' strict limit checks on numerical
        drift alone.
        """
        if ctx.min_position is None:
            return q_full

        margin = np.minimum(
            self.get_parameter("joint_limit_margin").value,
            0.5 * (ctx.max_position - ctx.min_position),
        )
        q_nudged = q_full.copy()
        q_nudged[ctx.q_indices] = np.clip(
            q_full[ctx.q_indices],
            ctx.min_position + margin,
            ctx.max_position - margin,
        )
        return q_nudged

    def _on_ik_feedback(self, feedback):
        q = self._ik_marker.process_feedback(feedback)
        if q is not None:
            # Warm-start the next solve from this solution so consecutive drag
            # updates stay on the same IK branch. On failure the seed is held,
            # so a bad solve never snaps to a random branch.
            self._ik_marker.set_seed_configuration(q)
            self._target_q = q
            # The marker pose is reported in the marker's frame (the base link).
            marker_pose = PoseStamped()
            marker_pose.header.frame_id = feedback.header.frame_id or self._base_link
            marker_pose.pose = feedback.pose
            self._target_marker_pose = marker_pose
            self._ik_marker_pub.publish(self._ik_visualizer.markers_from_configuration(q))

    def _sync_to_hardware(self):
        """Sets the scene to the latest joint state and returns the full configuration."""
        joint_config = fromJointState(self._js_subscriber.last_joint_state, self._scene, self._conversion_map)

        # MuJoCo, in particular, can push joints an epsilon past their limits, so this
        # is a little hacky but prevents planning failures due to constraint violations.
        self._latest_joint_positions = self._scene.clampToValidConfiguration(joint_config.positions)

        self._scene.setJointPositions(self._latest_joint_positions)
        return self._latest_joint_positions

    def _pose_stamped_to_base_frame(self, pose_stamped, q_full):
        """
        Converts a PoseStamped into an SE3 target in the base link frame.

        An empty frame_id is interpreted as the base link. Raises RuntimeError
        if the frame is not known to the scene.
        """
        tform = poseToSE3(pose_stamped.pose)
        frame = pose_stamped.header.frame_id
        if frame in ("", self._base_link):
            return tform
        base_T_frame = self._scene.forwardKinematics(q_full, frame, self._base_link)
        return base_T_frame @ tform

    def _solve_ik(self, ctx, base_T_target, q_seed_full):
        """
        Solves IK for the tip link from the seed configuration, returning a
        full configuration or None. A failed solve is reported as-is; callers
        (e.g. behavior trees) are expected to handle retries.
        """
        goal = CartesianConfiguration()
        goal.base_frame = self._base_link
        goal.tip_frame = self._tip_link
        goal.tform = base_T_target
        seed = JointConfiguration()
        seed.positions = q_seed_full[ctx.q_indices]
        solution = JointConfiguration()
        if ctx.ik_solver.solveIk(goal, seed, solution):
            return self._scene.toFullJointPositions(ctx.name, np.array(solution.positions))
        return None

    def _set_planned_trajectory(self, ctx, traj):
        """Stores the last planned trajectory and publishes its end-effector path."""
        self._planned_traj = traj
        self._planned_ctx = ctx
        self._planned_path_pub.publish(
            markerFromJointTrajectory(
                self._scene,
                traj,
                [self._tip_link],
                frame_id="world",
                ns="planned_trajectory",
                color=self._planned_path_color,
            )
        )

    def _to_ros_trajectory(self, traj):
        """
        Converts a roboplan trajectory to a ROS JointTrajectory, zeroing any
        residual velocities/accelerations on the final point. The Cartesian
        planner can leave ~1e-3 residuals there, which the joint trajectory
        controller rejects as a non-stopping trajectory.
        """
        ros_traj = toJointTrajectory(traj)
        if ros_traj.points:
            last = ros_traj.points[-1]
            last.velocities = [0.0] * len(last.velocities)
            last.accelerations = [0.0] * len(last.accelerations)
        return ros_traj

    def _project_to_top_down(self, ctx, q_full):
        """
        Returns the configuration projected onto the group's top-down gripper
        constraint, or None if the projection did not converge. Configurations
        already satisfying the constraint are returned unchanged.
        """
        if ctx.constraint_projector.satisfies(q_full):
            return q_full
        return ctx.constraint_projector.project(q_full)

    def _plan_to_configuration(
        self, ctx, q_target_full, velocity_scaling=0.0, acceleration_scaling=0.0, constrain_gripper_top_down=False
    ):
        """
        Plans from the current hardware state to a target full configuration
        using RRT, shortcutting, and TOPP-RA time parameterization.

        With the top-down gripper constraint enabled, the planner switches to
        the constrained option set and shortcutting is skipped, since a
        straight configuration-space shortcut leaves the constraint manifold.
        """
        with self._planning_lock:
            q_start_full = self._sync_to_hardware()

            constraints = []
            include_shortcutting = self._include_shortcutting
            if constrain_gripper_top_down:
                constraints = [ctx.top_down_constraint]
                include_shortcutting = False

                # The planner roots its trees at the start and goal, so both
                # must sit on the constraint. IK converges to its own tolerance
                # and hardware states carry noise, so project them first.
                q_start_full = self._project_to_top_down(ctx, q_start_full)
                if q_start_full is None:
                    return False, "The current configuration could not be projected onto the top-down constraint."
                q_target_full = self._project_to_top_down(ctx, q_target_full)
                if q_target_full is None:
                    return False, "The goal configuration could not be projected onto the top-down constraint."

            # Reconfiguring the planner is cheap, so just set the option set
            # matching the request every time.
            ctx.rrt.setOptions(ctx.constrained_rrt_options if constrain_gripper_top_down else ctx.rrt_options)

            start = JointConfiguration()
            start.positions = q_start_full[ctx.q_indices]

            goal = JointConfiguration()
            goal.positions = q_target_full[ctx.q_indices]

            constraint_note = " with the top-down gripper constraint" if constrain_gripper_top_down else ""
            self.get_logger().info(f"Planning for group '{ctx.name}'{constraint_note}...")
            plan_start_time = time.time()

            # A failed plan is reported as-is; callers (e.g. behavior trees)
            # are expected to handle retries.
            try:
                start_time = time.time()
                path = ctx.rrt.plan(start, goal, constraints)
                self.get_logger().info(f"  Finished planning in {time.time() - start_time} seconds.")
            except RuntimeError as e:
                return False, f"Planning failed: {e}"

            try:
                if include_shortcutting:
                    self.get_logger().info("Shortcutting...")
                    start_time = time.time()
                    path = ctx.shortcutter.shortcut(path)
                    self.get_logger().info(f"  Finished shortcutting in {time.time() - start_time} seconds.")

                self.get_logger().info("Generating trajectory...")
                start_time = time.time()
                toppra_options = TOPPRAOptions(
                    self._traj_dt,
                    mode=SplineFittingMode.Adaptive,
                    max_adaptive_iterations=5,
                )
                if velocity_scaling > 0.0:
                    toppra_options.velocity_scale = min(velocity_scaling, 1.0)
                if acceleration_scaling > 0.0:
                    toppra_options.acceleration_scale = min(acceleration_scaling, 1.0)
                traj = ctx.toppra.generate(path, toppra_options)
                self.get_logger().info(f"  Finished generating trajectory in {time.time() - start_time} seconds.")
            except RuntimeError as e:
                return False, f"Trajectory generation failed: {e}"

            self.get_logger().info(f"Total planning time: {time.time() - plan_start_time} seconds.")

            self._set_planned_trajectory(ctx, traj)
            return True, f"Planned trajectory with {len(traj.positions)} points"

    def _plan_to_joint_state(
        self, group_name, joint_names, joint_positions, velocity_scaling=0.0, acceleration_scaling=0.0
    ):
        """Plans to a joint configuration given by (possibly partial) name/position pairs."""
        ctx, error = self._get_group_context(group_name)
        if ctx is None:
            return False, error

        if len(joint_names) != len(joint_positions):
            return False, "Joint names and joint positions must have the same length."
        if len(joint_names) == 0:
            return False, "No target joints specified."

        unknown = [name for name in joint_names if name not in ctx.joint_names]
        if unknown:
            return (
                False,
                f"Joints {unknown} are not in group '{ctx.name}' (joints: {ctx.joint_names}).",
            )

        with self._planning_lock:
            # Start from the current positions so unspecified joints hold still.
            q_start_full = self._sync_to_hardware()
            group_positions = q_start_full[ctx.q_indices].copy()
            for name, position in zip(joint_names, joint_positions):
                group_positions[ctx.joint_names.index(name)] = position

            q_target_full = self._scene.toFullJointPositions(ctx.name, group_positions)
            return self._plan_to_configuration(ctx, q_target_full, velocity_scaling, acceleration_scaling)

    def _plan_to_pose(
        self,
        group_name,
        pose_stamped,
        velocity_scaling=0.0,
        acceleration_scaling=0.0,
        constrain_gripper_top_down=False,
    ):
        """Plans a free-space motion to a target pose via IK."""
        ctx, error = self._get_group_context(group_name)
        if ctx is None:
            return False, error

        with self._planning_lock:
            q_start_full = self._sync_to_hardware()
            try:
                base_T_target = self._pose_stamped_to_base_frame(pose_stamped, q_start_full)
            except RuntimeError as e:
                return False, f"Could not resolve target pose frame '{pose_stamped.header.frame_id}': {e}"

            q_target_full = self._solve_ik(ctx, base_T_target, q_start_full)
            if q_target_full is None:
                position = base_T_target[:3, 3]
                return False, (
                    "IK failed to find a valid goal configuration for tip position "
                    f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) in frame '{self._base_link}'."
                )

            # IK solutions can rest exactly on a joint limit, which makes any
            # follow-up Cartesian motion prone to spurious limit violations.
            q_target_full = self._nudge_within_limits(ctx, q_target_full)

            return self._plan_to_configuration(
                ctx, q_target_full, velocity_scaling, acceleration_scaling, constrain_gripper_top_down
            )

    def _plan_cartesian(self, group_name, target_poses, max_linear_speed=0.0, max_angular_speed=0.0):
        """Plans a straight-line Cartesian motion through one or more target poses."""
        ctx, error = self._get_group_context(group_name)
        if ctx is None:
            return False, error

        # Either a list or a single pose stamped.
        if isinstance(target_poses, PoseStamped):
            target_poses = [target_poses]

        if len(target_poses) < 1:
            return False, "At least one target pose is required."

        with self._planning_lock:
            q_start_full = self._nudge_within_limits(ctx, self._sync_to_hardware())

            base_T_start = self._scene.forwardKinematics(q_start_full, self._tip_link, self._base_link)
            poses = [base_T_start]

            for i, pose_stamped in enumerate(target_poses):
                try:
                    poses.append(self._pose_stamped_to_base_frame(pose_stamped, q_start_full))
                except RuntimeError as e:
                    return False, f"Could not resolve frame for target pose {i}: {e}"

            path = CartesianPath(
                [self._base_link],
                [self._tip_link],
                [poses],
            )

            speed_mode_name = self.get_parameter("cartesian_speed_mode").value
            if speed_mode_name not in ("time_optimal", "bounded"):
                return False, f"Invalid cartesian_speed_mode '{speed_mode_name}'; use 'time_optimal' or 'bounded'."
            speed_mode = (
                CartesianSpeedMode.TimeOptimal if speed_mode_name == "time_optimal" else CartesianSpeedMode.Bounded
            )

            options = CartesianPlannerOptions(
                group_name=ctx.name,
                dt=self._traj_dt,
                speed_mode=speed_mode,
                max_linear_speed=(
                    max_linear_speed if max_linear_speed > 0.0 else self.get_parameter("max_linear_speed").value
                ),
                max_angular_speed=(
                    max_angular_speed if max_angular_speed > 0.0 else self.get_parameter("max_angular_speed").value
                ),
                max_linear_acceleration=self.get_parameter("max_linear_acceleration").value,
                max_angular_acceleration=self.get_parameter("max_angular_acceleration").value,
                max_position_error=self.get_parameter("max_position_error").value,
                max_orientation_error=self.get_parameter("max_orientation_error").value,
                orientation_cost=0.1,
            )
            planner = CartesianPathPlanner(self._scene, options)

            q_start = JointConfiguration()
            q_start.positions = q_start_full

            self.get_logger().info(
                f"Planning Cartesian path through {len(target_poses)} pose(s) for group '{ctx.name}'..."
            )
            start_time = time.time()
            try:
                traj = planner.plan(path, q_start)
            except RuntimeError as e:
                return False, f"Cartesian planning failed: {e}"
            self.get_logger().info(f"  Finished Cartesian planning in {time.time() - start_time:.3f} seconds.")

            self._set_planned_trajectory(ctx, traj)
            return True, (
                f"Planned Cartesian trajectory through {len(target_poses)} pose(s) " f"({len(traj.positions)} points)"
            )

    def _preview(self):
        if self._planned_traj is None:
            return False, "No trajectory to preview. Plan first."

        self.get_logger().info("Previewing trajectory...")
        self._planned_ctx.player.play(
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
        goal.trajectory = self._to_ros_trajectory(self._planned_traj)

        self.get_logger().info("Sending trajectory for execution...")
        future = self._execute_client.send_goal_async(goal)
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

    def _execute_result(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory execution complete.")
        else:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.error_code}")

    def _reset(self):
        """Clears all plans and resets to a hardware state."""
        if self._js_subscriber.last_joint_state is None:
            raise RuntimeError("No joint states received, cannot reset to hw state.")

        with self._planning_lock:
            # Reset joint positions to the latest joint state
            self._sync_to_hardware()

            # Update the IK marker's seed to the current state
            self._ik_marker.set_seed_configuration(self._latest_joint_positions)

            # Compute FK for the current state to get the marker pose
            fk = self._scene.forwardKinematics(self._latest_joint_positions, self._tip_link, self._base_link)
            pose = se3ToPose(fk)

            # Update the IK to the current pose
            self._ik_server.setPose("ik_target", pose)
            self._ik_server.applyChanges()
            self._ik_marker_pub.publish(self._ik_visualizer.markers_from_configuration(self._latest_joint_positions))

            # Clear the planned trajectory and target
            self._target_q = None
            self._target_marker_pose = None
            self._planned_traj = None
            self._planned_ctx = None
            self._traj_marker_pub.publish(self._default_ctx.visualizer.clear_markers())
            delete_marker = Marker()
            delete_marker.header.frame_id = "world"
            delete_marker.action = Marker.DELETEALL
            self._planned_path_pub.publish(delete_marker)

    def _plan_to_marker_target(self):
        """Free-space plan to the current interactive marker target."""
        if self._target_q is None:
            return False, "No target set. Move the interactive marker first."
        return self._plan_to_configuration(self._default_ctx, self._target_q)

    def _plan_cartesian_to_marker_target(self):
        """Cartesian plan to the current interactive marker pose."""
        if self._target_marker_pose is None:
            return False, "No target set. Move the interactive marker first."
        return self._plan_cartesian(self._joint_group, [self._target_marker_pose])

    # Menu callbacks
    def _on_plan_menu(self, feedback):
        _, msg = self._plan_to_marker_target()
        self.get_logger().info(msg)

    def _on_plan_cartesian_menu(self, feedback):
        _, msg = self._plan_cartesian_to_marker_target()
        self.get_logger().info(msg)

    def _on_preview_menu(self, feedback):
        _, msg = self._preview()
        self.get_logger().info(msg)

    def _on_execute_menu(self, feedback):
        _, msg = self._execute()
        self.get_logger().info(msg)

    def _on_reset_menu(self, feedback):
        self._reset()
        self.get_logger().info("Reset node to current state.")

    # Planning service callbacks. Hold the (reentrant) planning lock across
    # both planning and conversion so a concurrent plan request cannot swap
    # out the stored trajectory in between.
    def _on_plan_to_joint_state(self, request, response):
        with self._planning_lock:
            response.success, response.message = self._plan_to_joint_state(
                request.group_name,
                request.joint_names,
                request.joint_positions,
                request.velocity_scaling,
                request.acceleration_scaling,
            )
            if response.success:
                response.trajectory = self._to_ros_trajectory(self._planned_traj)
            else:
                self.get_logger().error(response.message)
        return response

    def _on_plan_to_pose(self, request, response):
        with self._planning_lock:
            response.success, response.message = self._plan_to_pose(
                request.group_name,
                request.target_pose,
                request.velocity_scaling,
                request.acceleration_scaling,
                request.constrain_gripper_top_down,
            )
            if response.success:
                response.trajectory = self._to_ros_trajectory(self._planned_traj)
            else:
                self.get_logger().error(response.message)
        return response

    def _on_plan_cartesian_path(self, request, response):
        with self._planning_lock:
            response.success, response.message = self._plan_cartesian(
                request.group_name,
                list(request.target_poses),
                request.max_linear_speed,
                request.max_angular_speed,
            )
            if response.success:
                response.trajectory = self._to_ros_trajectory(self._planned_traj)
            else:
                self.get_logger().error(response.message)
        return response

    def _on_set_collisions(self, request, response):
        """Toggle collision checking for a list of body pairs."""
        if len(request.body1) != len(request.body2):
            response.success = False
            response.message = (
                f"body1 and body2 must have the same length " f"(got {len(request.body1)} and {len(request.body2)})."
            )
            self.get_logger().error(response.message)
            return response

        if len(request.body1) == 0:
            response.success = False
            response.message = "No body pairs specified."
            self.get_logger().error(response.message)
            return response

        action = "Enabling" if request.enable else "Disabling"
        with self._planning_lock:
            failed = []
            for b1, b2 in zip(request.body1, request.body2):
                try:
                    self._scene.setCollisions(b1, b2, request.enable)
                    self.get_logger().info(f"{action} collision checking: '{b1}' <-> '{b2}'")
                except Exception as e:
                    failed.append(f"'{b1}' <-> '{b2}': {e}")

            if failed:
                response.success = False
                response.message = (
                    f"Failed to set collisions for {len(failed)}/{len(request.body1)} " f"pair(s): {'; '.join(failed)}"
                )
                self.get_logger().error(response.message)
            else:
                action_past = "Enabled" if request.enable else "Disabled"
                pairs = ", ".join(f"'{b1}'<->'{b2}'" for b1, b2 in zip(request.body1, request.body2))
                response.success = True
                response.message = f"{action_past} collision checking for {len(request.body1)} pair(s): {pairs}"
                self.get_logger().info(response.message)

        return response

    # Trigger service callbacks
    def _on_plan(self, request, response):
        response.success, response.message = self._plan_to_marker_target()
        return response

    def _on_plan_cartesian(self, request, response):
        response.success, response.message = self._plan_cartesian_to_marker_target()
        return response

    def _on_preview(self, request, response):
        response.success, response.message = self._preview()
        return response

    def _on_execute(self, request, response):
        response.success, response.message = self._execute()
        return response

    def _on_reset(self, request, response):
        self._reset()
        response.success = True
        response.message = "Reset node to current state."
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        for ctx in self._group_contexts.values():
            if ctx.player is not None:
                ctx.player.stop()
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
    run_node(RoboplanPlanningServer())
