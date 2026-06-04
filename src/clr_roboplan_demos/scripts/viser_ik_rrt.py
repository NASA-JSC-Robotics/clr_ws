#!/usr/bin/env python3

import os
import queue
import time
import tyro
import xacro

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from ament_index_python.packages import get_package_share_directory
from roboplan.core import (
    CartesianConfiguration,
    JointConfiguration,
    PathShortcutter,
    PathShortcuttingOptions,
    Scene,
)
from roboplan.rrt import RRTOptions, RRT, visualizeTree
from roboplan.simple_ik import SimpleIkOptions, SimpleIk
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode
from roboplan.visualization import (
    visualizeJointTrajectory,
    visualizePath,
    plotJointTrajectory,
)


def main(
    # IK options
    max_iters: int = 100,
    step_size: float = 0.25,
    max_linear_error_norm: float = 0.001,
    max_angular_error_norm: float = 0.001,
    check_collisions: bool = True,
    # RRT options
    max_connection_distance: float = 3.0,
    collision_check_step_size: float = 0.05,
    collision_check_use_bisection: bool = True,
    goal_biasing_probability: float = 0.05,
    max_nodes: int = 10000,
    max_planning_time: float = 5.0,
    rrt_connect: bool = True,
    include_shortcutting: bool = True,
    max_shortcutting_iters: int = 100,
    toppra_mode: SplineFittingMode = SplineFittingMode.Adaptive,
    rng_seed: int | None = None,
    # Visualization options
    host: str = "localhost",
    port: str = "8000",
):
    """
    Run the combined IK + RRT example for CLR.

    Move the interactive IK marker to a desired end-effector pose, then click
    "Set Start Pose" or "Set Goal Pose" to capture the current joint configuration.
    Once both are set, click "Plan path" to run RRT between them, then
    "Animate trajectory" to play back the resulting motion.

    Parameters:
        max_iters: Maximum number of iterations for the IK solver.
        step_size: Integration step size for the IK solver.
        max_linear_error_norm: The maximum linear error norm for the IK solver.
        max_angular_error_norm: The maximum angular error norm for the IK solver.
        check_collisions: Whether to check for collisions when solving IK.
        max_connection_distance: Maximum connection distance between two search nodes.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        collision_check_use_bisection: If true, uses bisection instead of linear search for collision checking along edges.
            This can be helpful in collision-dense environments, but has a lower worst-case performance.
        goal_biasing_probability: Weighting of the goal node during random sampling.
        max_nodes: The maximum number of nodes to add to the search tree.
        max_planning_time: The maximum time (in seconds) to search for a path.
        rrt_connect: Whether or not to use RRT-Connect.
        include_shortcutting: Whether or not to include path shortcutting for found paths.
        max_shortcutting_iters: The maximum number of path shortcutting iterations.
        toppra_mode: The trajectory generation mode for TOPP-RA. Can be `Hermite`, `Cubic`, or `Adaptive` (default).
        rng_seed: The seed for solving RRT.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """
    model_xacro_path = os.path.join(
        get_package_share_directory("clr_mujoco_config"),
        "urdf",
        "clr_mujoco_xacro.urdf",
    )
    urdf_xml = xacro.process_file(model_xacro_path).toxml()

    srdf_xacro_path = os.path.join(
        get_package_share_directory("clr_moveit_config"),
        "srdf",
        "clr_and_sim_mockups.srdf.xacro",
    )
    srdf_xml = xacro.process_file(srdf_xacro_path).toxml()

    yaml_config_path = os.path.join(
        get_package_share_directory("clr_roboplan_demos"),
        "config",
        "clr_roboplan_config.yaml",
    )
    package_paths = [
        get_package_share_directory("clr_mujoco_config"),
    ]
    scene = Scene(
        "test_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=yaml_config_path,
    )

    group_name = "clr"
    tip_link = "grasp_frame"
    q_indices = scene.getJointGroupInfo(group_name).q_indices

    # Create a redundant Pinocchio model just for visualization with mimic joints.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths)

    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Hide the room ceiling so it does not occlude the scene.
    ceiling = viz.viewer.scene.get_handle_by_name("/pinocchio/visual/room_ceiling_0")
    if ceiling is not None:
        ceiling.visible = False

    # Set up an IK solver.
    ik_options = SimpleIkOptions(
        group_name=group_name,
        max_iters=max_iters,
        step_size=step_size,
        max_linear_error_norm=max_linear_error_norm,
        max_angular_error_norm=max_angular_error_norm,
        check_collisions=check_collisions,
    )
    ik_solver = SimpleIk(scene, ik_options)

    # Set up an RRT planner, shortcutter, and time parameterization.
    rrt_options = RRTOptions(
        group_name=group_name,
        max_nodes=max_nodes,
        max_connection_distance=max_connection_distance,
        collision_check_step_size=collision_check_step_size,
        collision_check_use_bisection=collision_check_use_bisection,
        goal_biasing_probability=goal_biasing_probability,
        max_planning_time=max_planning_time,
        rrt_connect=rrt_connect,
    )
    rrt = RRT(scene, rrt_options)
    if rng_seed:
        rrt.setRngSeed(rng_seed)

    toppra = PathParameterizerTOPPRA(scene, group_name)
    traj_dt = 0.01

    if include_shortcutting:
        shortcutting_options = PathShortcuttingOptions(
            group_name=group_name,
            max_step_size=collision_check_step_size,
            max_iters=max_shortcutting_iters,
        )
        shortcutter = PathShortcutter(scene, shortcutting_options)

    # Initialize the robot at a fixed configuration.
    q_full = np.zeros_like(scene.randomPositions())
    q_full[:12] = [
        8.55225251e-19,
        9.97381986e-03,
        3.19268333e00,
        -2.39129980e00,
        -1.77600054e00,
        -4.69997566e-01,
        -1.61764763e00,
        -1.61770000e00,
        2.43603275e-02,
        -1.00354533e-02,
        2.16578104e-02,
        1.07625658e-03,
    ]
    # The latest full configuration shown in the visualizer. Keep this updated
    # everywhere we display so the marker can always snap back to what is shown.
    current_q_full = np.array(q_full)

    def display_full(q):
        nonlocal current_q_full
        current_q_full = np.array(q)
        scene.setJointPositions(current_q_full)
        viz.display(current_q_full)

    display_full(q_full)

    # The IK seed and the most recent valid IK solution for the joint group.
    ik_seed = JointConfiguration()
    ik_seed.positions = np.array(q_full)[q_indices]
    current_q_group = np.array(ik_seed.positions)

    # The captured RRT start and goal configurations.
    start_config = JointConfiguration()
    goal_config = JointConfiguration()

    # Trajectory animation state.
    traj_queue = queue.Queue()
    cur_traj = None
    animate = False

    # Set up the IK goal and interactive transform control marker.
    goal = CartesianConfiguration()
    goal.base_frame = ""
    goal.tip_frame = tip_link

    solution = JointConfiguration()

    def solveIk(_):
        nonlocal current_q_group
        goal.tform = pin.SE3(pin.Quaternion(controls.wxyz[[1, 2, 3, 0]]), controls.position).homogeneous
        result = ik_solver.solveIk([goal], ik_seed, solution)
        if result:
            display_full(scene.toFullJointPositions(group_name, solution.positions))
            ik_seed.positions = solution.positions
            current_q_group = np.array(solution.positions)

    controls = viz.viewer.scene.add_transform_controls(
        f"/ik_markers/{tip_link}",
        depth_test=False,
        scale=0.2,
        disable_sliders=True,
        visible=True,
    )
    controls.on_update(solveIk)

    def snap_marker_to_robot():
        fk_tform = scene.forwardKinematics(current_q_full, tip_link)
        controls.position = fk_tform[:3, 3]
        controls.wxyz = pin.Quaternion(fk_tform[:3, :3]).coeffs()[[3, 0, 1, 2]]

    def move_marker_to_group_config(q_group):
        """Drive the robot and marker to a captured joint-group configuration."""
        nonlocal current_q_group
        display_full(scene.toFullJointPositions(group_name, q_group))
        ik_seed.positions = np.array(q_group)
        current_q_group = np.array(q_group)
        snap_marker_to_robot()

    # Marker reset button: snap the marker back onto the current robot pose.
    reset_button = viz.viewer.gui.add_button("Reset Marker")

    @reset_button.on_click
    def reset_position(_):
        snap_marker_to_robot()

    def update_plan_button():
        plan_button.disabled = start_config.positions is None or goal_config.positions is None

    # Button to capture the current configuration as the RRT start pose.
    set_start_button = viz.viewer.gui.add_button("Set Start Pose")

    @set_start_button.on_click
    def set_start_pose(_):
        start_config.positions = np.array(current_q_group)
        q_start_full = scene.toFullJointPositions(group_name, start_config.positions)
        viz.viewer.scene.add_icosphere(
            f"/rrt/start/{tip_link}",
            radius=0.03,
            color=(0, 200, 0),
            position=scene.forwardKinematics(q_start_full, tip_link)[:3, 3],
        )
        print("Set start pose.")
        move_to_start_button.disabled = False
        update_plan_button()

    # Button to drive the marker and robot to the captured start pose.
    move_to_start_button = viz.viewer.gui.add_button("Move Marker to Start Pose")
    move_to_start_button.disabled = True

    @move_to_start_button.on_click
    def move_to_start_pose(_):
        if start_config.positions is not None:
            move_marker_to_group_config(start_config.positions)

    # Button to capture the current configuration as the RRT goal pose.
    set_goal_button = viz.viewer.gui.add_button("Set Goal Pose")

    @set_goal_button.on_click
    def set_goal_pose(_):
        goal_config.positions = np.array(current_q_group)
        q_goal_full = scene.toFullJointPositions(group_name, goal_config.positions)
        viz.viewer.scene.add_icosphere(
            f"/rrt/goal/{tip_link}",
            radius=0.03,
            color=(200, 0, 0),
            position=scene.forwardKinematics(q_goal_full, tip_link)[:3, 3],
        )
        print("Set goal pose.")
        move_to_goal_button.disabled = False
        update_plan_button()

    # Button to drive the marker and robot to the captured goal pose.
    move_to_goal_button = viz.viewer.gui.add_button("Move Marker to Goal Pose")
    move_to_goal_button.disabled = True

    @move_to_goal_button.on_click
    def move_to_goal_pose(_):
        if goal_config.positions is not None:
            move_marker_to_group_config(goal_config.positions)

    # Path planning button (disabled until both start and goal are set).
    plan_button = viz.viewer.gui.add_button("Plan path")
    plan_button.disabled = True

    @plan_button.on_click
    def plan_path(_):
        nonlocal animate
        animate = False
        plan_button.disabled = True
        animate_button.disabled = True

        assert start_config.positions is not None
        assert goal_config.positions is not None

        # Clear any visualization from a previous plan (the start/goal markers
        # live under separate node names, so they are left untouched).
        for node_name in ("/rrt/start_tree", "/rrt/goal_tree", "/rrt/path", "/rrt/shortcut_path"):
            viz.viewer.scene.remove_by_name(node_name)

        print("\nPlanning...")
        t_start = time.time()
        try:
            path = rrt.plan(start_config, goal_config)
        finally:
            plan_button.disabled = False
        print(f"Found a path in {time.time() - t_start:.3f} s")

        # Optionally include path shortening.
        if include_shortcutting:
            print("Shortcutting path...")
            t_start = time.time()
            shortened_path = shortcutter.shortcut(path)
            print(f"Shortcutted path in {time.time() - t_start:.3f} s")

        # Set up TOPP-RA to time-parameterize the path.
        print("Generating trajectory...")
        t_start = time.time()
        traj = toppra.generate(shortened_path if include_shortcutting else path, traj_dt, toppra_mode)
        print(f"Generated trajectory in {time.time() - t_start:.3f} s")

        # Visualize the tree and path. Restore the robot to the start pose.
        display_full(scene.toFullJointPositions(group_name, start_config.positions))
        visualizeTree(viz, scene, rrt, [tip_link], 0.05)

        if include_shortcutting:
            visualizePath(viz, scene, path, [tip_link], 0.05, (100, 0, 0), "/rrt/path")
            visualizeJointTrajectory(
                viz,
                scene,
                traj,
                [tip_link],
                (0, 100, 0),
                "/rrt/shortcut_path",
            )
        else:
            visualizeJointTrajectory(viz, scene, traj, [tip_link], (100, 0, 0), "/rrt/path")

        traj_queue.put(traj)
        animate_button.disabled = False

    # Trajectory animation button.
    animate_button = viz.viewer.gui.add_button("Animate trajectory")
    animate_button.disabled = True

    @animate_button.on_click
    def animate_trajectory(_):
        nonlocal animate
        plan_button.disabled = True
        animate_button.disabled = True
        animate = True

    # Place the marker on the robot's starting pose.
    snap_marker_to_robot()

    # Main display and animation loop.
    plt.figure()
    plt.ion()
    while True:
        if not traj_queue.empty():
            plt.clf()
            cur_traj = traj_queue.get()
            fig = plotJointTrajectory(cur_traj, scene)
            plt.draw()
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.1)
        elif animate and cur_traj is not None:
            print("Animating trajectory...")
            for q in cur_traj.positions:
                display_full(scene.toFullJointPositions(group_name, q))
                time.sleep(traj_dt)
            animate = False
            plan_button.disabled = False
            animate_button.disabled = False
            print("...done!")
        else:
            time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
