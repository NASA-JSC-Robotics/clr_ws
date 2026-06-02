#!/usr/bin/env python3

import os
import time
import tyro
import xacro

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from roboplan.core import Scene, JointConfiguration, CartesianConfiguration
from roboplan.simple_ik import SimpleIkOptions, SimpleIk


def main(
    max_iters: int = 100,
    step_size: float = 0.25,
    max_linear_error_norm: float = 0.001,
    max_angular_error_norm: float = 0.001,
    check_collisions: bool = True,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Run the IK example with the provided parameters.

    Parameters:
        max_iters: Maximum number of iterations for the IK solver.
        step_size: Integration step size for the IK solver.
        max_linear_error_norm: The maximum linear error norm for the IK solver.
        max_angular_error_norm: The maximum angular error norm for the IK solver.
        check_collisions: Whether to check for collisions when solving IK.
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

    # Specify argument names to distinguish overloaded Scene constructors from python.
    scene = Scene(
        "test_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=yaml_config_path,
    )

    group_name = "clr"
    base_link = "vention_rail_base_link"
    tip_link = "tool0"
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

    # Set up an IK solver
    options = SimpleIkOptions(
        group_name=group_name,
        max_iters=max_iters,
        step_size=step_size,
        max_linear_error_norm=max_linear_error_norm,
        max_angular_error_norm=max_angular_error_norm,
        check_collisions=check_collisions,
    )
    ik_solver = SimpleIk(scene, options)

    starting_joint_config = np.array(
        [
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
            2.02910551e-11,
            -2.66777419e-04,
            1.56383145e00,
            -4.45405786e-03,
            -4.97470896e-04,
            -4.93714552e-06,
            3.21778639e-02,
            -6.74079130e-04,
            0.00000000e00,
            0.00000000e00,
            0.00000000e00,
            0.00000000e00,
            0.00000000e00,
            0.00000000e00,
            0.00000000e00,
            0.00000000e00,
            0.00000000e00,
            -6.73367315e-38,
            1.29458704e-14,
        ]
    )

    start = JointConfiguration()
    start.positions = np.array(starting_joint_config)[q_indices]

    goals = []
    transform_controls = []
    for ee_name in [tip_link]:
        goal = CartesianConfiguration()
        goal.base_frame = base_link
        goal.tip_frame = ee_name
        goals.append(goal)

    solution = JointConfiguration()

    # Create interactive markers.
    def solveIk(_):
        for goal, controls in zip(goals, transform_controls):
            goal.tform = pin.SE3(pin.Quaternion(controls.wxyz[[1, 2, 3, 0]]), controls.position).homogeneous
        result = ik_solver.solveIk(goals, start, solution)
        if result:
            q_full = scene.toFullJointPositions(group_name, solution.positions)
            viz.display(q_full)
            scene.setJointPositions(q_full)
            start.positions = solution.positions

    for ee_name in [tip_link]:
        controls = viz.viewer.scene.add_transform_controls(
            f"/ik_markers/{ee_name}",
            depth_test=False,
            scale=0.2,
            disable_sliders=True,
            visible=True,
        )
        controls.on_update(solveIk)
        transform_controls.append(controls)

    # Create a marker reset button.
    reset_button = viz.viewer.gui.add_button("Reset Marker")

    @reset_button.on_click
    def reset_position(_):
        for goal, controls in zip(goals, transform_controls):
            fk_tform = scene.forwardKinematics(scene.getCurrentJointPositions(), goal.tip_frame)
            controls.position = fk_tform[:3, 3]
            controls.wxyz = pin.Quaternion(fk_tform[:3, :3]).coeffs()[[3, 0, 1, 2]]

    random_button = viz.viewer.gui.add_button("Randomize Pose")

    @random_button.on_click
    def randomize_position(pos):
        q_rand = scene.randomCollisionFreePositions()[q_indices]
        q_full = scene.toFullJointPositions(group_name, q_rand)
        scene.setJointPositions(q_full)
        viz.display(q_full)
        start.positions = q_rand
        reset_position(pos)

    # Display the arm and marker at the starting position, then sleep forever.
    randomize_position(None)
    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
