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
Small helper functions for CLR's roboplan demos.
"""

import os
from dataclasses import dataclass

import xacro

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from roboplan.core import Scene


BEST_EFFORT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
)


@dataclass
class RobotConfig:
    """Description of a robot configuration within the CLR cell."""

    name: str
    joint_group: str
    base_link: str
    tip_link: str
    supports_streaming: bool = False
    controller_joint_trajectory_topic: str = ""
    controller_action: str = ""


# Available robot configurations
ROBOT_CONFIGS = {
    "clr": RobotConfig(
        name="clr",
        joint_group="clr",
        base_link="vention_rail_base_link",
        tip_link="grasp_frame",
        supports_streaming=False,
        controller_joint_trajectory_topic="/clr_joint_trajectory_controller/joint_trajectory",
        controller_action="/clr_joint_trajectory_controller/follow_joint_trajectory",
    ),
    "chonkur": RobotConfig(
        name="chonkur",
        joint_group="chonkur_grasp",
        base_link="base_link",
        tip_link="grasp_frame",
        supports_streaming=True,
        controller_joint_trajectory_topic="/joint_trajectory_controller/joint_trajectory",
        controller_action="/joint_trajectory_controller/follow_joint_trajectory",
    ),
}


def get_robot_config(name: str) -> RobotConfig:
    """Look up a robot configuration by name, raising on unknown names."""
    if name not in ROBOT_CONFIGS:
        available = ", ".join(ROBOT_CONFIGS.keys())
        raise ValueError(f"Unknown robot config '{name}'. Available: {available}")
    return ROBOT_CONFIGS[name]


def run_node(node):
    """Spins a node using a MultiThreadedExecutor and handles exceptions shutdowns."""
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


def spin_executor(executor, logger=None):
    """Spin an executor until shutdown, swallowing expected exceptions."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if logger:
            logger.warning(f"Executor caught exception, shutting down: {e}")


def create_scene():
    """Build and return (scene, urdf_xml, package_paths) for the CLR cell."""
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
    package_paths = [get_package_share_directory("clr_mujoco_config")]

    scene = Scene(
        name="clr_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=yaml_config_path,
    )
    return scene, urdf_xml, package_paths

