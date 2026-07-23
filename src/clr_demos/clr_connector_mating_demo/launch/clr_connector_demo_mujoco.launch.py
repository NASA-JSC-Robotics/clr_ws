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

from launch import LaunchDescription
from chonkur_deploy.launch_helpers import include_launch_file
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import (
    FindPackageShare,
)
from launch.conditions import UnlessCondition


def generate_launch_description():

    declared_arguments = []

    # this launch arg doesn't do anything right now because I don't want to enable passing
    # it through to the main control.launch.py... Same would go for something like headless
    # mode or for trying to increase running speed
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_pregenerated_mjcf",
            default_value="false",
            description="Use pre-generated mjcf instead of converting it on the fly.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_speed",
            default_value="1.0",
            description="Percentage speed to run the simulation at. 1.0 is 100 percent speed.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Optionally run headless, primarily for use in CI.",
        )
    )

    use_pregenerated_mjcf = LaunchConfiguration("use_pregenerated_mjcf")
    sim_speed = LaunchConfiguration("sim_speed")
    headless = LaunchConfiguration("headless")
    model_env = LaunchConfiguration("model_env")

    clr_mujoco_package_name = "clr_mujoco_config"

    generate_mjcf = include_launch_file(
        package_name="clr_mujoco_config",
        launch_file="generate_clr_mjcf.launch.py",
        launch_arguments={
            "save_only": "false",
            "use_pregenerated_assets_dir": "true",
        }.items(),
        condition=UnlessCondition(use_pregenerated_mjcf),
    )

    extra_xacro_args = [
        " use_pregenerated_mjcf:=",
        use_pregenerated_mjcf,
        " sim_speed:=",
        sim_speed,
        " headless:=",
        headless,
        " model_env:=",
        model_env,
    ]

    extra_controller_params_file = PathJoinSubstitution(
        [FindPackageShare(clr_mujoco_package_name), "config", "mujoco_plugins.yaml"]
    )

    clr_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "robot_description_package": "clr_connector_mating_demo",
            "robot_description_file": "clr_connector_demo_mujoco.xacro.urdf",
            "use_fake_hardware": "false",
            "use_sim_time": "true",
            "is_sim": "true",
            "control_node_package": "mujoco_ros2_control",
            "extra_xacro_args": extra_xacro_args,
            "extra_controller_params_file": extra_controller_params_file,
        }.items(),
    )

    point_cloud_proc = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("rgb/image_rect_color", "/wrist_mounted_camera/color/image_raw"),
            ("rgb/camera_info", "/wrist_mounted_camera/color/camera_info"),
            ("depth_registered/image_rect", "/wrist_mounted_camera/aligned_depth_to_color/image_raw"),
            ("points", "/wrist_mounted_camera/depth/color/points"),
        ],
    )

    return LaunchDescription(declared_arguments + [generate_mjcf, clr_launch, point_cloud_proc])
