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
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            try:
                return yaml.safe_load(file)
            except yaml.YAMLError as exc:
                print(exc)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        print("Was not able to load the yaml file at " + absolute_file_path)
        return None


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="If the robot is running in simulation, use the published clock. /"
            "If this flag is false, the mockups managers will be launched.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "wait_for_prompt",
            default_value="true",
            description="Whether to prompt before executing a trajectory.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "scaling_factor",
            default_value="1.0",
            description="Factor (<=1.0) by which to scale velocity and acceleration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "waypoint_cfg",
            default_value="waypoints.yaml",
        )
    )

    sim = LaunchConfiguration("sim")
    wp_cfg_file_name = LaunchConfiguration("waypoint_cfg")
    waypoint_cfg = PathJoinSubstitution(
        [get_package_share_directory("clr_pick_and_place_demo"), "config", wp_cfg_file_name]
    )

    description_package = "clr_imetro_environments"
    description_file = "clr_trainer_multi_hatch.urdf.xacro"
    moveit_config_file_path = "srdf/clr_and_sim_mockups.srdf.xacro"
    description_full_path = os.path.join(get_package_share_directory(description_package), "urdf", description_file)

    moveit_config = (
        MoveItConfigsBuilder("clr", package_name="clr_moveit_config")
        .robot_description(file_path=description_full_path)
        .robot_description_semantic(file_path=moveit_config_file_path)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/clr_moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    nodes_to_start = [
        Node(
            package="clr_pick_and_place_demo",
            executable="run_demo",
            output="both",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": sim},
                {"waypoint_cfg": waypoint_cfg},
                {"wait_for_prompt": LaunchConfiguration("wait_for_prompt")},
                {"scaling_factor": LaunchConfiguration("scaling_factor")},
                {"hw": NotSubstitution(sim)},
            ],
            on_exit=Shutdown(),
        ),
        Node(
            package="color_blob_centroid",
            executable="color_blob_node",
            output="both",
            parameters=[
                {
                    "mock_hardware": False,
                    "show_image": False,
                    "debug": False,
                    "continuous_output": False,
                    "use_sim_time": sim,
                }
            ],
        ),
    ]

    hw_launch = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("clr_imetro_environments"), "launch", "mockups_managers.launch.py"
                )
            ),
            launch_arguments={
                "hatch_4040": "true",
                "trainer": "true",
                "second_trainer": "false",
                "tf_prefix": "",
            }.items(),
            condition=UnlessCondition(sim),
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start + hw_launch)
