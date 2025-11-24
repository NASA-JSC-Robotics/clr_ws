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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


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
    sim = LaunchConfiguration("sim")

    package_share_dir = get_package_share_directory("clr_pick_and_place_demo")

    description_package = "clr_imetro_environments"
    description_file = "clr_trainer_multi_hatch.urdf.xacro"
    moveit_config_file_path = "srdf/clr_and_sim_mockups.srdf.xacro"

    description_full_path = os.path.join(get_package_share_directory(description_package), "urdf", description_file)
    rviz_config_file = os.path.join(package_share_dir, "rviz", "demo_config.rviz")
    kinematics_config_file = os.path.join(package_share_dir, "config", "demo_kinematics.yaml")

    moveit_config = (
        MoveItConfigsBuilder("clr", package_name="clr_moveit_config")
        .robot_description(file_path=description_full_path)
        .robot_description_semantic(file_path=moveit_config_file_path)
        .robot_description_kinematics(file_path=kinematics_config_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/clr_moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    launches = [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="both",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": sim},
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="both",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": sim},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launches)
