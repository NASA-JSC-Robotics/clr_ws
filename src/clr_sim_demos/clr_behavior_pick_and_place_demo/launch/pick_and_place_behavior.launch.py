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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    include_mockups_in_description = LaunchConfiguration("include_mockups_in_description")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    move_group_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("clr_moveit_config"),
                "launch",
                "clr_moveit.launch.py",
            ),
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "include_mockups_in_description": include_mockups_in_description,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    color_blob_node = Node(
        package="color_blob_centroid",
        executable="color_blob_node",
        output="both",
        parameters=[
            {
                "mock_hardware": False,
                "show_image": False,
                "debug": False,
                "continuous_output": False,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    drt_behavior_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("drt_behavior"),
                "launch",
                "drt_behavior_executor.py",
            ),
        )
    )

    return [move_group_nodes, color_blob_node, drt_behavior_nodes]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch rviz?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If the robot is running in simulation, use the published clock",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "include_mockups_in_description",
            default_value="true",
            description="Represent the iMETRO mockup environment in the robot description.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
