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
from launch_ros.actions import Node


def generate_launch_description():

    point_cloud_proc = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
        remappings=[
            ("rgb/image_rect_color", "/wrist_mounted_camera/color/image_raw"),
            ("rgb/camera_info", "/wrist_mounted_camera/color/camera_info"),
            ("depth_registered/image_rect", "/wrist_mounted_camera/aligned_depth_to_color/image_raw"),
            ("points", "/wrist_mounted_camera/depth/color/points"),
        ],
    )

    return LaunchDescription([point_cloud_proc])
