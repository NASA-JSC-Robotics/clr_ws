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

import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from imetro_behavior.executor import BehaviorTreeExecutor

# These imports auto-register behaviors, so we should grab them all
from imetro_behavior.basic_behaviors import *  # noqa: F401,F403
from imetro_behavior.color_behaviors import *  # noqa: F401,F403
from imetro_behavior.decorators import *  # noqa: F401,F403
from imetro_behavior.geometry_behaviors import *  # noqa: F401,F403
from imetro_behavior.moveit_behaviors import *  # noqa: F401,F403
from imetro_behavior.nav_behaviors import *  # noqa: F401,F403
from imetro_behavior.ros_behaviors.perception import *  # noqa: F401,F403
from imetro_behavior.ros_behaviors.service_client import *  # noqa: F401,F403


if __name__ == "__main__":
    rclpy.init()
    node = Node("run_behavior")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    bt_exec = BehaviorTreeExecutor(
        node,
        dt=0.1,
        search_paths=[
            os.path.join(get_package_share_directory("imetro_behavior"), "trees"),
            os.path.join(get_package_share_directory("clr_behavior_demos"), "trees"),
        ],
    )

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
