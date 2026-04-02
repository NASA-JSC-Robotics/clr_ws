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

import os
import sys
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
import pytest
import rclpy
from sensor_msgs.msg import JointState

sys.path.append(os.path.dirname(__file__))
from clr_test import spin_until


@pytest.mark.rostest
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("clr_deploy"), "launch", "clr_sim.launch.py")
        ),
    )

    return LaunchDescription([launch_include, KeepAliveProc(), ReadyToTest()])


class TestFixture(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")
        self._latest_js = None
        self._js_sub = self.node.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def joint_state_cb(self, msg):
        self._latest_js = msg

    def test_basic_sim_launch(self):
        self.assertTrue(
            spin_until(lambda: self._latest_js is not None, self.node, timeout=30.0),
            "No joint states received",
        )
        expected_joints = {
            "vention_rail_base_to_carriage",
            "ewellix_lift_lower_to_higher",
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "finger_1_joint",
        }
        self.assertEqual(
            set(self._latest_js.name),
            expected_joints,
            f"Unexpected joints: {self._latest_js.name}",
        )
