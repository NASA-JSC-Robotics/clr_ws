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

import time
import rclpy


def spin_until(predicate, node, timeout=20.0, spin_period=0.05):
    """Spin the node until predicate() returns True or timeout is reached.

    Returns True if the predicate was satisfied, False on timeout.
    """
    end_time = time.time() + timeout
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=spin_period)
        if predicate():
            return True
    return False
