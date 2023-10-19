#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import time

from isaac_ros_vda5050_nav2_client.action import MissionAction
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class DummyActionServer(Node):
    """
    Dummy Action Server node to test execution of missions with actions.

    The server provides result message according to the value for parameter
    'success' in the request message. If the value is 1, the server sends
    success result. Otherwise failure.

    """

    def __init__(self):
        super().__init__('dummy_action_server')
        self._action_server = ActionServer(self, MissionAction, 'dummy_action_server',
                                           self.callback)

    def callback(self, goal_handle):
        result = MissionAction.Result()
        params = {}
        for key, value in zip(goal_handle.request.keys, goal_handle.request.values):
            params[key] = value
        result.success = params.get('success') == '1'
        time.sleep(0.2)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    dummy_action_server = DummyActionServer()
    rclpy.spin(dummy_action_server)
    dummy_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
