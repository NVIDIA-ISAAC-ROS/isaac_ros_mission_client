#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


NUM_ITERATIONS = 10


class Nav2SimpleServer(Node):
    """
    Simple Nav2 Server node that responds to NavigateToPose actions.

    The server provides feedback messages in each iteration by incrementing the position.x
    of the previous feedback message until the current position.x is greater than the goal
    message's position.x.

    ROS Parameters
    ----------
    start_x_pos: The x position of the first feedback message.
    x_pos_increment: The increment of x position in each feedback iteration.
    """

    def __init__(self):
        super().__init__('nav2_simple_server')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_x_pos', 0.0),
                ('x_pos_increment', 0.5)
            ]
        )
        self._action_server = ActionServer(self, NavigateThroughPoses,
                                           'navigate_through_poses',
                                           self.callback)
        self.tf_broadcaster = TransformBroadcaster(self)

    def callback(self, goal_handle):
        if len(goal_handle.request.poses) == 0:
            return GoalResponse.REJECT
        start_pos = self.get_parameter('start_x_pos').value
        feedback_msg = NavigateThroughPoses.Feedback()
        feedback_msg.current_pose.pose.position.x = start_pos

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = goal_handle.request.poses[-1].pose.position.x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        current_pos = start_pos
        while current_pos <= goal_handle.request.poses[-1].pose.position.x:
            t.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(t)
            goal_handle.publish_feedback(feedback_msg)
            current_pos += self.get_parameter('x_pos_increment').value
            feedback_msg.current_pose.pose.position.x = current_pos
            time.sleep(0.5)

        goal_handle.succeed()

        result = NavigateThroughPoses.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    nav2_simple_server = Nav2SimpleServer()
    rclpy.spin(nav2_simple_server)
    nav2_simple_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
