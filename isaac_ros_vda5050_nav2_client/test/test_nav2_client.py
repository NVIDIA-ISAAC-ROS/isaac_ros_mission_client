# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaac_ros_test import IsaacROSBaseTest
import launch_ros.actions

import pytest
import rclpy

from vda5050_msgs.msg import Action, ActionParameter, ActionState, AGVState, Node, Order

ORDER_TOPIC = 'orders'
ORDER_INFO_TOPIC = 'agv_state'
START_X_POS = 1.0


@pytest.mark.rostest
def generate_test_description():

    nav2_simple_server = launch_ros.actions.Node(
        name='nav2_server',
        package='isaac_ros_vda5050_nav2_client',
        namespace=Nav2ClientTest.generate_namespace(),
        executable='nav2_simple_server.py',
        parameters=[{
            'start_x_pos': START_X_POS
        }],
        output='screen'
    )

    dummy_action_server = launch_ros.actions.Node(
        name='dummy_action_server',
        package='isaac_ros_vda5050_nav2_client',
        namespace=Nav2ClientTest.generate_namespace(),
        executable='dummy_action_server.py',
        output='screen'
    )

    nav2_client = launch_ros.actions.Node(
        name='nav2_client',
        package='isaac_ros_vda5050_nav2_client',
        namespace=Nav2ClientTest.generate_namespace(),
        executable='vda5050_nav2_client',
        parameters=[{
            'action_server_names': ['dummy_action_server'],
            'dummy_action_server': ['dummy_action'],
            'update_feedback_period': 0.1
        }],
        remappings=[('client_commands', ORDER_TOPIC)],
        output='screen'
    )

    return Nav2ClientTest.generate_test_description([
        nav2_simple_server, nav2_client, dummy_action_server])


class Nav2ClientTest(IsaacROSBaseTest):
    received_msgs = []

    def create_order(self, order_id: str, node_positions, node_actions):
        """
        Create order for VDA5050 client node.

        Parameters
        ----------
        order_id : str
            The order_id used for creating this order
        node_positions : List[tuple[float]]
            The positions that the robot should reach. The first element of the tuple
            should represent the x-coordinate, and the second element should represent
            the y-coordinate.
        node_actions : List[List[vda5050_msgs/Action]]
            The actions to be performed at each node.

        Returns
        -------
        Order
            The vda5050_msgs/Order message

        """
        order_msg = Order()
        order_msg.order_id = order_id
        nodes = []
        for i, node_pos in enumerate(node_positions):
            node = Node()
            node.node_id = str(i)
            node.node_position.x = node_pos[0]
            node.node_position.y = node_pos[1]
            node.node_position.map_id = 'map'
            node.actions = node_actions[i]
            nodes.append(node)
        order_msg.nodes = nodes
        return order_msg

    def create_agv_state(self, order_id, last_node_id, position_initialized,
                         position_x=0.0, position_y=0.0):
        """
        Create AGVState message.

        Parameters
        ----------
        order_id : str
            The order_id for the created AGVState
        last_node_id : str
            The last_node_id for the created AGVState
        position_initialized : bool
            The value of position_initialized in agv_position
        position_x : float
            The value of agv_position.x
        position_y : float
            The value of agv_position.y

        Returns
        -------
        AGVState
            The vda5050_msgs/AGVState message

        """
        agv_state = AGVState()
        agv_state.order_id = order_id
        agv_state.agv_position.x = position_x
        agv_state.agv_position.y = position_y
        agv_state.agv_position.position_initialized = position_initialized
        agv_state.last_node_id = last_node_id
        return agv_state

    def verify_results(self, received_messages, expected_last_msg):
        """
        Verify the messages received from the client node.

        Parameters
        ----------
        received_messages : List[AGVState]
            A list of AGVState messages from the client node.
        expected_last_msg : AGVState
            The expected last received message.

        """
        first_pos_init = received_messages[ORDER_INFO_TOPIC][0].agv_position.position_initialized
        self.assertEqual(first_pos_init, False, 'Initial position should be uninitialized')
        last_order_id = received_messages[ORDER_INFO_TOPIC][-1].order_id
        self.assertEqual(last_order_id, expected_last_msg.order_id, 'Order ID does not match')

        last_pos = received_messages[ORDER_INFO_TOPIC][-1].agv_position
        self.assertEqual(last_pos, expected_last_msg.agv_position, 'End pose does not match')
        self.assertEqual(received_messages[ORDER_INFO_TOPIC][-1].action_states,
                         expected_last_msg.action_states,
                         'Action state does not match')
        last_node_id = received_messages[ORDER_INFO_TOPIC][-1].last_node_id
        self.assertEqual(last_node_id, expected_last_msg.last_node_id,
                         'Last node ID does not match')

    def test_nav2_client(self):
        """
        Test for the client node.

        This test sends an Order message to the client node, and subscribes to the robot state
        published by the client node through AGVState messages. The AGVState
        messages are validated through the verify_results() message implemented above.
        """
        STARTUP_TIME = 3
        TIMEOUT = 10
        received_messages = {}

        self.generate_namespace_lookup([ORDER_TOPIC, ORDER_INFO_TOPIC])

        order_pub = self.node.create_publisher(
            Order, self.namespaces[ORDER_TOPIC], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers(
            [(ORDER_INFO_TOPIC, AGVState)], received_messages,
            accept_multiple_messages=True)

        # Allow nodes to initialize
        time.sleep(STARTUP_TIME)

        # Create orders for test and expected states for verification
        orders = []
        expected_last_msgs = []
        # Test order which has two nodes and no actions
        end_x = 2.0
        node_positions = [(START_X_POS, 0.0), (end_x, 0.0)]
        order_id = '0'
        orders.append(self.create_order(order_id, node_positions, [[], []]))
        # Expected state
        expected_last_msg = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, end_x)
        expected_last_msgs.append(expected_last_msg)

        # Test order with actions that will succeed
        order_id = '1'
        params = [ActionParameter(key='success', value='1')]
        node1_actions = [Action(action_type='dummy_action',
                                action_id='0',
                                action_parameters=params)]
        node_actions = [node1_actions, []]
        orders.append(self.create_order(order_id, node_positions, node_actions))
        # Expected state
        expected_last_msg = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, end_x)
        expected_last_msg.action_states.append(
            ActionState(action_id='0', action_status='FINISHED')
        )
        expected_last_msgs.append(expected_last_msg)

        # Test order with actions that will fail.
        # DummyActionServer will send failed result if the first parameter
        # value is not '1'.
        order_id = '2'
        node2_actions = [Action(action_type='dummy_action',
                                action_id='0',
                                action_parameters=params),
                         Action(action_type='dummy_action',
                                action_id='1')]
        node_actions = [[], node2_actions]
        orders.append(self.create_order(order_id, node_positions, node_actions))
        # Expected state
        expected_last_msg = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, end_x)
        expected_last_msg.action_states.append(
            ActionState(action_id='0', action_status='FINISHED')
        )
        expected_last_msg.action_states.append(
            ActionState(action_id='1', action_status='FAILED')
        )
        expected_last_msgs.append(expected_last_msg)

        try:
            for i, order in enumerate(orders):
                received_messages[ORDER_INFO_TOPIC].clear()
                last_node_id = str(len(order.nodes) - 1)
                end_time = time.time() + TIMEOUT
                published = False
                while time.time() < end_time:
                    if not published:
                        order_pub.publish(order)
                        published = True
                    rclpy.spin_once(self.node, timeout_sec=(0.1))

                    has_failed_action = False
                    has_unfinished_action = False
                    if len(received_messages[ORDER_INFO_TOPIC]) > 0:
                        for action_state in received_messages[ORDER_INFO_TOPIC][-1].action_states:
                            if action_state.action_status == 'FAILED':
                                has_failed_action = True
                            elif action_state.action_status != 'FINISHED':
                                has_unfinished_action = True
                    if has_failed_action:
                        break
                    if (len(received_messages[ORDER_INFO_TOPIC]) and
                            received_messages[ORDER_INFO_TOPIC][-1].last_node_id ==
                            last_node_id and not has_unfinished_action):
                        break

                self.assertGreater(len(received_messages[ORDER_INFO_TOPIC]), 0,
                                   'Appropriate output not received')
                self.verify_results(received_messages, expected_last_msgs[i])

        finally:
            self.node.destroy_subscription(subs)
            self.node.destroy_publisher(order_pub)
