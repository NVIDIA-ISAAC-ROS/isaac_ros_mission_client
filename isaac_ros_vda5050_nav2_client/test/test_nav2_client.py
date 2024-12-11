# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from sensor_msgs.msg import BatteryState as ROSBatteryState
from vda5050_msgs.msg import (
    Action, ActionParameter, ActionState, AGVState, BatteryState, InstantActions, Node, Order)

ORDER_TOPIC = 'orders'
INSTANT_ORDER_TOPIC = 'instant_orders'
ORDER_INFO_TOPIC = 'agv_state'
BATTERY_STATE_TOPIC = 'battery_state'
START_X_POS = 1.0
STARTUP_TIME = 3
TIMEOUT = 10


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
        remappings=[('client_commands', ORDER_TOPIC),
                    ('instant_actions_commands', INSTANT_ORDER_TOPIC)],
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

    def create_battery_state(self, percentage, voltage, power_supply_status):
        """
        Create BatteryState message.

        Parameters
        ----------
        percentage : float
            Charge percentage on 0 to 1 range
        voltage : float
            Voltage in Volts
        power_supply_status : int
            The charging status as reported. Values defined below.

            POWER_SUPPLY_STATUS_UNKNOWN = 0
            POWER_SUPPLY_STATUS_CHARGING = 1
            POWER_SUPPLY_STATUS_DISCHARGING = 2
            POWER_SUPPLY_STATUS_NOT_CHARGING = 3
            POWER_SUPPLY_STATUS_FULL = 4

        Returns
        -------
        BatteryState
            The sensor_msgs/BatteryState message

        """
        battery_state = ROSBatteryState()
        battery_state.percentage = percentage
        battery_state.voltage = voltage
        battery_state.power_supply_status = power_supply_status
        return battery_state

    def verify_battery_state(self, received_messages, expected_battery_state):
        """
        Verify the battery state messages received from the client node.

        Parameters
        ----------
        received_messages : List[AGVState]
            A list of AGVState messages from the client node.
        expected_battery_state: AGVState
            The expected battery state message.

        """
        battery_state = received_messages[ORDER_INFO_TOPIC][-1].battery_state
        self.assertEqual(battery_state.battery_charge, expected_battery_state.battery_charge)
        self.assertEqual(battery_state.battery_voltage, expected_battery_state.battery_voltage)
        self.assertEqual(battery_state.charging, expected_battery_state.charging)

    def is_order_completed(self, received_messages, last_node_id):
        has_failed_action = False
        has_unfinished_action = False
        if len(received_messages[ORDER_INFO_TOPIC]) > 0:
            for action_state in received_messages[ORDER_INFO_TOPIC][-1].action_states:
                if action_state.action_status == 'FAILED':
                    has_failed_action = True
                elif action_state.action_status != 'FINISHED':
                    has_unfinished_action = True
        if has_failed_action:
            return True
        if (len(received_messages[ORDER_INFO_TOPIC]) and
                received_messages[ORDER_INFO_TOPIC][-1].last_node_id ==
                last_node_id and not has_unfinished_action):
            return True
        return False

    def is_action_running(self, received_messages, action_idx=0):
        if len(received_messages[ORDER_INFO_TOPIC]) > 0 and \
           len(received_messages[ORDER_INFO_TOPIC][-1].action_states) > action_idx:
            if received_messages[ORDER_INFO_TOPIC][-1].action_states[action_idx].action_status \
               != 'WAITING':
                return True
        return False

    def teleop_with_api(self, received_messages, order_pub, instant_order_pub, node_positions):
        """Test Teleop with StartTeleop/StopTeleop instant actions."""
        order_id = 'teleop_with_api'
        received_messages[ORDER_INFO_TOPIC].clear()
        node_actions = [[], []]
        teleop_order = self.create_order(order_id, node_positions, node_actions)
        # Test startTeleop instant action.
        start_teleop_instant_order = InstantActions()
        start_teleop_instant_order.instant_actions = [
            Action(action_type='startTeleop', action_id='start_teleop_0')]
        # Test stopTeleop instant action.
        stop_teleop_instant_order = InstantActions()
        stop_teleop_instant_order.instant_actions = [
            Action(action_type='stopTeleop', action_id='stop_teleop_0')]
        # Expected state.
        expected_teleop = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, node_positions[-1][0])
        expected_teleop.action_states = [
            ActionState(action_id='start_teleop_0', action_type='startTeleop',
                        action_status='FINISHED'),
            ActionState(action_id='stop_teleop_0', action_type='stopTeleop',
                        action_status='FINISHED')
        ]
        order_pub.publish(teleop_order)
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        instant_order_pub.publish(start_teleop_instant_order)
        end_time = time.time() + TIMEOUT
        i = 0
        while time.time() < end_time:
            # Simulate teleop
            if i == 3:
                instant_order_pub.publish(stop_teleop_instant_order)
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(received_messages, '1') and \
               received_messages[ORDER_INFO_TOPIC][-1].agv_position.x == node_positions[-1][0]:
                break
            i += 1
        self.assertGreater(len(received_messages[ORDER_INFO_TOPIC]), 0,
                           'Appropriate output not received')
        self.verify_results(received_messages, expected_teleop)

    def teleop_with_mission_node(self, received_messages, order_pub,
                                 instant_order_pub, node_positions):
        """Test Teleop with pause order mission node."""
        order_id = 'teleop_with_mission_node'
        received_messages[ORDER_INFO_TOPIC].clear()
        pause_order_action = [Action(action_type='pause_order',
                              action_id='pause_order_1',
                              action_parameters=[])]
        node_actions = [[], pause_order_action]
        teleop_order = self.create_order(order_id, node_positions, node_actions)
        stop_teleop_instant_order = InstantActions()
        stop_teleop_instant_order.instant_actions = [
            Action(action_type='stopTeleop', action_id='stop_teleop')]
        # Expected state
        expected_teleop = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, node_positions[-1][0])
        expected_teleop.action_states = [
            ActionState(action_id='pause_order_1', action_type='pause_order',
                        action_status='FINISHED'),
            ActionState(action_id='stop_teleop', action_type='stopTeleop',
                        action_status='FINISHED')
        ]
        order_pub.publish(teleop_order)
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        end_time = time.time() + TIMEOUT
        stop_order_published = False
        while time.time() < end_time:
            if self.is_action_running(received_messages, 0) and not stop_order_published:
                stop_order_published = True
                instant_order_pub.publish(stop_teleop_instant_order)
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(received_messages, '1'):
                break
        # spin twice for the order to complete
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        self.assertGreater(len(received_messages[ORDER_INFO_TOPIC]), 0,
                           'Appropriate output not received')
        self.verify_results(received_messages, expected_teleop)

    def test_nav2_client(self):
        """
        Test for the client node.

        This test sends an Order message to the client node, and subscribes to the robot state
        published by the client node through AGVState messages. The AGVState
        messages are validated through the verify_results() message implemented above.
        """
        received_messages = {}

        self.generate_namespace_lookup([ORDER_TOPIC, INSTANT_ORDER_TOPIC, ORDER_INFO_TOPIC,
                                        BATTERY_STATE_TOPIC])

        order_pub = self.node.create_publisher(
            Order, self.namespaces[ORDER_TOPIC], self.DEFAULT_QOS)

        instant_order_pub = self.node.create_publisher(
            InstantActions, self.namespaces[INSTANT_ORDER_TOPIC], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers(
            [(ORDER_INFO_TOPIC, AGVState)], received_messages,
            accept_multiple_messages=True)

        battery_state_pub = self.node.create_publisher(
            ROSBatteryState, self.namespaces[BATTERY_STATE_TOPIC], self.DEFAULT_QOS)

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
            ActionState(action_id='0', action_type='dummy_action', action_status='FINISHED')
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
            ActionState(action_id='0', action_type='dummy_action', action_status='FINISHED')
        )
        expected_last_msg.action_states.append(
            ActionState(action_id='1', action_type='dummy_action', action_status='FAILED')
        )
        expected_last_msgs.append(expected_last_msg)

        # Test cancelOrder instant action.
        instant_order = InstantActions()
        instant_order.instant_actions = [Action(action_type='cancelOrder',
                                                action_id='2')]

        # Expected state
        expected_last_msg = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, end_x)
        expected_last_msg.action_states = [
            ActionState(action_id='0', action_type='dummy_action', action_status='FAILED'),
            ActionState(action_id='1', action_type='dummy_action', action_status='FAILED'),
            ActionState(action_id='2', action_type='cancelOrder', action_status='FINISHED')
        ]

        # Test battery state
        battery_state_messages = [
            self.create_battery_state(0.25, 12.0, 1),
            self.create_battery_state(1.0, 5.0, 3)
        ]

        # Expected battery messages
        expected_battery_state = [
            BatteryState(battery_charge=25.0, battery_voltage=12.0, charging=True),
            BatteryState(battery_charge=100.0, battery_voltage=5.0, charging=False)
        ]

        try:
            # Test teleop with mission node
            self.teleop_with_mission_node(
                received_messages, order_pub, instant_order_pub, [(2.0, 0.0), (1.0, 0.0)])

            # Test Teleop with API calls
            self.teleop_with_api(
                received_messages, order_pub, instant_order_pub, [(1.0, 0.0), (2.0, 0.0)])

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

                    if self.is_order_completed(received_messages, last_node_id):
                        break
                self.assertGreater(len(received_messages[ORDER_INFO_TOPIC]), 0,
                                   'Appropriate output not received')
                self.verify_results(received_messages, expected_last_msgs[i])

            # Test cancelOrder action
            received_messages[ORDER_INFO_TOPIC].clear()
            last_node_id = str(len(orders[2].nodes) - 1)
            order_pub.publish(orders[2])
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            instant_order_pub.publish(instant_order)
            end_time = time.time() + TIMEOUT
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                self.is_order_completed(received_messages, last_node_id)
            self.verify_results(received_messages, expected_last_msg)

            # Test battery state reporting
            battery_state_pub.publish(battery_state_messages[0])
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            self.verify_battery_state(received_messages, expected_battery_state[0])

            battery_state_pub.publish(battery_state_messages[1])
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            self.verify_battery_state(received_messages, expected_battery_state[1])

        finally:
            self.node.destroy_subscription(subs)
            self.node.destroy_publisher(order_pub)
            self.node.destroy_publisher(battery_state_pub)
