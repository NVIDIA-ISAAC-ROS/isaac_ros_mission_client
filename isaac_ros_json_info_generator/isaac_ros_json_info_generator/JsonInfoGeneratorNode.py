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

"""Implementation for JsonInfoGeneratorNode."""
from collections import deque
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosbridge_library.internal import message_conversion
from rosbridge_library.internal import ros_loader
from std_msgs.msg import String


class JsonInfoGeneratorNode(Node):
    """Node that collects messages from different topics and sends them to mission client."""

    def __init__(self, name='json_info_generator_node'):
        """Construct the JsonInfoGeneratorNode."""
        super().__init__(name)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ros_subscriber_types', [
                 'std_msgs/String', 'std_msgs/String']),
                ('ros_subscriber_topics', ['data1', 'data2']),
                ('ros_pub_sub_queue', 10),
                ('messages_aggregated_count', 10),
                ('update_period', 1)
            ]
        )

        self.messages = {}
        self.order_id = -1
        self.lock = threading.Lock()
        self.order_subscriber = self.create_subscription(
            String, 'order_id', self.__order_id_callback,
            self.get_parameter('ros_pub_sub_queue').value)
        self.subscription = []
        ros_subscriber_types_param_type = self.get_parameter_or('ros_subscriber_types').type_
        if ros_subscriber_types_param_type == Parameter.Type.NOT_SET:
            self.get_logger().info('Parameter ros_subscriber_types is not set.')
            return
        ros_subscriber_types = self.get_parameter('ros_subscriber_types').value
        ros_subscriber_topics = self.get_parameter(
            'ros_subscriber_topics').value
        # The number of values in `ros_subscriber_types` should equal the
        # number of values in `ros_subscriber_topics`
        assert len(ros_subscriber_types) == len(ros_subscriber_topics)
        # Create a subscriber for each pair of subscriber type and topic given
        # in the `ros_subscriber_types` and `ros_subscriber_topics` parameters
        for i in range(len(ros_subscriber_types)):
            self.subscription.append(self.create_subscription(
                ros_loader.get_message_class(
                    ros_subscriber_types[i]),
                ros_subscriber_topics[i], self.__ros_subscriber_callback(
                    ros_subscriber_topics[i]),
                self.get_parameter('ros_pub_sub_queue').value))

        self.create_timer(self.get_parameter(
            'update_period').value, self.__timer_callback)
        self.publisher = self.create_publisher(
            String, 'info', self.get_parameter('ros_pub_sub_queue').value)

    def __order_id_callback(self, msg):
        with self.lock:
            if msg.data != self.order_id:
                self.messages = {}
                self.order_id = msg.data

    def __ros_subscriber_callback(self, topic_name):
        def callback(msg):
            extracted = message_conversion.extract_values(msg)
            with self.lock:
                if topic_name not in self.messages:
                    self.messages[topic_name] = deque([extracted])
                else:
                    self.messages[topic_name].append(extracted)
                    if len(self.messages[topic_name]) > \
                            self.get_parameter('messages_aggregated_count').value:
                        self.messages[topic_name].popleft()
        return callback

    def __timer_callback(self):
        encoded_json = String()
        with self.lock:
            # Deque is not JSON serializable, so convert the deques stored to
            # list so the JSON library can serialize them.
            messages_transformed = {topic: list(deque) for (
                topic, deque) in self.messages.items()}
            encoded_json.data = json.dumps(messages_transformed)
            self.publisher.publish(encoded_json)


def main(args=None):
    """Execute the JsonInfoGeneratorNode."""
    rclpy.init(args=args)
    node = JsonInfoGeneratorNode('json_info_generator_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
