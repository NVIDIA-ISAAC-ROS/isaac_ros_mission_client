# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import json
import os

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from opentelemetry import metrics
from opentelemetry.exporter.otlp.proto.grpc.metric_exporter import OTLPMetricExporter
from opentelemetry.sdk.metrics import MeterProvider
from opentelemetry.sdk.metrics.export import PeriodicExportingMetricReader
from opentelemetry.sdk.resources import Resource
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class NodeMonitorService(Node):
    """A ROS2 service that monitors the state of nodes."""

    def __init__(self):
        super().__init__('mega_node_monitor_service')

        # Create a callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters and set up metrics
        self.declare_parameter(
            'monitored_nodes',
            descriptor=ParameterDescriptor(
                description='List of regular nodes to monitor',
                additional_constraints='List of node names to monitor',
                dynamic_typing=True
            )
        )
        self.declare_parameter(
            'monitored_lifecycle_nodes',
            descriptor=ParameterDescriptor(
                description='List of lifecycle nodes to monitor',
                additional_constraints='List of lifecycle node names to monitor',
                dynamic_typing=True
            )
        )
        self.declare_parameter('enable_metrics', False)
        self.declare_parameter('metrics_interval', 15.0)
        self.declare_parameter('enable_3d_lidar_costmap', False)
        self.monitored_nodes = self.get_parameter(
            'monitored_nodes').get_parameter_value().string_array_value
        self.monitored_lifecycle_nodes = self.get_parameter(
            'monitored_lifecycle_nodes').get_parameter_value().string_array_value
        self.enable_metrics = self.get_parameter('enable_metrics').get_parameter_value().bool_value
        self.metrics_interval = self.get_parameter(
            'metrics_interval').get_parameter_value().double_value
        self.enable_3d_lidar_costmap = self.get_parameter('enable_3d_lidar_costmap').value

        # Add 3D lidar nodes to monitored nodes if enabled
        if self.enable_3d_lidar_costmap:
            self.get_logger().info('3D lidar costmap enabled, adding lidar nodes to monitoring')
            lidar_nodes = [
                '/front_3d_lidar/pointcloud_to_flatscan',
                '/front_3d_lidar/flatscan_to_laserscan'
            ]
            # Add lidar nodes
            self.monitored_nodes.extend(lidar_nodes)
            self.get_logger().info(f'Updated monitored nodes: {self.monitored_nodes}')

        # Create the service with the callback group
        self.srv = self.create_service(
            Trigger,
            'check_nodes_alive',
            self.check_nodes_callback,
            callback_group=self.callback_group)

        self.get_logger().info('Node Monitor Service has started')
        self.get_logger().info(f'Monitoring regular nodes: {self.monitored_nodes}')
        self.get_logger().info(f'Monitoring lifecycle nodes: {self.monitored_lifecycle_nodes}')

        # Verify that metrics env are configured in environment
        if self.enable_metrics:
            if not os.environ.get('OTEL_EXPORTER_OTLP_METRICS_ENDPOINT', ''):
                self.get_logger().warning(
                    'Metrics is enabled but metrics endpoint is not set. Disabling metrics.')
                self.enable_metrics = False
            if not os.environ.get('OTEL_EXPORTER_OTLP_PROTOCOL', ''):
                self.get_logger().warning(
                    'Metrics is enabled but telemetry protocol is not set. Disabling metrics.')
                self.enable_metrics = False

        # Initialize OpenTelemetry
        if self.enable_metrics:
            self.get_logger().info('Enabling metrics collection')
            otel_service_name = 'mega-amr-brain'
            telemetry_resource = Resource(
                attributes={
                    'service.name': otel_service_name,
                    'mega.component.name': otel_service_name}
            )
            exporter = OTLPMetricExporter()
            metric_reader = PeriodicExportingMetricReader(exporter, export_interval_millis=5000)
            provider = MeterProvider(metric_readers=[metric_reader], resource=telemetry_resource)
            metrics.set_meter_provider(provider)
            self.meter = metrics.get_meter(otel_service_name)

            # Metrics to collect
            self.up_metric = self.meter.create_gauge(
                name='up',
                unit='',
                description='Nodes are running'
            )
            self.up_metric.set(0)

            # Create a timer with the callback group
            self.timer = self.create_timer(
                self.metrics_interval,
                self.timer_callback,
                callback_group=self.callback_group)

    async def get_lifecycle_node_state(self, node_name: str) -> int:
        """Get the state of a lifecycle node using the get_state service."""
        # Format service name - ensure no double slashes
        clean_name = node_name.lstrip('/')
        service_name = f'/{clean_name}/get_state'

        self.get_logger().debug(f'Checking lifecycle state via service: {service_name}')

        try:
            client = self.create_client(GetState, service_name)

            # Wait for service availability with a short timeout
            if not client.wait_for_service(timeout_sec=0.2):
                self.get_logger().debug(
                    f'get_state service not available for node: {node_name}')
                return State.PRIMARY_STATE_UNKNOWN

            # Create request and call service
            request = GetState.Request()
            try:
                self.get_logger().debug(f'Calling service for node: {node_name}')
                future = client.call_async(request)

                # Add timeout for the async operation
                try:
                    # Wait for the future with a very short timeout (0.5 seconds)
                    response = await future
                    self.get_logger().debug(f'Received response for node: {node_name}')
                except Exception as e:
                    self.get_logger().debug(
                        f'Error waiting for service response for node {node_name}: {str(e)}')
                    return State.PRIMARY_STATE_UNKNOWN

                if response is not None:
                    state = response.current_state.id
                    self.get_logger().debug(
                        f'Node {node_name} state: {state}')
                    return state
                else:
                    self.get_logger().debug(
                        f'No result from get_state service for node: {node_name}')
                    return State.PRIMARY_STATE_UNKNOWN

            except Exception as e:
                self.get_logger().debug(
                    f'Failed to get state for node {node_name}: {str(e)}')
                return State.PRIMARY_STATE_UNKNOWN
            finally:
                # Cleanup client
                try:
                    self.destroy_client(client)
                except Exception as e:
                    self.get_logger().debug(f'Error destroying client: {str(e)}')
        except Exception as e:
            self.get_logger().debug(f'Error creating client for {node_name}: {str(e)}')
            return State.PRIMARY_STATE_UNKNOWN

    def get_formatted_node_names(self):
        """Get a list of all node names with consistent formatting."""
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        node_names = [f'{ns}/{name}' if ns != '/' else f'/{name}'
                      for name, ns in node_names_and_namespaces]
        return node_names

    def check_node_exists(self, node_name, node_names):
        """Check if a node exists in the list of nodes."""
        # Ensure consistent formatting with leading slash
        if not node_name.startswith('/'):
            node_name = f'/{node_name}'

        # Check if node exists (case-insensitive comparison)
        for name in node_names:
            if node_name.lower() == name.lower():
                return True
        return False

    async def check_all_nodes(self):
        """Check all nodes and return their status."""
        # Get all node names
        node_names = self.get_formatted_node_names()

        # Check regular nodes
        alive_regular = []
        dead_regular = []
        for node in self.monitored_nodes:
            if self.check_node_exists(node, node_names):
                alive_regular.append(node)
            else:
                dead_regular.append(node)

        # Check lifecycle nodes
        alive_lifecycle = []
        inactive_lifecycle = []
        dead_lifecycle = []

        for node in self.monitored_lifecycle_nodes:
            if self.check_node_exists(node, node_names):
                # Get state asynchronously
                state = await self.get_lifecycle_node_state(node.lstrip('/'))
                if state == State.PRIMARY_STATE_ACTIVE:
                    alive_lifecycle.append(node)
                else:
                    inactive_lifecycle.append(node)
            else:
                dead_lifecycle.append(node)

        # Combine results
        alive_nodes = alive_regular + alive_lifecycle
        dead_nodes = dead_regular + dead_lifecycle

        return {
            'alive_nodes': alive_nodes,
            'dead_nodes': dead_nodes,
            'inactive_lifecycle_nodes': inactive_lifecycle
        }

    async def timer_callback(self):
        """Async function for timer callback."""
        try:
            # Use the common check function
            result = await self.check_all_nodes()

            # Update metrics
            success = len(result['dead_nodes']) == 0 and len(
                result['inactive_lifecycle_nodes']) == 0
            if self.enable_metrics:
                self.up_metric.set(1 if success else 0)

            # Log the success result
            self.get_logger().info(f"Health check result: {'healthy' if success else 'unhealthy'}")
            self.get_logger().debug(f"Alive: {result['alive_nodes']}")
            self.get_logger().debug(f"Dead: {result['dead_nodes']}")
            self.get_logger().debug(f"Inactive: {result['inactive_lifecycle_nodes']}")

        except Exception as e:
            self.get_logger().error(f'Error in async timer check: {str(e)}')

    async def check_nodes_callback(self, request, response):
        """Service callback to check nodes."""
        try:
            self.get_logger().info('Service callback started')

            # Use the common check function
            result = await self.check_all_nodes()

            # Format the message for Trigger response
            status_dict = {
                'status': 'healthy' if not (result['dead_nodes'] or result[
                    'inactive_lifecycle_nodes']) else 'unhealthy',
                'dead_nodes': result['dead_nodes'],
                'inactive_lifecycle_nodes': result['inactive_lifecycle_nodes'],
                'alive_nodes': result['alive_nodes']
            }

            # Log the final status
            self.get_logger().info(f"Health check result: {status_dict['status']}")
            self.get_logger().info(f"Dead nodes: {result['dead_nodes']}")
            self.get_logger().info(
                f"Inactive lifecycle nodes: {result['inactive_lifecycle_nodes']}")
            self.get_logger().info(f"Alive nodes: {result['alive_nodes']}")

            # Set Trigger response fields
            response.success = len(result['dead_nodes']) == 0 and len(
                result['inactive_lifecycle_nodes']) == 0
            response.message = json.dumps(status_dict, indent=2)

            self.get_logger().info('Service callback completed successfully')
            return response

        except Exception as e:
            self.get_logger().error(f'Error in service callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

            # Return a basic response in case of error
            response.success = False
            response.message = f'Error checking nodes: {str(e)}'
            return response


def main(args=None):
    rclpy.init(args=args)
    node_monitor = NodeMonitorService()
    executor = MultiThreadedExecutor()
    executor.add_node(node_monitor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
