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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('isaac_ros_mega_node_monitor'),
        'config',
        'mega_node_monitor_config.yaml'
    )

    launch_args = [
        DeclareLaunchArgument('enable_metrics', default_value='false',
                              description='Enable metrics collection'),
        DeclareLaunchArgument('metrics_interval', default_value='15.0',
                              description='Metrics interval in seconds'),
        DeclareLaunchArgument('enable_3d_lidar_costmap', default_value='false',
                              description='Enable 3D lidar costmap')
    ]

    enable_metrics = LaunchConfiguration('enable_metrics')
    metrics_interval = LaunchConfiguration('metrics_interval')
    enable_3d_lidar_costmap = LaunchConfiguration('enable_3d_lidar_costmap')

    return LaunchDescription(launch_args + [
        Node(
            package='isaac_ros_mega_node_monitor',
            executable='isaac_ros_mega_node_monitor',
            name='mega_node_monitor_service',
            parameters=[
                config,
                {
                    'enable_metrics': enable_metrics,
                    'metrics_interval': metrics_interval,
                    'enable_3d_lidar_costmap': enable_3d_lidar_costmap
                }
            ],
            output='screen',
            remappings=[
                ('check_nodes_alive', '/check_nodes_alive')
            ]
        )
    ])
