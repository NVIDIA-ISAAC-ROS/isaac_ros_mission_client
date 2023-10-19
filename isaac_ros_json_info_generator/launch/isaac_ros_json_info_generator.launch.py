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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Json Info Generator node."""
    config = os.path.join(
        get_package_share_directory('isaac_ros_json_info_generator'),
        'config',
        'json_info_generator_params.yaml'
    )

    namespace = LaunchConfiguration('namespace')

    json_info_generator_node = Node(
        name='json_info_generator_node',
        package='isaac_ros_json_info_generator',
        executable='json_info_generator_node',
        parameters=[config],
        namespace=namespace,
        remappings=[('ros_sub_topic', 'agv_state')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for ROS nodes in this launch script'),
        json_info_generator_node
    ])
