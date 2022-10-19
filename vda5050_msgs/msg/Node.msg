# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# This repository implements data types and logic specified in the VDA5050
# protocol, which is specified here
# https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md

# Unique node identification
string node_id

# Number to track the sequence of nodes and edges in an order and to simplify
# order updates.
int32 sequence_id

# Additional information on the node
string node_description

# "true" indicates that the node is part of the base.
# "false" indicates that the node is part of the horizon.
bool released

# Node position.
# Optional for vehicle-types that do not require the node position
# (e.g., line-guided vehicles).
vda5050_msgs/NodePosition node_position

# Array of actions to be executed on a node. Empty array, if no actions
# required.
vda5050_msgs/Action[] actions
