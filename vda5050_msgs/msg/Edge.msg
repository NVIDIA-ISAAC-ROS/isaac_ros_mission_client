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

# Directional connection between two nodes.

# Unique edge identification.
string edge_id

# Number to track the sequence of nodes and edges in an order and to simplify
# order updates.
# The variable sequenceId runs across all nodes and edges of the same order
# and is reset when a new orderId is issued.
int32 sequence_id

# Additional information on the edge.
string edge_description

# "true" indicates that the edge is part of the base.
# "false" indicates that the edge is part of the horizon.
bool released

# nodeId of startNode.
string start_node_id

# nodeId of endNode.
string end_node_id

# Permitted maximum speed on the edge.
# Speed is defined by the fastest measurement of the vehicle.
float64 max_speed

# Permitted maximum height of the vehicle, including the load, on edge.
float64 max_height

# Permitted minimal height of the load handling device on the edge.
float64 min_height

# Orientation of the AGV on the edge.
float64 orientation

# Enum {GLOBAL, TANGENTIAL}:
# "GLOBAL"- relative to the global project specific map coordinate system;
# "TANGENTIAL"- tangential to the edge.
# If not defined, the default value is "TANGENTIAL".
string orientation_type

# Sets direction at junctions for line-guided or wire-guided vehicles, to be
# defined initially (vehicle-individual).
# Examples: left, right, straight, 433MHz.
string direction

# “true”: rotation is allowed on the edge.
# “false”: rotation is not allowed on the edge.
# Optional: No limit, if not set.
bool rotation_allowed

# Maximum rotation speed
# Optional: No limit, if not set.
float64 max_rotation_speed

# Trajectory JSON-object for this edge as a NURBS.
# Defines the curve, on which the AGV should move between startNode and
# endNode.
# Optional: Can be omitted, if AGV cannot process trajectories or if AGV plans
# its own trajectory.
vda5050_msgs/Trajectory trajectory

# Length of the path from startNode to endNode
# Optional: This value is used by line-guided AGVs to decrease their speed
# before reaching a stop position.
float64 length

# Array of actionIds to be executed on the edge.
# Empty array, if no actions required.
vda5050_msgs/Action[] actions

# Enums for orientationType
string NONE = "NONE"
string SOFT = "SOFT"
string HARD = "HARD"
