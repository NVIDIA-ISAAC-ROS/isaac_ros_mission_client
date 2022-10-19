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

# Header ID of the message. The headerId is defined per topic and incremented
# by 1 with each sent (but not necessarily received) message.
int32 header_id

# Timestamp after ISO8601 in the format YYYY-MM-DDTHH:mm:ss.ssZ
# (e.g.“2017-04-15T11:40:03.12Z”)
string timestamp

# Version of the protocol [Major].[Minor].[Patch] (e.g. 1.3.2)
string version

# Manufacturer of the AGV
string manufacturer

# Serial Number of the AGV
string serial_number

# Unique order identification of the current order or the previous finished
# order. The orderId is kept until a new order is received. Empty string (""),
# if no previous orderId is available.
string order_id

# Order identification. Is unique per orderId. If an order update is rejected,
# this field is to be passed in the rejection message
uint32 order_update_id

# Unique identifier of the zone set, that the AGV has to use for navigation or
# that was used by master control for planning.
# Optional: If the AGV does not use zones, this field can be omitted.
string zone_set_id

# Node ID of last reached node or, if AGV is currently on a node, current node
# (e.g., „node7”). Empty string (""), if no lastNodeId is available.
string last_node_id

# Sequence ID of the last reached node or, if AGV is currently on a node,
# Sequence ID of current node. "o" if no lastNodeSequenced is available.
int32 last_node_sequence_id

# Array of nodeState-Objects, that need to be traversed for fulfilling the
# order (empty list if idle)
vda5050_msgs/NodeState[] node_states

# Array of edgeState-Objects, that need to be traversed for fulfilling the
# order (empty list if idle)
vda5050_msgs/EdgeState[] edge_states

# Current position of the AGV on the map.
# Optional: Can only be omitted for AGV without the capability to localize
# themselves, e.g., line guided AGVs.
vda5050_msgs/AGVPosition agv_position

# The AGV velocity in vehicle coordinates.
vda5050_msgs/Velocity velocity

# Loads, that are currently handled by the AGV.
# Optional: If AGV cannot determine load state, leave the array out of the
# state. If the AGV can determine the load state, but the array is empty, the
# AGV is considered unloaded.
vda5050_msgs/Load[] loads

# “true”: indicates, that the AGV is driving and/or rotating. Other movements
# of the AGV (e.g., lift movements) are not included here.
# “false”: indicates that the AGV is neither driving nor rotating.
bool driving

# “true”: AGV is currently in a paused state, either because of the push of a
# physical button on the AGV or because of an instantAction. The AGV can
# resume the order.
# “false”: The AGV is currently not in a paused state.
bool paused

# “true”: AGV is almost at the end of the base and will reduce speed, if no new
# base is transmitted. Trigger for master control to send a new base.
# “false”: no base update required.
bool new_base_requested

# Used by line guided vehicles to indicate the distance it has been driving
# past the „lastNodeId“. Distance is in meters.
float64 distance_since_last_node

# Contains a list of the current actions and the actions, which are yet to be
# finished.
# This may include actions from previous nodes, that are still in progress.
vda5050_msgs/ActionState[] action_states

# Contains all battery-related information.
vda5050_msgs/BatteryState battery_state

# Enum {AUTOMATIC, SEMIAUTOMATIC, MANUAL, SERVICE, TEACHIN}
string operating_mode

# Array of error-objects.
# All active errors of the AGV should be in the list.
# An empty array indicates that the AGV has no active errors.
vda5050_msgs/Error[] errors

# Array of info-objects.
# An empty array indicates, that the AGV has no information.
# This should only be used for visualization or debugging – it must not be used
# for logic in master control.
vda5050_msgs/Info[] informations

# Contains all safety-related information.
vda5050_msgs/SafetyState safety_state

# Enums for operatingMode

# AGV is under full control of the master control.
# AGV drives and executes actions based on orders from the master control.
string AUTOMATIC="AUTOMATIC"

# AGV is under control of the master control.
# AGV drives and executes actions based on orders from the master control.
string SEMIAUTOMATIC="SEMIAUTOMATIC"

# Master control is not in control of the AGV.
# Supervisor doesn’t send driving order or actions to the AGV.
# HMI can be used to control the steering and velocity and handling device of
# the AGV.
string MANUAL="MANUAL"

# Master control is not in control of the AGV.
# Master control doesn’t send driving order or actions to the AGV.
# Authorized personal can reconfigure the AGV.
string SERVICE="SERVICE"

# Master control is not in control of the AGV.
# Supervisor doesn’t send driving order or actions to the AGV.
# The AGV is being taught, e.g., mapping is done by a master control.
string TEACHIN="TEACHIN"
