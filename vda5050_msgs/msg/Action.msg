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

# Describes an action that the AGV can perform.

# Name of action as described in the first column of “Actions and Parameters”.
# Identifies the function of the action.
string action_type

# Unique ID to identify the action and map them to the actionState in the
# state. Suggestion: Use UUIDs.
string action_id

# Additional information on the action
string action_description

# Enum {NOTE, SOFT, HARD}:
# "NONE"- allows driving and other actions;
# "SOFT"- allows other actions, but not driving;
# "HARD"- is the only allowed action at that time.
string blocking_type

# Array of actionParameter-objects for the indicated action, e.g., deviceId,
# loadId, external Triggers.
vda5050_msgs/ActionParameter[] action_parameters

# Enums for blockingType
string NONE = "NONE"
string SOFT = "SOFT"
string HARD = "HARD"
