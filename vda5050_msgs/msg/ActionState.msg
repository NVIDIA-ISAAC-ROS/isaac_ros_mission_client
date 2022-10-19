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

# action_ID
string action_id

# actionType of the action.
# Optional: Only for informational or visualization purposes.
string action_type

# Additional information on the current action.
string action_description

# Enum {WAITING; INITIALIZING; RUNNING; PAUSED; FINISHED; FAILED}
# WAITING: waiting for the trigger (passing the mode, entering the edge)
# PAUSED: paused by instantAction or external trigger
# FAILED: action could not be performed.
string action_status

# Description of the result, e.g., the result of a RFID-read.
string result_description

# Enums for actionStatus
string WAITING="WAITING"
string INITIALIZING="INITIALIZING"
string RUNNING="RUNNING"
string PAUSED="PAUSED"
string FINISHED="FINISHED"
string FAILED="FAILED"
