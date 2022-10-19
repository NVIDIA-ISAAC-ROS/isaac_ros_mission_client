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

# Enum {AUTOACK,MANUAL,REMOTE,NONE}
# Acknowledge-Type of eStop:
# AUTOACK: auto-acknowledgeable e-stop is activated, e.g., by bumper or
# protective field.
# MANUAL: e-stop hast to be acknowledged manually at the vehicle.
# REMOTE: facility e-stop has to be acknowledged remotely.
# NONE: no e-stop activated.
string e_stop

# Protective field violation.
# "true":field is violated
# "false":field is not violated.
bool field_violation

# Enums for eStop
string AUTO_ACK="autoAck"
string MANUAL="manual"
string REMOTE="remote"
string NONE="none"
