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

# Type/name of error
string error_type

# Array of references to identify the source of the error
vda5050_msgs/ErrorReference[] error_references

# Error description.
string error_description

# Enum {WARNING, FATAL}
# WARNING: AGV is ready to start (e.g. maintenance cycle expiration warning).
# FATAL: AGV is not in running condition, user intervention required
# (e.g. laser scanner is contaminated).
string error_level

# Enums for error_level
string WARNING="WARNING"
string FATAL="FATAL"
