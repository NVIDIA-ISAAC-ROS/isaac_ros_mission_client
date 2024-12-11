# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# Series Name of the AGV
string series_name

# Series Description of the AGV
# Optional: This field can be ommitted
string series_description

# Simplified description of AGV kinematics-type.
string agv_kinematics

# Simplified description of AGV class.
string agv_class

# [kg], Maximum loadable mass.
float64 max_load_mass

# Simplified description of localization type.
string[] localization_types

# List of path planning types supported by the
# AGV, sorted by priority
string[] navigation_types