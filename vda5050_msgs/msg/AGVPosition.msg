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

# Defines the position on a map in world coordinates.
# Each floor has its own map.

# “true”: position is initialized.
# “false”: position is not initialized.
bool position_initialized

# Range: [0.0 ... 1.0]
# Describes the quality of the localization
# Optional for vehicles, that cannot estimate their localization score.
float64 localization_score

# Value for the deviation range of the position in meters.
# Optional for vehicles that cannot estimate their deviation e.g. grid-based
# localization.
float64 deviation_range

# X-position on the map in reference to the map coordinate system.
# Precision is up to the specific implementation.
float64 x

# Y-position on the map in reference to the map coordinate system.
# Precision is up to the specific implementation.
float64 y

# Range: [-Pi ... Pi]
# Orientation of the AGV.
float64 theta

# Unique identification of the map in which the position is referenced.
string map_id

# Additional information on the map.
string map_description
