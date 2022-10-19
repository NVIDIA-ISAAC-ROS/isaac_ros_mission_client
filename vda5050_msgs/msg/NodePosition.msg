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

# Defines the position on a map in a global project specific world coordinate
# system.

# X-position on the map in reference to the map coordinate system.
# Precision is up to the specific implementation.
float64 x

# Y-position on the map in reference to the map coordinate system.
# Precision is up to the specific implementation.
float64 y

# Range: [-Pi ... Pi]
# Absolute orientation of the AGV on the node.
# Optional: vehicle can plan the path by itself.
float64 theta

# Indicates how exact an AGV has to drive over a node in order for it to count
# as traversed.
float32 allowed_deviation_x_y

# Range: [0 ... Pi]
# Indicates how big the deviation of theta angle can be.
# The lowest acceptable angle is theta - allowedDeviationTheta and the highest
# acceptable angle is theta + allowedDeviationTheta.
float32 allowed_deviation_theta

# Unique identification of the map in which the position is referenced.
# Each map has the same project specific global origin of coordinates.
string map_id

# Additional information on the map.
string map_description
