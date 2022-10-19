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

# Unique identification number of the load (e.g., barcode or RFID).
# Empty field, if the AGV can identify the load, but didn’t identify the load
# yet.
# Optional, if the AGV cannot identify the load.
string load_id

# Type of load.
string load_type

# Indicates, which load handling/carrying unit of the AGV is used, e.g., in
# case the AGV has multiple spots/positions to carry loads.
string load_position

# Point of reference for the location of the bounding box.
vda5050_msgs/BoundingBoxReference bounding_box_reference

# Dimensions of the load´s bounding box in meters.
vda5050_msgs/LoadDimensions load_dimensions

# Range: [0.0 ... infinity)
# Absolute weight of the load measured in kg.
uint32 weight
