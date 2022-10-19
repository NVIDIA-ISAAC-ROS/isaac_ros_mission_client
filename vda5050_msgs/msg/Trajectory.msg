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

# Range: [1 ... infinity]
# Defines the number of control points that influence any given point on the
# curve. Increasing the degree increases continuity.
# If not defined, the default value is 1.
int32 degree

# Range: [ 0.0 ... 1.0]
# Sequence of parameter values that determines where and how the control points
# affect the NURBS curve.
# knotVector has size of number of control points + degree + 1.
float64[] knot_vector

# List of JSON controlPoint objects defining the control points of the NURBS,
# which includes the beginning and end point.
vda5050_msgs/ControlPoint[] control_points
