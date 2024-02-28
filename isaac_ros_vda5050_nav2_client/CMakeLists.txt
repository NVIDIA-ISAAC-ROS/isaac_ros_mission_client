# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_vda5050_nav2_client LANGUAGES C CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

# The FindPythonInterp and FindPythonLibs modules are removed
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

execute_process(
  COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

set(CUDA_MIN_VERSION "11.4")

# find dependencies
find_package(nav_msgs)
find_package(nav2_msgs)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MissionAction.action"
)
ament_auto_add_library(vda5050_nav2_client_node SHARED
  src/vda5050_nav2_client_node.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(vda5050_nav2_client_node "${cpp_typesupport_target}")

target_compile_definitions(vda5050_nav2_client_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(vda5050_nav2_client_node
  "isaac_ros::mission_client::Vda5050toNav2ClientNode")
set(node_plugins "${node_plugins}isaac_ros::mission_client::Vda5050toNav2ClientNode;$<TARGET_FILE:vda5050_nav2_client_node>\n")

ament_auto_add_executable("vda5050_nav2_client" src/vda5050_nav2_client_main.cpp)

install(TARGETS vda5050_nav2_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  test/test_nodes/dummy_action_server.py
  test/test_nodes/nav2_simple_server.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_nav2_client.py TIMEOUT "30")
endif()

ament_auto_package(INSTALL_TO_SHARE)
