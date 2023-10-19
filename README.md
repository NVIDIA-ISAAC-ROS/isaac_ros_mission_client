# Isaac ROS Mission Client

VDA5050-compatible mission controller

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_mission_client/MD.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_mission_client/MD.png/" width="800px"/></a></div>

---

## Webinars

Learn more about missions by watching our on-demand webinar: [Build Connected Robots with NVIDIA Isaac Dispatch and Client](https://gateway.on24.com/wcc/experience/elitenvidiabrill/1407606/3998202/isaac-ros-webinar-series)

## Overview

[Isaac ROS Mission Client](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mission_client) provides the ROS 2 packages for Mission Client, which
communicates to a robot fleet management service. Mission Client
receives tasks and actions from the fleet management service and updates
its progress, state, and errors. Mission Client performs navigation
actions with [Nav2](https://github.com/ros-planning/navigation2) and
can be integrated with other ROS actions.

The communication to Mission Client is based on the [VDA5050
protocol](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md)
and uses MQTT fundamentals as the industry standard for a highly
efficient, scalable protocol for connecting devices over the Internet.

Mission Client is provided with a matching Mission Dispatch available
[here](https://github.com/NVIDIA-ISAAC/isaac_mission_dispatch), or
can be integrated with other fleet management systems using VDA5050 over
MQTT.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/index.html) to learn how to use this repository.

---

## Packages

* [Isaac ROS Mission Client](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html#overview)
  * [Usage](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html#usage)
  * [ROS Parameters](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html#ros-parameters)
* [Isaac ROS Scene Recorder](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_scene_recorder/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_scene_recorder/index.html#quickstart)
* [Isaac ROS VDA5050 NAV2 Client](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_vda5050_nav2_client/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_vda5050_nav2_client/index.html#overview)
  * [Usage](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_vda5050_nav2_client/index.html#usage)
  * [ROS Parameters](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_vda5050_nav2_client/index.html#ros-parameters)

## Quickstart

A Quickstart with Isaac Sim is [here](https://nvidia-isaac-ros.github.io/concepts/missions/isaac_ros_mission_client.html).

## Latest

Update 2023-10-18: Bugfixes, Mission Cancellation, initial pose
