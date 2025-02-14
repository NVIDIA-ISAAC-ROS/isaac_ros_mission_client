# Isaac ROS Mission Client

VDA5050-compatible mission controller

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_mission_client/MD.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_mission_client/MD.png/" width="800px"/></a></div>

---

## Webinars

Learn more about missions by watching our on-demand webinar: [Build Connected Robots with NVIDIA Isaac Dispatch and Client](https://gateway.on24.com/wcc/experience/elitenvidiabrill/1407606/3998202/isaac-ros-webinar-series)

## Overview

Isaac ROS Mission Client provides the ROS 2 packages for Mission Client, which
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

* [`isaac_ros_mission_client`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html#quickstart)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_mission_client/index.html#api)
* [`isaac_ros_scene_recorder`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_scene_recorder/index.html)
  * [Usage](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mission_client/isaac_ros_scene_recorder/index.html#usage)

## Quickstart

A Quickstart with Isaac Sim is [here](https://nvidia-isaac-ros.github.io/concepts/missions/isaac_ros_mission_client.html).

## Latest

Update 2024-12-10: Added actions to support object pick and place

## Contributors

- Add velocities to the status message [(#10)]([https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mission_client/pull/10](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mission_client/pull/10))
  Contributors: [Burak Guler]([https://github.com/gulerburak](https://github.com/gulerburak))
