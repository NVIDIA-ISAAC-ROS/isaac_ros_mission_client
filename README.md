# Isaac ROS Mission Client

<div align="center"><img alt="Architecture" src="resources/architecture.png" width="650px"/></div>

## Overview

This repository provides the ROS2 packages for Mission Client, which communicates to a robot fleet management service. Mission Client receives tasks and actions from the fleet management service and updates its progress, state, and errors. Mission Client performs navigation actions with [Nav2](https://github.com/ros-planning/navigation2) and can be integrated with other ROS actions.

The communication to Mission Client is based on the [VDA5050 protocol](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md) and uses MQTT fundamentals as the industry standard for a highly efficient, scalable protocol for connecting devices over the Internet.

Mission Client is provided with a matching Mission Dispatch available [here](https://github.com/NVIDIA-ISAAC/isaac_mission_dispatch), or can be integrated with other fleet management systems using VDA5050 over MQTT.

## Table of Contents

- [Isaac ROS Mission Client](#isaac-ros-mission-client)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
  - [Tutorial with Isaac Sim](#tutorial-with-isaac-sim)
    - [Tutorial Walkthrough](#tutorial-walkthrough)
    - [Customize your Dev Environment](#customize-your-dev-environment)
  - [Package Reference](#package-reference)
    - [isaac_ros_mission_client](#isaac_ros_mission_client)
      - [Usage](#usage)
      - [ROS Parameters](#ros-parameters)
    - [isaac_ros_vda5050_nav2_client](#isaac_ros_vda5050_nav2_client)
      - [Usage](#usage-1)
      - [ROS Parameters](#ros-parameters-1)
    - [isaac_ros_scene_recorder](#isaac_ros_scene_recorder)
      - [Usage](#usage-2)
  - [Troubleshooting](#troubleshooting)
    - [Isaac ROS Troubleshooting](#isaac-ros-troubleshooting)
  - [Updates](#updates)

## Latest Update

Update 2022-10-19: Initial release

## Supported Platforms

This package is designed and tested to be compatible with ROS2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system.

| Platform | Hardware                                                                                                                                                                                                 | Software                                                       | Notes                                                                                                                                                                                   |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.0.2](https://developer.nvidia.com/embedded/jetpack) | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | x86 CPU                                                                                                                                                                                                  | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/)            |
> **Note:** Mission Client does not require GPU.

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note:** All Isaac ROS quick-start guides, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Tutorial with Isaac Sim

<div align="center"><img alt="Tutorial Visual" src="resources/tutorial.png" width="650px"/></div>
This tutorial will walk you through steps to send missions from [Mission Dispatch](https://github.com/NVIDIA-ISAAC/isaac_mission_dispatch) to Isaac ROS Mission Client using the [VDA5050 protocol](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md). The Mission Client will be attached to an Isaac Sim environment running the [Nav2 stack](https://navigation.ros.org/). The steps from a high-level are as follows:

1. Set up Isaac Sim
2. Run Mission Client
3. Run Mission Dispatch
4. Send a position setup to Nav2 using Mission Dispatch and Mission Client

### Tutorial Walkthrough

1. Install and launch Isaac Sim following the steps in the [Isaac ROS Isaac Sim Setup Guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/isaac-sim-sil-setup.md)
2. Open the Isaac ROS Common USD scene (using the **content** window) located at:

   `omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd`

   Wait for the scene to load completely.
   > **Note:** To use a different server, replace `localhost` with `<your_nucleus_server>`
3. Press **Play** to start publishing data from Isaac Sim.

   <div align="center"><img src="resources/Isaac_sim_init_screen.png" width="800px"/></div>

4. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).

5. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src`.

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mission_client
    ```

6. Start MQTT broker

    The MQTT broker is used for communication between the Mission Dispatch and the robots. There are many ways to run an MQTT broker, including as a system daemon, a standalone application, or a Docker container. Here we use `mosquitto` as our MQTT broker. Start the `mosquitto` broker by running the following:
    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_mission_client
    docker run -it --network host -v ${PWD}/utils/mosquitto.sh:/mosquitto.sh -d eclipse-mosquitto:latest sh mosquitto.sh 1883 9001
    ```

7. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

8. Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

9. (Optional) Run tests to verify complete and correct installation:

    ```bash
    colcon test --executor sequential
    ```

10. Run the following launch files to spin up ``mission_client`` and Nav2:

    ```bash
    ros2 launch isaac_ros_vda5050_nav2_client_bringup isaac_ros_vda5050_nav2_client.launch.py
    ```
11. Start [Mission Dispatch](https://github.com/NVIDIA-ISAAC/isaac_mission_dispatch) with Docker.
- Postgres database

    Set the following environment variable:

    ```bash
    export POSTGRES_PASSWORD=<Any password>
    ```

    Start the Postgres database by running the following:

    ```bash
    docker run --rm --name postgres \
      --network host \
      -p 5432:5432 \
      -e POSTGRES_USER=postgres \
      -e POSTGRES_PASSWORD \
      -e POSTGRES_DB=mission \
      -d postgres:14.5
    ```
- Launch the Mission Database microservice:

    Start the API and database server with the official Docker container.
    ```bash
    docker run -it --network host nvcr.io/nvidia/isaac/mission-database:2022.10.17_de4892b

    # To see what configuration options are, run
    # docker run -it --network host nvcr.io/nvidia/isaac/mission-database:2022.10.17_de4892b --help
    # For example, if you want to change the port for the user API from the default 5000 to 5002, add `--port 5002` configuration option in the command.
    ```
- Launch the Mission Dispatch microservice:

  Start the Mission Dispatch server with the official Docker container.
    ```bash
    docker run -it --network host nvcr.io/nvidia/isaac/mission-dispatch:2022.10.17_de4892b
    # To see what configuration options are, add --help option after the command.
    ```

  > **Note:** Read [this tutorial](https://github.com/NVIDIA-ISAAC/isaac_mission_dispatch#getting-started-with-deployment-recommended) for more deployment options for Mission Dispatch.

12. Open `http://localhost:5000/docs` in a web browser.
13. Use the `POST /robots` endpoint to create robot objects. See the video below for exact steps.
14. Get the status of the robots using the `GET /robots`
    endpoint. If the robots are connected, the state should reflect the actual position of the robots.

    https://user-images.githubusercontent.com/77975110/196787009-4e19199c-deb7-4bee-9f21-bda06b4791b0.mp4
    > **Note:** When using the interactive documentation page, the default value for the the robot object
    `name` in the `spec` is 'string', so make sure to change it from 'string' to another name that has
    more meaning, like 'carter01'. Make sure to delete the `prefix` entry as shown in the video.

15. Send a mission to the robot using the `POST /missions` endpoint. See the video below for exact steps.

    https://user-images.githubusercontent.com/77975110/196791774-060749d6-4274-4d6d-8380-cff6207bfed6.mp4

    > **Note:** By default, the value for the `robot` is 'string', so make sure to change it to
    the `name` you used for one of the robot objects you created earlier. For example, if you set the `name` of the robot
    object to 'carter01', use that to fill in the `robot` field for the mission. Also make sure to delete the `prefix`,
    `selector`, `sequence` and `action` entries as shown in the video.

    >**Note:** Mission Client cannot cancel or update a running mission and will disregard incoming missions if it is busy.

16. Go back to the Isaac Sim screen. You should see the robot move to the set goal position, as shown below.

    <div align="center"><img alt="Isaac ROS Mission Client Sample Isaac SimOutput" src="resources/sim_output.gif" width="800px"/></div>

### Customize your Dev Environment

To customize your development environment, reference [this guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/modify-dockerfile.md).

## Package Reference

### isaac_ros_mission_client

This launch script under the launch directory brings up the MQTT bridge nodes and the VDA5050 Nav2 Client node.

#### Usage

```bash
ros2 launch isaac_ros_vda5050_nav2_client_bringup isaac_ros_vda5050_client.launch.py ros_namespace:=<"namespace for ros graph"> mqtt_host_name:=<"mqtt_host_name"> mqtt_transport:=<"mqtt_transport"> mqtt_pub_topic:=<"mqtt_pub_topic"> ros_subscriber_type:=<"ros_subscriber_type"> ros_to_mqtt_name:=<"ros_to_mqtt_name"> mqtt_sub_topic:=<"mqtt_sub_topic"> ros_publisher_type:=<"ros_publisher_type"> mqtt_to_ros_name:=<"mqtt_to_ros_name">
```

#### ROS Parameters

| ROS Parameter         | Type     | Default                    | Description                                                                                                                                                |
| --------------------- | -------- | -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ros_namespace`       | `string` | `""(Empty string)`         | The ROS namespace used for the ROS graph. <br/> e.g. `carter`                                                                                              |
| `mqtt_host_name`      | `string` | `localhost`                | The MQTT broker IP to connect to.  <br/> e.g. `192.168.25.32`                                                                                              |
| `mqtt_port`           | `string` | `1883`                     | The MQTT broker port                                                                                                                                       |
| `mqtt_transport`      | `string` | `tcp`                      | The protocol to use for sending MQTT messages (either `tcp` or `websockets`)                                                                               |
| `mqtt_ws_path`        | `string` | `''`                       | The path for the websocket if the protocol set in `mqtt_transport` is `websockets`                                                                         |
| `mqtt_pub_topic`      | `string` | `/uagv/v1/carter01/state`  | The MQTT topic to publish MQTT messages                                                                                                                    |
| `mqtt_sub_topic`      | `string` | `/uagv/v1/carter01/order`  | The MQTT topic to subscribe for incoming MQTT messages                                                                                                     |
| `ros_publisher_type`  | `string` | `vda5050_msgs/Order`       | The ROS message type to convert incoming MQTT messages to                                                                                                  |
| `ros_subscriber_type` | `string` | `vda5050_msgs/AGVState`    | The ROS message type that outgoing MQTT message are converted from                                                                                         |
| `ros_to_mqtt_name`    | `string` | `Carter01_RosToMqttBridge` | The MQTT client name for the RosToMqtt Node                                                                                                                |
| `mqtt_to_ros_name`    | `string` | `Carter01_MqttToRosBridge` | The MQTT client name for the MqttToRos Node                                                                                                                |
| `retry_forever`       | `bool`   | `true`                     | Retry connecting forever if connection to MQTT message broker is not established                                                                           |
| `reconnect_period`    | `int`    | `5`                        | The period of time to wait before retrying to connect to MQTT message broker (in seconds)                                                                  |
| `num_retries`         | `int`    | `10`                       | The number of reconnection retries to connect to the MQTT message broker before giving up. This setting is only valid if `retry_forever` is set to `false` |
| `ros_recorder`        | `bool`   | `false`                    | Launches [isaac_ros_scene_recorder](./isaac_ros_scene_recorder/) if set to `true`                                                                          |

### isaac_ros_vda5050_nav2_client

Besides launching `isaac_ros_vda5050_client.py`, this launch script also launches the `nav2_bringup` launch script (through the Nav2 ``bringup_launch.py`` script) and JsonInfoGeneratorNode in the `isaac_ros_json_info_generator` package (the README for this package is in the `isaac_ros_json_info_generator` folder). This script has the following parameters in addition to the parameters listed above.

#### Usage

```bash
ros2 launch isaac_ros_vda5050_nav2_client_bringup isaac_ros_vda5050_nav2_client.launch.py ros_namespace:=<"namespace for ros graph"> mqtt_host_name:=<"mqtt_host_name"> mqtt_pub_topic:=<"mqtt_pub_topic"> ros_subscriber_type:=<"ros_subscriber_type"> ros_to_mqtt_name:=<"ros_to_mqtt_name"> mqtt_sub_topic:=<"mqtt_sub_topic"> ros_publisher_type:=<"ros_publisher_type"> mqtt_to_ros_name:=<"mqtt_to_ros_name">
```

#### ROS Parameters

| ROS Parameter                | Type     | Default                                                                                                                                          | Description                                                     |
| ---------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------- |
| `use_namespace`              | `bool`   | `false`                                                                                                                                          | Whether to apply a namespace to the navigation stack            |
| `use_composition`            | `bool`   | `false`                                                                                                                                          | Whether to use composed Nav2 bringup                            |
| `use_sim_time`               | `bool`   | `false`                                                                                                                                          | Whether to use simulation (Omniverse Isaac Sim) clock           |
| `init_pose_x`                | `float`  | `0.0`                                                                                                                                            | The initial position X coordinate                               |
| `init_pose_y`                | `float`  | `0.0`                                                                                                                                            | The initial position Y coordinate                               |
| `init_pose_z`                | `float`  | `0.0`                                                                                                                                            | The initial position Z coordinate                               |
| `map`                        | `string` | `/home/$USER/workspaces/isaac_ros-dev/src/isaac_ros_mission_client/isaac_ros_vda5050_nav2_client_bringup/maps/carter_warehouse_navigation.yaml`  | The full path to the occupancy map file to load                 |
| `nav_params_file`            | `string` | `/home/$USER/workspaces/isaac_ros-dev/src/isaac_ros_mission_client/isaac_ros_vda5050_nav2_client_bringup/config/carter_navigation_params.yaml`   | The full path to the navigation parameter file to load          |
| `info_generator_params_file` | `string` | `/home/$USER/workspaces/isaac_ros-dev/src/isaac_ros_mission_client/isaac_ros_vda5050_nav2_client_bringup/config/json_info_generator_params.yaml` | The full path to the JSON Info Generator parameter file to load |
| `launch_rviz`                | `bool`   | `false`                                                                                                                                          | Launches RViz if ``true``                                       |

### isaac_ros_scene_recorder

This node will record data published to given topics and save them as ``rosbag``.

#### Usage

To run Mission Client with the recorder, set the `ros_recorder` launch file parameter to ``true``.

  ```bash
  ros2 launch isaac_ros_vda5050_nav2_client_bringup isaac_ros_vda5050_nav2_client.launch.py ros_recorder:=true
  ```

  or

  ```bash
  ros2 launch isaac_ros_vda5050_nav2_client_bringup isaac_ros_vda5050_client.launch.py ros_recorder:=true
  ```

To start the recorder, post a mission from mission dispatch. Here is an example mission:

  ```bash
  {
    "robot": "carter01",
    "mission_tree": [
      {
        "name": "string",
        "parent": "root",
        "action": {
          "action_type": "start_recording",
          "action_parameters": {"path": "/tmp/data", "topics": "/rgb_left", "time":3}
        }
      }
    ],
    "timeout": 300,
    "deadline": "2022-10-07T00:21:31.112Z",
    "needs_canceled": false,
    "name": "mission01"
  }
  ```

The `action_type` should be `start_recording` and the parameters `path, topics, time` are required. The recorder will save data published by `topics` in folder `path` for `time` seconds.

To stop the recorder, post a mission with action of type `stop_recording` from Mission Dispatch. Here is an example:

  ```bash
  {
    "robot": "carter01",
    "mission_tree": [
      {
        "name": "string",
        "parent": "root",
        "action": {
          "action_type": "stop_recording"
        }
      }
    ],
    "timeout": 300,
    "deadline": "2022-10-07T00:21:31.112Z",
    "needs_canceled": false,
    "name": "mission02"
  }
  ```

## Troubleshooting

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

## Updates

| Date       | Changes         |
| ---------- | --------------- |
| 2022-10-19 | Initial release |
