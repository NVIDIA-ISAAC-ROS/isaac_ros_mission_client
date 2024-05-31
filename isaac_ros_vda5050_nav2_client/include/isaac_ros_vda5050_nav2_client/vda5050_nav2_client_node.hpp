// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * This repository implements data types and logic specified in the VDA5050
 * protocol, which is specified here
 * https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md
 */

#ifndef ISAAC_ROS_VDA5050_NAV2_CLIENT__VDA5050_NAV2_CLIENT_NODE_HPP_
#define ISAAC_ROS_VDA5050_NAV2_CLIENT__VDA5050_NAV2_CLIENT_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "vda5050_msgs/msg/error.hpp"
#include "vda5050_msgs/msg/order.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/agv_state.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "vda5050_msgs/msg/node_state.hpp"
#include "vda5050_msgs/msg/node.hpp"
#include "vda5050_msgs/msg/edge_state.hpp"
#include "vda5050_msgs/msg/instant_actions.hpp"

#include "isaac_ros_vda5050_nav2_client/action/mission_action.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "opennav_docking_msgs/action/undock_robot.hpp"

namespace isaac_ros
{
namespace mission_client
{

class Vda5050toNav2ClientNode : public rclcpp::Node
{
public:
  using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavThroughPoses = rclcpp_action::ClientGoalHandle<NavThroughPoses>;

  using VDAActionState = vda5050_msgs::msg::ActionState;

  // Add generic action handle
  using MissionAction =
    isaac_ros_vda5050_nav2_client::action::MissionAction;
  using GoalHandleMissionAction = rclcpp_action::ClientGoalHandle<MissionAction>;
  using DockAction =
    opennav_docking_msgs::action::DockRobot;
  using GoalHandleDockAction = rclcpp_action::ClientGoalHandle<DockAction>;

  using UndockAction =
    opennav_docking_msgs::action::UndockRobot;
  using GoalHandleUndockAction = rclcpp_action::ClientGoalHandle<UndockAction>;

  explicit Vda5050toNav2ClientNode(const rclcpp::NodeOptions & options);

  ~Vda5050toNav2ClientNode();

private:
  rclcpp_action::Client<NavThroughPoses>::SharedPtr client_ptr_;
  GoalHandleNavThroughPoses::SharedPtr nav_goal_handle_;

  rclcpp::Publisher<vda5050_msgs::msg::AGVState>::SharedPtr order_info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_id_pub_;
  rclcpp::Subscription<vda5050_msgs::msg::Order>::SharedPtr order_sub_;
  rclcpp::Subscription<vda5050_msgs::msg::InstantActions>::SharedPtr
    instant_actions_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr info_sub_;
  // Topic to get robot battery state
  std::string battery_state_topic_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_valid_error_sub_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr dock_detector_switch_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr amcl_get_parameters_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr localization_client_;

  // Service client to pause/resume docking server
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr docking_lifecycle_manager_client_;
  // Callback group for service reqeust callbacks
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // Executor for service reqeust callbacks
  rclcpp::executors::SingleThreadedExecutor executor_;
  // Publish a vda5050_msgs/AGVState based on the current state of the robot
  void PublishRobotState();
  // Timer callback function to publish a vda5050_msgs/AGVState message
  void StateTimerCallback();
  // Timer callback function to publish a std_msgs/String message containing the order_id
  void OrderIdCallback();
  // Return true if there is currently a running order, otherwise return false. This function is
  // NOT thread-safe, and need to be called in a function that has acquired the `order_mutex_`
  // lock.
  bool RunningOrder();
  // Function that creates the NavigateThroughPoses goal message for Nav2 and sends that goal
  // asynchronously
  void NavigateThroughPoses();
  // Vda5050 action handler: check actions in the current node and send requests to trigger
  // different servers based on the action type
  void Vda5050ActionsHandler(const vda5050_msgs::msg::Action & vda5050_action);
  // Update action status and description based on the action id
  void UpdateActionState(
    const size_t & action_state_idx, const std::string & status,
    const std::string & action_description = {});
  // Initialization the order state once received a new order
  void InitAGVState();
  // The callback function when the node receives a vda5050_msgs/Order message and processes it
  void Vda5050toNav2ClientCallback(const vda5050_msgs::msg::Order::ConstSharedPtr msg);
  // The callback function when the node receives a std_msgs/String info message and appends it to
  // the status message that gets published
  void InfoCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  // The callback function when the node receives a sensor_msgs/BatteryState message and processes
  // it into a VDA5050 BatteryState message
  void BatteryStateCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  // The callback function when the node receives a nav_msgs/Odometry message and appends it's
  // velotity to the status message's velocity that gets published
  void OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  // Goal response callback for NavigateThroughPoses goal message
  void NavPoseGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<NavThroughPoses>::SharedPtr & goal);
  // Feedback callback for NavigateThroughPoses goal message
  void NavPoseFeedbackCallback(
    GoalHandleNavThroughPoses::SharedPtr,
    const NavThroughPoses::Feedback::ConstSharedPtr);
  // Result callback for NavigateThroughPoses goal message
  void NavPoseResultCallback(const GoalHandleNavThroughPoses::WrappedResult & result);
  // Execute order message
  void execute_order();
  // Goal response callback for Action goal message
  template<typename ActionType>
  void ActionResponseCallback(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr & goal,
    const size_t & action_state_idx);
  // Result callback for Action goal message
  template<typename ResultType>
  void ActionResultCallback(
    const ResultType & result,
    const size_t & action_state_idx,
    const std::string & description = "");
  void CancelOrder();
  void UpdateActionStatebyId(const std::string & action_id, const std::string & action_status);
  void InstantActionsCallback(const vda5050_msgs::msg::InstantActions::ConstSharedPtr msg);
  // Handle teleop instant actions
  void TeleopActionHandler(const vda5050_msgs::msg::Action & teleop_action);
  // The callback function when the node receives an order error message.
  void OrderValidErrorCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  // The callback function for switch service result
  void SwitchServiceCallback(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future,
    const size_t action_state_idx,
    const DockAction::Goal & goal_msg,
    const rclcpp_action::Client<DockAction>::SendGoalOptions & send_goal_options);
  // Sync service request. Return request result
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr SendServiceRequest(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request);

  // Sync service request. Return result->success
  template<typename ServiceT>
  bool SendBoolRequest(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request);

  // The length of a period before the client publishes the robot state and mission status messages
  // (in seconds)
  double update_feedback_period_{};
  // The length of a period before the client publishes the order ID (in seconds)
  double update_order_id_period_{};
  // Setting this to true will allow the client to print out more descriptive logs such as sending
  // goals and goal completion
  bool verbose_;
  // If true, pause docking server in the contructor.
  bool docking_server_enabled_;
  // Read action names as parameter
  std::vector<std::string> action_server_names_;
  // odom topic to get robot velocity
  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  // Timer to call PublishRobotState periodically
  rclcpp::TimerBase::SharedPtr robot_state_timer_;
  // Timer to publish order_id to JsonInfoGenerator
  rclcpp::TimerBase::SharedPtr order_id_timer_;
  // Mutex to protect private member variables when they are read or written to
  std::mutex order_mutex_;

  // The value to populate in the first element of AGVState's field informations so the
  // mission dispatcher can correctly parse the messages from the JsonInfoGenerator
  const std::string info_messages_type_ = "user_info";
  vda5050_msgs::msg::Order::ConstSharedPtr current_order_;
  // Order information for feedback of the mission
  vda5050_msgs::msg::AGVState::SharedPtr agv_state_;
  // Cancel action
  vda5050_msgs::msg::Action::SharedPtr cancel_action_;
  // Reached current waypoint flag
  bool reached_waypoint_;
  // Pause order
  bool pause_order_;
  std::string pause_order_action_id_ = "";
  // Current node the robot is working on
  size_t current_node_{};
  // Then last node of navigate_through_poses action
  size_t next_stop_{};
  // Current action the robot is working on
  size_t current_node_action_{};
  // Current action state to update
  size_t current_action_state_{};
  // Supported action server and type
  std::unordered_map<std::string, std::string> action_server_map_;
  std::unordered_map<std::string, rclcpp_action::Client<MissionAction>::SharedPtr> action_clients_;
  rclcpp_action::Client<DockAction>::SharedPtr dock_client_;
  rclcpp_action::Client<UndockAction>::SharedPtr undock_client_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // True if current order is canceled
  bool current_order_canceled_;
};

}  // namespace mission_client
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VDA5050_NAV2_CLIENT__VDA5050_NAV2_CLIENT_NODE_HPP_
