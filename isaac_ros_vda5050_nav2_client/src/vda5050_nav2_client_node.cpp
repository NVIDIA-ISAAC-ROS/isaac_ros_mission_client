// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_vda5050_nav2_client/vda5050_nav2_client_node.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vda5050_msgs/msg/error_reference.hpp"
#include "vda5050_msgs/msg/info.hpp"

namespace isaac_ros
{
namespace mission_client
{

enum ErrorLevel { WARNING, FATAL };

Vda5050toNav2ClientNode::Vda5050toNav2ClientNode(
  const rclcpp::NodeOptions & options)
: Node("nav2_client_node", options),
  client_ptr_(
    rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose")),
  order_info_pub_(
    create_publisher<vda5050_msgs::msg::AGVState>("agv_state", 1)),
  order_id_pub_(create_publisher<std_msgs::msg::String>("order_id", 1)),
  order_sub_(create_subscription<vda5050_msgs::msg::Order>(
      "client_commands", rclcpp::SensorDataQoS(),
      std::bind(&Vda5050toNav2ClientNode::Vda5050toNav2ClientCallback, this,
      std::placeholders::_1))),
  instant_actions_sub_(
    create_subscription<vda5050_msgs::msg::InstantActions>(
      "instant_actions_commands", rclcpp::SensorDataQoS(),
      std::bind(&Vda5050toNav2ClientNode::InstantActionsCallback, this,
      std::placeholders::_1))),
  info_sub_(create_subscription<std_msgs::msg::String>(
      "info", rclcpp::SensorDataQoS(),
      std::bind(&Vda5050toNav2ClientNode::InfoCallback, this,
      std::placeholders::_1))),
  battery_state_sub_(create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", rclcpp::SensorDataQoS(),
      std::bind(&Vda5050toNav2ClientNode::BatteryStateCallback, this,
      std::placeholders::_1))),
  update_feedback_period_(
    declare_parameter<double>("update_feedback_period", 1.0)),
  update_order_id_period_(
    declare_parameter<double>("update_order_id_period", 0.5)),
  verbose_(declare_parameter<bool>("verbose", false)),
  action_server_names_(declare_parameter<std::vector<std::string>>(
      "action_server_names", std::vector<std::string>({}))),
  robot_state_timer_(create_wall_timer(
      std::chrono::duration<double>(update_feedback_period_),
      std::bind(&Vda5050toNav2ClientNode::StateTimerCallback, this))),
  order_id_timer_(create_wall_timer(
      std::chrono::duration<double>(update_order_id_period_),
      std::bind(&Vda5050toNav2ClientNode::OrderIdCallback, this))),
  agv_state_(std::make_shared<vda5050_msgs::msg::AGVState>()),
  cancel_action_(std::make_shared<vda5050_msgs::msg::Action>()),
  reached_waypoint_(false),
  current_node_(0),
  current_node_action_(0),
  current_action_state_(0)
{
  // tf_buffer and listener to get current robot positon
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create action clients
  for (auto & action_server_name : action_server_names_) {
    RCLCPP_DEBUG(
      get_logger(), "Action server name: %s",
      action_server_name.c_str());
    action_clients_[action_server_name] =
      rclcpp_action::create_client<MissionAction>(this, action_server_name);
    declare_parameter<std::vector<std::string>>(action_server_name);
    std::vector<std::string> action_types =
      get_parameter(action_server_name).as_string_array();
    for (std::string action_type : action_types) {
      RCLCPP_DEBUG(get_logger(), "Action type: %s", action_type.c_str());
      action_server_map_[action_type] = action_server_name;
    }
  }

  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Vda5050 Client Node initialized!");
  }
}

std::string CreateISO8601Timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  struct tm buf;
  gmtime_r(&itt, &buf);
  ss << std::put_time(&buf, "%FT%TZ");
  return ss.str();
}

vda5050_msgs::msg::ErrorReference CreateErrorReference(
  const std::string & reference_key, const std::string & reference_value)
{
  auto error_reference = vda5050_msgs::msg::ErrorReference();
  error_reference.reference_key = reference_key;
  error_reference.reference_value = reference_value;
  return error_reference;
}

vda5050_msgs::msg::Error CreateError(
  ErrorLevel level, const std::string & error_msg,
  const std::vector<vda5050_msgs::msg::ErrorReference> & error_refs,
  const std::string & error_type = "")
{
  auto error = vda5050_msgs::msg::Error();
  switch (level) {
    case ErrorLevel::WARNING:
      error.error_level = error.WARNING;
      break;
    case ErrorLevel::FATAL:
      error.error_level = error.FATAL;
      break;
    default:
      error.error_level = error.FATAL;
      break;
  }
  error.error_description = error_msg;
  error.error_references = error_refs;
  error.error_type = error_type;
  return error;
}

void Vda5050toNav2ClientNode::execute_order()
{
  std::unique_lock<std::mutex> lock(order_mutex_);
  if (!RunningOrder()) {
    return;
  }
  // Reached target waypoint of the current node
  if (reached_waypoint_) {
    // Check if current node has actions
    if (current_node_action_ < current_order_->nodes[current_node_].actions.size()) {
      // Check if action has been finished
      auto & action_status =
        agv_state_->action_states[current_action_state_].action_status;
      if (action_status == VDAActionState().FINISHED) {
        RCLCPP_INFO(
          get_logger(), "Finished action: %s",
          agv_state_->action_states[current_action_state_].action_id.c_str());
        current_node_action_++;
        current_action_state_++;
      } else if (action_status == VDAActionState().WAITING) {
        // Trigger action if it's still waiting
        Vda5050ActionsHandler(
          current_order_->nodes[current_node_].actions[current_node_action_]);
        action_status = VDAActionState().INITIALIZING;
        agv_state_->driving = false;
      }
    } else {
      // Move to next node if no action
      current_node_action_ = 0;
      current_node_++;
      reached_waypoint_ = false;
      lock.unlock();
      NavigateToPose();
    }
  }
}

void Vda5050toNav2ClientNode::PublishRobotState()
{
  agv_state_->timestamp = CreateISO8601Timestamp();
  order_info_pub_->publish(*agv_state_);
}

void Vda5050toNav2ClientNode::StateTimerCallback()
{
  // Get robot position
  try {
    // Find the latest map_T_base_link transform
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
      "map", "base_link",
      tf2::TimePointZero);
    agv_state_->agv_position.x = t.transform.translation.x;
    agv_state_->agv_position.y = t.transform.translation.y;
    // Calculate robot orientation
    tf2::Quaternion quaternion;
    tf2::fromMsg(t.transform.rotation, quaternion);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getEulerYPR(yaw, pitch, roll);
    agv_state_->agv_position.theta = yaw;
    agv_state_->agv_position.position_initialized = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not get robot position: %s", ex.what());
  }
  if (cancel_action_) {
    CancelOrder();
    return;
  }
  execute_order();
  PublishRobotState();
}

void Vda5050toNav2ClientNode::OrderIdCallback()
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (current_order_) {
    auto order_id_msg = std_msgs::msg::String();
    order_id_msg.data = current_order_->order_id;
    order_id_pub_->publish(order_id_msg);
  }
}

bool Vda5050toNav2ClientNode::RunningOrder()
{
  // If client has not received an order
  if (!current_order_) {
    return false;
  }
  // If client has completed the currently assigned order
  if (current_node_ >= current_order_->nodes.size()) {
    return false;
  }
  // If the order has failed
  if (!agv_state_->errors.empty()) {
    return false;
  }

  return true;
}

void Vda5050toNav2ClientNode::NavigateToPose()
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (!client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(get_logger(), "Navigation server not available");
    return;
  }

  if (current_node_ >= current_order_->nodes.size()) {
    RCLCPP_INFO(get_logger(), "Navigation completed");
    return;
  }
  auto goal_msg = NavToPose::Goal();
  auto pose_stamped = geometry_msgs::msg::PoseStamped();

  pose_stamped.pose.position.x =
    current_order_->nodes[current_node_].node_position.x;
  pose_stamped.pose.position.y =
    current_order_->nodes[current_node_].node_position.y;
  pose_stamped.pose.position.z = 0.0;
  pose_stamped.header.frame_id = "map";

  // Convert theta into a quaternion for goal pose's orientation
  tf2::Quaternion orientation;
  orientation.setRPY(
    0, 0,
    current_order_->nodes[current_node_].node_position.theta);
  pose_stamped.pose.orientation = tf2::toMsg(orientation);
  pose_stamped.header.stamp = rclcpp::Clock().now();
  goal_msg.pose = pose_stamped;

  auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(
    &Vda5050toNav2ClientNode::NavPoseGoalResponseCallback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &Vda5050toNav2ClientNode::NavPoseFeedbackCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &Vda5050toNav2ClientNode::NavPoseResultCallback, this,
    std::placeholders::_1);
  if (verbose_) {
    RCLCPP_INFO(
      get_logger(), "Sending goal for (x: %f, y: %f, t: %f)",
      current_order_->nodes[current_node_].node_position.x,
      current_order_->nodes[current_node_].node_position.y,
      current_order_->nodes[current_node_].node_position.theta);
  }
  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void Vda5050toNav2ClientNode::Vda5050ActionsHandler(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  // Check if action server exists
  if (action_server_map_.count(vda5050_action.action_type) == 0) {
    RCLCPP_DEBUG(
      this->get_logger(), "Action type %s is not supported",
      vda5050_action.action_type.c_str());
    UpdateActionState(
      current_action_state_, VDAActionState().FAILED,
      "This action is not supported");
  }
  // Trigger action
  size_t action_state_idx = current_action_state_;
  auto send_goal_options =
    rclcpp_action::Client<MissionAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, action_state_idx](
    const rclcpp_action::ClientGoalHandle<MissionAction>::SharedPtr &
    goal) {MissionActionResponseCallback(goal, action_state_idx);};
  send_goal_options.result_callback =
    [this,
      action_state_idx](const GoalHandleMissionAction::WrappedResult & result) {
      MissionActionResultCallback(result, action_state_idx);
    };
  auto goal_msg = MissionAction::Goal();
  goal_msg.keys.push_back("action_type");
  goal_msg.values.push_back(vda5050_action.action_type);
  for (const auto & action_param : vda5050_action.action_parameters) {
    goal_msg.keys.push_back(action_param.key);
    goal_msg.values.push_back(action_param.value);
  }
  auto client_ptr =
    action_clients_[action_server_map_[vda5050_action.action_type]];
  client_ptr->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(
    this->get_logger(), "Send %s request",
    vda5050_action.action_type.c_str());
}

void Vda5050toNav2ClientNode::InitAGVState()
{
  RCLCPP_DEBUG(this->get_logger(), "Initialization order information");
  agv_state_.reset(new vda5050_msgs::msg::AGVState());
  agv_state_->order_id = current_order_->order_id;
  agv_state_->last_node_id = current_order_->nodes[0].node_id;
  agv_state_->driving = false;
  agv_state_->informations.resize(1);
  for (const auto & vda5050_node : current_order_->nodes) {
    auto node_state = vda5050_msgs::msg::NodeState();
    node_state.node_id = vda5050_node.node_id;
    node_state.sequence_id = vda5050_node.sequence_id;
    node_state.node_description = vda5050_node.node_description;
    node_state.released = vda5050_node.released;
    agv_state_->node_states.push_back(node_state);
    for (const auto & vda5050_action : vda5050_node.actions) {
      auto actionState = VDAActionState();
      actionState.action_id = vda5050_action.action_id;
      actionState.action_status =
        VDAActionState().WAITING;
      agv_state_->action_states.push_back(actionState);
    }
  }
  agv_state_->node_states.erase(
    agv_state_->node_states.begin());
  reached_waypoint_ = true;
  RCLCPP_DEBUG(
    this->get_logger(),
    "Obtained %ld node states and %ld action states",
    agv_state_->node_states.size(),
    agv_state_->action_states.size());
}

void Vda5050toNav2ClientNode::Vda5050toNav2ClientCallback(
  const vda5050_msgs::msg::Order::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(
    get_logger(), "Order with order_id %s received",
    msg->order_id.c_str());
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (!RunningOrder() && !msg->nodes.empty()) {
    current_order_ = msg;
    current_node_ = 0;
    current_node_action_ = 0, current_action_state_ = 0;
    InitAGVState();
  }
}

void Vda5050toNav2ClientNode::BatteryStateCallback(
  const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  agv_state_->battery_state.battery_charge = msg->percentage * 100;
  agv_state_->battery_state.battery_voltage = msg->voltage;
  // POWER_SUPPLY_STATUS_CHARGING = 1
  agv_state_->battery_state.charging = (msg->power_supply_status == 1) ? true : false;

  // battery_health and reach are currently not supported
  agv_state_->battery_state.battery_health = 0;
  agv_state_->battery_state.reach = 0;
}

void Vda5050toNav2ClientNode::InstantActionsCallback(
  const vda5050_msgs::msg::InstantActions::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Instant actions with header_id %d received",
    msg->header_id);
  std::lock_guard<std::mutex> lock(order_mutex_);

  for (const vda5050_msgs::msg::Action & action : msg->instant_actions) {
    RCLCPP_INFO(
      this->get_logger(), "Processing action %s of type %s.",
      action.action_id.c_str(), action.action_type.c_str());
    // Add action to action_states
    auto action_state = VDAActionState();
    action_state.action_id = action.action_id;
    action_state.action_description = action.action_description;
    action_state.action_status = VDAActionState().WAITING;
    action_state.action_type = action.action_type;
    agv_state_->action_states.push_back(action_state);
    if (action.action_type == "cancelOrder") {
      cancel_action_ = std::make_shared<vda5050_msgs::msg::Action>(action);
    }
  }
}

void Vda5050toNav2ClientNode::CancelOrder()
{
  /*
  Process cancel order as per VDA5050 specification:
  https://github.com/VDA5050/VDA5050/blob/main/assets/Figure9.png
  */
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (!RunningOrder()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "cancelOrder action request failed. There is no active order running.");
    UpdateActionStatebyId(cancel_action_->action_id, VDAActionState().FAILED);
    // The AGV must report a “noOrderToCancel” error with the errorLevel set to
    // warning. The actionId of the instantAction must be passed as an errorReference.
    auto error = vda5050_msgs::msg::Error();
    error = CreateError(
      ErrorLevel::WARNING, "There is no active order running.",
      {CreateErrorReference("action_id", cancel_action_->action_id)});
    agv_state_->errors.push_back(error);
    PublishRobotState();
    cancel_action_.reset();
    return;
  }
  // Set cancelOrder action state to running
  UpdateActionStatebyId(cancel_action_->action_id, VDAActionState().RUNNING);
  // Set waiting actions to failed
  for (auto & action_state : agv_state_->action_states) {
    if (action_state.action_status ==
      VDAActionState().WAITING)
    {
      UpdateActionStatebyId(action_state.action_id, VDAActionState().FAILED);
    }
  }
  // Interrupt any running action
  for (auto it = action_clients_.begin(); it != action_clients_.end(); ++it) {
    it->second->async_cancel_all_goals();
  }
  // vda_action_goal_handles_.clear();
  // Interrupt any running navigation goal
  if (nav_goal_handle_) {
    client_ptr_->async_cancel_goal(nav_goal_handle_);
  }
  // Once all the VDA actions and navigation goal requests have finished,
  // the cancel order will be mark as finished
  UpdateActionStatebyId(cancel_action_->action_id, VDAActionState().FINISHED);
  cancel_action_.reset();
  RCLCPP_INFO(
    this->get_logger(), "Finished executing cancelOrder.");
}

void Vda5050toNav2ClientNode::UpdateActionStatebyId(
  const std::string & action_id,
  const std::string & action_status)
{
  /*
  Update action status on the current state given action's ID.
  */

  // Get action state from current state
  auto it = std::find_if(
    agv_state_->action_states.begin(),
    agv_state_->action_states.end(),
    [&action_id](const VDAActionState & action) {
      return action.action_id == action_id;
    });

  if (it == agv_state_->action_states.end()) {
    RCLCPP_ERROR(
      this->get_logger(), "Error while processing action state. Couldn't find action with id: %s",
      action_id.c_str());
    auto error = vda5050_msgs::msg::Error();
    error = CreateError(
      ErrorLevel::WARNING, "VDA5050 action with id " + action_id + " not found.",
      {CreateErrorReference("action_id", action_id)}, "noOrderToCancel");
    agv_state_->errors.push_back(error);
  } else {
    // If found, update action status
    RCLCPP_INFO(
      this->get_logger(), "Update action %s with state %s ",
      action_id.c_str(), action_status.c_str());
    it->action_status = action_status;
  }
  PublishRobotState();
}

void Vda5050toNav2ClientNode::UpdateActionState(
  const size_t & action_state_idx, const std::string & status,
  const std::string & action_description)
{
  if (action_state_idx >= agv_state_->action_states.size()) {
    return;
  }
  agv_state_->action_states[action_state_idx].action_status = status;
  agv_state_->action_states[action_state_idx].result_description =
    action_description;
  if (status == VDAActionState().FAILED) {
    auto error = vda5050_msgs::msg::Error();
    error = CreateError(
      ErrorLevel::WARNING, "Action Error",
      {CreateErrorReference(
          "node_id",
          current_order_->nodes[current_node_].node_id),
        CreateErrorReference(
          "action_id",
          agv_state_->action_states[action_state_idx].action_id)});
    agv_state_->errors.push_back(error);
  }
  PublishRobotState();
}

void Vda5050toNav2ClientNode::MissionActionResponseCallback(
  const rclcpp_action::ClientGoalHandle<MissionAction>::SharedPtr & goal,
  const size_t & action_state_idx)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  auto action_id =
    agv_state_->action_states[action_state_idx].action_id;
  if (!goal) {
    RCLCPP_ERROR(
      this->get_logger(), "MissionAction %s was rejected",
      action_id.c_str());
    UpdateActionState(action_state_idx, VDAActionState().FAILED, "Action was rejected");
  } else {
    // vda_action_goal_handles_.push_back(goal);
    RCLCPP_INFO(
      this->get_logger(), "MissionAction %s was accepted",
      action_id.c_str());
    UpdateActionState(action_state_idx, VDAActionState().RUNNING, "Action was rejected");
  }
}

void Vda5050toNav2ClientNode::MissionActionResultCallback(
  const GoalHandleMissionAction::WrappedResult & result,
  const size_t & action_state_idx)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  auto action_id =
    agv_state_->action_states[action_state_idx].action_id;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result.result->success) {
        RCLCPP_INFO(
          get_logger(), "MissionAction %s was succeeded",
          action_id.c_str());
        UpdateActionState(
          action_state_idx, VDAActionState().FINISHED,
          result.result->result_description);
      } else {
        RCLCPP_ERROR(
          get_logger(), "MissionAction %s was failed",
          action_id.c_str());
        UpdateActionState(
          action_state_idx, VDAActionState().FAILED,
          result.result->result_description);
      }
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        get_logger(), "MissionAction %s was aborted",
        action_id.c_str());
      UpdateActionState(action_state_idx, VDAActionState().FAILED, "Action is aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(
        get_logger(), "MissionAction %s was canceled",
        action_id.c_str());
      UpdateActionState(action_state_idx, VDAActionState().FAILED, "Action is canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED,
        "Unknown action result code");
      return;
  }
}

void Vda5050toNav2ClientNode::InfoCallback(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (agv_state_->informations.size()) {
    auto & info = agv_state_->informations.at(0);
    info.info_type = info_messages_type_;
    info.info_description = msg->data;
    info.info_level = info.INFO;
  }
}

void Vda5050toNav2ClientNode::NavPoseGoalResponseCallback(
  const rclcpp_action::ClientGoalHandle<
    nav2_msgs::action::NavigateToPose>::SharedPtr & goal)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (!goal) {
    RCLCPP_WARN(get_logger(), "Goal was rejected by server");
    auto error = vda5050_msgs::msg::Error();
    error = CreateError(
      ErrorLevel::FATAL, "Goal rejected",
      {CreateErrorReference(
          "node_id",
          current_order_->nodes[current_node_].node_id)});
    agv_state_->errors.push_back(error);
    nav_goal_handle_.reset();
    PublishRobotState();
  } else {
    nav_goal_handle_ = goal;
    agv_state_->driving = true;
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }
}

void Vda5050toNav2ClientNode::NavPoseFeedbackCallback(
  GoalHandleNavToPose::SharedPtr,
  const NavToPose::Feedback::ConstSharedPtr)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  agv_state_->driving = true;
}

void Vda5050toNav2ClientNode::NavPoseResultCallback(
  const GoalHandleNavToPose::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  auto error = vda5050_msgs::msg::Error();
  nav_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        get_logger(), "Goal was aborted at node_id %s",
        agv_state_->last_node_id.c_str());
      current_node_ = 1;
      error = CreateError(
        ErrorLevel::FATAL, "Goal aborted",
        {CreateErrorReference(
            "node_id",
            current_order_->nodes[current_node_].node_id)});
      agv_state_->errors.push_back(error);
      PublishRobotState();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(
        get_logger(), "Goal was canceled at node_id %s",
        agv_state_->last_node_id.c_str());
      current_node_ = 1;
      error = CreateError(
        ErrorLevel::WARNING, "Goal canceled",
        {CreateErrorReference(
            "node_id",
            current_order_->nodes[current_node_].node_id)});
      agv_state_->errors.push_back(error);
      PublishRobotState();
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
  }
  // Logic if navigation was successful
  RCLCPP_INFO(get_logger(), "Reached order node: %ld", current_node_);
  if (!agv_state_->node_states.empty()) {
    agv_state_->node_states.erase(
      agv_state_->node_states.begin());
  }
  agv_state_->last_node_id =
    current_order_->nodes[current_node_].node_id;
  agv_state_->last_node_sequence_id =
    current_order_->nodes[current_node_].sequence_id;
  reached_waypoint_ = true;
}

Vda5050toNav2ClientNode::~Vda5050toNav2ClientNode() = default;

}  // namespace mission_client
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  isaac_ros::mission_client::Vda5050toNav2ClientNode)
