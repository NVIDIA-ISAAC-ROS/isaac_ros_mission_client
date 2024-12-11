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

#include "isaac_ros_vda5050_nav2_client/vda5050_nav2_client_node.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
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
namespace
{
constexpr char kValidationError[] = "validationError";
constexpr char kOrderUpdateError[] = "orderUpdateError";

constexpr char kRobotDockAction[] = "dock_robot";
constexpr char kRobotUndockAction[] = "undock_robot";
constexpr char kDockType[] = "dock_type";
constexpr char kDockPose[] = "dock_pose";
constexpr char kTriggerGlobalLocalization[] = "trigger_global_localization";

constexpr char kGetObjectsAction[] = "get_objects";
constexpr char kPickAndPlaceAction[] = "pick_and_place";

constexpr char kObjectId[] = "object_id";
constexpr char kClassId[] = "class_id";
constexpr char kPlacePose[] = "place_pose";

constexpr char kClearObjects[] = "clear_objects";
constexpr char kObjectIds[] = "object_ids";


std::vector<std::string> split(const std::string & s, char delimiter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// Check the format of dock_pose parameter
bool isValidDockPose(const std::string & str)
{
  std::regex pattern(R"([-+]?\d*\.?\d+,\s*[-+]?\d*\.?\d+,\s*[-+]?\d*\.?\d+)");
  return std::regex_match(str, pattern);
}

// Convert string to lowercase and return true is the lowercase string is 'true'
bool stringToBool(const std::string & s)
{
  std::string lower_str = s;
  std::transform(
    lower_str.begin(), lower_str.end(), lower_str.begin(),
    [](unsigned char c) {return std::tolower(c);});
  return lower_str == "true" || lower_str == "1";
}

std::string jsonFromDetection2D(const vision_msgs::msg::Detection2D & detection)
{
  std::ostringstream json_result;
  if (detection.results.size() == 0) {
    return "";
  }
  json_result << std::fixed << std::setprecision(3);  // Setting precision for double values
  json_result << "  \"bbox2d\": {\n"
              << "    \"center\": {\n"
              << "      \"x\": " << detection.bbox.center.position.x << ",\n"
              << "      \"y\": " << detection.bbox.center.position.y << ",\n"
              << "      \"theta\": " << detection.bbox.center.theta << "\n"
              << "    },\n"
              << "    \"size_x\": " << detection.bbox.size_x << ",\n"
              << "    \"size_y\": " << detection.bbox.size_y << "\n"
              << "  }";
  return json_result.str();
}

std::string jsonFromDetection3D(const vision_msgs::msg::Detection3D & detection)
{
  std::ostringstream json_result;
  if (detection.results.size() == 0) {
    return "";
  }
  json_result << std::fixed << std::setprecision(3);  // Setting precision for double values
  json_result << "  \"bbox3d\": {\n"
              << "    \"center\": {\n"
              << "      \"position\": {\n"
              << "        \"x\": " << detection.bbox.center.position.x << ",\n"
              << "        \"y\": " << detection.bbox.center.position.y << ",\n"
              << "        \"z\": " << detection.bbox.center.position.z << "\n"
              << "      },\n"
              << "      \"orientation\": {\n"
              << "        \"x\": " << detection.bbox.center.orientation.x << ",\n"
              << "        \"y\": " << detection.bbox.center.orientation.y << ",\n"
              << "        \"z\": " << detection.bbox.center.orientation.z << ",\n"
              << "        \"w\": " << detection.bbox.center.orientation.w << "\n"
              << "      }\n"
              << "    },\n"
              << "    \"size_x\": " << detection.bbox.size.x << ",\n"
              << "    \"size_y\": " << detection.bbox.size.y << ",\n"
              << "    \"size_z\": " << detection.bbox.size.z << "\n"
              << "  }";
  return json_result.str();
}

std::string getClassId(const std::vector<vision_msgs::msg::ObjectHypothesisWithPose> & objects)
{
  std::string class_id = "";
  double score = -1;
  for (size_t i = 0; i < objects.size(); i++) {
    if (objects[i].hypothesis.score > score) {
      score = objects[i].hypothesis.score;
      class_id = objects[i].hypothesis.class_id;
    }
  }
  return class_id;
}

std::string jsonFromObjectInfo(const isaac_manipulator_interfaces::msg::ObjectInfo & object_info)
{
  std::ostringstream json_result;
  std::string detection_2d = jsonFromDetection2D(object_info.detection_2d);
  std::string detection_3d = jsonFromDetection3D(object_info.detection_3d);
  std::string class_id = "";
  if (object_info.detection_2d.results.size() > 0) {
    class_id = getClassId(object_info.detection_2d.results);
  } else {
    class_id = getClassId(object_info.detection_3d.results);
  }
  json_result << "{\n"
              << "  \"object_id\": \"" << std::to_string(object_info.object_id) << "\",\n"
              << "  \"class_id\": \"" << class_id << "\",\n";
  if (detection_2d.length() > 0) {
    json_result << detection_2d << ((detection_3d.length() > 0) ? ",\n" : "");
  }
  if (detection_3d.length() > 0) {
    json_result << detection_3d;
  }
  json_result << "\n}";
  return json_result.str();
}

std::string jsonFromGetObjectsResult(
  const isaac_ros::mission_client::Vda5050toNav2ClientNode::GoalHandleGetObjectsAction::
  WrappedResult & result)
{
  std::string json_result = "[\n";
  for (size_t i = 0; i < result.result->objects.size(); i++) {
    if (i != 0) {
      json_result += ", ";
    }
    json_result += jsonFromObjectInfo(result.result->objects[i]);
  }
  json_result += "\n]";
  return json_result;
}

}  // namespace
enum ErrorLevel { WARNING, FATAL };

template<typename ActionType>
void Vda5050toNav2ClientNode::ActionResponseCallback(
  const typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr & goal,
  const size_t & action_state_idx)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  auto action_id =
    agv_state_->action_states[action_state_idx].action_id;
  if (!goal) {
    RCLCPP_ERROR(
      this->get_logger(), "Action %s was rejected",
      action_id.c_str());
    UpdateActionState(action_state_idx, VDAActionState().FAILED, "Action was rejected");
  } else {
    // vda_action_goal_handles_.push_back(goal);
    RCLCPP_INFO(
      this->get_logger(), "Action %s was accepted",
      action_id.c_str());
    UpdateActionState(action_state_idx, VDAActionState().RUNNING, "Action was accepted");
  }
}

template<typename ResultType>
void Vda5050toNav2ClientNode::ActionResultCallback(
  const ResultType & result,
  const bool & success,
  const size_t & action_state_idx,
  const std::string & description,
  const int & error_code)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  auto action_id =
    agv_state_->action_states[action_state_idx].action_id;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (success) {
        RCLCPP_INFO(
          get_logger(), "Action %s was succeeded",
          action_id.c_str());
        UpdateActionState(
          action_state_idx, VDAActionState().FINISHED, description);
      } else {
        RCLCPP_ERROR(
          get_logger(), "Action %s was failed",
          action_id.c_str());
        UpdateActionState(
          action_state_idx, VDAActionState().FAILED, description, error_code);
      }
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        get_logger(), "Action %s was aborted",
        action_id.c_str());
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, "Action is aborted", error_code);
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(
        get_logger(), "Action %s was canceled",
        action_id.c_str());
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, "Action is canceled", error_code);
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED,
        "Unknown action result code");
      return;
  }
}

Vda5050toNav2ClientNode::Vda5050toNav2ClientNode(
  const rclcpp::NodeOptions & options)
: Node("nav2_client_node", options),
  client_ptr_(
    rclcpp_action::create_client<NavThroughPoses>(this, "navigate_through_poses")),
  order_info_pub_(
    create_publisher<vda5050_msgs::msg::AGVState>("agv_state", 1)),
  factsheet_info_pub_(
    create_publisher<vda5050_msgs::msg::Factsheet>("factsheet", 1)),
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
  battery_state_topic_(declare_parameter<std::string>("battery_state_topic", "battery_state")),
  battery_state_sub_(create_subscription<sensor_msgs::msg::BatteryState>(
      battery_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Vda5050toNav2ClientNode::BatteryStateCallback, this,
      std::placeholders::_1))),
  order_valid_error_sub_(create_subscription<std_msgs::msg::String>(
      "order_valid_error", rclcpp::SensorDataQoS(),
      std::bind(&Vda5050toNav2ClientNode::OrderValidErrorCallback, this,
      std::placeholders::_1))),
  update_feedback_period_(
    declare_parameter<double>("update_feedback_period", 1.0)),
  update_order_id_period_(
    declare_parameter<double>("update_order_id_period", 0.5)),
  verbose_(declare_parameter<bool>("verbose", false)),
  docking_server_enabled_(declare_parameter<bool>("docking_server_enabled_", false)),
  action_server_names_(declare_parameter<std::vector<std::string>>(
      "action_server_names", std::vector<std::string>({}))),
  odom_topic_(declare_parameter<std::string>("odom_topic", "odom")),
  odometry_sub_(create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Vda5050toNav2ClientNode::OdometryCallback, this,
      std::placeholders::_1))),
  robot_state_timer_(create_wall_timer(
      std::chrono::duration<double>(update_feedback_period_),
      std::bind(&Vda5050toNav2ClientNode::StateTimerCallback, this))),
  order_id_timer_(create_wall_timer(
      std::chrono::duration<double>(update_order_id_period_),
      std::bind(&Vda5050toNav2ClientNode::OrderIdCallback, this))),
  agv_state_(std::make_shared<vda5050_msgs::msg::AGVState>()),
  factsheet_(std::make_shared<vda5050_msgs::msg::Factsheet>()),
  reached_waypoint_(false),
  pause_order_(false),
  current_node_(0),
  next_stop_(0),
  current_node_action_(0),
  current_action_state_(0),
  robot_type_(declare_parameter<std::string>("robot_type", "amr"))
{
  agv_state_->operating_mode = vda5050_msgs::msg::AGVState().AUTOMATIC;
  agv_state_->safety_state.e_stop = vda5050_msgs::msg::SafetyState().NONE;
  agv_state_->safety_state.field_violation = false;
  dock_client_ = rclcpp_action::create_client<DockAction>(this, kRobotDockAction);
  undock_client_ = rclcpp_action::create_client<UndockAction>(this, kRobotUndockAction);
  get_objects_client_ = rclcpp_action::create_client<GetObjectsAction>(
    this,
    kGetObjectsAction);
  pick_and_place_client_ = rclcpp_action::create_client<PickPlaceAction>(
    this,
    kPickAndPlaceAction);
  clear_objects_client_ = create_client<ClearObjectsService>(kClearObjects);
  // Callback group for services
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(callback_group_, this->get_node_base_interface());

  dock_detector_switch_client_ = create_client<std_srvs::srv::SetBool>(
    "switch", rmw_qos_profile_services_default, callback_group_);
  docking_lifecycle_manager_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(
    "lifecycle_manager_docking/manage_nodes", rmw_qos_profile_services_default, callback_group_);
  amcl_get_parameters_client_ = create_client<rcl_interfaces::srv::GetParameters>(
    "amcl/get_parameters", rmw_qos_profile_services_default, callback_group_);
  localization_client_ = create_client<std_srvs::srv::Empty>(
    "/trigger_grid_search_localization", rmw_qos_profile_services_default, callback_group_);
  // Pause docking server
  if (docking_server_enabled_) {
    while (!dock_detector_switch_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(get_logger(), "Waiting for docking server lifecycle manager service to appear.");
    }
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::PAUSE;
    docking_lifecycle_manager_client_->async_send_request(request);
  }

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

  current_order_canceled_ = false;

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

  // If the robot is in teleop mode
  if (pause_order_) {
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
        agv_state_->driving = false;
      }
      if (action_status == VDAActionState().FAILED) {
        RCLCPP_ERROR(
          get_logger(), "Action '%s' failed.",
          agv_state_->action_states[current_action_state_].action_id.c_str());
        RCLCPP_ERROR(
          get_logger(), "Order '%s' failed.", agv_state_->order_id.c_str());
        return;
      }
    } else {
      // Move to next node if no action
      current_node_action_ = 0;
      next_stop_++;
      // Check if the order is completed.
      if (next_stop_ >= current_order_->nodes.size()) {
        RCLCPP_INFO(
          get_logger(), "Order %s is completed.", current_order_->order_id.c_str());
      } else {
        reached_waypoint_ = false;
      }
      lock.unlock();
      if (next_stop_ < current_order_->nodes.size()) {
        NavigateThroughPoses();
      }
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
  if (robot_type_ == "amr") {
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
  if (next_stop_ >= current_order_->nodes.size()) {
    return false;
  }
  // If the order has failed
  for (auto & error : agv_state_->errors) {
    if (error.error_type != kValidationError && error.error_type != kOrderUpdateError) {
      return false;
    }
  }
  // If current order is canceled
  if (current_order_canceled_) {
    return false;
  }
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "An order is running");
  }
  return true;
}

void Vda5050toNav2ClientNode::NavigateThroughPoses()
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Navigation server not available");
    return;
  }
  if (!current_order_) {
    RCLCPP_ERROR(get_logger(), "Navigation was called when no order exists.");
  }
  if (next_stop_ >= current_order_->nodes.size()) {
    RCLCPP_INFO(get_logger(), "Navigation completed");
    return;
  }
  auto goal_msg = NavThroughPoses::Goal();
  for (size_t i = current_node_ + 1; i < current_order_->nodes.size(); i++) {
    auto pose_stamped = geometry_msgs::msg::PoseStamped();

    pose_stamped.pose.position.x =
      current_order_->nodes[i].node_position.x;
    pose_stamped.pose.position.y =
      current_order_->nodes[i].node_position.y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.header.frame_id = "map";
    // Convert theta into a quaternion for goal pose's orientation
    tf2::Quaternion orientation;
    orientation.setRPY(
      0, 0,
      current_order_->nodes[i].node_position.theta);
    pose_stamped.pose.orientation = tf2::toMsg(orientation);
    pose_stamped.header.stamp = rclcpp::Clock().now();
    goal_msg.poses.push_back(pose_stamped);
    if (current_order_->nodes[i].actions.size() > 0 ||
      i == current_order_->nodes.size() - 1 ||
      current_order_->nodes[i].node_position.allowed_deviation_x_y == 0)
    {
      next_stop_ = i;
      break;
    }
  }
  auto send_goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
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
  RCLCPP_INFO(
    get_logger(), "Sending goal for (x: %f, y: %f, t: %f)",
    current_order_->nodes[next_stop_].node_position.x,
    current_order_->nodes[next_stop_].node_position.y,
    current_order_->nodes[next_stop_].node_position.theta);
  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void Vda5050toNav2ClientNode::Vda5050ActionsHandler(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  // Handle action that does not need a server
  if (vda5050_action.action_type == "pause_order") {
    RCLCPP_INFO(
      this->get_logger(), "Get %s request", vda5050_action.action_type.c_str());
    pause_order_ = true;
    // Set the pause_order to running untill stopTeleop action is triggered
    UpdateActionStatebyId(vda5050_action.action_id, VDAActionState().RUNNING);
    pause_order_action_id_ = vda5050_action.action_id;
    return;
  }

  // Update action state to initializing
  size_t action_state_idx = current_action_state_;
  UpdateActionState(action_state_idx, VDAActionState().INITIALIZING);
  std::unordered_map<std::string, std::string> action_parameters_map;
  for (const auto & action_param : vda5050_action.action_parameters) {
    action_parameters_map[action_param.key] = action_param.value;
  }
  // Handle robotDock action
  if (vda5050_action.action_type == kRobotDockAction) {
    auto send_goal_options =
      rclcpp_action::Client<DockAction>::SendGoalOptions();
    // Prepare dock_robot parameters and callbacks
    send_goal_options.goal_response_callback =
      [this, action_state_idx](
      const rclcpp_action::ClientGoalHandle<DockAction>::SharedPtr &
      goal) {
        ActionResponseCallback<DockAction>(goal, action_state_idx);
      };
    send_goal_options.result_callback =
      [this,
        action_state_idx](const GoalHandleDockAction::WrappedResult & result) {
        std::string result_description = "";
        switch (result.result->error_code) {
          case DockAction::Result::UNKNOWN:
            result_description = "Unknown";
            break;
          case DockAction::Result::DOCK_NOT_IN_DB:
            result_description = "Dock not in DB";
            break;
          case DockAction::Result::DOCK_NOT_VALID:
            result_description = "Dock not valid";
            break;
          case DockAction::Result::FAILED_TO_STAGE:
            result_description = "Failed to stage";
            break;
          case DockAction::Result::FAILED_TO_DETECT_DOCK:
            result_description = "Failed to detect dock";
            break;
          case DockAction::Result::FAILED_TO_CONTROL:
            result_description = "Failed to control";
            break;
          case DockAction::Result::FAILED_TO_CHARGE:
            result_description = "Failed to charge";
            break;
          default:
            break;
        }
        ActionResultCallback<GoalHandleDockAction::WrappedResult>(
          result, result.result->success, action_state_idx, result_description,
          result.result->error_code);
        // Disable switch
        auto switch_request = std::make_shared<std_srvs::srv::SetBool::Request>();
        switch_request->data = false;
        SendBoolRequest<std_srvs::srv::SetBool>(dock_detector_switch_client_, switch_request);

        // Disable docking server
        auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::PAUSE;
        SendBoolRequest<nav2_msgs::srv::ManageLifecycleNodes>(
          docking_lifecycle_manager_client_, server_request);
      };
    auto goal_msg = DockAction::Goal();
    goal_msg.use_dock_id = false;
    // Parse dock_robot parameters
    double yaw;
    bool trigger_global_localization = true;
    try {
      goal_msg.dock_type = action_parameters_map.at(kDockType);
      goal_msg.dock_pose = geometry_msgs::msg::PoseStamped();
      if (!isValidDockPose(action_parameters_map.at(kDockPose))) {
        throw std::invalid_argument("Failed to parse 'dock_pose'.");
      }
      std::vector<std::string> dock_pose_strings = split(action_parameters_map[kDockPose], ',');
      goal_msg.dock_pose.pose.position.x = std::stod(dock_pose_strings[0]);
      goal_msg.dock_pose.pose.position.y = std::stod(dock_pose_strings[1]);
      yaw = std::stod(dock_pose_strings[2]);

      if (action_parameters_map.find(kTriggerGlobalLocalization) != action_parameters_map.end()) {
        trigger_global_localization = stringToBool(
          action_parameters_map[kTriggerGlobalLocalization]);
      }
    } catch (std::invalid_argument & e) {
      RCLCPP_ERROR(get_logger(), e.what());
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, e.what());
      return;
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR(get_logger(), e.what());
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, e.what());
      return;
    } catch (...) {
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, "Invalid dock_robot parameters");
      return;
    }
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);
    goal_msg.dock_pose.pose.orientation = tf2::toMsg(orientation);
    goal_msg.dock_pose.header.stamp = rclcpp::Clock().now();
    goal_msg.dock_pose.header.frame_id = "map";
    goal_msg.navigate_to_staging_pose = true;
    // Send service request to reinitialize global localization
    if (trigger_global_localization) {
      auto trigger_localization_request = std::make_shared<std_srvs::srv::Empty::Request>();
      SendServiceRequest<std_srvs::srv::Empty>(localization_client_, trigger_localization_request);
    }

    // Send service request to enable docking server
    auto enable_docking_server_request =
      std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    enable_docking_server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::RESUME;
    if (!SendBoolRequest<nav2_msgs::srv::ManageLifecycleNodes>(
        docking_lifecycle_manager_client_, enable_docking_server_request))
    {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to enable docking server");
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, "Failed to enable docking server");
      return;
    }

    // Send service request to open switch node
    auto enable_switch_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    enable_switch_request->data = true;
    if (!SendBoolRequest<std_srvs::srv::SetBool>(
        dock_detector_switch_client_, enable_switch_request))
    {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to enable switch node");
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, "Failed to enable switch node");
      return;
    }

    // Send dock_robot action
    // Get transform_tolerance value
    auto get_param_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_param_request->names.push_back("transform_tolerance");
    int transform_tolerance = 0.0;
    try {
      auto get_param_result = SendServiceRequest<rcl_interfaces::srv::GetParameters>(
        amcl_get_parameters_client_, get_param_request);
      transform_tolerance = static_cast<int>(ceil(get_param_result->values[0].double_value));
    } catch (...) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to get transform_tolerance");
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, "Failed to get transform_tolerance");
      return;
    }
    // Wait transform_tolerance seconds for the tf2_listener in docking server to startup.
    std::this_thread::sleep_for(std::chrono::seconds(transform_tolerance));
    dock_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(
      this->get_logger(), "Send dock_robot request");
    return;
  }
  // Handle undock_robot action
  if (vda5050_action.action_type == kRobotUndockAction) {
    bool trigger_global_localization = true;
    if (action_parameters_map.find(kTriggerGlobalLocalization) != action_parameters_map.end()) {
      trigger_global_localization = stringToBool(
        action_parameters_map[kTriggerGlobalLocalization]);
    }

    auto send_goal_options =
      rclcpp_action::Client<UndockAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this, action_state_idx](
      const rclcpp_action::ClientGoalHandle<UndockAction>::SharedPtr &
      goal) {ActionResponseCallback<UndockAction>(goal, action_state_idx);};
    send_goal_options.result_callback =
      [this, action_state_idx, trigger_global_localization](
      const GoalHandleUndockAction::WrappedResult & result) {
        // Disable docking server
        auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::PAUSE;
        SendBoolRequest<nav2_msgs::srv::ManageLifecycleNodes>(
          docking_lifecycle_manager_client_, server_request);
        // Parse error code
        std::string result_description = "";
        switch (result.result->error_code) {
          case UndockAction::Result::UNKNOWN:
            result_description = "Unknown";
            break;
          case UndockAction::Result::DOCK_NOT_VALID:
            result_description = "Dock not valid";
            break;
          case UndockAction::Result::FAILED_TO_CONTROL:
            result_description = "Failed to control";
            break;
          default:
            break;
        }
        ActionResultCallback<GoalHandleUndockAction::WrappedResult>(
          result, result.result->success, action_state_idx, result_description,
          result.result->error_code);
        if (trigger_global_localization) {
          auto trigger_localization_request = std::make_shared<std_srvs::srv::Empty::Request>();
          SendServiceRequest<std_srvs::srv::Empty>(
            localization_client_,
            trigger_localization_request);
        }
      };
    auto goal_msg = UndockAction::Goal();
    // Enable docking server
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::RESUME;
    if (!SendBoolRequest<nav2_msgs::srv::ManageLifecycleNodes>(
        docking_lifecycle_manager_client_, request))
    {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to enable docking server");
      UpdateActionState(
        action_state_idx, VDAActionState().FAILED, "Failed to enable docking server");
      return;
    }
    undock_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(
      this->get_logger(), "Send %s request",
      vda5050_action.action_type.c_str());
    return;
  }
  // Handle detect object action
  if (vda5050_action.action_type == kGetObjectsAction) {
    getObjectsActionHandler();
    return;
  }
  // Handle pick and place action
  if (vda5050_action.action_type == kPickAndPlaceAction) {
    pickPlaceActionHandler(action_parameters_map);
    return;
  }
  // Handle clear objects action
  if (vda5050_action.action_type == kClearObjects) {
    clearObjectsHandler(action_parameters_map);
    return;
  }
  // Check if action server exists
  if (action_server_map_.count(vda5050_action.action_type) == 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Action type %s is not supported",
      vda5050_action.action_type.c_str());
    UpdateActionState(
      current_action_state_, VDAActionState().FAILED,
      "This action is not supported");
    return;
  }
  // Trigger action
  auto send_goal_options =
    rclcpp_action::Client<MissionAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, action_state_idx](
    const rclcpp_action::ClientGoalHandle<MissionAction>::SharedPtr &
    goal) {ActionResponseCallback<MissionAction>(goal, action_state_idx);};
  send_goal_options.result_callback =
    [this,
      action_state_idx](const GoalHandleMissionAction::WrappedResult & result) {
      ActionResultCallback<GoalHandleMissionAction::WrappedResult>(
        result, result.result->success, action_state_idx, result.result->result_description);
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
  current_order_canceled_ = false;
  agv_state_.reset(new vda5050_msgs::msg::AGVState());
  agv_state_->operating_mode = vda5050_msgs::msg::AGVState().AUTOMATIC;
  agv_state_->safety_state.e_stop = vda5050_msgs::msg::SafetyState().NONE;
  agv_state_->safety_state.field_violation = false;
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
      actionState.action_type = vda5050_action.action_type;
      actionState.action_description = vda5050_action.action_description;
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
  RCLCPP_INFO(
    get_logger(), "Order with order_id %s received",
    msg->order_id.c_str());
  std::lock_guard<std::mutex> lock(order_mutex_);
  bool is_running = RunningOrder();
  if (is_running) {
    if (msg->order_id != current_order_->order_id) {
      RCLCPP_INFO(
        get_logger(), "One order is running. Order %s is ignored",
        msg->order_id.c_str());
      vda5050_msgs::msg::Error error = CreateError(
        ErrorLevel::WARNING, "An order is running",
        {CreateErrorReference("orderId", msg->order_id)},
        kOrderUpdateError);
      agv_state_->errors.push_back(error);
    }
  }
  if (!is_running && !msg->nodes.empty()) {
    current_order_ = msg;
    current_node_ = 0;
    next_stop_ = 0;
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

void Vda5050toNav2ClientNode::OdometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  agv_state_->velocity.vx = msg->twist.twist.linear.x;
  agv_state_->velocity.vy = msg->twist.twist.linear.y;
  agv_state_->velocity.omega = msg->twist.twist.angular.z;
}

void Vda5050toNav2ClientNode::InstantActionsCallback(
  const vda5050_msgs::msg::InstantActions::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Instant actions with header_id %d received",
    msg->header_id);
  std::unique_lock<std::mutex> lock(order_mutex_);

  for (const vda5050_msgs::msg::Action & action : msg->instant_actions) {
    auto it = std::find_if(
      agv_state_->action_states.begin(),
      agv_state_->action_states.end(),
      [&action](const VDAActionState & processed_action) {
        return processed_action.action_id == action.action_id;
      });
    // Ignore if the action is already processed.
    if (it != agv_state_->action_states.end()) {
      continue;
    }
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
    } else if (action.action_type == "startTeleop" || action.action_type == "stopTeleop") {
      TeleopActionHandler(action);
    } else if (action.action_type == "factsheetRequest") {
      FactsheetRequestHandler(action);
    }
  }
}

void Vda5050toNav2ClientNode::TeleopActionHandler(const vda5050_msgs::msg::Action & teleop_action)
{
  UpdateActionStatebyId(teleop_action.action_id, VDAActionState().RUNNING);
  if (teleop_action.action_type == "startTeleop") {
    // Cancel any running navigation goal
    pause_order_ = true;
    if (nav_goal_handle_) {
      client_ptr_->async_cancel_goal(nav_goal_handle_);
      RCLCPP_INFO(
        this->get_logger(), "Start teleop: cancelling current mission node.");
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Finished teleop. Resume order.");
    pause_order_ = false;
    // Set the pause_order action if exists to finished
    if (pause_order_action_id_ != "") {
      UpdateActionStatebyId(pause_order_action_id_, VDAActionState().FINISHED);
      pause_order_action_id_ = "";
    }
    // Reset states to resume navigation
    next_stop_ = current_node_;
    reached_waypoint_ = true;
  }
  UpdateActionStatebyId(teleop_action.action_id, VDAActionState().FINISHED);
}

void Vda5050toNav2ClientNode::FactsheetRequestHandler(
  const vda5050_msgs::msg::Action & factsheet_request)
{
  UpdateActionStatebyId(factsheet_request.action_id, VDAActionState().RUNNING);

  if (factsheet_request.action_type == "factsheetRequest") {
    if (robot_type_ == "arm") {
      factsheet_->physical_parameters.speed_max = 0;
      factsheet_->type_specification.agv_class = "FORKLIFT";
    } else {
      factsheet_->physical_parameters.speed_max = 1;
      factsheet_->type_specification.agv_class = "CARRIER";
    }

    // publish the factsheet over the "factsheet" topic
    PublishRobotFactsheet();
  }

  UpdateActionStatebyId(factsheet_request.action_id, VDAActionState().FINISHED);
}

void Vda5050toNav2ClientNode::PublishRobotFactsheet()
{
  factsheet_->timestamp = CreateISO8601Timestamp();
  factsheet_info_pub_->publish(*factsheet_);
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
  current_order_canceled_ = true;
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
  const std::string & result_description, const int & error_code)
{
  if (action_state_idx >= agv_state_->action_states.size()) {
    return;
  }
  agv_state_->action_states[action_state_idx].action_status = status;
  agv_state_->action_states[action_state_idx].result_description =
    result_description;
  if (status == VDAActionState().FAILED) {
    auto error = vda5050_msgs::msg::Error();
    error = CreateError(
      ErrorLevel::WARNING, "Action Error",
      {CreateErrorReference(
          "node_id",
          current_order_->nodes[current_node_].node_id),
        CreateErrorReference(
          "action_id",
          agv_state_->action_states[action_state_idx].action_id),
        CreateErrorReference(
          "error_code", std::to_string(error_code))});
    agv_state_->errors.push_back(error);
  }
  PublishRobotState();
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
    nav2_msgs::action::NavigateThroughPoses>::SharedPtr & goal)
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
  GoalHandleNavThroughPoses::SharedPtr,
  const NavThroughPoses::Feedback::ConstSharedPtr feedback)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  current_node_ = next_stop_ - feedback->number_of_poses_remaining;
  agv_state_->last_node_id =
    current_order_->nodes[current_node_].node_id;
  agv_state_->last_node_sequence_id =
    current_order_->nodes[current_node_].sequence_id;
  agv_state_->driving = true;
  while (agv_state_->node_states.size() > current_order_->nodes.size() - current_node_ - 1) {
    agv_state_->node_states.erase(
      agv_state_->node_states.begin());
  }
}

void Vda5050toNav2ClientNode::NavPoseResultCallback(
  const GoalHandleNavThroughPoses::WrappedResult & result)
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
      // Reset current_node if the nav2 action is not cancelled by teleop
      if (pause_order_ != true) {
        current_node_ = 1;
        error = CreateError(
          ErrorLevel::WARNING, "Goal canceled",
          {CreateErrorReference(
              "node_id", current_order_->nodes[current_node_].node_id)});
        agv_state_->errors.push_back(error);
      }
      PublishRobotState();
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
  }
  // Logic if navigation was successful
  RCLCPP_INFO(get_logger(), "Reached order node: %ld", next_stop_);
  current_node_ = next_stop_;
  while (agv_state_->node_states.size() > current_order_->nodes.size() - current_node_ - 1) {
    agv_state_->node_states.erase(
      agv_state_->node_states.begin());
  }
  agv_state_->last_node_id =
    current_order_->nodes[current_node_].node_id;
  agv_state_->last_node_sequence_id =
    current_order_->nodes[current_node_].sequence_id;
  reached_waypoint_ = true;
}

void Vda5050toNav2ClientNode::OrderValidErrorCallback(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  vda5050_msgs::msg::Error error = CreateError(
    ErrorLevel::WARNING, "Malformed order",
    {CreateErrorReference("Error", msg->data)},
    kValidationError);
  agv_state_->errors.push_back(error);
}

template<typename ServiceT>
typename ServiceT::Response::SharedPtr Vda5050toNav2ClientNode::SendServiceRequest(
  typename rclcpp::Client<ServiceT>::SharedPtr client,
  typename ServiceT::Request::SharedPtr request)
{
  if (!client->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Service is not available.");
    throw std::runtime_error("Service is not available.");
  }
  auto result = client->async_send_request(request);
  if (executor_.spin_until_future_complete(result) != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Failed to get service result.");
  }
  return result.get();
}

template<typename ServiceT>
bool Vda5050toNav2ClientNode::SendBoolRequest(
  typename rclcpp::Client<ServiceT>::SharedPtr client,
  typename ServiceT::Request::SharedPtr request)
{
  try {
    auto result = SendServiceRequest<ServiceT>(client, request);
    return result->success;
  } catch (...) {
    return false;
  }
}

void Vda5050toNav2ClientNode::getObjectsActionHandler()
{
  // Check if action server is available
  if (!get_objects_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Get Objects server not available");
    UpdateActionState(
      current_action_state_, VDAActionState().FAILED, "Get Objects server not available");
    return;
  }
  auto send_goal_options = rclcpp_action::Client<GetObjectsAction>::SendGoalOptions();
  size_t action_state_idx = current_action_state_;
  send_goal_options.goal_response_callback =
    [this, action_state_idx](const GoalHandleGetObjectsAction::SharedPtr & goal) {
      ActionResponseCallback<GetObjectsAction>(goal, action_state_idx);
    };
  send_goal_options.result_callback =
    [this,
      action_state_idx](const GoalHandleGetObjectsAction::WrappedResult & result) {
      std::string json_result;
      json_result = jsonFromGetObjectsResult(result);
      ActionResultCallback<GoalHandleGetObjectsAction::WrappedResult>(
        result, true, action_state_idx, json_result);
    };
  auto goal_msg = GetObjectsAction::Goal();
  get_objects_client_->async_send_goal(goal_msg, send_goal_options);
  return;
}

void Vda5050toNav2ClientNode::pickPlaceActionHandler(
  std::unordered_map<std::string, std::string> & action_parameters_map)
{
  // Check if action server is available
  if (!pick_and_place_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Pick Place server not available");
    UpdateActionState(
      current_action_state_, VDAActionState().FAILED, "Pick Place server not available");
    return;
  }
  auto send_goal_options = rclcpp_action::Client<PickPlaceAction>::SendGoalOptions();
  size_t action_state_idx = current_action_state_;
  send_goal_options.goal_response_callback =
    [this, action_state_idx](const GoalHandlePickPlaceAction::SharedPtr & goal) {
      ActionResponseCallback<PickPlaceAction>(goal, action_state_idx);
    };
  send_goal_options.result_callback =
    [this,
      action_state_idx](const GoalHandlePickPlaceAction::WrappedResult & result) {
      ActionResultCallback<GoalHandlePickPlaceAction::WrappedResult>(
        result, true, action_state_idx, "");
    };
  auto goal_msg = PickPlaceAction::Goal();

  try {
    goal_msg.object_id = std::stoi(action_parameters_map[kObjectId]);
    goal_msg.class_id = action_parameters_map[kClassId];

    std::vector<std::string> place_pose = split(action_parameters_map[kPlacePose], ',');

    goal_msg.place_pose.position.x = std::stod(place_pose[0]);
    goal_msg.place_pose.position.y = std::stod(place_pose[1]);
    goal_msg.place_pose.position.z = std::stod(place_pose[2]);

    goal_msg.place_pose.orientation.x = std::stod(place_pose[3]);
    goal_msg.place_pose.orientation.y = std::stod(place_pose[4]);
    goal_msg.place_pose.orientation.z = std::stod(place_pose[5]);
    goal_msg.place_pose.orientation.w = std::stod(place_pose[6]);
  } catch (std::out_of_range & e) {
    UpdateActionState(
      current_action_state_, VDAActionState().FAILED, "Invalid action parameters");
    return;
  } catch (std::invalid_argument & e) {
    UpdateActionState(
      current_action_state_, VDAActionState().FAILED, "Invalid action parameters");
    return;
  }
  pick_and_place_client_->async_send_goal(goal_msg, send_goal_options);
  return;
}

void Vda5050toNav2ClientNode::clearObjectsHandler(
  std::unordered_map<std::string, std::string> & action_parameters_map)
{
  if (!clear_objects_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Clear Objects Service is not available.");
    UpdateActionState(
      current_action_state_, VDAActionState().FAILED, "Clear Objects Service is not available.");
  }
  UpdateActionState(current_action_state_, VDAActionState().RUNNING);
  auto request = std::make_shared<ClearObjectsService::Request>();
  std::vector<std::string> object_ids = split(action_parameters_map[kObjectIds], ',');
  for (auto & object_id : object_ids) {
    request->object_ids.push_back(std::stoi(object_id));
  }
  clear_objects_client_->async_send_request(
    request,
    [this](const rclcpp::Client<ClearObjectsService>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response) {
          UpdateActionState(
            current_action_state_, VDAActionState().FINISHED);
        } else {
          UpdateActionState(
            current_action_state_, VDAActionState().FAILED, "Failed to call ClearObjects service");
        }
      } catch (std::exception & e) {
        UpdateActionState(
          current_action_state_, VDAActionState().FAILED, "Failed to call ClearObjects service");
      }
    });
  return;
}

Vda5050toNav2ClientNode::~Vda5050toNav2ClientNode() = default;

}  // namespace mission_client
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  isaac_ros::mission_client::Vda5050toNav2ClientNode)
