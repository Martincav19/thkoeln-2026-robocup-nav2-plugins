
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <tf2/utils.h>

#include "nav2_behavior_tree/plugins/action/dock_to_station_action.hpp"

namespace nav2_behavior_tree
{

DockToStationAction::DockToStationAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::DockToStation>(xml_tag_name, action_name, conf) {

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "global_frame", this);

  RCLCPP_INFO(node->get_logger(), "Constructor DockToStationAction");

}

void DockToStationAction::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);
  node_->get_parameter("ws", ws_flat_);

  if (ws_flat_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Parameter 'ws' is empty");
  }

  // NEW: ws is [x1,y1,x2,y2] per station
  if (ws_flat_.size() % 4 != 0) {
    throw std::runtime_error(
      "Parameter 'ws' must contain a multiple of 4 values: [x1,y1,x2,y2] per station");
  }

  double time_allowance = 0.0;
  getInput("time_allowance", time_allowance);

  double standoff_distance = 0.0;
  getInput("standoff_distance", standoff_distance);

  bool use_docking = false;
  getInput("use_docking", use_docking);

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_WARN(node_->get_logger(), "DockToStation: missing input [goal]");
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
  }

  geometry_msgs::msg::Vector3 current_vector;
  const double yaw_robot = tf2::getYaw(current_pose.pose.orientation);
  current_vector.x = std::cos(yaw_robot);
  current_vector.y = std::sin(yaw_robot);
  current_vector.z = 0.0;

  geometry_msgs::msg::Vector3 goal_vector;
  const double yaw_goal = tf2::getYaw(goal.pose.orientation);
  goal_vector.x = std::cos(yaw_goal);
  goal_vector.y = std::sin(yaw_goal);
  goal_vector.z = 0.0;

  bool near_station = false;
  // tuning parameter (meters)
  const double near_ws_dist = 0.5;
  const double near_ws_dist_sq = near_ws_dist * near_ws_dist;

  const double rx = goal.pose.position.x;
  const double ry = goal.pose.position.y;

  RCLCPP_INFO(node_->get_logger(),
    "WS check: ws_flat_.size()=%zu (stations=%zu)",
    ws_flat_.size(), ws_flat_.size() / 4);

  // NEW loop stride: 4, and use only (x1,y1) for distance check
  for (size_t i = 0; i < ws_flat_.size(); i += 4) {
    const double x1 = ws_flat_[i + 0];
    const double y1 = ws_flat_[i + 1];
    // const double x2 = ws_flat_[i + 2];
    // const double y2 = ws_flat_[i + 3];

    const double dx = x1 - rx;
    const double dy = y1 - ry;

    if ((dx * dx + dy * dy) <= near_ws_dist_sq) {
      near_station = true;
      break;
    }
  }

  use_docking = near_station && use_docking;

  goal_.docking_side = decideDockingSide(current_vector, goal_vector);
  goal_.standoff_distance = standoff_distance;
  goal_.max_linear_speed = 0.20;
  goal_.max_angular_speed = 0.60;  // (unchanged, but double-check this literal)
  goal_.maintain_alignment = use_docking;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void DockToStationAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}

BT::NodeStatus DockToStationAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DockToStationAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DockToStationAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

int DockToStationAction::decideDockingSide(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b)
{
  // Front
  double angle_front = angleBetween(a, b);

  // Back
  geometry_msgs::msg::Vector3 back_vector;
  back_vector.x = -a.x;
  back_vector.y = -a.y;
  back_vector.z = 0.0;
  double angle_back = angleBetween(back_vector, b);

  // Left (+90 deg rotation)
  geometry_msgs::msg::Vector3 left_vector;
  left_vector.x = -a.y;
  left_vector.y =  a.x;
  left_vector.z = 0.0;
  double angle_left = angleBetween(left_vector, b);

  // Right (-90 deg rotation)
  geometry_msgs::msg::Vector3 right_vector;
  right_vector.x =  a.y;
  right_vector.y = -a.x;
  right_vector.z = 0.0;
  double angle_right = angleBetween(right_vector, b);

  RCLCPP_INFO(node_->get_logger(),
  "angles [front %.2f, back %.2f, left %.2f, right %.2f]",
  angle_front, angle_back, angle_left, angle_right);

  // Find minimum angle
  double min_angle = angle_front;
  int side = 0;

  if (angle_back < min_angle) {
    min_angle = angle_back;
    side = 1;
  }
  if (angle_left < min_angle) {
    min_angle = angle_left;
    side = 2;
  }
  if (angle_right < min_angle) {
    min_angle = angle_right;
    side = 3;
  }

  return side;
}

double DockToStationAction::angleBetween(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b)
{
    double dot = a.x * b.x + a.y * b.y + a.z * b.z;
    double magA = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    double magB = std::sqrt(b.x*b.x + b.y*b.y + b.z*b.z);

    if (magA < 1e-9 || magB < 1e-9)
        return 0.0; // undefined, but safe fallback

    double c = dot / (magA * magB);

    // Clamp to [-1,1] to avoid NaN due to float precision
    c = std::max(-1.0, std::min(1.0, c));

    return std::acos(c);  // radians
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::DockToStationAction>(name, "dock_to_station", config);
    };

  factory.registerBuilder<nav2_behavior_tree::DockToStationAction>("DockToStation", builder);
}
