
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <tf2/utils.h>

#include "nav2_behavior_tree/plugins/condition/is_goal_station_critical_condition.hpp"

namespace nav2_behavior_tree
{

IsGoalStationCriticalCondition::IsGoalStationCriticalCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
  
  RCLCPP_INFO(node->get_logger(), "Constructor IsGoalStationCriticalCondition");
}

IsGoalStationCriticalCondition::~IsGoalStationCriticalCondition()
{
  cleanup();
}

void IsGoalStationCriticalCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  //node_->get_parameter("min_angle_for_rotation", min_angle_for_rotation);

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  //RCLCPP_INFO(node_->get_logger(), "IsGoalStationCriticalCondition initialized");

  node_->get_parameter("ws", ws_flat_);

  if (ws_flat_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Parameter 'ws' is empty");
  }

  node_->get_parameter("critical_ws", critical_ws_);

  if (critical_ws_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Parameter 'critical_ws_' is empty");
  }


}

BT::NodeStatus IsGoalStationCriticalCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (isStationCritical()) {
    //RCLCPP_INFO(node_->get_logger(), "IsGoalStationCriticalCondition: SUCCESS – rotation is needed");
    return BT::NodeStatus::SUCCESS;
  } else {
    //RCLCPP_INFO(node_->get_logger(), "IsGoalStationCriticalCondition: FAILURE – rotation not needed");
    return BT::NodeStatus::FAILURE;
  }
}
bool IsGoalStationCriticalCondition::isStationCritical()
{
  // Get goal from BT input
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_WARN(node_->get_logger(), "IsGoalStationCritical: missing input [goal]");
    return false;
  }

  double goal_dist_threshold;
  getInput("goal_dist_threshold", goal_dist_threshold);

  if (ws_flat_.empty()) {
    return false;
  }

  // ws is [x1,y1,x2,y2] per station
  if (ws_flat_.size() % 4 != 0) {
    RCLCPP_ERROR(node_->get_logger(),
      "Parameter 'ws' must be multiple of 4: [x1,y1,x2,y2] per station. Got size=%zu",
      ws_flat_.size());
    return false;
  }

  const size_t station_count = ws_flat_.size() / 4;

  // near check threshold
  const double near_ws_dist_sq = goal_dist_threshold * goal_dist_threshold;

  const double rx = goal.pose.position.x;
  const double ry = goal.pose.position.y;

  RCLCPP_INFO(node_->get_logger(),
    "WS check: ws_flat_.size()=%zu (stations=%zu), critical_ws_.size()=%zu, thresh=%.3f",
    ws_flat_.size(), station_count, critical_ws_.size(), goal_dist_threshold);

  // Find closest station index (using x1,y1 only)
  bool goal_near_station = false;
  size_t matched_station_idx = 0;  // 0-based
  double best_dist_sq = std::numeric_limits<double>::infinity();

  for (size_t s = 0; s < station_count; ++s) {
    const size_t i = s * 4;
    const double x1 = ws_flat_[i + 0];
    const double y1 = ws_flat_[i + 1];

    const double dx = x1 - rx;
    const double dy = y1 - ry;
    const double dist_sq = dx * dx + dy * dy;

    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      matched_station_idx = s;
    }
  }

  // Accept only if closest station is within threshold
  if (best_dist_sq <= near_ws_dist_sq) {
    goal_near_station = true;
  } else {
    goal_near_station = false;
  }

  if (!goal_near_station) {
    return false;
  }

  if (critical_ws_.empty()) {
    return false;
  }

  // Decide whether critical_ws_ stores station IDs as:
  //  - 0-based indices (0..N-1)  OR
  //  - 1-based station numbers (1..N)
  //
  // Pick ONE convention and stick to it. Most people use 1-based "station numbers".
  // If your critical_ws_ uses station numbers (1-based), use matched_station_number = matched_station_idx + 1.
  const int matched_station_number = static_cast<int>(matched_station_idx) + 1;  // 1-based

  const bool is_critical = std::any_of(
    critical_ws_.begin(), critical_ws_.end(),
    [&](const auto & v) { return static_cast<int>(v) == matched_station_number; });

  RCLCPP_INFO(node_->get_logger(),
    "Goal near station #%d -> critical=%s",
    matched_station_number, is_critical ? "true" : "false");

  return is_critical;
}

}  // namespace nav2_behavior_tree


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsGoalStationCriticalCondition>("IsGoalStationCritical");
}

//  Possible improvements:
//  -When the goal is near stop the function
//  -Change the N of the number of points to be a parameter
//  -Change the threshold angle to be a parameter
//  -Important assumption of the package: path is always computed before entering this package, this way current_pose = initial_pose

//Continuar manana: poner los argumentos en el config!!!