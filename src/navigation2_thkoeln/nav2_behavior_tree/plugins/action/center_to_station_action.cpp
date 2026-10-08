
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>  

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <tf2/utils.h>

#include "nav2_behavior_tree/plugins/action/center_to_station_action.hpp"

namespace nav2_behavior_tree
{

CenterToStationAction::CenterToStationAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::DriveOnLateral>(xml_tag_name, action_name, conf) {

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "global_frame", this);

  RCLCPP_INFO(node->get_logger(), "Constructor CenterToStationAction");

}

void CenterToStationAction::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);
  node_->get_parameter("ws", ws_flat_);
  node_->get_parameter("center_station_tolerance", tol_); 

  if (ws_flat_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Parameter 'ws' is empty");
  }

  // NOW: ws is [x1,y1,x2,y2] repeated per station
  if (ws_flat_.size() % 4 != 0) {
    throw std::runtime_error(
      "Parameter 'ws' must contain a multiple of 4 values: [x1,y1,x2,y2] per station");
  }

  double time_allowance = 0.0;
  getInput("time_allowance", time_allowance);

  double speed;
  getInput("center_to_station_speed", speed);

  // Get current pose in global frame (map)
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
  }

  const double rx = current_pose.pose.position.x;
  const double ry = current_pose.pose.position.y;

  const double yaw_robot = tf2::getYaw(current_pose.pose.orientation);

  // Robot left axis in map frame (base_link +Y)
  // base_link forward: (cos(yaw), sin(yaw))
  // base_link left:    (-sin(yaw), cos(yaw))
  const double left_x = -std::sin(yaw_robot);
  const double left_y =  std::cos(yaw_robot);

  // Tuning parameter (meters): how close you must be to P1 to "select" that station
  const double near_ws_dist = 0.5;
  const double near_ws_dist_sq = near_ws_dist * near_ws_dist;

  RCLCPP_INFO(node_->get_logger(),
    "WS check: ws_flat_.size()=%zu (stations=%zu)",
    ws_flat_.size(), ws_flat_.size() / 4);

  // Iterate per station: [x1,y1,x2,y2] and pick the closest (using only x1,y1 for proximity)
  double best_dist_sq = std::numeric_limits<double>::infinity();
  bool near_station = false;

  // Selected station line points
  double x1_sel = 0.0, y1_sel = 0.0, x2_sel = 0.0, y2_sel = 0.0;

  for (size_t i = 0; i < ws_flat_.size(); i += 4) {
    const double x1 = ws_flat_[i + 0];
    const double y1 = ws_flat_[i + 1];
    const double x2 = ws_flat_[i + 2];
    const double y2 = ws_flat_[i + 3];

    const double dx = x1 - rx;
    const double dy = y1 - ry;
    const double dist_sq = dx * dx + dy * dy;

    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      x1_sel = x1; y1_sel = y1;
      x2_sel = x2; y2_sel = y2;
    }
  }

  // Accept only if closest is within threshold
  if (best_dist_sq <= near_ws_dist_sq) {
    near_station = true;
  } else {
    near_station = false;
  }

  if (!near_station) {
    RCLCPP_WARN(node_->get_logger(),
      "Not near any station (threshold=%.2fm). Robot=[%.3f, %.3f] closest_dist=%.3f",
      near_ws_dist, rx, ry, std::sqrt(best_dist_sq));
  } else {
    RCLCPP_INFO(node_->get_logger(),
      "Selected closest station line: [%.6f, %.6f, %.6f, %.6f] dist=%.3f",
      x1_sel, y1_sel, x2_sel, y2_sel, std::sqrt(best_dist_sq));
  }

  // ---- Compute lateral correction to bring robot onto the selected line ----
  // Line: P1=(x1_sel,y1_sel), P2=(x2_sel,y2_sel)
  const double vx = x2_sel - x1_sel;
  const double vy = y2_sel - y1_sel;
  const double v_norm_sq = vx * vx + vy * vy;

  if (v_norm_sq < 1e-12) {
    RCLCPP_ERROR(node_->get_logger(),
      "Invalid station line: P1 and P2 are too close (degenerate).");
  }

  // Project robot position onto infinite line to get closest point P
  const double wx = rx - x1_sel;
  const double wy = ry - y1_sel;

  const double t = (wx * vx + wy * vy) / v_norm_sq;

  const double px = x1_sel + t * vx;
  const double py = y1_sel + t * vy;

  // Displacement from robot to line (in map frame)
  const double dx_line = px - rx;
  const double dy_line = py - ry;

  // Signed lateral correction in base_link (+left, -right)
  const double lateral = dx_line * left_x + dy_line * left_y;

  // Optional: deadband / clamp
  const double max_corr = 0.30; // 30 cm safety clamp
  double lateral_cmd = lateral;

  if (std::fabs(lateral_cmd) < tol_) {
    lateral_cmd = 0.0;
    speed = 0.0;
  } else {
    lateral_cmd = std::clamp(lateral_cmd, -max_corr, max_corr);
  }

  RCLCPP_INFO(node_->get_logger(),
    "Selected station line: [%.6f, %.6f, %.6f, %.6f]",
    x1_sel, y1_sel, x2_sel, y2_sel);

  RCLCPP_INFO(node_->get_logger(),
    "Robot: [%.6f, %.6f] yaw=%.6f | Closest-on-line: [%.6f, %.6f] | lateral=%.6f (cmd=%.6f)",
    rx, ry, yaw_robot, px, py, lateral, lateral_cmd);

  if(!near_station || ws_flat_.empty() || v_norm_sq < 1e-12 ){
    goal_.target.x = 0.0;
    goal_.target.y = 0.0;
    goal_.target.z = 0.0;

    goal_.speed = 0.0;
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  }
  else{
    goal_.target.x = 0.0;
    goal_.target.y = lateral_cmd;
    goal_.target.z = 0.0;

    goal_.speed = std::copysign(std::fabs(speed), lateral_cmd);
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  }
}

void CenterToStationAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}

BT::NodeStatus CenterToStationAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CenterToStationAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CenterToStationAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::CenterToStationAction>(name, "drive_on_lateral", config);
    };

  factory.registerBuilder<nav2_behavior_tree::CenterToStationAction>("CenterToStation", builder);
}
