
#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "nav2_behaviors/plugins/can_rotate.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors
{

CanRotate::CanRotate()
: TimedBehavior<CanRotateAction>(),
  spinning_simulation_time_(0.0)
{
}

CanRotate::~CanRotate() = default;

void CanRotate::onConfigure()
{
  RCLCPP_INFO(logger_,"Configure CanRotate.");
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    "spinning_simulation_time", rclcpp::ParameterValue(20.0));
  node->get_parameter("spinning_simulation_time", spinning_simulation_time_);

  RCLCPP_INFO(logger_,"Finish Configure CanRotate.");

}

ResultStatus CanRotate::onRun(const std::shared_ptr<const CanRotateActionGoal> command)
{

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, CanRotateActionResult::TF_ERROR};
  }

  vel = command->angular_speed;
  angle = command->relative_yaw;   

  return ResultStatus{Status::SUCCEEDED, CanRotateActionResult::NONE};
}

ResultStatus CanRotate::onCycleUpdate()
{

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, CanRotateActionResult::TF_ERROR};
  }

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(pose2d)) {
    RCLCPP_WARN(logger_, "Rotation not possible in this position");
    return ResultStatus{Status::FAILED, CanRotateActionResult::COLLISION_AHEAD};
  }
  else{
    RCLCPP_INFO(logger_,"Rotation possible in this position.");
    return ResultStatus{Status::SUCCEEDED, CanRotateActionResult::NONE};
  }

  return ResultStatus{Status::RUNNING, CanRotateActionResult::NONE};
}

bool CanRotate::isCollisionFree(geometry_msgs::msg::Pose2D & pose2d)
{
  // Simulate ahead by spinning_simulation_time_ in cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  const int max_cycle_count = static_cast<int>(cycle_frequency_ * spinning_simulation_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change = vel * (cycle_count / cycle_frequency_);
    pose2d.theta = init_pose.theta + sim_position_change;
    cycle_count++;

    if (angle - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::CanRotate, nav2_core::Behavior)
