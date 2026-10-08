
#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "nav2_behaviors/plugins/where_am_i.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors
{

WhereAmI::WhereAmI()
: TimedBehavior<WhereAmIAction>(){
}

WhereAmI::~WhereAmI() = default;

void WhereAmI::onConfigure()
{
  RCLCPP_INFO(logger_,"Configure WhereAmI.");
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Add this:
  nav2_util::declare_parameter_if_not_declared(
    node, "forward_distance", rclcpp::ParameterValue(1.0));
  node->get_parameter("forward_distance", forward_distance_);

  RCLCPP_INFO(logger_, "WhereAmI forward_distance: %.3f m", forward_distance_);
  RCLCPP_INFO(logger_, "Finish Configure WhereAmI.");

}

ResultStatus WhereAmI::onRun(const std::shared_ptr<const WhereAmIActionGoal> command)
{
  command_activate_plugin_     = command->activate_plugin;
  return ResultStatus{Status::SUCCEEDED, WhereAmIActionResult::NONE};
}

ResultStatus WhereAmI::onCycleUpdate()
{

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, WhereAmIActionResult::TF_ERROR};
  }
  
  const auto & p = current_pose.pose.position;
  const auto & q = current_pose.pose.orientation;

  const double yaw = tf2::getYaw(q);

  const double x1 = p.x;
  const double y1 = p.y;

  const double x2 = p.x + forward_distance_ * std::cos(yaw);
  const double y2 = p.y + forward_distance_ * std::sin(yaw);

  RCLCPP_INFO(
    logger_,
    "POSE: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n"
    "LINE: [%.6f, %.6f, %.6f, %.6f]",
    p.x, p.y, p.z,
    q.x, q.y, q.z, q.w,
    x1, y1, x2, y2
  );

  return ResultStatus{Status::SUCCEEDED, WhereAmIActionResult::NONE};
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::WhereAmI, nav2_core::Behavior)
