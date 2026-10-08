
#include "nav2_behaviors/plugins/move_forward.hpp"

namespace nav2_behaviors
{

ResultStatus MoveForward::onRun(const std::shared_ptr<const MoveForwardAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Driving forward in Y and Z not supported, will only move in X.");
    return ResultStatus{Status::FAILED, MoveForwardActionResult::INVALID_INPUT};
  }

  // Silently ensure that both the speed and direction are positive.
  command_x_ = std::fabs(command->target.x);
  command_speed_ = std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return ResultStatus{Status::FAILED, MoveForwardActionResult::TF_ERROR};
  }

  return ResultStatus{Status::SUCCEEDED, MoveForwardActionResult::NONE};
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::MoveForward, nav2_core::Behavior)
