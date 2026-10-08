
#include "nav2_behaviors/plugins/move_left.hpp"

namespace nav2_behaviors
{

ResultStatus MoveLeft::onRun(const std::shared_ptr<const MoveLeftAction::Goal> command)
{
  if (command->target.x != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Driving left in X and Z not supported, will only move in Y.");
    return ResultStatus{Status::FAILED, MoveLeftActionResult::INVALID_INPUT};
  }

  // Silently ensure that both the speed and direction are positive.
  command_y_ = std::fabs(command->target.y);
  command_speed_ = std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return ResultStatus{Status::FAILED, MoveLeftActionResult::TF_ERROR};
  }
  RCLCPP_INFO(
    logger_,
    "OnRun successful");
  return ResultStatus{Status::SUCCEEDED, MoveLeftActionResult::NONE};
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::MoveLeft, nav2_core::Behavior)
