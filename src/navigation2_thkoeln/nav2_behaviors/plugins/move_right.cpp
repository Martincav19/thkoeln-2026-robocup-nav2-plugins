
#include "nav2_behaviors/plugins/move_right.hpp"

namespace nav2_behaviors
{

ResultStatus MoveRight::onRun(const std::shared_ptr<const MoveRightAction::Goal> command)
{
  if (command->target.x != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Driving right in X and Z not supported, will only move in Y.");
    return ResultStatus{Status::FAILED, MoveRightActionResult::INVALID_INPUT};
  }

  // Silently ensure that both the speed and direction are negative.
  command_y_ = -std::fabs(command->target.y);
  command_speed_ = -std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return ResultStatus{Status::FAILED, MoveRightActionResult::TF_ERROR};
  }

  return ResultStatus{Status::SUCCEEDED, MoveRightActionResult::NONE};
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::MoveRight, nav2_core::Behavior)
