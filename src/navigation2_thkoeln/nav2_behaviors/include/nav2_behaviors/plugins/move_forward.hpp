
#ifndef NAV2_BEHAVIORS__PLUGINS__MOVE_FORWARD_HPP_
#define NAV2_BEHAVIORS__PLUGINS__MOVE_FORWARD_HPP_

#include <memory>

#include "drive_on_heading.hpp"
#include "nav2_msgs/action/move_forward.hpp"

using MoveForwardAction = nav2_msgs::action::MoveForward;

namespace nav2_behaviors
{
  class MoveForward : public DriveOnHeading<nav2_msgs::action::MoveForward>
  {
  public:
    using MoveForwardActionGoal = MoveForwardAction::Goal;
    using MoveForwardActionResult = MoveForwardAction::Result;

    ResultStatus onRun(const std::shared_ptr<const MoveForwardActionGoal> command) override;
  };
}   // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__MOVE_FORWARD_HPP_

