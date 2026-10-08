
#ifndef NAV2_BEHAVIORS__PLUGINS__MOVE_RIGHT_HPP_
#define NAV2_BEHAVIORS__PLUGINS__MOVE_RIGHT_HPP_

#include <memory>

#include "drive_on_lateral.hpp"
#include "nav2_msgs/action/move_right.hpp"

using MoveRightAction = nav2_msgs::action::MoveRight;

namespace nav2_behaviors
{
  class MoveRight : public DriveOnLateral<nav2_msgs::action::MoveRight>
  {
  public:
    using MoveRightActionGoal = MoveRightAction::Goal;
    using MoveRightActionResult = MoveRightAction::Result;

    ResultStatus onRun(const std::shared_ptr<const MoveRightActionGoal> command) override;
  };
}   // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__MOVE_RIGHT_HPP_

