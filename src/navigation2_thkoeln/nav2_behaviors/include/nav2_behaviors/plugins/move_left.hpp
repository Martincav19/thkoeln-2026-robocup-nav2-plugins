
#ifndef NAV2_BEHAVIORS__PLUGINS__MOVE_LEFT_HPP_
#define NAV2_BEHAVIORS__PLUGINS__MOVE_LEFT_HPP_

#include <memory>

#include "drive_on_lateral.hpp"
#include "nav2_msgs/action/move_left.hpp"

using MoveLeftAction = nav2_msgs::action::MoveLeft;

namespace nav2_behaviors
{
  class MoveLeft : public DriveOnLateral<nav2_msgs::action::MoveLeft>
  {
  public:
    using MoveLeftActionGoal = MoveLeftAction::Goal;
    using MoveLeftActionResult = MoveLeftAction::Result;

    ResultStatus onRun(const std::shared_ptr<const MoveLeftActionGoal> command) override;
  };
}   // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__MOVE_LEFT_HPP_

