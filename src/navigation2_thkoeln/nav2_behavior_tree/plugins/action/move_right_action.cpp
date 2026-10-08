
#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/move_right_action.hpp"

namespace nav2_behavior_tree
{

MoveRightAction::MoveRightAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::MoveRight>(xml_tag_name, action_name, conf)
{
}

void nav2_behavior_tree::MoveRightAction::initialize()
{
  double dist;
  getInput("move_right_dist", dist);
  double speed;
  getInput("move_right_speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  goal_.target.x = 0.0;
  goal_.target.y = dist;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void MoveRightAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  increment_recovery_count();
}

BT::NodeStatus MoveRightAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveRightAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveRightAction::on_cancelled()
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
      return std::make_unique<nav2_behavior_tree::MoveRightAction>(
        name, "move_right", config);
    };

  factory.registerBuilder<nav2_behavior_tree::MoveRightAction>("MoveRight", builder);
}
