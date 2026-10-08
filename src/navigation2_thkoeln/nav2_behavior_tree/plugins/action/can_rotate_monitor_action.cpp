
#include <memory>

#include "nav2_behavior_tree/plugins/action/can_rotate_monitor_action.hpp"

namespace nav2_behavior_tree
{

CanRotateMonitorAction::CanRotateMonitorAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::CanRotate>(xml_tag_name, action_name, conf) {}

void CanRotateMonitorAction::initialize()
{
  double angular_speed;
  getInput("angular_speed", angular_speed);

  double relative_yaw;
  getInput("relative_yaw", relative_yaw);

  goal_.angular_speed = angular_speed;
  goal_.relative_yaw = relative_yaw;
}

void CanRotateMonitorAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}

BT::NodeStatus CanRotateMonitorAction::on_success()
{
  //RCLCPP_INFO(node_->get_logger(),"Can Rotate here");
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("can_rotate", true);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CanRotateMonitorAction::on_aborted()
{
  RCLCPP_INFO(node_->get_logger(),"Can NOT Rotate here");
  setOutput("error_code_id", result_.result->error_code);
  setOutput("can_rotate", false);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CanRotateMonitorAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("can_rotate", true);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::CanRotateMonitorAction>(name, "can_rotate", config);
    };

  factory.registerBuilder<nav2_behavior_tree::CanRotateMonitorAction>("CanRotateMonitor", builder);
}

//Make it possible to give the angle and speed params in the bt