
#include "nav2_behavior_tree/plugins/condition/can_rotate_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

CanRotateCondition::CanRotateCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{}

void CanRotateCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

}

BT::NodeStatus CanRotateCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  bool can_rotate;
  getInput("can_rotate", can_rotate);

  if (can_rotate) {
    RCLCPP_INFO(node_->get_logger(), "CanRotate: SUCCESS - possible to rotatey");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(node_->get_logger(), "CanRotate: FAILURE – cannot rotate");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CanRotateCondition>("CanRotate");
}
