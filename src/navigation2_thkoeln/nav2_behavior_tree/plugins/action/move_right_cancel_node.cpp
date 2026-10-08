
#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/move_right_cancel_node.hpp"

namespace nav2_behavior_tree
{

MoveRightCancel::MoveRightCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_msgs::action::MoveRight>(xml_tag_name, action_name, conf)
{
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::MoveRightCancel>(
        name, "move_right", config);
    };

  factory.registerBuilder<nav2_behavior_tree::MoveRightCancel>(
    "CancelMoveRight", builder);
}
