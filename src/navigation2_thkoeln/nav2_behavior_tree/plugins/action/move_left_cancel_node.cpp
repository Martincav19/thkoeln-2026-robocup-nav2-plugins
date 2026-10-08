
#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/move_left_cancel_node.hpp"

namespace nav2_behavior_tree
{

MoveLeftCancel::MoveLeftCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_msgs::action::MoveLeft>(xml_tag_name, action_name, conf)
{
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::MoveLeftCancel>(
        name, "move_left", config);
    };

  factory.registerBuilder<nav2_behavior_tree::MoveLeftCancel>(
    "CancelMoveLeft", builder);
}
