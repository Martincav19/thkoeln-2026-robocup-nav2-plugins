
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_RIGHT_CANCEL_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_RIGHT_CANCEL_NODE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/move_right.hpp"

#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::MoveRight
 */
class MoveRightCancel : public BtCancelActionNode<nav2_msgs::action::MoveRight>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::MMoveRightAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  MoveRightCancel(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_RIGHT_CANCEL_NODE_HPP_
