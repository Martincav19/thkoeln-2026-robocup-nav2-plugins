
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_RIGHT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_RIGHT_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/move_right.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::MoveRight
 */
class MoveRightAction : public BtActionNode<nav2_msgs::action::MoveRight>
{
  using Action = nav2_msgs::action::MoveRight;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::MoveRightAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  MoveRightAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;


  /**
 * @brief Function to perform some user-defined operation upon successful completion of the action
 */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("move_right_dist", 0.15, "Distance to move right"),
        BT::InputPort<double>("move_right_speed", 0.025, "Speed at which to move right"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for reversing"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The move right behavior server error code")
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_RIGHT_ACTION_HPP_
