
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CAN_ROTATE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CAN_ROTATE_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the CanRotate
 * service returns true and FAILURE otherwise
 */
class CanRotateCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CanRotateCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  CanRotateCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  CanRotateCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("can_rotate", false ,"Answers the question: Can the vehicle Rotate right now?")
    };
  }


private:
  //Variables
  rclcpp::Node::SharedPtr node_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CAN_ROTATE_CONDITION_HPP_
