
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__AM_I_NEAR_GOAL_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__AM_I_NEAR_GOAL_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_behavior_tree/bt_utils.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the AmINearGoal
 * service returns true and FAILURE otherwise
 */
class AmINearGoalCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::AmINearGoalCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  AmINearGoalCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  AmINearGoalCondition() = delete;

  ~AmINearGoalCondition() override;

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

  bool isNearGoal();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
      BT::InputPort<double>("near_goal_dist", 1.0, "Distance to be considered to be near the goal")
    };
  }

protected:
  /**
   * @brief Cleanup function
   */
  void cleanup()
  {}

private:
  //Variables
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  //double min_angle_for_rotation;
  std::string robot_base_frame_;
  double transform_tolerance_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WHERE_AM_I_CONDITION_HPP_
