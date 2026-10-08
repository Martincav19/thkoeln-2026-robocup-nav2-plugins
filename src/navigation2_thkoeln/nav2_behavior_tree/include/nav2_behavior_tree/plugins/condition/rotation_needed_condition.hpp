
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ROTATION_NEEDED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ROTATION_NEEDED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "tf2_ros/buffer.h"
#include "nav2_behavior_tree/bt_utils.hpp"
#include <cmath>

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a rotation is eventually needed
 * to reach the desired goal and FAILURE otherwise
 */
class RotationNeededCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RotationNeededCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RotationNeededCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  RotationNeededCondition() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::RotationNeededCondition
   */
  ~RotationNeededCondition() override;

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
   * @brief Checks if the robot needs to rotate
   * @return bool true when rotation needed, false otherwise
   */
  bool isRotationNeeded();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to Follow"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
      BT::InputPort<double>("threshold_angle", M_PI / 4, "Angle as threshold for the rotation needed"),
      BT::InputPort<double>("threshold_goal_angle", M_PI / 18, "Angle as threshold for the rotation needed"),
      BT::InputPort<double>("near_goal_dist", 1.2, "Distance to be considered near the goal"),
      BT::InputPort<bool>("always_rotate", true ,"If set to false, avoids aligning to path if initial pose is not completely opposite to goal pose"),
      BT::OutputPort<std::string>("ideal_heading_direction", "Direction considered as the heading of the robot, can be either front or back")
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
  double transform_tolerance_;
  //double min_angle_for_rotation;
  std::string robot_base_frame_;

  //Functions
  std::vector<geometry_msgs::msg::PoseStamped> getFirstN(const nav_msgs::msg::Path &, size_t);
  std::vector<geometry_msgs::msg::PoseStamped> getLastN(const nav_msgs::msg::Path &, size_t);
  geometry_msgs::msg::Vector3 getAveragedDirection(const std::vector<geometry_msgs::msg::PoseStamped> &, bool);
  double angleBetween(const geometry_msgs::msg::Vector3 &, const geometry_msgs::msg::Vector3 &);
  bool checkAngles(double, double, double, bool);

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ROTATION_NEEDED_CONDITION_HPP_
