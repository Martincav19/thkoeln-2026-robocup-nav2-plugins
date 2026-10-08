
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ALIGN_HEADING_TO_PATH_OPEN_LOOP_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ALIGN_HEADING_TO_PATH_OPEN_LOOP_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::AlignHeadingToPathOpenLoop
 */
class AlignHeadingToPathOpenLoopAction : public BtActionNode<nav2_msgs::action::Spin>
{
  using Action = nav2_msgs::action::Spin;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::AlignHeadingToPathOpenLoopAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  AlignHeadingToPathOpenLoopAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

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
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
        BT::InputPort<std::string>("ideal_heading_direction", "front", "Direction considered as the heading of the robot, can be either front or back"),
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to Follow"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for aligning"),
        BT::InputPort<double>("near_goal_dist", 1.0, "Distance to be considered to be near the goal"),
        BT::InputPort<bool>("is_recovery", false, "True if recovery"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The spin behavior error code")
      });
  }

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

private:
  bool is_recovery_;
  std::string global_frame_, robot_base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  int path_points_;
  std::string ideal_heading_direction_;

  std::vector<geometry_msgs::msg::PoseStamped> getFirstN(const nav_msgs::msg::Path &, size_t);
  geometry_msgs::msg::Vector3 getAveragedDirection(const std::vector<geometry_msgs::msg::PoseStamped> &, bool);
  double signedAngle(const geometry_msgs::msg::Vector3 &, const geometry_msgs::msg::Vector3 &);
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ALIGN_HEADING_TO_PATH_OPEN_LOOP_ACTION_HPP_
