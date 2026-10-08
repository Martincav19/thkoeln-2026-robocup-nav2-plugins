
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_TO_STATION_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_TO_STATION_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/dock_to_station.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::DockToStation
 */
class DockToStationAction : public BtActionNode<nav2_msgs::action::DockToStation>
{
  using Action = nav2_msgs::action::DockToStation;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::DockToStationAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  DockToStationAction(
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
        BT::InputPort<bool>("use_docking", false, "True if dock to station wanted"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for docking to the station"),
        BT::InputPort<double>("standoff_distance", 0.200, "Distance to be separated from the station"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The dock to station behavior error code")
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
  std::string global_frame_, robot_base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;

  std::vector<double> ws_flat_;
  
  double angleBetween(const geometry_msgs::msg::Vector3 &, const geometry_msgs::msg::Vector3 &);
  int decideDockingSide(const geometry_msgs::msg::Vector3 &, const geometry_msgs::msg::Vector3 &);

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_TO_STATION_ACTION_HPP_
