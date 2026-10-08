
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CENTER_TO_STATION_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CENTER_TO_STATION_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_lateral.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::CenterToStation
 */
class CenterToStationAction : public BtActionNode<nav2_msgs::action::DriveOnLateral>
{
  using Action = nav2_msgs::action::DriveOnLateral;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::CenterToStationAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  CenterToStationAction(
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
        BT::InputPort<double>("center_to_station_speed", 0.15, "Speed at which to center the station"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for docking to the station"),
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
  double tol_;
  std::vector<double> ws_flat_;
  
  double angleBetween(const geometry_msgs::msg::Vector3 &, const geometry_msgs::msg::Vector3 &);

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CENER_TO_STATION_ACTION_HPP_
