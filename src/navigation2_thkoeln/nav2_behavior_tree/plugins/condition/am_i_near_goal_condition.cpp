
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <tf2/utils.h>

#include "nav2_behavior_tree/plugins/condition/am_i_near_goal_condition.hpp"

namespace nav2_behavior_tree
{

AmINearGoalCondition::AmINearGoalCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
  
  RCLCPP_INFO(node->get_logger(), "Constructor AmINearGoalCondition");
}

AmINearGoalCondition::~AmINearGoalCondition()
{
  cleanup();
}

void AmINearGoalCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  //node_->get_parameter("min_angle_for_rotation", min_angle_for_rotation);

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  //RCLCPP_INFO(node_->get_logger(), "AmINearGoalCondition initialized");

}

BT::NodeStatus AmINearGoalCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (isNearGoal()) {
    //RCLCPP_INFO(node_->get_logger(), "AmINearGoalCondition: SUCCESS – rotation is needed");
    return BT::NodeStatus::SUCCESS;
  } else {
    //RCLCPP_INFO(node_->get_logger(), "AmINearGoalCondition: FAILURE – rotation not needed");
    return BT::NodeStatus::FAILURE;
  }
}

bool AmINearGoalCondition::isNearGoal()
{
  //Get Variables from Blackboard
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_WARN(node_->get_logger(), "AmINearGoal: missing input [goal]");
    return false;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal.header.frame_id, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  double near_goal_dist;
  getInput("near_goal_dist", near_goal_dist); //1.2

  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;

  //if the car is inside the goal reached tolerance and the alignment is greater than 10 degrees
  if((dx * dx + dy * dy) <= (near_goal_dist * near_goal_dist)){
    RCLCPP_INFO(node_->get_logger(), "Robot Near the Goal!");
    return true;
  }
  else{
    return false;
  }
}

}  // namespace nav2_behavior_tree


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::AmINearGoalCondition>("AmINearGoal");
}

//  Possible improvements:
//  -When the goal is near stop the function
//  -Change the N of the number of points to be a parameter
//  -Change the threshold angle to be a parameter
//  -Important assumption of the package: path is always computed before entering this package, this way current_pose = initial_pose

//Continuar manana: poner los argumentos en el config!!!