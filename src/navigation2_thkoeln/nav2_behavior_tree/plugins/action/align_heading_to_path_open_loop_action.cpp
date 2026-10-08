
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

#include "nav2_behavior_tree/plugins/action/align_heading_to_path_open_loop_action.hpp"

namespace nav2_behavior_tree
{

AlignHeadingToPathOpenLoopAction::AlignHeadingToPathOpenLoopAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf) {

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "global_frame", this);
  
  RCLCPP_INFO(node->get_logger(), "Constructor AlignHeadingToPathOpenLoopAction");

}

void AlignHeadingToPathOpenLoopAction::initialize()
{

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_); 
  node_->get_parameter("path_points", path_points_); 

  if (getInput("ideal_heading_direction", ideal_heading_direction_)) {
      RCLCPP_INFO(
          node_->get_logger(),
          "AlignHeadingToPathOpenLoop: ideal_heading_direction = '%s'",
          ideal_heading_direction_.c_str()
      );
  } else {
      RCLCPP_WARN(
          node_->get_logger(),
          "AlignHeadingToPathOpenLoop: missing or empty parameter [ideal_heading_direction]"
      );
      ideal_heading_direction_ = "front";
  }

  double time_allowance;
  getInput("time_allowance", time_allowance);
  
  //HERE: COMPUTE THE YAW USING THE PATH AND THE FUNCTION OF THE VECTOR YOU USE IN ROTATION NEEDED CONDITION AND GIVE IT AS THE TARGET YAW
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "AlignHeadingToPathOpenLoop: missing or empty [path]");
  }

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_WARN(node_->get_logger(), "RotationNeeded: missing input [goal]");
  }

  getInput("is_recovery", is_recovery_);

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
  }

  double near_goal_dist;
  getInput("near_goal_dist", near_goal_dist);

  auto firstN = getFirstN(path, path_points_);
  auto target_vector = getAveragedDirection(firstN, true);   // p[0] → p[1..n]

  geometry_msgs::msg::Vector3 current_vector;
  double yaw_robot = tf2::getYaw(current_pose.pose.orientation);
  current_vector.x = std::cos(yaw_robot);
  current_vector.y = std::sin(yaw_robot);
  current_vector.z = 0.0;

  geometry_msgs::msg::Vector3 current_vector_inversed;
  current_vector_inversed.x = -current_vector.x;
  current_vector_inversed.y = -current_vector.y;
  current_vector_inversed.z = 0.0;

  geometry_msgs::msg::Vector3 goal_vector;
  double yaw_goal = tf2::getYaw(goal.pose.orientation);
  goal_vector.x = std::cos(yaw_goal);
  goal_vector.y = std::sin(yaw_goal);
  goal_vector.z = 0.0;

  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;

  geometry_msgs::msg::Vector3 source_vector;

  //Orient to goal if close enough
  if((dx * dx + dy * dy) <= (near_goal_dist * near_goal_dist)){
    //Target vector is the final orientation
    ideal_heading_direction_ = "front";
    target_vector = goal_vector;
    RCLCPP_INFO(
      node_->get_logger(),
      "AlignHeadingToPathOpenLoop: inside near_goal_dist"
    );
    double dist = signedAngle(current_vector, target_vector);
    goal_.target_yaw = dist;
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
    return;
  }

  //Do recovery taken the side of the robot shortest to path
  if(is_recovery_){
    //Source vector is the shortest of the current vectors to the path
    RCLCPP_INFO(
      node_->get_logger(),
      "AlignHeadingToPathOpenLoop: being used as recovery"
    );
    double dist1 = signedAngle(current_vector, target_vector);
    double dist2 = signedAngle(current_vector_inversed, target_vector);

    if (std::fabs(dist1) < std::fabs(dist2)){
      goal_.target_yaw = dist1;
    }
    else{
      goal_.target_yaw = dist2;
    }
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
    return;
  }

  //Source vector is the one given by the blackboard
  if(ideal_heading_direction_ == "back"){
    source_vector = current_vector_inversed;
  }
  else{
    source_vector = current_vector;
  }

  double dist = signedAngle(source_vector, target_vector);
  goal_.target_yaw = dist;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

}

void AlignHeadingToPathOpenLoopAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus AlignHeadingToPathOpenLoopAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AlignHeadingToPathOpenLoopAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus AlignHeadingToPathOpenLoopAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

std::vector<geometry_msgs::msg::PoseStamped> AlignHeadingToPathOpenLoopAction::getFirstN(const nav_msgs::msg::Path & path, size_t n) {
    n = std::min(n, path.poses.size());
    return std::vector<geometry_msgs::msg::PoseStamped>(path.poses.begin(), path.poses.begin() + n);
}

geometry_msgs::msg::Vector3 AlignHeadingToPathOpenLoopAction::getAveragedDirection(const std::vector<geometry_msgs::msg::PoseStamped> &pts, bool from_first)
{
    geometry_msgs::msg::Vector3 dir;
    dir.x = dir.y = dir.z = 0.0;

    if (pts.size() < 2) {
        // fallback: arbitrary unit vector
        dir.x = 1.0;
        return dir;
    }

    double sumX = 0.0;
    double sumY = 0.0;

    if (from_first) {
        //  Case 1: first N points → direction from pts[0] outward
        const auto &p0 = pts.front().pose.position;

        for (size_t i = 1; i < pts.size(); ++i) {
            const auto &pi = pts[i].pose.position;

            double vx = pi.x - p0.x;
            double vy = pi.y - p0.y;

            double norm = std::sqrt(vx * vx + vy * vy);
            if (norm > 1e-6) {
                vx /= norm;
                vy /= norm;
            }

            sumX += vx;
            sumY += vy;
        }
    } else {
        //  Case 2: last N points → direction from last point inwards
        const auto &pn = pts.back().pose.position;

        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const auto &pi = pts[i].pose.position;

            // vector from last point -> inner points (inwards)
            double vx = pi.x - pn.x;
            double vy = pi.y - pn.y;

            double norm = std::sqrt(vx * vx + vy * vy);
            if (norm > 1e-6) {
                vx /= norm;
                vy /= norm;
            }

            sumX += vx;
            sumY += vy;
        }
    }

    double norm = std::sqrt(sumX * sumX + sumY * sumY);
    if (norm > 1e-6) {
        sumX /= norm;
        sumY /= norm;
    } else {
        // degenerate: all points equal → just pick something
        sumX = 1.0;
        sumY = 0.0;
    }

    dir.x = sumX;
    dir.y = sumY;
    dir.z = 0.0;
    return dir;
}

double AlignHeadingToPathOpenLoopAction::signedAngle(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b)
{
    double dot = a.x*b.x + a.y*b.y;
    double det = a.x*b.y - a.y*b.x; // 2D cross product (scalar)

    return std::atan2(det, dot);  
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::AlignHeadingToPathOpenLoopAction>(name, "align_heading_to_path_open_loop", config);
    };

  factory.registerBuilder<nav2_behavior_tree::AlignHeadingToPathOpenLoopAction>("AlignHeadingToPathOpenLoop", builder);
}
