
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

#include "nav2_behavior_tree/plugins/condition/rotation_needed_condition.hpp"

namespace nav2_behavior_tree
{

RotationNeededCondition::RotationNeededCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
  
  RCLCPP_INFO(node->get_logger(), "Constructor RotationNeededCondition");
}

RotationNeededCondition::~RotationNeededCondition()
{
  cleanup();
}

void RotationNeededCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  //node_->get_parameter("min_angle_for_rotation", min_angle_for_rotation);

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  //RCLCPP_INFO(node_->get_logger(), "RotationNeededCondition initialized");

}

BT::NodeStatus RotationNeededCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (isRotationNeeded()) {
    //RCLCPP_INFO(node_->get_logger(), "RotationNeededCondition: SUCCESS – rotation is needed");
    return BT::NodeStatus::SUCCESS;
  } else {
    //RCLCPP_INFO(node_->get_logger(), "RotationNeededCondition: FAILURE – rotation not needed");
    return BT::NodeStatus::FAILURE;
  }
}

bool RotationNeededCondition::isRotationNeeded()
{
  //Get Variables from Blackboard
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_WARN(node_->get_logger(), "RotationNeeded: missing input [goal]");
    return false;
  }

  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "RotationNeeded: missing or empty [path]");
    return false;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal.header.frame_id, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  double threshold_goal_angle;
  getInput("threshold_goal_angle", threshold_goal_angle); //10 degrees

  double near_goal_dist;
  getInput("near_goal_dist", near_goal_dist); //1.2

  double threshold_angle;
  getInput("threshold_angle", threshold_angle);   //90 degrees

  bool always_rotate;
  getInput("always_rotate", always_rotate);

  auto firstN = getFirstN(path, 5);
  auto lastN  = getLastN(path, 5);

  auto vector_firstN = getAveragedDirection(firstN, true);   // p[0] → p[1..n]
  auto vector_lastN = getAveragedDirection(lastN, false); // plast → inside

  geometry_msgs::msg::Vector3 vector_goal;
  double yaw_goal = tf2::getYaw(goal.pose.orientation);
  vector_goal.x = std::cos(yaw_goal);
  vector_goal.y = std::sin(yaw_goal);
  vector_goal.z = 0.0;

  geometry_msgs::msg::Vector3 vector_initial_pose;
  double yaw_robot = tf2::getYaw(current_pose.pose.orientation);
  vector_initial_pose.x = std::cos(yaw_robot);
  vector_initial_pose.y = std::sin(yaw_robot);
  vector_initial_pose.z = 0.0;

  double initial_angle = angleBetween(vector_firstN, vector_initial_pose);
  double final_angle = angleBetween(vector_lastN, vector_goal);
  double goal_diff_angle = angleBetween(vector_initial_pose, vector_goal);

  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;

  //if the car is inside the goal reached tolerance and the alignment is greater than 10 degrees
  if((dx * dx + dy * dy) <= (near_goal_dist * near_goal_dist) && goal_diff_angle >= threshold_goal_angle){
    return true;
  }
  else if((dx * dx + dy * dy) <= (near_goal_dist * near_goal_dist)){
    return false;
  }
  else{
    return checkAngles(initial_angle, final_angle, threshold_angle, always_rotate);
  }
}

std::vector<geometry_msgs::msg::PoseStamped> RotationNeededCondition::getFirstN(const nav_msgs::msg::Path & path, size_t n) {
    n = std::min(n, path.poses.size());
    return std::vector<geometry_msgs::msg::PoseStamped>(path.poses.begin(), path.poses.begin() + n);
}

std::vector<geometry_msgs::msg::PoseStamped> RotationNeededCondition::getLastN(const nav_msgs::msg::Path & path, size_t n) {
    n = std::min(n, path.poses.size());
    return std::vector<geometry_msgs::msg::PoseStamped>(path.poses.end() - n, path.poses.end());
}

geometry_msgs::msg::Vector3 RotationNeededCondition::getAveragedDirection(const std::vector<geometry_msgs::msg::PoseStamped> &pts, bool from_first)
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

double RotationNeededCondition::angleBetween(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b)
{
    double dot = a.x * b.x + a.y * b.y + a.z * b.z;
    double magA = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    double magB = std::sqrt(b.x*b.x + b.y*b.y + b.z*b.z);

    if (magA < 1e-9 || magB < 1e-9)
        return 0.0; // undefined, but safe fallback

    double c = dot / (magA * magB);

    // Clamp to [-1,1] to avoid NaN due to float precision
    c = std::max(-1.0, std::min(1.0, c));

    return std::acos(c);  // radians
}

bool RotationNeededCondition::checkAngles(double initial_angle, double final_angle, double T, bool always_rotate)
{

    //Case 2
    if(final_angle < T){

      //D Zone
      if(initial_angle < T){
        setOutput("ideal_heading_direction", "back");
        return true;
      }
      //C Zone
      else if(initial_angle > (M_PI - T)){
        setOutput("ideal_heading_direction", "back");
        return false;
      }
      //A-B Zone
      else{
        setOutput("ideal_heading_direction", "back");
        return true;
      }
    }
    //Case 1
    else if(final_angle > (M_PI - T)){

      //D Zone
      if(initial_angle < T){
        setOutput("ideal_heading_direction", "front");
        return false;
      }
      //C Zone
      else if(initial_angle > (M_PI - T)){
        setOutput("ideal_heading_direction", "front");
        return true;
      }
      //A-B Zone
      else{
        setOutput("ideal_heading_direction", "front");
        return true;
      }

    }
    //Case 3
    else{

      //D Zone
      if(initial_angle < T){
        setOutput("ideal_heading_direction", "front");
        return false;
      }
      //C Zone
      else if(initial_angle > (M_PI - T)){
        setOutput("ideal_heading_direction", "back");
        return false;
      }
      //A-B Zone
      else{
        setOutput("ideal_heading_direction", "front");
        return always_rotate;
      }

    }
    // return true if they match (both above OR both below)
    
}


}  // namespace nav2_behavior_tree


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RotationNeededCondition>("RotationNeeded");
}

//  Possible improvements:
//  -When the goal is near stop the function
//  -Change the N of the number of points to be a parameter
//  -Change the threshold angle to be a parameter
//  -Important assumption of the package: path is always computed before entering this package, this way current_pose = initial_pose

//Continuar manana: poner los argumentos en el config!!!