
#include "dwb_critics/make_progress.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"
#include "nav_2d_utils/path_ops.hpp"
#include <algorithm>  // std::min, std::max
#include <limits>

PLUGINLIB_EXPORT_CLASS(dwb_critics::MakeProgressCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void MakeProgressCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  front_window_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".front_window", 30);  //Analizing 30 index in the front direction of robot.

  back_window_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".back_window", 30);   //Analizing 30 index in the rear direction of robot.
  
  no_progress_cost_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".no_progress_cost", 10.0);  //As if no progress would be equal to going back 10 index.

  goal_threshold_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".goal_threshold", 1.2);  //Goal threshold to be considered "near Goal"
  
  reset();
}

void MakeProgressCritic::reset()
{

}

bool MakeProgressCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  enabled_ = true;
  // Densify (optional but recommended to reduce “ties”)
  plan_ = nav_2d_utils::adjustPlanResolution(global_plan, costmap_->getResolution());

  if (plan_.poses.size() != global_plan.poses.size()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "MakeProgressCritic"), "Adjusted global plan resolution, added %zu points",
      plan_.poses.size() - global_plan.poses.size());
  }

  // Find closest index to current pose (you can also window this using last start_idx_)
  start_idx_ = closestIndex(plan_, pose, 0, static_cast<int>(plan_.poses.size()) - 1);

  const int n = static_cast<int>(plan_.poses.size());
  const int remaining = (n - 1) - start_idx_;

  double dx = pose.x - goal.x;
  double dy = pose.y - goal.y;
  double sq_dist = dx * dx + dy * dy;

  if (sq_dist < goal_threshold_ * goal_threshold_ || remaining < front_window_){
    enabled_ = false;
  }

  return true;
}

double MakeProgressCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty() || plan_.poses.empty()) {
    return 0.0;
  }

  if (!enabled_) return 0.0;

  const auto & end_pose = traj.poses.back(); // predicted end pose :contentReference[oaicite:3]{index=3}

  const int n = static_cast<int>(plan_.poses.size());
  const int i0 = std::max(0, start_idx_ - back_window_);
  const int i1 = std::min(n - 1, start_idx_ + front_window_);

  const int end_idx = closestIndex(plan_, end_pose, i0, i1);
  const int delta = end_idx - start_idx_;

  if (delta <= 0) {
    // “no progress” → punish hard (this kills rotate-in-place nicely)
    return no_progress_cost_;
  }

  RCLCPP_INFO_THROTTLE(
  rclcpp::get_logger("MakeProgressCritic"),
  *node_.lock()->get_clock(),
  500,  // ms
  "start_idx=%d end_idx=%d delta=%d | vx=%.2f wz=%.2f",
  start_idx_,
  end_idx,
  delta,
  traj.velocity.x,
  traj.velocity.theta
  );

  // More progress should be LOWER cost (DWB minimizes)
  return static_cast<double>(front_window_ - delta);

}

int MakeProgressCritic::closestIndex(
  const nav_2d_msgs::msg::Path2D & plan,
  const geometry_msgs::msg::Pose2D & pose,
  int from, int to) const
{
  int best_i = from;
  double best_d2 = std::numeric_limits<double>::infinity();

  for (int i = from; i <= to; ++i) {
    const double dx = pose.x - plan.poses[i].x;
    const double dy = pose.y - plan.poses[i].y;
    const double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }
  return best_i;
}

}  // namespace dwb_critics
