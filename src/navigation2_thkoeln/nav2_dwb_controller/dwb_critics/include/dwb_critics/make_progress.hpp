
#ifndef DWB_CRITICS__MAKE_PROGRESS_HPP_
#define DWB_CRITICS__MAKE_PROGRESS_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{

/**
 * @class MakeProgressCritic
 * @brief Forces the commanded trajectories to make progress on the global plan
 *
 * This used to be built in to the DWA Local Planner as the LatchedStopRotate controller,
 * but has been moved to a critic for consistency.
 *
 * The critic has three distinct phases.
 */
class MakeProgressCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  /**
   * @brief Score the trajectory with greater weight as greater time passes by without progress.
   *
   * This (easily overridden) method assumes that the critic is in the third phase (as described above)
   * @param traj Trajectory to score
   * @return numeric score
   */
  int closestIndex(const nav_2d_msgs::msg::Path2D &, const geometry_msgs::msg::Pose2D &, int, int) const;
private:
  int front_window_;
  int back_window_;
  double no_progress_cost_;
  int start_idx_;
  bool enabled_;
  double goal_threshold_;


  nav_2d_msgs::msg::Path2D plan_;
  nav2_costmap_2d::Costmap2D * costmap_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__MAKE_PROGRESS_HPP_
