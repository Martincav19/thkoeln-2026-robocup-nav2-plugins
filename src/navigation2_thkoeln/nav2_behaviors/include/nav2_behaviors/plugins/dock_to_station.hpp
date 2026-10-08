
#ifndef NAV2_BEHAVIORS__PLUGINS__DOCK_TO_STATION_HPP_
#define NAV2_BEHAVIORS__PLUGINS__DOCK_TO_STATION_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include <cstdint>
#include <optional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/dock_to_station.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_behaviors
{
using DockToStationAction = nav2_msgs::action::DockToStation;

/**
 * @class nav2_behaviors::DockToStation
 * @brief An action server behavior for checking if robot can rotate
 */
class DockToStation : public TimedBehavior<DockToStationAction>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  using DockToStationActionGoal = DockToStationAction::Goal;
  using DockToStationActionResult = DockToStationAction::Result;

  /**
   * @brief A constructor for nav2_behaviors::DockToStation
   */
  DockToStation();
  ~DockToStation();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const DockToStationActionGoal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

protected:

  void publishStop();
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_a_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_b_;

  std::optional<int16_t> last_a_mm_;
  std::optional<int16_t> last_b_mm_;
  std::optional<double> a_filt_;
  std::optional<double> b_filt_;

  int stable_count_theta_ = 0;
  int stable_count_dist_ = 0;
  // --- Filter ---
  double alpha_;

  // --- Deadbands ---
  double deadband_theta_;
  double deadband_dist_;

  // --- Control gains ---
  double ke_theta_;
  double ke_dist_;

  // --- Stability logic ---
  int stable_cycles_;
  bool use_calibration_;
  double baseline_dx_m_;

  // --- Sensor A calibration ---
  double calib_mA_;
  double calib_bA_;

  // --- Sensor B calibration ---
  double calib_mB_;
  double calib_bB_;

  // --- Minimum speeds (stiction handling) ---
  double w_min_;
  double v_min_;

  // --- Gating thresholds ---
  double theta_gate_;

  // --- Sensor sanity limits (mm) ---
  int min_mm_;
  int max_mm_;

  //ACTION GOAL
  int   command_docking_side_;
  float command_standoff_distance_;
  float command_max_linear_speed_;
  float command_max_angular_speed_;
  bool  command_maintain_alignment_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;

};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__DOCK_TO_STATION_HPP_
