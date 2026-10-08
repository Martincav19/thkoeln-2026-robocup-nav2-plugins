
#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "nav2_behaviors/plugins/dock_to_station.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors
{

DockToStation::DockToStation()
: TimedBehavior<DockToStationAction>()
{
  for (auto & v : last_tof_mm_) { v.store(-1); }  // -1 means “not received yet”
}

DockToStation::~DockToStation() = default;

//Hacer la configuracion de parametros aqui

void DockToStation::onConfigure()
{
  RCLCPP_INFO(logger_,"Configure DockToStation.");
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // EMA filter factor
  nav2_util::declare_parameter_if_not_declared(
    node, "alpha", rclcpp::ParameterValue(0.2));
  node->get_parameter("alpha", alpha_);

  // Deadbands
  nav2_util::declare_parameter_if_not_declared(
    node, "deadband_theta", rclcpp::ParameterValue(0.008));
  node->get_parameter("deadband_theta", deadband_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, "deadband_dist", rclcpp::ParameterValue(0.04));
  node->get_parameter("deadband_dist", deadband_dist_);

  // Control gains
  nav2_util::declare_parameter_if_not_declared(
    node, "ke_theta", rclcpp::ParameterValue(8.0));
  node->get_parameter("ke_theta", ke_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, "ke_dist", rclcpp::ParameterValue(1.5));
  node->get_parameter("ke_dist", ke_dist_);

  // Stability logic
  nav2_util::declare_parameter_if_not_declared(
    node, "stable_cycles", rclcpp::ParameterValue(10));
  node->get_parameter("stable_cycles", stable_cycles_);

  nav2_util::declare_parameter_if_not_declared(
    node, "use_calibration", rclcpp::ParameterValue(true));
  node->get_parameter("use_calibration", use_calibration_);

  nav2_util::declare_parameter_if_not_declared(
    node, "baseline_dx_m", rclcpp::ParameterValue(0.0));
  node->get_parameter("baseline_dx_m", baseline_dx_m_);

  // Sensor A calibration
  nav2_util::declare_parameter_if_not_declared(
    node, "calib_mA", rclcpp::ParameterValue(0.9667));
  node->get_parameter("calib_mA", calib_mA_);

  nav2_util::declare_parameter_if_not_declared(
    node, "calib_bA", rclcpp::ParameterValue(2.0));
  node->get_parameter("calib_bA", calib_bA_);

  // Sensor B calibration
  nav2_util::declare_parameter_if_not_declared(
    node, "calib_mB", rclcpp::ParameterValue(0.9633));
  node->get_parameter("calib_mB", calib_mB_);

  nav2_util::declare_parameter_if_not_declared(
    node, "calib_bB", rclcpp::ParameterValue(6.1));
  node->get_parameter("calib_bB", calib_bB_);

  // Minimum speeds (stiction handling)
  nav2_util::declare_parameter_if_not_declared(
    node, "w_min", rclcpp::ParameterValue(0.05));
  node->get_parameter("w_min", w_min_);

  nav2_util::declare_parameter_if_not_declared(
    node, "v_min", rclcpp::ParameterValue(0.05));
  node->get_parameter("v_min", v_min_);

  // Angular gate for linear motion
  nav2_util::declare_parameter_if_not_declared(
    node, "theta_gate", rclcpp::ParameterValue(0.05));
  node->get_parameter("theta_gate", theta_gate_);

  // Sensor sanity limits (mm)
  nav2_util::declare_parameter_if_not_declared(
    node, "min_mm", rclcpp::ParameterValue(20));
  node->get_parameter("min_mm", min_mm_);

  nav2_util::declare_parameter_if_not_declared(
    node, "max_mm", rclcpp::ParameterValue(2000));
  node->get_parameter("max_mm", max_mm_);

  // -------- ROS interfaces (create ONCE) --------
  for (int i = 0; i < 8; ++i) {
    std::string topic = "/laser_tof_" + std::to_string(i);
    tof_subs_[i] = node->create_subscription<std_msgs::msg::Int16>(
      topic, rclcpp::SensorDataQoS(),
      [this, i](std_msgs::msg::Int16::SharedPtr msg){
        last_tof_mm_[i].store(msg->data, std::memory_order_relaxed);
      });
  }

  stable_count_theta_ = 0;
  stable_count_dist_  = 0;
  
  RCLCPP_INFO(logger_, "Finish Configure DockToStation.");

}

//Recibir el action goal y processarlo debidamente

ResultStatus DockToStation::onRun(const std::shared_ptr<const DockToStationActionGoal> command)
{
  
  command_docking_side_     = command->docking_side;
  command_standoff_distance_ = command->standoff_distance;
  command_max_linear_speed_  = command->max_linear_speed;
  command_max_angular_speed_ = command->max_angular_speed;
  command_maintain_alignment_ = command->maintain_alignment;
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (command->docking_side > 3) {
    return ResultStatus{Status::FAILED, DockToStationActionResult::INVALID_INPUT};
  }

  const auto & pair = DOCKING_SENSOR_MAP[command->docking_side];
  idx_a_ = pair.a;
  idx_b_ = pair.b;

  RCLCPP_INFO(logger_, "Using TOF A=%d B=%d", idx_a_, idx_b_);

  return ResultStatus{Status::SUCCEEDED, DockToStationActionResult::NONE};
}

//Hacer el ciclo de control aqui, remplaza OnTime
ResultStatus DockToStation::onCycleUpdate()
{
  if(!command_maintain_alignment_){
    return ResultStatus{Status::SUCCEEDED, DockToStationActionResult::NONE};
  }

  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
    return ResultStatus{Status::FAILED, DockToStationActionResult::TIMEOUT};
  }


  const int last_a_mm = last_tof_mm_[idx_a_].load(std::memory_order_relaxed);
  const int last_b_mm = last_tof_mm_[idx_b_].load(std::memory_order_relaxed);

      // Need both sensors
  if (last_a_mm < 0 || last_b_mm < 0) {
    publishStop();
    return ResultStatus{Status::RUNNING, DockToStationActionResult::INVALID_INPUT};
  }

  int a_mm_raw = last_a_mm;
  int b_mm_raw = last_b_mm;

  if (a_mm_raw < 20 || a_mm_raw > 2000 || b_mm_raw < 20 || b_mm_raw > 2000) {
    publishStop();
    stable_count_theta_ = 0;
    stable_count_dist_ = 0;
      RCLCPP_WARN(
    this->logger_,
    "No valid sensor data!");
    return ResultStatus{Status::RUNNING, DockToStationActionResult::INVALID_INPUT};
  }

  // Optional calibration (still in mm)
  const bool use_calib = use_calibration_;
  double a_mm = static_cast<double>(a_mm_raw);
  double b_mm = static_cast<double>(b_mm_raw);

  if (use_calib) {
    const double mA = calib_mA_;
    const double bA = calib_bA_;
    const double mB = calib_mB_;
    const double bB = calib_bB_;
    a_mm = mA * a_mm + bA;
    b_mm = mB * b_mm + bB;
  }

  // Convert to meters
  const double a_m = a_mm / 1000.0;
  const double b_m = b_mm / 1000.0;

  // Filter (EMA)
  if (!a_filt_.has_value()) a_filt_ = a_m;
  if (!b_filt_.has_value()) b_filt_ = b_m;

  *a_filt_ = alpha_ * a_m + (1.0 - alpha_) * (*a_filt_);
  *b_filt_ = alpha_ * b_m + (1.0 - alpha_) * (*b_filt_);

  const double dx = baseline_dx_m_;

  double a_final = *a_filt_;
  double b_final = *b_filt_ - dx;

  double e_theta = (a_final) - (b_final);                  // meters, has to be 0

  double d_avg   = 0.5 * (a_final + b_final);              // meters
  double e_dist  = d_avg - command_standoff_distance_;                         // meters, has to be 0


  bool theta_stable = false;
  if (std::fabs(e_theta) < deadband_theta_) {
    stable_count_theta_++;
    theta_stable = true;
    if (stable_count_theta_ == 1) {
      RCLCPP_INFO(this->logger_, "Within deadband_theta (|e_theta|=%.4fm). Holding...", std::fabs(e_theta));
    }
  }

  bool dist_stable = false;
  if (std::fabs(e_dist) <= deadband_dist_) {
    stable_count_dist_++;
    dist_stable = true;
    if (stable_count_dist_ == 1) {
      RCLCPP_INFO(this->logger_, "Within deadband_dist (|e_dist|=%.4fm). Holding...", std::fabs(e_dist));
    }
  }

  if(!theta_stable) stable_count_theta_ = 0;
  if(!dist_stable) stable_count_dist_  = 0;

  // P-control on distance difference
  double w = -ke_theta_ * e_theta;

  // Clamp + minimum angular speed
  w = std::clamp(
    w,
    -static_cast<double>(command_max_angular_speed_),
    static_cast<double>(command_max_angular_speed_));

  if (std::fabs(w) < w_min_) w = (w > 0.0) ? w_min_ : -w_min_;

  // P-control on distance difference
  double v = ke_dist_ * e_dist;

  // Clamp + minimum angular speed
  v = std::clamp(
    v,
    -static_cast<double>(command_max_linear_speed_),
    static_cast<double>(command_max_linear_speed_));

  if (std::fabs(v) < v_min_) v = (v > 0.0) ? v_min_ : -v_min_;

  // gate forward motion if not aligned enough
  if (std::fabs(e_theta) > theta_gate_) {
    v = 0.0;
  }

  // stop if both are within deadbands
  if (std::fabs(e_dist) <= deadband_dist_) {
    v = 0.0; 
  }

  // stop if both are within deadbands
  if (std::fabs(e_theta) < deadband_theta_) {
    w = 0.0;
  }

  if ((std::fabs(e_theta) < deadband_theta_) && (std::fabs(e_dist) <= deadband_dist_)) {
    RCLCPP_INFO(this->logger_, "Good Alignment, Increasing count...");
  }

  // If stable long enough, stop and keep stopped
  if (stable_count_theta_ >= stable_cycles_ && stable_count_dist_ >= stable_cycles_) {
    publishStop();
    RCLCPP_INFO(this->logger_, "Aligned: stable for %d cycles.", stable_cycles_);
    return ResultStatus{Status::SUCCEEDED, DockToStationActionResult::NONE};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = this->clock_->now();
  cmd_vel->header.frame_id = this->robot_base_frame_;

  // reset
  cmd_vel->twist.linear.x  = 0.0;
  cmd_vel->twist.linear.y  = 0.0;
  cmd_vel->twist.angular.z = w;

  switch (command_docking_side_) {
    case 0:  // FRONT
      cmd_vel->twist.linear.x =  v;
      break;

    case 1:  // BACK
      cmd_vel->twist.linear.x = -v;
      break;

    case 2:  // LEFT
      cmd_vel->twist.linear.y =  -v;
      break;

    case 3:  // RIGHT
      cmd_vel->twist.linear.y = v;
      break;

    default:
      RCLCPP_ERROR(this->logger_,
        "Invalid docking_side: %d", command_docking_side_);
      // publish stop for safety
      cmd_vel->twist.angular.z = 0.0;
      break;
  }

  this->vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, DockToStationActionResult::NONE};
}

void DockToStation::publishStop()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = this->clock_->now();
  cmd_vel->header.frame_id = this->robot_base_frame_;
  cmd_vel->twist.linear.y = 0.0;
  cmd_vel->twist.angular.z = 0.0;
  cmd_vel->twist.linear.x = 0.0;

  this->vel_pub_->publish(std::move(cmd_vel));
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::DockToStation, nav2_core::Behavior)
