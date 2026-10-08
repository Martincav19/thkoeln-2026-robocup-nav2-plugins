
#ifndef NAV2_BEHAVIORS__PLUGINS__CAN_ROTATE_HPP_
#define NAV2_BEHAVIORS__PLUGINS__CAN_ROTATE_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/can_rotate.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace nav2_behaviors
{
using CanRotateAction = nav2_msgs::action::CanRotate;

/**
 * @class nav2_behaviors::CanRotate
 * @brief An action server behavior for checking if robot can rotate
 */
class CanRotate : public TimedBehavior<CanRotateAction>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  using CanRotateActionGoal = CanRotateAction::Goal;
  using CanRotateActionResult = CanRotateAction::Result;

  /**
   * @brief A constructor for nav2_behaviors::CanRotate
   */
  CanRotate();
  ~CanRotate();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const CanRotateActionGoal> command) override;

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
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(geometry_msgs::msg::Pose2D &);

  float vel;
  float angle;
  double spinning_simulation_time_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__CAN_ROTATE_HPP_
