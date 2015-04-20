#include "diff_velocity_controller.h"

DiffVelocityController::DiffVelocityController(double l_max_vel, double l_tolerance,
                                               double a_max_vel, double a_tolerance)
: l_max_vel_(l_max_vel)
, l_tolerance_(l_tolerance)
, a_max_vel_(a_max_vel)
, a_tolerance_(a_tolerance)
{
}

void DiffVelocityController::setTargetPose(const Pose& pose)
{
  target_pose_ = pose;
  linear_complete_ = false;
  angular_complete_ = false;
}

bool DiffVelocityController::isTargetReached() const
{
  return linear_complete_ & angular_complete_;
}

Velocity DiffVelocityController::computeVelocity(const Pose& actual_pose)
{
  // Displacement and orientation to the target in world frame
  double dx = target_pose_.x - actual_pose.x;
  double dy = target_pose_.y - actual_pose.y;

  // Step 1: compute remaining distances
  double linear_dist = getDistance(target_pose_, actual_pose);
  double angular_dist = getShortestAngle(target_pose_.theta, actual_pose.theta);

  if (std::abs(linear_dist) < l_tolerance_ && std::abs(angular_dist) < a_tolerance_)
  {
    linear_complete_ = true;
    angular_complete_ = true;
    return Velocity();
  }

  if (std::abs(linear_dist) > l_tolerance_)
    // We still need to drive to the target, therefore we first need to make
    // sure that we are oriented towards it.
    angular_dist = getShortestAngle(atan2(dy, dx), actual_pose.theta);

  // Step 2: compute velocities
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  if (std::abs(linear_dist) > l_tolerance_)
    linear_vel = std::abs(linear_dist) > 5 * l_tolerance_ ? l_max_vel_ : l_tolerance_;

  if (std::abs(angular_dist) > a_tolerance_)
    angular_vel = std::abs(angular_dist) > 5 * a_tolerance_ ? a_max_vel_ : a_tolerance_;

  if (std::abs(angular_dist) > a_tolerance_ * 5)
  {
    // We need to rotate a lot, so stand still and rotate with max velocity.
    return Velocity(0, 0, std::copysign(angular_vel, angular_dist));
  }
  else
  {
    // We need to rotate just a bit (or do not need at all), so it is fine to
    // combine with linear motion if needed.
    return Velocity(std::copysign(linear_vel, linear_dist), 0, std::copysign(angular_vel, angular_dist));
  }
}

