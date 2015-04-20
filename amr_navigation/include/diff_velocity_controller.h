#ifndef DIFF_VELOCITY_CONTROLLER_H
#define DIFF_VELOCITY_CONTROLLER_H

#include "velocity_controller.h"

/** A simple implementation of velocity controller that drives the robot as if
  * it had a differential drive base.
  *
  * The base is assumed to have 2 degrees of freedom, i.e. can mave forwards
  * and rotate. The controller tries to orient the robot towards the goal and
  * then move it forwards until it is reached.
  *
  * The robot drives at a constant (max) velocity until it has almost reached
  * the goal pose, then it switches to the minimum velocity. */
class DiffVelocityController : public VelocityController
{

public:

  DiffVelocityController(double l_max_vel, double l_tolerance,
                         double a_max_vel, double a_tolerance);

  virtual void setTargetPose(const Pose& pose);

  virtual bool isTargetReached() const;

  virtual Velocity computeVelocity(const Pose& actual_pose);

private:

  Pose target_pose_;

  bool linear_complete_;
  bool angular_complete_;

  double l_max_vel_;
  double l_tolerance_;

  double a_max_vel_;
  double a_tolerance_;

};

#endif /* DIFF_VELOCITY_CONTROLLER_H */

