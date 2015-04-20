#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <cmath>

#include "pose.h"
#include "velocity.h"

/** Abstract class which declares an interface for velocity controllers.
  *
  * Velocity controller is an object that can compute the velocity that the
  * robot should have in order to reach some goal given the current pose.
  *
  * Different controllers may implement different velocity profiles. */
class VelocityController
{

public:

  typedef std::unique_ptr<VelocityController> UPtr;

  virtual void setTargetPose(const Pose& pose) = 0;

  virtual bool isTargetReached() const = 0;

  virtual Velocity computeVelocity(const Pose& actual_pose) = 0;

protected:

  /** Helper function to compute the Euclidean distance between two points. */
  static float getDistance(const Pose& p1, const Pose& p2)
  {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  }

  /** Helper function to compute the angular distance between two angles (in
    * radians). */
  static float getShortestAngle(float a1, float a2)
  {
    return atan2(sin(a1 - a2), cos(a1 - a2));
  }

};

#endif /* VELOCITY_CONTROLLER_H */

