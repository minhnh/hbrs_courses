#ifndef VELOCITY_H
#define VELOCITY_H

#include <geometry_msgs/Twist.h>

/** This structure reprosents velocity in 2d space. */
struct Velocity
{

  float x;
  float y;
  float theta;

  Velocity() : x(0), y(0), theta(0) { }

  Velocity(float x, float y, float theta) : x(x), y(y), theta(theta) { }

  Velocity(const Velocity& other) : x(other.x), y(other.y), theta(other.theta) { }

  const Velocity& operator=(const Velocity& other)
  {
    x = other.x;
    y = other.y;
    theta = other.theta;
    return *this;
  }

  /** Convenience cast operator to ROS Twist message. */
  operator geometry_msgs::Twist()
  {
    geometry_msgs::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = theta;
    return twist;
  }

  friend std::ostream& operator<<(std::ostream& out, const Velocity& p)
  {
    return out << "[" << p.x << ", " << p.y << ", " << p.theta << "]";
  }

};

#endif /* VELOCITY_H */

