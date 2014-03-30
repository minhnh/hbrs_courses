#ifndef BRAITENBERG_VEHICLE_H
#define BRAITENBERG_VEHICLE_H

#include <memory>

class BraitenbergVehicle
{

public:

  typedef std::unique_ptr<BraitenbergVehicle> UPtr;

  /** Braitenberg vehicle type. */
  enum Type
  {
    TYPE_A, ///< direct connections
    TYPE_B, ///< cross connections
    TYPE_C, ///< direct and cross connections
  };

  /** Default constructor creates a vehicle of type A with the connection
    * factor equal to 1. */
  BraitenbergVehicle();

  /** Construct a braitenberg vehicle of the desired type.
    *
    * @a factor2 has an effect only for vehicles of type C, therefore this
    * parameter may be omitted when constructing vehicles of other types. */
  BraitenbergVehicle(Type type, float factor1, float factor2 = 0.0);

  ~BraitenbergVehicle() { };

  /** Compute wheel speeds of the vehicle depending on the input from sonars.
    *
    * @param left_in : left sonar reading scaled by its maximum range, i.e.
    *        proximity to an obstacle (in interval [0..1]), where 0 means
    *        contact, and 1 means that there are no obstacles in the sonar
    *        range.
    * @param right_in : same as @a left_in, but for the right sonar.
    * @param left_out : computed left wheel speed.
    * @param right_out : computed right wheel speed. */
  void computeWheelSpeeds(float left_in, float right_in, float& left_out, float& right_out);

private:

  Type type_;
  float factor1_;
  float factor2_;

};

#endif /* BRAITENBERG_VEHICLE_H */

