#ifndef SONAR_MAP_H
#define SONAR_MAP_H

#include <memory>

#include "map_store.h"
#include "map_store_cone.h"

/** This class implements the sonar map algorithm as described in "Sonar-Based
  * Real-World Mapping and Navigation" by Alberto Elfes, IEEE Journal Of
  * Robotics And Automation, Vol. RA-3, No. 3, June 1987.
  *
  * Map coordinates vs. map cells
  *
  * The map works internaly on a discrete, integer-based grid, but exposes a
  * more natural continuous coordinates interface. This allows an application
  * to work with the map using its own units (in this documentation refered to
  * as "meters"), without taking care of details of the map storage
  * implementation.
  *
  * Each cell with integer coordinates (c_x, c_y) occupies the space from
  * ((c_x - 0.5, c_y - 0.5) * resolution) exclusive to
  * ((c_x - 0.5, c_y + 0.5) * resolution) inclusive.
  *
  * The value resolution is the length of a cells edge. All cells are considered
  * to be squares.
  *
  * Note: if a variable name starts with the prefix "m_", then this variable
  * contains a map coordinate. If the name starts with "c_" then this variable
  * contains a cell coordinate. This convention applies both to the functions'
  * arguments and internal/local variables. */
class SonarMap
{

public:

  typedef std::unique_ptr<SonarMap> UPtr;

  /** This constructor creates a map of given dimensions.
    *
    * @param resolution : size of a cell, measured in meters, i.e. the length of
    * the edge of a cell.
    *
    * @param m_size_x : initial size of the map in x direction (meters).
    *
    * @param m_size_y : initial size of the map in y direction (meters). */
  SonarMap(double resolution, double m_size_x, double m_size_y);

  /** Update map using a sonar reading.
    *
    * If the position of the sonar is outside of the current may, the map will
    * be grown.
    *
    * @param m_sonar_x : x coordinate of the sonar in map coordinates.
    *
    * @param m_sonar_y : y coordinate of the sonar in map coordinates.
    *
    * @param sonar_theta : orientation of the sonar in map coordinates.
    *
    * @param fov : opening angle of the sonar (radians).
    *
    * @param max_range : maximum possible range of the sonar (meters).
    *
    * @param distance : range reading returned from the sensor (meters).
    *
    * @param uncertainty : the noise associated with the sensed distance,
    * expressed as the standard deviation. */
  void addScan(double m_sonar_x, double m_sonar_y, double sonar_theta, double fov, double max_range, double distance, double uncertainty);

  double getResolution() const { return resolution_; }

  int getMinX() const { return m_min_x_; }

  int getMinY() const { return m_min_y_; }

  int getGridSizeX() const { return c_size_x_; }

  int getGridSizeY() const { return c_size_y_; }

  const double* getMapData() const { return map_.getRawData(); }

  const double* getMapFreeData() const { return map_free_.getRawData(); }

  const double* getMapOccupiedData() const { return map_occupied_.getRawData(); }

private:

  /** Determine the map cell that contains the point given by a map coordinate.
    *
    * @param m_x : map coordinate to convert (meters).
    *
    * @param m_y : map coordinate to convert (meters).
    *
    * @param[out] c_x : the cell coordinate corresponding to @a m_x.
    *
    * @param[out] c_y : the cell coordinate corresponding to @a m_y.
    *
    * @return flag if the coordinate (@a m_x, @a m_y) is in the map (return
    * value is true) or not (return value is false). If the coordinate is
    * outside the map, then @a c_x and @a c_y are not valid cell coordinates for
    * this map. */
  bool convertToCell(const double m_x, const double m_y, int &c_x, int &c_y) const;

  /** Determine the map coordinates given map cell.
    *
    * @param c_x : cell coordinate to convert.
    *
    * @param c_x : cell coordinate to convert.
    *
    * @param m_x[out] : the map coordinate corresponding to @a c_x.
    *
    * @param m_y[out] : the map coordinate corresponding to @a c_y.
    *
    * @return flag if the cell with index (@a c_x, @a c_y) is in the map (return
    * value is true) or not (return value is false). If the cell is outside the
    * map, then @a m_x and @a m_y are not valid map coordinates for this map. */
  bool convertToMap(const int c_x, const int c_y, double &m_x, double &m_y) const;

  /** Expand map.
    *
    * This function expands the map around point @a x, @a y by @a size meters.
    * It adds a square of edge length @a size, no matter where @a x, @a y is
    * located. The map size and origin will be updated accordingly. The new
    * space is initialized as zero (unknown occupancy).
    *
    * @param m_x : x coordinate of point where the map should grow.
    *
    * @param m_y : y coordinate of point where the map should grow.
    *
    * @param size : length of the dge of the square which will be added to the
    * map.
    *
    * Note: it is fine to add space which is already in the map. Any overlap
    * between the area specified by @a x, @a y and @a size with the map will be
    * ignored. */
  void growMap(double m_x, double m_y, double size);

  /** Calculate free-space probability.
    *
    * This function calculates the probability to be free for a point that is
    * @a delta meters away from the sonar's origin when the sonar has measured a
    * distance of @a sensed_distance with @a uncertainty. This function only
    * computes the translational component of the probability. To fully specify
    * a point you need a distance and an angle and as such for the full
    * probability you need the angular probability of the point to be the cause
    * of the measured distance @a sensed_distance. This is calculated by Ea().
    * The full probability is the product of the result from Ea() and from this
    * function.
    *
    * @param sensed_distance : distance in meters measured by the sonar.
    *
    * @param delta : distance from the sonar's origin for which the probability
    * should be calculated.
    *
    * @param uncertainty : uncertainty (variance) of measured distance.
    *
    * @return The probability to be free for a point @a delta meters away from
    * the sonar's origin. The value is in the range 0 to 1. */
  double ErFree(double sensed_distance, double delta, double uncertainty) const;

  /** Calculate occupied-space probability.
    *
    * This function calculates the probability to be occupied for a point that
    * is @a delta meters away from the sonar's origin when the sonar has
    * measured a distance of @a sensed_distance with an uncertainty of
    * @a uncertainty. This function only computes the translational component of
    * the probability. To fully specify a point you need a distance and an angle
    * and as such for the full probability you need the angular probability of
    * the point to be the cause of the measured distance @a sensed_distance.
    * This is calculated by Ea(). The full probability is the product of the
    * result from Ea() and from this function.
    *
    * @param sensed_distance : distance in meters measured by the sonar.
    *
    * @param delta : distance from the sonar's origin for which the probability
    * should be calculated.
    *
    * @param uncertainty : uncertainty (variance) of measured distance.
    *
    * @return The probability to be occupied for a point @a delta meters away
    * from the sonar's origin. The value is in the range 0 to 1. */
  double ErOcc(double sensed_distance, double delta, double uncertainty) const;

  /** Probability for a point in the sonar cone to be actually measured.
    *
    * This function calculates the probability of a point @a theta radians away
    * from he center beam of a sonar cone of @a sonar_fov angular width, to be
    * the cause of a sonar measurement.
    *
    * @param sonar_fov : the opening angle of the sonar cone in radians.
    *
    * @param theta : the angular distance of a point from the center of the
    * sonar cone, measured in radians. This value must lie within plus/minus
    * @a sonar_fov / 2.
    *
    * @sa ErFree(), ErOcc() */
  double Ea(double sonar_fov, double theta) const;

  /** Helper function to clamp a variable to a given range. */
  template<typename T>
  static void clamp(T& value, T min, T max)
  {
    if (value < min)
      value = min;
    else if (value > max)
      value = max;
  }

  /** Helper function to compute the Euclidean distance between two points. */
  static double computeEuclideanDistance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  /** Helper function to compute the angular distance between two angles (in
    * radians). */
  static double computeAngularDistance(double a1, double a2)
  {
    return atan2(sin(a1 - a2), cos(a1 - a2));
  }

  /// Size of a cell in meters
  double resolution_;
  /// Width of the map in cells
  int c_size_x_;
  /// Height of the map in cells
  int c_size_y_;
  /// Width of the map in meters
  double m_size_x_;
  /// Height of the map in meters
  double m_size_y_;
  /// X coordinate of bottom-left corner of the map in meters
  double m_min_x_;
  /// Y coordinate of bottom-left corner of the map in meters
  double m_min_y_;

  mapstore::MapStore map_;
  mapstore::MapStore map_free_;
  mapstore::MapStore map_occupied_;

};

#endif /* SONAR_MAP_H */

