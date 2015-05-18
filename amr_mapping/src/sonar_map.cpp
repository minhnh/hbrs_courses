#include <vector>

#include "sonar_map.h"

SonarMap::SonarMap(double resolution, double m_size_x, double m_size_y)
: resolution_(resolution)
, c_size_x_(lround(m_size_x / resolution) + 2)
, c_size_y_(lround(m_size_y / resolution) + 2)
, m_size_x_(resolution_ * c_size_x_)
, m_size_y_(resolution_ * c_size_y_)
, m_min_x_(-m_size_x_ / 2.0)
, m_min_y_(-m_size_y_ / 2.0)
, map_(c_size_x_, c_size_y_)
, map_free_(c_size_x_, c_size_y_)
, map_occupied_(c_size_x_, c_size_y_)
{
}

void SonarMap::addScan(double sonar_x, double sonar_y, double sonar_theta, double fov, double max_range, double distance, double uncertainty)
{
  //============================== YOUR CODE HERE ==============================
  // Instructions: implement the routine that performs map update based on a
  //               single sonar reading.
  //
  // Hint: use convertToCell() and convertToMap() functions to convert between
  //       map and cell coordinates.
  //
  // Hint: use the other helper functions defined in the header file.
  //
  // Hint: use mapstore::MapStoreCone class to iterate over the cells covered
  //       by the sonar cone.
  //


  //============================================================================
}

double SonarMap::ErFree(double sensed_distance, double delta, double uncertainty) const
{
  //============================== YOUR CODE HERE ==============================
  // Instructions: compute the distance probability function for the "probably
  //               empty" region.

  return 0.0;

  //============================================================================
}

double SonarMap::ErOcc(double sensed_distance, double delta, double uncertainty) const
{
  //============================== YOUR CODE HERE ==============================
  // Instructions: compute the distance probability function for the "probably
  //               occupied" region.

  return 0.0;

  //============================================================================
}

double SonarMap::Ea(double sonar_fov, double theta) const
{
  //============================== YOUR CODE HERE ==============================
  // Instructions: compute the angular probability function (it is same for both
  //               the "probably empty" and the "probably occupied" region.

  return 0.0;

  //============================================================================
}

bool SonarMap::convertToCell(const double m_x, const double m_y, int &c_x, int &c_y) const
{
  c_x = lround(m_x / resolution_);
  c_y = lround(m_y / resolution_);
  return (map_.isInX(c_x) && map_.isInY(c_y));
}

bool SonarMap::convertToMap(const int c_x, const int c_y, double &m_x, double &m_y) const
{
  m_x = c_x * resolution_;
  m_y = c_y * resolution_;
  return (map_.isInX(c_x) && map_.isInY(c_y));
}

