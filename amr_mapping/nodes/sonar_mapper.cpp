#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <amr_msgs/Ranges.h>
#include <amr_srvs/SwitchRanger.h>
#include <tf/transform_listener.h>

#include "sonar_map.h"

class SonarMapperNode
{

public:

  SonarMapperNode()
  : transform_listener_(ros::Duration(10))
  {
    // Read settings from the parameter server
    ros::NodeHandle pn("~");
    double resolution;
    double size_x;
    double size_y;
    pn.param<std::string>("frame_id", frame_id_, "odom");
    pn.param<double>("resolution", resolution, 0.06);
    pn.param<double>("size_x", size_x, 16);
    pn.param<double>("size_y", size_y, 16);
    pn.param<double>("map_publication_period", map_publication_period_, 3);
    // Create empty map
    map_ = SonarMap::UPtr(new SonarMap(resolution, size_x, size_y));
    // Publishers and subscribers
    map_publisher_ = pn.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_free_publisher_ = pn.advertise<nav_msgs::OccupancyGrid>("map_free", 1, true);
    map_occupied_publisher_ = pn.advertise<nav_msgs::OccupancyGrid>("map_occupied", 1, true);
    sonar_subscriber_ = nh_.subscribe<amr_msgs::Ranges>("/sonar_pioneer", 10, boost::bind(&SonarMapperNode::sonarCallback, this, _1));
    // Force publish (initially empty) maps
    publishMaps(true);
  }

  void sonarCallback(const amr_msgs::Ranges::ConstPtr& msg)
  {
    for (const auto& range : msg->ranges)
    {
      // Get sonar position in the map frame
      tf::StampedTransform transform;
      try
      {
        ros::Time time;
        std::string str;
        transform_listener_.getLatestCommonTime(frame_id_, range.header.frame_id, time, &str);
        transform_listener_.lookupTransform(frame_id_, range.header.frame_id, time, transform);
      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("Unable to incorporate sonar reading in the map because of unavailable transform. Reason: %s.", ex.what());
        continue;
      }

      // Incorporate range reading in the map
      map_->addScan(transform.getOrigin().getX(),
                    transform.getOrigin().getY(),
                    tf::getYaw(transform.getRotation()),
                    range.field_of_view,
                    range.max_range,
                    range.range,
                    calculateRangeUncertainty(range.range, range.max_range));
    }
    publishMaps();
  }

private:

  void publishMaps(bool force = false)
  {
    if (force || last_map_publication_ + ros::Duration(map_publication_period_) <= ros::Time::now())
    {
      // Query map properties
      int width = map_->getGridSizeX();
      int height = map_->getGridSizeY();
      double min_x = map_->getMinX();
      double min_y = map_->getMinY();
      double resolution = map_->getResolution();
      // Publish maps
      map_publisher_.publish(createOccupancyGridMessage(width, height, min_x, min_y, resolution, -1.0, 1.0, map_->getMapData()));
      map_free_publisher_.publish(createOccupancyGridMessage(width, height, min_x, min_y, resolution, 0.0, 1.0, map_->getMapFreeData()));
      map_occupied_publisher_.publish(createOccupancyGridMessage(width, height, min_x, min_y, resolution, 0.0, 1.0, map_->getMapOccupiedData()));
      last_map_publication_ = ros::Time::now();
    }
  }

  double calculateRangeUncertainty(double range, double max_range) const
  {
    if (range < 0.1 * max_range)
      return 0.01 * max_range;
    else if (range < 0.5 * max_range)
      return 0.1 * range;
    else
      return 0.05 * max_range;
  }

  nav_msgs::OccupancyGridPtr createOccupancyGridMessage(int width, int height, double origin_x, double origin_y, double resolution, double min, double max, const double* data) const
  {
    const double EPSILON = 1e-5;
    const double range = max - min;
    nav_msgs::OccupancyGridPtr grid_msg(new nav_msgs::OccupancyGrid);
    grid_msg->info.width = width;
    grid_msg->info.height = height;
    grid_msg->info.resolution = resolution;
    grid_msg->info.map_load_time = ros::Time::now();
    grid_msg->header.stamp = ros::Time::now();
    grid_msg->header.frame_id = frame_id_;
    grid_msg->info.origin.position.x = origin_x;
    grid_msg->info.origin.position.y = origin_y;
    grid_msg->data.resize(width * height);
    int i = 0;
    for (int x = 0; x < width; x++)
      for (int y = 0; y < height; y++, i++)
      {
        double d = data[x * height + y];
        if (d > max - EPSILON)
          grid_msg->data[i] = 100;
        else if (d < min + EPSILON)
          grid_msg->data[i] = 0;
        else
          grid_msg->data[i] = (d - min) / range * 100;
      }
    return grid_msg;
  }

  SonarMap::UPtr map_;
  std::string frame_id_;
  tf::TransformListener transform_listener_;
  double sonar_uncertainty_;
  double map_publication_period_;
  ros::Time last_map_publication_;

  ros::NodeHandle nh_;
  ros::Publisher map_publisher_;
  ros::Publisher map_free_publisher_;
  ros::Publisher map_occupied_publisher_;
  ros::Subscriber sonar_subscriber_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_mapper");
  ros::NodeHandle nh;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the /switch_ranger service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<amr_srvs::SwitchRanger>("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the pioneer sonars are available and enable them.
  amr_srvs::SwitchRanger srv;
  srv.request.name = "sonar_pioneer";
  srv.request.state = true;
  if (switch_ranger_client.call(srv))
  {
    ROS_INFO("Enabled pioneer sonars.");
  }
  else
  {
    ROS_ERROR("Pioneer sonars are not available, shutting down.");
    return 1;
  }
  SonarMapperNode smn;
  ros::spin();
  return 0;
}

