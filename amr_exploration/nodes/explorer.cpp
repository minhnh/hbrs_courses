#include <ros/ros.h>
#include <ros/console.h>

#include "clustered_point_cloud_visualizer.h"
#include "frontier.h"

class ExplorerNode
{

public:

  ExplorerNode()
  : frontier_clusters_publisher_("frontier_clusters", "odom")
  , world_is_explored_(false)
  {
    frontier_publisher_ = nh_.advertise<Frontier::PointCloud>("frontier_points", 1);
    map_subscriber_ = nh_.subscribe("sonar_mapper/map", 1, &ExplorerNode::mapCallback, this);
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    Frontier frontier(msg);
    frontier_publisher_.publish(frontier.getPointCloud());
    frontier_clusters_publisher_.publish<Frontier::Point>(frontier.getClusterPointClouds());

    // Process frontier clusters (if any)
    // ...
  }

  void explore()
  {
    while (ros::ok() && !world_is_explored_)
    {
      // Choose a target
      // Plan a path to it
      // Execute the path
      // ...
      ros::spinOnce();
    }
    ROS_INFO_COND(world_is_explored_, "World is completely explored, exiting...");
  }

private:

  ros::NodeHandle nh_;

  ros::Subscriber map_subscriber_;
  ros::Publisher frontier_publisher_;
  ClusteredPointCloudVisualizer frontier_clusters_publisher_;

  bool world_is_explored_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer");
  ExplorerNode en;
  en.explore();
  return 0;
}
