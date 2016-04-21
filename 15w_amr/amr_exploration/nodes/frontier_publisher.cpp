#define _USE_MATH_DEFINES
#include <cmath>
#include <random>
#include <list>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Pose2D.h>
#include <amr_msgs/ExecutePathAction.h>
#include <amr_msgs/PathExecutionFailure.h>
#include <amr_srvs/PlanPath.h>
#include <amr_msgs/Frontiers.h>


#include <pcl/common/centroid.h>

#include "clustered_point_cloud_visualizer.h"
#include "frontier.h"

class FrontierPublisherNode
{

public:

  FrontierPublisherNode()
  : frontier_clusters_publisher_("frontier_clusters", "odom")
  {
    frontier_publisher_ = nh_.advertise<Frontier::PointCloud>("frontier_points", 1);
    frontier_centroid_publisher_ = nh_.advertise<amr_msgs::Frontiers>("frontier_centroids", 1);
    map_subscriber_ = nh_.subscribe("sonar_mapper/map", 1, &FrontierPublisherNode::mapCallback, this);
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    Frontier frontier(msg);
    frontier_publisher_.publish(frontier.getPointCloud());
    frontier_clusters_publisher_.publish<Frontier::Point>(frontier.getClusterPointClouds());

    auto centroids = frontier.getClusterCentroids();
    auto centroid_sizes = frontier.getClusterSizes();
    //world_is_explored_ = centroids.size() == 0;

    amr_msgs::Frontiers targets;

    for (const auto& centroid : centroids)
    {
      geometry_msgs::Pose2D ctr;
      ctr.x = centroid.x;
      ctr.y = centroid.y;
      ctr.theta = 0;
      targets.centroids.push_back(ctr);
    }
    
    for (const auto& c_size : centroid_sizes)
    {
      std_msgs::UInt16 siz;
      siz.data = c_size;
      targets.sizes.push_back(siz);
    }
    
    frontier_centroid_publisher_.publish(targets);
    ROS_INFO("Received new frontier, created %zu targets.", targets.centroids.size());
  }


private:

  ros::NodeHandle nh_;

  ros::Subscriber map_subscriber_;
  ros::Publisher frontier_publisher_;
  ros::Publisher frontier_centroid_publisher_;
  ClusteredPointCloudVisualizer frontier_clusters_publisher_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frontier_publisher");
  FrontierPublisherNode fp;
  ros::spin();
  return 0;
}
