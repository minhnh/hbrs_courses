#ifndef CLUSTERED_POINT_CLOUD_VISUALIZER_H
#define CLUSTERED_POINT_CLOUD_VISUALIZER_H

#include <string>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "color.h"

/** A helper class which publishes a set of clusters (that is, PCL point
  * clouds) to RViz.
  *
  * The clusters are merged in one big point cloud, where points belonging to
  * different clusters get different colors. */
class ClusteredPointCloudVisualizer
{

public:

  ClusteredPointCloudVisualizer(const std::string& topic_name, const std::string& frame_id);

  template<typename PointT>
  void publish(const std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters);

  template<typename PointT>
  void publish(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::vector<pcl::PointIndices>& cluster_indices);

private:

  ros::Publisher cloud_publisher_;

  const std::string frame_id_;

  static const size_t COLORS_NUM = 32;
  float COLORS[COLORS_NUM];

};

#include "impl/clustered_point_cloud_visualizer.hpp"

#endif /* CLUSTERED_POINT_CLOUD_VISUALIZER_H */

