#ifndef FRONTIER_H
#define FRONTIER_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <nav_msgs/OccupancyGrid.h>

class Frontier
{

public:

  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;

  /** Process an occupancy grid determining the frontier between free and
    * unexplored part, and split it into Euclidean clusters.
    *
    * The output could be retrieved with @ref getPointCloud(),
    * @ref getClusterPointClouds(), and @ref getClusterCentroids(). */
  Frontier(const nav_msgs::OccupancyGridConstPtr& occupancy_grid);

  /** Get a point cloud of all cells that belong to the frontier. */
  const PointCloud::Ptr getPointCloud() const
  {
    return frontier_;
  }

  /** Get a vector of clusters in which the frontier was split. */
  const std::vector<PointCloud::Ptr>& getClusterPointClouds() const
  {
    return frontier_clusters_;
  }

  /** Get a vector of centroids of clusters in which the frontier was split. */
  const PointCloud::VectorType& getClusterCentroids() const
  {
    return frontier_clusters_centroids_;
  }

  /** Get a vector of centroids of clusters in which the frontier was split. */
  const std::vector<uint>& getClusterSizes() const
  {
    return frontier_clusters_sizes_;
  }


private:

  PointCloud::Ptr frontier_;
  std::vector<PointCloud::Ptr> frontier_clusters_;
  PointCloud::VectorType frontier_clusters_centroids_;
  std::vector<uint> frontier_clusters_sizes_;

};

#endif /* FRONTIER_H */

