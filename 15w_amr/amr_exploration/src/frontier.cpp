#include "frontier.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <typeinfo>

Frontier::Frontier(const nav_msgs::OccupancyGridConstPtr& occupancy_grid)
: frontier_(new PointCloud)
{
  // Step 0: prepare variables
  int width = occupancy_grid->info.width;
  int height = occupancy_grid->info.height;
  int size = width * height;
  double offset_x = occupancy_grid->info.origin.position.x;
  double offset_y = occupancy_grid->info.origin.position.y;
  double resolution = occupancy_grid->info.resolution;

  // Step 1: compute the frontier, create a pointcloud
  for (int y = 1; y < height - 1; y++)
  {
    for (int x = 1; x < width - 1; x++)
    {
      int idx = y * width + x;
      if (occupancy_grid->data[idx] <= 50 && occupancy_grid->data[idx] > 0)
      {
        for (int n : {-width - 1, -width, -width + 1, -1, 1, width - 1, width, width + 1})
        {
          if (idx + n < size && occupancy_grid->data[idx + n] == 0)
          {
            frontier_->points.push_back(pcl::PointXYZ(offset_x + x * resolution, offset_y + y * resolution, 0.0));
            continue;
          }
        }
      }
    }
  }
  frontier_->header = pcl_conversions::toPCL(occupancy_grid->header);
  frontier_->width = frontier_->points.size();
  frontier_->height = 1;
  if (!frontier_->points.size())
  {
    frontier_clusters_.resize(0);
    frontier_clusters_centroids_.resize(0);
    frontier_clusters_sizes_.resize(0);
    return;
  }

  // Step 2: split the frontier into clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(frontier_);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(resolution * 2);
  ec.setMinClusterSize(8);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(frontier_);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  // Step 3: create a pointcloud for each cluster and compute its centroid
  frontier_clusters_.resize(cluster_indices.size());
  frontier_clusters_centroids_.resize(cluster_indices.size());
  frontier_clusters_sizes_.resize(cluster_indices.size());

  for (size_t i = 0; i < cluster_indices.size(); i++)
  {
    frontier_clusters_[i] = boost::make_shared<PointCloud>();
    pcl::copyPointCloud(*frontier_, cluster_indices[i], *frontier_clusters_[i]);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*frontier_clusters_[i], centroid);
    frontier_clusters_centroids_[i].getVector4fMap() = centroid;
    frontier_clusters_sizes_[i] = frontier_clusters_[i]->points.size()/3;
  }
}

