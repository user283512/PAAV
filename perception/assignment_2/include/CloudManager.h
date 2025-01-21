#ifndef MANAGER_H_
#define MANAGER_H_

#include <chrono>
#include <thread>
#include <mutex>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

#include "viewer/Renderer.h"

class CloudManager
{
public:
  CloudManager(
      const std::filesystem::path &path,
      int64_t freq,
      viewer::Renderer &renderer);

  void startCloudManager();

  // getters
  pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() { return cloud_; }
  viewer::Color getColor() { return color_; }
  const std::vector<viewer::Box> &getBoxes() { return boxes_; }
  const std::vector<double> &getCentroidsX() { return centroids_x_; }
  const std::vector<double> &getCentroidsY() { return centroids_y_; }
  const std::vector<double> &getCentroidsZ() { return centroids_z_; }

  // data handlers
  bool new_measurement = false;
  std::mutex mtxData;

private:
  // process the pointcloud and perform clustering
  void processAndRenderPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  // cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  // viewer
  viewer::Renderer *renderer_;
  std::vector<viewer::Box> boxes_;
  viewer::Color color_ = viewer::Color(0, 0, 1);

  // clusters
  std::vector<double> centroids_x_;
  std::vector<double> centroids_y_;
  std::vector<double> centroids_z_;

  // processing frequency
  int64_t freq_;

  // path to logs
  std::filesystem::path path_;
};

#endif // MANAGER_H_
