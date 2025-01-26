#ifndef HELPER_CLOUD_HPP_
#define HELPER_CLOUD_HPP_

#include "map.hpp"

#include <filesystem>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// extractReflectors This function extracts the reflectors from the point cloud
// @param point cloud from the LiDAR
// @output point cloud with just the reflectors
pcl::PointCloud<pcl::PointXYZI> extractReflectors(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

// createMap This function creates a map and extracts the reflectors from the point cloud
// @param cloud point cloud with the reflectors-landmarks
// @param filename writes in this file the coordinates sof the landmarks
// @param map with the landmarks (reflectors)
// @output True if the function is executed succesfuly
bool createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
							 const std::filesystem::path &path,
							 Map &map);
#endif