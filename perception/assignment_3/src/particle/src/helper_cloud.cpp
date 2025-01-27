#include <vector>
#include <fstream>
#include <cmath>
#include <string>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "particle/helper_cloud.hpp"
#include "particle/circle_fit.hpp"
#include "Globals.hpp"

pcl::PointCloud<pcl::PointXYZI> extractReflectors(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZI> cloudReflector;
  for (auto &point : *cloud)
  {
    if (point.intensity > 0.12f)
      cloudReflector.push_back(point);
  }

  // cluster reflector points, circle fitting and compute the center
  pcl::PointCloud<pcl::PointXYZI> reflectorCenter;
  // clustering
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloudReflector.makeShared());
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.5);
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloudReflector.makeShared());
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  int pp = 0;
  // for each cluster perform circle fitting
  for (auto &idx : cluster_indices)
  {
    // extract points
    std::vector<float> xComponent;
    std::vector<float> yComponent;
    for (auto &j : idx.indices)
    {
      xComponent.push_back(cloudReflector.at(j).x);
      yComponent.push_back(cloudReflector.at(j).y);
    }

    // Circle fitting
    Circle circle;
    circle = CircleFitByCeres(xComponent, yComponent, 0.04);

    float xAvg = 0.0f;
    for (auto &x : xComponent)
      xAvg += x;
    xAvg /= xComponent.size();
    float yAvg = 0.0f;
    for (auto &y : yComponent)
      yAvg += y;
    yAvg /= yComponent.size();

    if (sqrt(xAvg * xAvg + yAvg * yAvg) > sqrt(circle.x * circle.x + circle.y * circle.y))
      continue; // skip reflector, circle center is nearest that average point

    pcl::PointXYZI pt;
    pt.x = circle.x;
    pt.y = circle.y;
    pt.z = 0.0f;
    reflectorCenter.push_back(pt);
    if (pp > g_num_reflectors)
      break;
    pp++;
  }
  return reflectorCenter;
}

bool createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
               const std::filesystem::path &path,
               Map &map)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  tree->setInputCloud(cloud);
  ec.setClusterTolerance(0);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(2);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::ofstream myfile(path);
  int i = 0;
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    std::vector<float> xComponent;
    std::vector<float> yComponent;
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      xComponent.push_back(cloud->points[*pit].x);
      yComponent.push_back(cloud->points[*pit].y);
      myfile << cloud->points[*pit].x << " " << cloud->points[*pit].y << " " << i << "\n";
    }
    i++;
  }
  myfile.close();

  return true;
}
