#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <unordered_set>
#include <algorithm>

#include "../include/Renderer.hpp"

#define USE_PCL_LIBRARY

using namespace lidar_obstacle_detection;

extern Renderer renderer;
extern float g_ransac_threshold;
extern float g_cluster_tolerance;
extern int g_cluster_min_size;
extern int g_cluster_max_size;

// This function sets up the custom kdtree using the point cloud
// static void setupKdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree *tree, int dimension)
// {
//   // insert point cloud points into tree
//   for (int i = 0; i < cloud->size(); ++i)
//   {
//     const pcl::PointXYZ &point = cloud->at(i);
//     tree->insert({point.x, point.y, point.z}, i);
//   }
// }

static void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float downscaling)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(downscaling, downscaling, downscaling);
  voxel_filter.filter(*cloud);
}
static void cropCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::CropBox<pcl::PointXYZ> crob_filter(true);
  crob_filter.setInputCloud(cloud);
  crob_filter.setMin(Eigen::Vector4f(-20, -6, -2, 1));
  crob_filter.setMax(Eigen::Vector4f(30, 7, 5, 1));
  crob_filter.filter(*cloud);
}
static void ransacSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float threshold = 0.1f)
{
  static pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  static pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  coefficients->values.clear();
  inliers->indices.clear();

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  // A point will be considered plane inlier if its distance from the estimated plane
  // is less-equal to 1cm.
  // Points that are more than 1cm away from the plane are discarded as outliners
  seg.setDistanceThreshold(threshold);

  static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_plane->clear();
  cloud_plane->height = 1;
  cloud_plane->is_dense = true;

  int nr_points = cloud->size();
  while (cloud->size() > 0.3f * nr_points)
  {
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())
      break;

    cloud_plane->points.clear();
    for (const auto &idx : inliers->indices)
      cloud_plane->points.push_back(cloud->points[idx]);

    cloud_plane->width = cloud_plane->points.size();

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // Remove inliers from the cloud
    extract.filter(*cloud);
  }
}
static void clusterExtraction(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<pcl::PointIndices> &indices,
    float cluster_tolerance,
    int cluster_min_size,
    int cluster_max_size)
{
  static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // Spatial tollerance for the distance between cluster points:
  // if the distance between two points if less-equal to the tollerance set,
  // those two points will be considered to belong to the same cluster
  ec.setClusterTolerance(cluster_tolerance);
  // Set the minimum number of points required to consider a cluster valid:
  // if a cluster contains less than <g_cluster_min_size> points, it will be discarded.
  ec.setMinClusterSize(cluster_min_size);
  // Set the maximum number of points required to consider a cluster valid:
  // if a cluster contains more than <g_cluster_max_size> points, it will be discarded.
  ec.setMaxClusterSize(cluster_max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(indices);
}

static void processClusters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<pcl::PointIndices> &cluster_indices)
{
  static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;

  int cluster_id = 0;
  char cluster_name[32]{};
  for (const pcl::PointIndices &cluster : cluster_indices)
  {
    cloud_cluster->clear();
    for (const int &idx : cluster.indices)
    {
      pcl::PointXYZ &point = cloud->at(idx);
      cloud_cluster->push_back(point);
    }
    cloud_cluster->width = cloud_cluster->size();

    std::fill_n(cluster_name, 0, sizeof(cluster_name));
    std::snprintf(cluster_name, sizeof(cluster_name), "cluster_%d", cluster_id);

    // Plot the distance of each cluster w.r.t ego vehicle and
    // color the vehicles that are both in front and 5 meters away from the ego vehicle.
    static const Eigen::Vector3f ego_position(0.0f, 0.0f, 0.0f); // vehicle position
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
    Box box{
        minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};

    renderer.RenderBox(box, cluster_id, Color(1.f, 0.f, 0.f), 1.f);

    // Calculate the center of the bounding box
    Eigen::Vector3f cluster_center(
        (box.x_min + box.x_max) / 2.0f, // x
        (box.y_min + box.y_max) / 2.0f, // y
        (box.z_min + box.z_max) / 2.0f  // z
    );

    // Calculates the distance between the center of the cluster and the ego vehicle
    float distance_to_vehicle = (cluster_center - ego_position).norm();
    // pcl::console::print_info("\x1B[34m5)Distance cluster_%d - vehicle: %.2f meters.\x1B[0m\n", cluster_id, distance_to_vehicle);

    // if (box.x_min > 0.0f && distance_to_vehicle > 5.0f)
    //   renderer.RenderBox(box, cluster_id, Color(1.f / (cluster_id + 1), 0.f, 0.f), 1.f);
    // else
    //   renderer.RenderBox(box, cluster_id, Color(1.0f, 1.0f, 1.0f), 1.f);

    cluster_id++;
  }
}

// TODO (mandatory): complete this function
void ProcessAndRenderPointCloud(Renderer &renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // =========================================
  // 1) Downsample the dataset with VoxelGrid
  // =========================================
  filterCloud(cloud, 0.1f);
  pcl::console::print_info("\x1B[34m1)After filtering: %d points.\x1B[0m\n", cloud->size());

  // ===============================================================================
  // 2) we crop the points that are far away from us, in which we are not interested
  // ===============================================================================
  cropCloud(cloud);
  pcl::console::print_info("\x1B[34m2)After cropping: %d points.\x1B[0m\n", cloud->size());

  // ==============================================================
  // 3) Segmentation and apply RANSAC:
  // iterate over the cloud, segment and remove the planar inliers
  // ==============================================================
  ransacSegmentation(cloud, g_ransac_threshold);
  pcl::console::print_info("\x1B[34m3)After segmentation: %d points.\x1B[0m\n", cloud->size());

  //renderer.RenderPointCloud(cloud, "cloud", Color(1, 0, 1));
  //return;

  // =====================================================
  // 4) Extract the clusters from the cloud
  // =====================================================
  static std::vector<pcl::PointIndices> cluster_indices;
  cluster_indices.clear();

  clusterExtraction(cloud,
                    cluster_indices,
                    g_cluster_tolerance,
                    g_cluster_min_size,
                    g_cluster_max_size);

  if (cluster_indices.empty())
  {
    PCL_WARN("No clusters detected\n");
    return;
  }
  pcl::console::print_info("\x1B[34m4)Number of clusters: %d.\x1B[0m\n", cluster_indices.size());

  // ====================================================================
  // 5) Render the cluster and plane without rendering the original cloud
  // 6) Plot the distance of each cluster w.r.t ego vehicle and
  // color the vehicles that are both in front and 5 meters away from the ego vehicle.
  // =================================================================================
  processClusters(cloud, cluster_indices);

#ifdef USE_PCL_LIBRARY
  // PCL functions
  // HERE 6)
#else
  // Optional assignment
  my_pcl::KdTree treeM;
  treeM.set_dimension(3);
  setupKdtree(cloud_filtered, &treeM, 3);
  cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
#endif
}
