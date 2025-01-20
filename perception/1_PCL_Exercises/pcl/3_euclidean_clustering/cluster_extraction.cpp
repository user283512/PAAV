#include <iostream>
#include <numeric>
#include <filesystem>
#include <string>
#include <format>
#include <sstream>
#include <iomanip>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace fs = std::filesystem;

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		std::cerr << "2 args required: <path-to-pcd-file>\n";
		return -1;
	}
	fs::path pcd_path = fs::path(argv[1]);
	if (!fs::exists(pcd_path) && !fs::is_regular_file(pcd_path))
	{
		std::cerr << "Invalid pcd file path: " << pcd_path << "\n";
		return 1;
	}

	pcl::PCDWriter writer;
	pcl::PCDReader reader;

	// Read in the cloud data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(pcd_path.string(), *cloud);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.01f);

	int i = 0;
	int nr_points = cloud_filtered->size();
	while (cloud_filtered->size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.empty())
		{
			PCL_WARN("Could not estimate a planar model for the given dataset.\n");
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_plane);

		std::stringstream ss;
		ss << "plane_" << i << ".pcd";
		writer.write(ss.str(), *cloud_plane, true);

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_filtered);
		i++;
	}
	writer.write("cloud_filtered.pcd", *cloud_filtered, true);

	// KdTree is an optimized data structure for searching nearest neighbors within a 3D space.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	// It's a vector where each element (pcl::PointIndices) represents a cluster.
	// Each cluster contains the indices of the points that belong to that group.
	// A single object pcl::PointIndices is a vector of integers where each number corresponds
	// to the index of the point in the cloud.
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	// Specify the max distance between points from them to be considered part of the same cluster
	ec.setClusterTolerance(0.02f); // 2cm
	// Every cluster must contain from 100 to 25'000 points.
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25'000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	int j = 0;
	for (const pcl::PointIndices &cluster : cluster_indices)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_cluster->reserve(cluster.indices.size());
		for (const int &idx : cluster.indices)
		{
			pcl::PointXYZ &point = cloud_filtered->at(idx);
			cloud_cluster->push_back(point);
		}
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::stringstream ss;
		ss << "cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, true);
		j++;
	}

	return 0;
}
