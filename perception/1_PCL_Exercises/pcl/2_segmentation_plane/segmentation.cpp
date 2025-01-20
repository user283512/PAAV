#include <iostream>
#include <numeric>
#include <filesystem>
#include <string>
#include <format>
#include <sstream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

namespace fs = std::filesystem;

// Objective: Segment the ground from the input cloud.
// Input: point cloud
// Output:
// 				1) point cloud/s representing the ground
// 				2) point cloud without the ground

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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);

	// 1. Init the cloud
	pcl::PCDReader reader;
	reader.read(pcd_path.string(), *cloud_input);
	// 2. Downsample the cloud
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_input);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_input);
	// 3. Create the SACSegmentation object and set the model and method type.
	// We specify the “distance threshold”, which determines how close a point must be
	// to the model in order to be considered an inlier.
	// We will use the RANSAC method (pcl::SAC_RANSAC) as the robust estimator of choice
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.1f);

	// cloud that contains only inlier points
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_plane->height = 1;
	cloud_plane->is_dense = true;

	// Used to remove plane from the cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::PCDWriter writer;

	int i = 0;
	int nr_points = cloud_input->size();
	while (cloud_input->size() > 0.3f * nr_points)
	{
		seg.setInputCloud(cloud_input);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.empty())
		{
			PCL_WARN("Could not estimate a planar model for the given dataset.\n");
			break;
		}

		cloud_plane->points.reserve(inliers->indices.size());
		cloud_plane->points.clear();
		for (const auto &idx : inliers->indices)
			cloud_plane->points.push_back(cloud_input->points[idx]);

		cloud_plane->width = cloud_plane->points.size();

		std::stringstream ss;
		ss << "plane_" << i << ".pcd";
		std::string filename = ss.str();
		writer.write(filename, *cloud_plane, true);

		extract.setInputCloud(cloud_input);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_input);

		i++;
	}

	writer.write("cloud_remaining.pcd", *cloud_input, true);

	return 0;
}
