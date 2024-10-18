#include <iostream>
#include <filesystem>
#include <format>
#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

namespace fs = std::filesystem;

static const fs::path res_path = fs::current_path().parent_path().parent_path() / "res";
static const fs::path pcd_file_path = res_path / "table_scene_lms400_downsampled.pcd"; // "1639663212.029186000.pcd"

/*
	Objective: Segment the ground from the input cloud.
	Input: point cloud
	Output:
				1) point cloud/s representing the ground
				2) point cloud without the ground

*/

int main ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	reader.read(pcd_file_path.string(), *cloud_input);

	// Downsample point cloud using a voxel grid filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_input);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered);

	// Segmentation using RANSAC
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.1f);

	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

	int nr_segmented = 0;
	int nr_points = cloud_filtered->size();
	pcl::PCDWriter writer;
	while (cloud_filtered->size() > 0.3f * nr_points)
	{
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.empty())
		{
			PCL_ERROR("Could not estimate a planar model for the given dataset\n");
			break;
		}

		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_plane);
		PCL_INFO("PointCloud cloud_plane has %d points\n", cloud_plane->size());

		char out[32]{};
		std::format_to_n(out, sizeof(out), "cloud_plane_{}.pcd", nr_segmented);
		writer.write<pcl::PointXYZ>(out, *cloud_plane, false);

		// Remove points that belong to the plane from the original cloud
    extract.setNegative(true);  // Remove inliers from the filtered cloud
    extract.filter(*cloud_filtered);

		nr_segmented++;
	}

	PCL_INFO("Total number of segmentations: %d\n", nr_segmented);

	// We write the point cloud without the segmented planes
	writer.write<pcl::PointXYZ>("cloud_remaining.pcd", *cloud_filtered, false);
	return 0;
}