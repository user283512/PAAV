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
#include <iostream>
#include <vector>
#include <format>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;
static const fs::path res_path = fs::current_path().parent_path().parent_path() / "res";
static const fs::path pcd_file_path = res_path / "table_scene_lms400_downsampled.pcd"; // "1639663212.029186000.pcd"

int main()
{
	// 1. Read the cloud data
	// -----------------------------------------------------------
	PCL_INFO("Loading %s...\n", pcd_file_path.string().c_str());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PCDReader reader;
	reader.read(pcd_file_path.string(), *cloud_input);
	PCL_INFO("PointCloud has %d points\n", cloud_input->size());

	// 2. Point cloud filtering (VoxelGrid)
	// -----------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud_input);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	PCL_INFO("PointCloud after voxel grid filtering: %d points\n", cloud_filtered->size());

	// 3. Segmentation of the main plane
	// -----------------------------------------------------------
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.01f);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

	int nr_segmented = 0;
	int nr_points = cloud_filtered->size();
	while (cloud_filtered->size() > 0.3 * nr_points)
	{
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.empty())
		{
			PCL_ERROR("Could not estimate a planar model for the given dataset\n");
			break;
		}

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_plane);
		PCL_INFO("Cloud plane has %d points\n", cloud_plane->size());

		// Remove points that belong to the plane from the original cloud
    extract.setNegative(true);  // Remove inliers from the filtered cloud
    extract.filter(*cloud_filtered);

		nr_segmented++;
	}
	
	PCL_INFO("Total number of segmentations: %d\n", nr_segmented);

	// Create a KDTree for clustering
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	if (tree->getInputCloud()->empty())
	{
		PCL_ERROR("KDTree input cloud is empty\n");
		return 0;
	}

	// Perform clustering using the KDTree
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02f); // 2cm
	ec.setMinClusterSize(100);     // Minimum 100 points for a cluster
	ec.setMaxClusterSize(25000);   // Maximum 25000 points for a cluster
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);
	if (cluster_indices.empty())
	{
		PCL_ERROR("cluster_indices empty\n");
		return 0;
	}

	// 6. Saving of extracted clusters
	// -----------------------------------------------------------
	pcl::PCDWriter writer;
	int cluster_id = 0;
	for (const auto& cluster : cluster_indices)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& index : cluster.indices)
			cloud_cluster->push_back((*cloud_filtered)[index]);

		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		char out[32]{};
		std::format_to_n(out, sizeof(out), "cluster_{}.pcd", cluster_id);

		writer.write<pcl::PointXYZ>(out, *cloud_cluster, false);
		PCL_INFO("Saved cluster %d with %d points\n", cluster_id, cloud_cluster->size());

		cluster_id++;
	}

	return 0;
}
