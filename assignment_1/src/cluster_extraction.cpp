#include <iostream>
#include <chrono>
#include <unordered_set>
#include <filesystem>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "../include/Renderer.hpp"
#include "../include/tree_utilities.hpp"

#define USE_PCL_LIBRARY

namespace fs = std::filesystem;
using namespace lidar_obstacle_detection;
using my_visited_set_t = std::unordered_set<int>;

/**
 * This function sets up the custom kdtree using the point cloud
 */
static void setupKdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
	/* insert point cloud points into tree */ 
	for (int i = 0; i < cloud->size(); ++i)
		tree->insert({ cloud->at(i).x, cloud->at(i).y, cloud->at(i).z }, i);
}

/**
 * OPTIONAL
 * This function computes the nearest neighbors and builds the clusters
 * Input:
 * 	+cloud: Point cloud to be explored
 * 	+target_ndx: i-th point to visit
 * 	+tree: kd tree for searching neighbors
 * 	+distanceTol: Distance tolerance to build the clusters 
 * 	+visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
 * 	+cluster: Here we add points that will represent the cluster
 * 	+max: Max cluster size
 * 
 * Output:
 * 	+visited: already visited points
 * 	+cluster: at the end of this function we will have one cluster
 */
static void proximity(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	int target_ndx, 
	my_pcl::KdTree* tree, 
	float distanceTol, 
	my_visited_set_t& visited, 
	std::vector<int>& cluster, 
	int max
)
{
	if (cluster.size() < max)
	{
		cluster.push_back(target_ndx);
		visited.insert(target_ndx);

		std::vector<float> point { cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z };
		/* Get all neighboring indices of point */ 
		std::vector<int> neighborNdxs = tree->search(point, distanceTol);

		for (int neighborNdx : neighborNdxs)
		{
			/* If point was not visited */
			if (visited.find(neighborNdx) == visited.end())
				proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);

			if (cluster.size() >= max)
				return;
		}
	}
}

/**
 * OPTIONAL
 * This function builds the clusters following a euclidean clustering approach
 * Input:
 * 	+cloud: Point cloud to be explored
 * 	+tree: kd tree for searching neighbors
 * 	+distanceTol: Distance tolerance to build the clusters 
 * 	+setMinClusterSize: Minimum cluster size
 * 	+setMaxClusterSize: Max cluster size
 * 
 * Output:
 * 	+cluster: at the end of this function we will have a set of clusters
 * 
 * TODO: Complete the function
 */
static std::vector<pcl::PointIndices> euclideanCluster(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	my_pcl::KdTree* tree, 
	float distanceTol, 
	int setMinClusterSize, 
	int setMaxClusterSize
)
{
	my_visited_set_t visited{};         // already visited points
	std::vector<pcl::PointIndices> clusters; // vector of pcl::PointIndices that will contain all the clusters
	std::vector<int> cluster;           // vector of int that is used to store the points that the function proximity will give me back

	//for every point of the cloud
	//  if the point has not been visited (use the function called "find")
	//    find clusters using the proximity function
	//
	//    if we have more clusters than the minimum
	//      Create the cluster and insert it in the vector of clusters. 
	//			You can extract the indices from the cluster returned by the proximity funciton (use pcl::pcl::PointIndices)   
	//    end if
	//  end if
	//end for
	return clusters;	
}

static void ProcessAndRenderPointCloud(
	Renderer& renderer, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud
)
{
	/**
	 * 1) Downsample the dataset
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	
	std::cout << "1) Before filtering: " << input_cloud->width * input_cloud->height << " points\n";
	constexpr float downscaling = 0.1f;
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filtering;
	voxel_filtering.setInputCloud(input_cloud);
	voxel_filtering.setLeafSize(downscaling, downscaling, downscaling);
	voxel_filtering.filter(*cloud_filtered);
	std::cout << "1) After filtering: " << cloud_filtered->width * cloud_filtered->height << " points\n";

	/**
	 * 2) here we crop the points that are far away from us, in which we are not interested
	 */
	std::cout << "2) Before cropping: " << cloud_filtered->width * cloud_filtered->height << " points\n";
	pcl::CropBox<pcl::PointXYZ> cb(true);
	cb.setInputCloud(cloud_filtered);
	cb.setMin(Eigen::Vector4f(-20.f, -6.f, -2.f, 1.f));
	cb.setMax(Eigen::Vector4f( 30.f, 7.f, 5.f, 1.f));
	cb.filter(*cloud_filtered);
	std::cout << "2) After cropping: " << cloud_filtered->width * cloud_filtered->height << " points\n";

	/**
	 * 3) Segmentation and apply RANSAC
	 */
#if 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1f);
	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0) 
	{
		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		return;
	}

	// The indices of the points that belong to the plane are extracted with pcl::ExtractIndices.
	// PCL defines a way to define a region of interest (list of point indices)
	// that the algorithm should operate on, rather than the entire cloud, via setIndices.
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(false);	// Retrieve indices to all points in cloud_filtered but only those referenced by inliers
	extract.filter(*cloud_segmented); // We effectively retrieve JUST the plane

	std::cout << "PointCloud representing the planar component: " 
		<< cloud_segmented->width * cloud_segmented->height << " points\n";

	renderer.RenderPointCloud(cloud_segmented, "cloud_segmented", Color(1.f, 1.f, 1.f));
#endif

	/**
	 * 4) Iterate on the point cloud, segment the main plane each time, remove inliers 
	 * (those that belong to the plane) and repeat until there are no more significant planes to segment.
	 */

	// segment the plans until more than 30 percent of the points remain in the original point cloud
	std::cout << "3-4) Segmentation of the plane\n";
	int nr_points = cloud_filtered->size();
	while (cloud_filtered->size() > 0.3f * nr_points)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.1f);
		seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Pulling out the plan inliers
		pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false); // setNegative(false) to extract only the points in the plane
    extract.filter(*cloud_plane);
		
		std::cout << "PointCloud representing the planar component: " 
			<< cloud_plane->width * cloud_plane->height << " points\n";

		// setNegative(true) to remove those points from the original cloud, 
		// leaving all other points that do not belong to the plane
    extract.setNegative(true);				
    extract.filter(*cloud_filtered); 	// We write into cloud_filtered the cloud without the extracted plane
	}
	return;

	/**
	 * 5) Create the KDTree and the vector of pcl::PointIndices
	 */
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud_filtered); 
	// Vector to contain clusters (each represented by a pcl::PointIndices object)
	std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02f); 		// Maximum distance between points in a cluster (2cm)
	ec.setMinClusterSize(100);				// Minimum cluster size
	ec.setMaxClusterSize(25000); 			// Maximum cluster size
	ec.setSearchMethod(kdtree);				// Set the search method (KDTree)
	ec.setInputCloud(cloud_filtered);	// Set the remaining point cloud as input
	ec.extract(cluster_indices);			// Extract the clusters and save the results in cluster_indices

	// Stampa i risultati
	std::cout << "Numero di cluster trovati: " << cluster_indices.size() << std::endl;



	/* TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!) */ 
	//std::vector<pcl::PointIndices> cluster_indices;

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

	//const Color colors[] = { Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1) };

	/**
	 * Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 
	 * To separate each cluster out of the vector<pcl::PointIndices> we have to iterate through cluster_indices, 
	 * create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
	 * Compute euclidean distance
	 */

	// int j = 0;
	// int clusterId = 0;
	// for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	// {
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	// 	for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	// 		cloud_cluster->push_back((*cloud_filtered)[*pit]); 
		
	// 	cloud_cluster->width = cloud_cluster->size();
	// 	cloud_cluster->height = 1;
	// 	cloud_cluster->is_dense = true;

	// 	char buff[32]{};
	// 	std::format_to_n(buff, sizeof(buff), "Input_Cloud_{}", clusterId);
	// 	renderer.RenderPointCloud(input_cloud, buff, colors[2]);
		
	// 	/* TODO: 7) render the cluster and plane without rendering the original cloud  */ 

	// 	/* Here we create the bounding box on the detected clusters */
	// 	pcl::PointXYZ minPt, maxPt;
	// 	pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

	// 	/* TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle */
	// 	Box box;
	// 	box.min_pos = Eigen::Vector3f{ minPt.x, minPt.y, minPt.z };
	// 	box.max_pos = Eigen::Vector3f{ maxPt.x, maxPt.y, maxPt.z };

	// 	/**
	// 	 * TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle.
	// 	 * Please take a look at the function RenderBox to see how to color the box
	// 	 */
	// 	renderer.RenderBox(box, j);

	// 	clusterId++;
	// 	j++;
	// }  
}


int main()
{
	static const auto dataset_dir = fs::current_path().parent_path() / "res/dataset_1";

	Renderer renderer;
	renderer.InitCamera(CameraAngle::XY);
	renderer.ClearViewer();

	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	std::vector<fs::path> stream(
		fs::directory_iterator{dataset_dir},
		fs::directory_iterator{}
	);
	std::sort(stream.begin(), stream.end()); /* Sort files in ascending (chronological) order */

	auto streamIterator = stream.begin();

	while (!renderer.WasViewerStopped())
	{
		renderer.ClearViewer();

		pcl::PCDReader reader;
		reader.read(streamIterator->string(), *input_cloud);
		auto startTime = std::chrono::steady_clock::now();

		//renderer.RenderPointCloud(input_cloud, "Input_Cloud", Color(255,255,255));
		ProcessAndRenderPointCloud(renderer, input_cloud);

		auto endTime = std::chrono::steady_clock::now();
		std::chrono::duration<float> elapsedTime = endTime - startTime;
		std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
			<< input_cloud->points.size() << " data points from " 
			<< streamIterator->string() <<  "plane segmentation took " 
			<< elapsedTime.count() << " milliseconds" << std::endl;

		streamIterator++;
		if(streamIterator == stream.end())
			streamIterator = stream.begin();

		renderer.SpinViewerOnce();
	}
}
