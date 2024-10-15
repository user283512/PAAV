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
#include <pcl/common/common.h>
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
void setupKdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
	/* insert point cloud points into tree */ 
	for (int i = 0; i < cloud->size(); ++i)
		tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
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
void proximity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
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
std::vector<pcl::PointIndices> euclideanCluster(
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

void ProcessAndRenderPointCloud(Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	/* TODO: 1) Downsample the dataset */ 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>());



	/* 2) here we crop the points that are far away from us, in which we are not interested */ 
	pcl::CropBox<pcl::PointXYZ> cb(true);
	cb.setInputCloud(cloud_filtered);
	cb.setMin(Eigen::Vector4f(-20, -6, -2, 1));
	cb.setMax(Eigen::Vector4f( 30, 7, 5, 1));
	cb.filter(*cloud_filtered); 

	/* TODO: 3) Segmentation and apply RANSAC */ 

	/* TODO: 4) iterate over the filtered cloud, segment and remove the planar inliers */ 

	/* TODO: 5) Create the KDTree and the vector of pcl::PointIndices */ 

	/* TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!) */ 
	std::vector<pcl::PointIndices> cluster_indices;

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

	std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1) };

	/**
	 * Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 
	 * To separate each cluster out of the vector<pcl::PointIndices> we have to iterate through cluster_indices, 
	 * create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
	 * Compute euclidean distance
	 */

	int j = 0;
	int clusterId = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->push_back((*cloud_filtered)[*pit]); 
		
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		renderer.RenderPointCloud(cloud,"originalCloud" + std::to_string(clusterId),colors[2]);
		
		/* TODO: 7) render the cluster and plane without rendering the original cloud  */ 

		/* Here we create the bounding box on the detected clusters */
		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

		/* TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle */
		Box box;
		box.min_pos = Eigen::Vector3f{ minPt.x, minPt.y, minPt.z };
		box.max_pos = Eigen::Vector3f{ maxPt.x, maxPt.y, maxPt.z };

		/**
		 * TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle.
		 * Please take a look at the function RenderBox to see how to color the box
		 */
		renderer.RenderBox(box, j);

		++clusterId;
		j++;
	}  
}


int main()
{
	static const auto dataset_dir = fs::current_path().parent_path() / "res/dataset_1";

	Renderer renderer;
	renderer.InitCamera(CameraAngle::XY);
	renderer.ClearViewer();

	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

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
