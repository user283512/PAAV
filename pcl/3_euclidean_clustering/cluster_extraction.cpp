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

template<typename T>
using vector = std::vector<T>;

using Point_XYZ             = pcl::PointXYZ;
using PointCloud_XYZ        = pcl::PointCloud<Point_XYZ>;
using VoxelGrid_XYZ         = pcl::VoxelGrid<Point_XYZ>;
using SACSegmentation_XYZ   = pcl::SACSegmentation<Point_XYZ>;
using ExtractIndices_XYZ    = pcl::ExtractIndices<Point_XYZ>;
using KdTree_XYZ            = pcl::search::KdTree<Point_XYZ>;
using EuclideanClusterExtraction_XYZ = pcl::EuclideanClusterExtraction<Point_XYZ>;
using ModelCoefficients     = pcl::ModelCoefficients;
using PointIndices          = pcl::PointIndices;
using PCDReader             = pcl::PCDReader;
using PCDWriter             = pcl::PCDWriter;

static const fs::path res_path = fs::current_path().parent_path().parent_path() / "res";
static const fs::path pcd_file_path = res_path / "table_scene_lms400_downsampled.pcd"; // "1639663212.029186000.pcd"

int main()
{
    PointCloud_XYZ::Ptr cloud(new PointCloud_XYZ); 
    PointCloud_XYZ::Ptr cloud_f(new PointCloud_XYZ);
    PointCloud_XYZ::Ptr cloud_filtered(new PointCloud_XYZ);

    // 1. Read the cloud data
    // -----------------------------------------------------------
    PCDReader reader;
    reader.read(pcd_file_path.string(), *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points\n";

    // 2. Point cloud filtering (VoxelGrid)
    // -----------------------------------------------------------
    VoxelGrid_XYZ vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points\n";

    // 3. Segmentation of the main plane
    // -----------------------------------------------------------
    SACSegmentation_XYZ seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);

    PointIndices::Ptr inliers(new PointIndices);
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointCloud_XYZ::Ptr cloud_plane(new PointCloud_XYZ);

    int i = 0;
    int nr_points = cloud_filtered->size();

    // While there are more than ground plane...
    while (cloud_filtered->size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // 4. Point cluster extraction (Euclidean Cluster Extraction)
        // -----------------------------------------------------------
        ExtractIndices_XYZ extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    KdTree_XYZ::Ptr tree(new KdTree_XYZ);
    tree->setInputCloud(cloud_filtered);

    EuclideanClusterExtraction_XYZ ec;
    // Set the spatial tolerance for new cluster candidates
    // If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
    // On the other hand, if you set the value too high, it could happen, 
    // that multiple objects are seen as one cluster
    ec.setClusterTolerance(0.02f); // 2cm
    // We impose that the clusters found must have at least setMinClusterSize() points 
    // and maximum setMaxClusterSize() points
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    
    // Here we are creating a vector of PointIndices, which contain the actual index information in a vector<int>. The indices of each detected cluster are saved here.
    vector<PointIndices> cluster_indices;
    ec.extract(cluster_indices);


    // 6. Saving of extracted clusters
    // -----------------------------------------------------------
    PCDWriter writer;
    int j = 0;
    for (const PointIndices& pointIndex : cluster_indices)
    {
        PointCloud_XYZ::Ptr cloud_cluster(new PointCloud_XYZ);
        for (const auto& indices : pointIndex.indices)
            cloud_cluster->push_back((*cloud_filtered)[indices]);

        cloud_cluster->width = cloud_cluster->size();  
        cloud_cluster->height = 1; 
        cloud_cluster->is_dense = true;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points\n";
        
        char out[32]{};
        std::format_to_n(out, sizeof(out), "cloud_cluster_{}.pcd", j);
        writer.write<Point_XYZ>(out, *cloud_cluster, false);
        j++;
    }

    return 0;
}
