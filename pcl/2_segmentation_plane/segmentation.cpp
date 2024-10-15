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

using Point_XYZ             = pcl::PointXYZ;
using PointCloud_XYZ        = pcl::PointCloud<Point_XYZ>;
using VoxelGrid_XYZ         = pcl::VoxelGrid<Point_XYZ>;
using SACSegmentation_XYZ   = pcl::SACSegmentation<Point_XYZ>;
using ExtractIndices_XYZ    = pcl::ExtractIndices<Point_XYZ>;
using ModelCoefficients     = pcl::ModelCoefficients;
using PointIndices          = pcl::PointIndices;
using PCDReader             = pcl::PCDReader;
using PCDWriter             = pcl::PCDWriter;

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
    PointCloud_XYZ::Ptr cloud_input(new PointCloud_XYZ);       //point cloud input
    PointCloud_XYZ::Ptr cloud_segmented(new PointCloud_XYZ);   //point cloud with planes
    PointCloud_XYZ::Ptr cloud_aux(new PointCloud_XYZ);         //aux point cloud
    PointCloud_XYZ::Ptr cloud_filtered(new PointCloud_XYZ);    //point cloud filtered
    
    PCDWriter writer;

    // Fill in the cloud data
    PCDReader reader;
    reader.read(pcd_file_path.string(), *cloud_input);

    // The point cloud is downsampled using a voxel grid filter, 
    // reducing the number of points so that each point represents a volume of 1 cm³
    VoxelGrid_XYZ sor;
    sor.setInputCloud(cloud_input);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    // An instance of SACSegmentation is created to segment the plane using the RANSAC algorithm. 
    // This algorithm searches for the plane that best fits the data points, 
    // with a distance tolerance of 10 cm.
    SACSegmentation_XYZ seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.1f);

    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices); 
    
    int i = 0; 
    int nr_points = cloud_filtered->size();
    
    // A cycle is run that segments the plans until more than 30 percent of the points 
    // remain in the original point cloud.
    while (cloud_filtered->size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud <-
        seg.setInputCloud(cloud_filtered);
        
        // Base method for segmentation of a model in a PointCloud given by <setInputCloud(), setIndices()>
        // [out]	inliers	the resultant point indices that support the model found (inliers)
        // [out]	model_coefficients	the resultant model coefficients that describe the plane 
        
        // we get one of the planes and we put it into the inliers variable
        seg.segment(*inliers, *coefficients); 
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // The indices of the points that belong to the plane are extracted with pcl::ExtractIndices
        ExtractIndices_XYZ extract;
        extract.setInputCloud(cloud_filtered); 
        // PCL defines a way to define a region of interest / list of point indices 
        // that the algorithm should operate on, rather than the entire cloud, via setIndices.
        extract.setIndices(inliers);
        extract.setNegative(false); // Retrieve indices to all points in cloud_filtered but only those referenced by inliers:
        extract.filter(*cloud_segmented);   // We effectively retrieve JUST the plane
        
        std::cout << "PointCloud representing the planar component: " 
            << cloud_segmented->width * cloud_segmented->height << " data points." << std::endl;

        char out[32]{};
        std::format_to_n(out, sizeof(out), "table_scene_lms400_plane_{}.pcd", i);
        writer.write<Point_XYZ>(out, *cloud_segmented, false);

        // Here we will extract the plane from the original filtered point cloud
        extract.setNegative(true); // original cloud - plane 
        extract.filter(*cloud_aux);  // We write into cloud_f the cloud without the extracted plane
        
        cloud_filtered.swap(cloud_aux); // Here we swap the cloud (the removed plane one) with the original
        i++;
    }
    
    // We write the point cloud without the ground
    writer.write<Point_XYZ>("table_scene_lms400_notable.pcd", *cloud_filtered, false);
    return 0;
}

