#include <iostream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace fs = std::filesystem;

using Point_XYZ = pcl::PointXYZ;
using PointCloud_XYZ = pcl::PointCloud<Point_XYZ>;
using VoxelGrid_XYZ = pcl::VoxelGrid<Point_XYZ>;
using PCDReader = pcl::PCDReader;
using PCDWriter = pcl::PCDWriter;

static const fs::path res_path = fs::current_path().parent_path().parent_path() / "res";
static const fs::path pcd_file_path = res_path / "table_scene_lms400_downsampled.pcd"; // "1639663212.029186000.pcd"

int main ()
{
   PointCloud_XYZ::Ptr cloud(new PointCloud_XYZ);
   PointCloud_XYZ::Ptr cloud_filtered(new PointCloud_XYZ);

   // Fill in the cloud data
   PCDReader reader;
   // Replace the path below with the path where you saved your file
   reader.read(pcd_file_path.string(), *cloud); // Remember to download the file first!

   std::cout << "PointCloud before filtering: " << cloud->width * cloud->height 
      << " data points (" << pcl::getFieldsList(*cloud) << ").\n";

   // Create the filtering object
   float downscaling = 0.05f;
   VoxelGrid_XYZ sor;
   sor.setInputCloud(cloud);
   sor.setLeafSize(downscaling, downscaling, downscaling); //this value defines how much the PC is filtered
   sor.filter(*cloud_filtered);

   std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
      << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

   PCDWriter writer;
   writer.write<Point_XYZ>("cloud_filtered.pcd", *cloud_filtered, false);

   return 0;
}