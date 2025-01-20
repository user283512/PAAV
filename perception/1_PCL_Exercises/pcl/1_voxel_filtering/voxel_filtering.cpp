#include <iostream>
#include <numeric>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
namespace fs = std::filesystem;

int main(int argc, char **argv)
{
	// Controllo argomenti
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// init the cloud
	pcl::PCDReader reader;
	reader.read(pcd_path.string(), *cloud);

	std::cerr << "PointCloud before filtering: "
						<< cloud->width * cloud->height
						<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.1f, 0.1f, 0.1f); // this value defines how much the PC is filtered
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: "
						<< cloud_filtered->width * cloud_filtered->height
						<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

	pcl::PCDWriter writer;
	std::stringstream ss;
	ss << "cloud_filtered.pcd";
	writer.write<pcl::PointXYZ>(ss.str(), *cloud_filtered, false);

	return 0;
}
