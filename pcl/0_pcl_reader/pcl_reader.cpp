#include <iostream>
#include <numeric>
#include <filesystem>
#include <thread>
#include <sstream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
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
	pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path.string(), *cloud);

	// Creazione del visualizzatore
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.setCameraPosition(0, 0, -3, 0, 1, 0);

	// Rendering
	viewer.spin();

	return 0;
}