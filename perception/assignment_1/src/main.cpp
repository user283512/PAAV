#include <iostream>
#include <filesystem>
#include <chrono>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include "../include/tree_utilities.hpp"
#include "../include/Renderer.hpp"

namespace fs = std::filesystem;
using namespace lidar_obstacle_detection;

extern void ProcessAndRenderPointCloud(Renderer &renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

Renderer renderer;

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		std::cerr << "2 args required: <dataset_directory>\n";
		return -1;
	}

	const char *dataset = argv[1];
	static const fs::path res_dir = fs::current_path() / "res";
	static const fs::path dataset_path = res_dir / dataset;
	if (!fs::exists(dataset_path))
	{
		std::cerr << "Invalid dataset path: " << dataset_path << std::endl;
		return -1;
	}

	pcl::PCDReader reader;

	std::vector<fs::path> pcd_files{
			fs::directory_iterator(dataset_path),
			fs::directory_iterator()};
	std::sort(pcd_files.begin(), pcd_files.end());
	assert(pcd_files.size() != 0);

	renderer.InitCamera(CameraAngle::XY);
	renderer.ClearViewer();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto stream_it = pcd_files.begin();
	while (!renderer.WasViewerStopped())
	{
		renderer.ClearViewer();

		reader.read(stream_it->string(), *cloud);
		auto startTime = std::chrono::steady_clock::now();

		ProcessAndRenderPointCloud(renderer, cloud);

		auto endTime = std::chrono::steady_clock::now();
		std::chrono::duration<double, std::milli> elapsedTime = endTime - startTime;

		PCL_INFO("Time elapsed: %fms\n", elapsedTime.count());

		stream_it++;
		if (stream_it == pcd_files.end())
			stream_it = pcd_files.begin();

		renderer.SpinViewerOnce();
	}
}
