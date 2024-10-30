#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <thread>
#include <cstdint>

#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"

namespace fs = std::filesystem;
const static fs::path dir_res_path = fs::current_path().parent_path() / "res";
const static fs::path dir_log_path = dir_res_path / "log";

int main()
{
	constexpr int64_t freq = 100;  // Frequency of the thread dedicated to process the point cloud
	const std::string dir_log_string = dir_log_path.string();

	std::ifstream dataFile(dir_log_string, std::ios::in | std::ios::binary);
	if (!dataFile)
	{
		std::cerr << "Error on opening file " << dir_log_string << "\n";
		return 1;
	}

	// Init renderer
	viewer::Renderer renderer;
	renderer.initCamera(viewer::CameraAngle::XY);
	renderer.clearViewer();

	// Instantiate the tracker
	Tracker tracker;

	// Spawn the thread that process the point cloud and performs the clustering
	CloudManager lidar_cloud(dir_log_string, freq, renderer);
	std::thread t(&CloudManager::startCloudManager, &lidar_cloud);

	while (true)
	{
		// Clear the render
		renderer.clearViewer();

		while (!lidar_cloud.new_measurement) { } // wait for new data (we will execute the following code each 100ms)

		// fetch data
		lidar_cloud.mtxData.lock();
		auto cloud = lidar_cloud.getCloud();
		auto color = lidar_cloud.getColor();
		auto boxes = lidar_cloud.getBoxes();
		auto centroids_x = lidar_cloud.getCentroidsX();
		auto centroids_y = lidar_cloud.getCentroidsY();
		lidar_cloud.new_measurement = false;
		lidar_cloud.mtxData.unlock();

		// render pointcloud and boxes
		renderer.renderPointCloud(cloud, "pointCloud", color);
		for (uint64_t i = 0; i < boxes.size(); ++i)
			renderer.renderBox(boxes[i], i);

		// Call the tracker on the detected clusters
		tracker.track(centroids_x, centroids_y, renderer.getLidarStatus());

		// retrieve tracklets and render the trackers
		auto tracks = tracker.getTracks();
		for (size_t i = 0; i < tracks.size(); ++i)
		{
			renderer.addCircle(tracks[i].getX(), tracks[i].getY(), tracks[i].getId());
			renderer.addText(tracks[i].getX() + 0.01, tracks[i].getY() + 0.01, tracks[i].getId());
		}

		renderer.spinViewerOnce();
	}

	t.join();
	return 0;
}
