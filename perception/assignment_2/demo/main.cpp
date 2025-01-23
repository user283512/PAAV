#include <iostream>
#include <fstream>
#include <filesystem>

#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"

namespace fs = std::filesystem;

extern double g_distance_threshold;
extern double g_covariance_threshold;
extern int g_loss_threshold;

int main()
{
	static const fs::path res_dir = fs::current_path() / "res";
	if (!fs::exists(res_dir))
	{
		std::cerr << "Invalid path: " << res_dir << std::endl;
		return -1;
	}

	// Init renderer
	viewer::Renderer renderer;
	renderer.initCamera(viewer::CameraAngle::XY);
	renderer.clearViewer();

	// Instantiate the tracker
	Tracker tracker{g_distance_threshold, g_covariance_threshold, g_loss_threshold};

	// Frequency of the thread dedicated to process the point cloud
	constexpr int64_t freq = 100;

	// Spawn the thread that process the point cloud and performs the clustering
	CloudManager lidar_cloud(res_dir, freq, renderer);
	std::thread t(&CloudManager::startCloudManager, &lidar_cloud);

	while (true)
	{
		// Clear the render
		renderer.clearViewer();

		while (!lidar_cloud.new_measurement)
			; // Wait for new data (we will execute the following code each 100ms)

		// Fetch data
		lidar_cloud.mtxData.lock();
		auto cloud = lidar_cloud.getCloud();
		auto color = lidar_cloud.getColor();
		auto boxes = lidar_cloud.getBoxes();
		auto centroids_x = lidar_cloud.getCentroidsX();
		auto centroids_y = lidar_cloud.getCentroidsY();
		lidar_cloud.new_measurement = false;
		lidar_cloud.mtxData.unlock();

		// Render pointcloud
		renderer.renderPointCloud(cloud, "pointCloud", color);

		// Render boxes
		for (size_t i = 0; i < boxes.size(); ++i)
			renderer.renderBox(boxes[i], i);

		// Call the tracker on the detected clusters
		tracker.track(centroids_x, centroids_y, renderer.getLidarStatus());

		renderer.addText(0.0, 0.0, 0);

		// Retrieve tracklets and render the trackers
		auto tracks = tracker.getTracks();
		// for (auto &track : tracks)
		// {
		// 	renderer.addCircle(track.getX(), track.getY(), track.getId());
		// 	//renderer.addText(track.getX() + 0.01, track.getY() + 0.01, track.getId());
		// }

		renderer.spinViewerOnce(100);
	}

	t.join();
	return 0;
}
