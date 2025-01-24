#include <iostream>
#include <fstream>
#include <filesystem>
#include <set>

#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"

namespace fs = std::filesystem;
extern double g_distance_threshold;
extern double g_covariance_threshold;
extern int g_loss_threshold;

// Define an area and count the persons that has entered in that area

static std::set<int> persons_inside; // IDs of the persons currently in the area
static int people_entered_count = 0;

static bool isTrackInside(const viewer::Box &area,
													const Tracklet &track)
{
	return (
			track.getX() <= area.x_max && track.getX() >= area.x_min &&
			track.getY() <= area.y_max && track.getY() >= area.y_min);
}

static int countTrackInArea(const std::vector<Tracklet> &tracks,
														const viewer::Box &area)
{
	auto lambda = [&](const Tracklet &track)
	{ return isTrackInside(area, track); };

	return std::count_if(tracks.begin(), tracks.end(), lambda);
}

static void updatePeopleInArea(const std::vector<Tracklet> &tracks,
															 const viewer::Box &area)
{
	std::set<int> current_inside;

	for (const auto &track : tracks)
	{
		if (isTrackInside(area, track))
		{
			current_inside.insert(track.getId());

			// If a person was not previously inside, increment the counter.
			if (persons_inside.find(track.getId()) == persons_inside.end())
				++people_entered_count;
		}
	}

	persons_inside = current_inside;
}

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
	constexpr int freq = 100;

	// Spawn the thread that process the point cloud and performs the clustering
	CloudManager lidar_cloud(res_dir, freq, renderer);
	std::thread t(&CloudManager::startCloudManager, &lidar_cloud);

	// Create an area of size 10x10
	viewer::Box area_of_interest = {
			-5.0f, -5.0f, 0.0f,
			5.0f, 5.0f, 0.0f};

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

		// Render area
		renderer.renderBox(area_of_interest, 0, viewer::Color(0, 1, 0), 0.25f);

		// Render boxes
		for (size_t i = 0; i < boxes.size(); ++i)
			renderer.renderBox(boxes[i], i + 1, viewer::Color(1, 0, 0));

		// Call the tracker on the detected clusters
		tracker.track(centroids_x, centroids_y, renderer.getLidarStatus());

		// Retrieve tracklets and render the trackers
		auto tracks = tracker.getTracks();
		for (auto &track : tracks)
		{
			renderer.addCircle(track.getX(), track.getY(), track.getId());
			renderer.addText(track.getX() + 0.01, track.getY() + 0.01, track.getId());
		}

		// Update the count of persons entering the area.
		updatePeopleInArea(tracks, area_of_interest);
		std::printf("countTrackInArea=%d, people_entered_count=%d\n",
								countTrackInArea(tracks, area_of_interest),
								people_entered_count);

		renderer.spinViewerOnce();
	}

	t.join();
	return 0;
}
