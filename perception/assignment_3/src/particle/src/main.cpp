#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <filesystem>

#include <boost/foreach.hpp>

#include <pcl/type_traits.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "Box.hpp"
#include "Renderer.hpp"
#include "particle/helper_cloud.hpp"
#include "particle/helper_functions.hpp"
#include "particle/particle_filter.hpp"
#include "Globals.hpp"

using namespace lidar_obstacle_detection;

DEFINE_double(robust_threshold, 0.0, "Robust loss parameter. Set to 0 for normal squared error (no robustification).");

static Map map_mille;
static ParticleFilter pf;
static Renderer renderer;
static control_s odom;

static bool init_odom = false;
static std::vector<Particle> best_particles;
static std::ofstream myfile;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_particles(new pcl::PointCloud<pcl::PointXYZ>);

// This function updates the position of the particles in the viewer
static void showPCstatus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
												 const std::vector<Particle> &particles)
{
	for (size_t i = 0; i < particles.size(); ++i)
	{
		cloud->points[i].x = particles[i].x;
		cloud->points[i].y = particles[i].y;
	}
	renderer.updatePointCloud(cloud, "particles");
}

// This function adds the observations from the LiDAR (reflectors) and updates its position
static void updateViewerReflector(pcl::PointCloud<pcl::PointXYZI> reflectorCenter)
{
	for (int i = 0; i < reflectorCenter.size(); i++)
	{
		// Update the pose of the reflectors with respect the best particle
		pcl::PointXYZI pt;
		pt.x = reflectorCenter[i].x;
		pt.y = reflectorCenter[i].y;
		pt.z = 0.0f;
		float gx = pt.x * cos(best_particles.back().theta) - pt.y * sin(best_particles.back().theta) + best_particles.back().x;
		float gy = pt.x * sin(best_particles.back().theta) + pt.y * cos(best_particles.back().theta) + best_particles.back().y;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << gx, gy, 0;

		// Apply the transformation
		renderer.updatePose("reflector_id" + std::to_string(i), transform);
		renderer.updateShape("reflector_id" + std::to_string(i), 1.0);
	}
}

// This functions processes the odometry from the forklift (Prediction phase)
static void OdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	static std::chrono::time_point<std::chrono::high_resolution_clock> t_start;
	static std::chrono::time_point<std::chrono::high_resolution_clock> t_end;

	odom.velocity = msg->twist.twist.linear.x;
	odom.yawrate = msg->twist.twist.angular.z;
	// t_start=msg->header.stamp.toSec();
	t_start = std::chrono::high_resolution_clock::now();
	// Prediction phase
	if (!init_odom)
	{
		pf.prediction(0, g_sigma_pos, odom.velocity, odom.yawrate);
		init_odom = true;
	}
	else
	{
		// double delta_t = t_start-t_end;
		double delta_t = (std::chrono::duration<double, std::milli>(t_start - t_end).count()) / 1000;
		pf.prediction(delta_t, g_sigma_pos, odom.velocity, odom.yawrate);
	}

	t_end = std::chrono::high_resolution_clock::now();
}

// This functions processes the point cloud (Update phase)
static void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	std::chrono::time_point<std::chrono::high_resolution_clock> t_start =
			std::chrono::high_resolution_clock::now();

	// Convert to PCL data type
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*cloud_msg, *cloud);

	// Extract landmarks
	pcl::PointCloud<pcl::PointXYZI> reflectorCenter = extractReflectors(cloud);

	// Let's hide all the reflectors
	for (int i = 0; i < g_num_reflectors; i++)
		renderer.updateShape("reflector_id" + std::to_string(i), 0.0);

	// Update the observations and shows the reflectors (this can be improved, this line can be executed after computing the best particle)
	updateViewerReflector(reflectorCenter);

	// Receive noisy observation data
	std::vector<LandmarkObs> noisy_observations;
	noisy_observations.reserve(reflectorCenter.size());
	for (int i = 0; i < reflectorCenter.size(); i++)
	{
		LandmarkObs obs;
		obs.x = reflectorCenter[i].x;
		obs.y = reflectorCenter[i].y;
		noisy_observations.push_back(obs);
	}

	// Update the weights of the particle
	pf.updateWeights(g_sigma_landmark, noisy_observations, map_mille);

	// Resample the particles
	pf.resample();

	// Calculate and output the average weighted error of the particle filter over all time steps so far.
	Particle best_particle;
	std::vector<Particle> particles = pf.particles;
	double highest_weight = -1.0;
	for (int i = 0; i < particles.size(); ++i)
	{
		if (particles[i].weight > highest_weight)
		{
			highest_weight = particles[i].weight;
			best_particle = particles[i];
		}
	}
	best_particles.push_back(best_particle);

	// Show the particles in the map
	showPCstatus(cloud_particles, particles);
	renderer.removeShape("circle_id" + std::to_string(g_num_particles + 1));
	renderer.addCircle(best_particles.back().x, best_particles.back().y, "circle_id" + std::to_string(g_num_particles + 1), 0.3, 1, 0, 0);

	// Log the execution time
	std::chrono::time_point<std::chrono::high_resolution_clock> t_end =
			std::chrono::high_resolution_clock::now();

	double delta_t = (std::chrono::duration<double, std::milli>(t_end - t_start).count()) / 1000;
	// Write the results in a file
	myfile << best_particle.x << " " << best_particle.y << " " << delta_t << '\n';

	renderer.SpinViewerOnce();
}

int main(int argc, char **argv)
{
	namespace fs = std::filesystem;
	static const fs::path data_dir = fs::current_path() / "data";
	static const fs::path map_data_file = data_dir / "map_data.txt";
	static const fs::path map_pepperl_file = data_dir / "map_pepperl.pcd";
	static const fs::path map_reflector_file = data_dir / "map_reflector.pcd";
	static const fs::path out_bag_file = data_dir / "out.bag";
	static const fs::path res_file = data_dir / "res.txt";
	static const fs::path pf_slam_file = data_dir / "pf_slam.txt";
	if (!fs::exists(map_data_file))
	{
		std::cout << "Error: data/map_data.txt file does not exist" << endl;
		return -1;
	}
	if (!fs::exists(map_pepperl_file))
	{
		std::cout << "Error: data/map_pepperl.pcd file does not exist" << endl;
		return -1;
	}
	if (!fs::exists(map_reflector_file))
	{
		std::cout << "Error: data/map_reflector.pcd file does not exist" << endl;
		return -1;
	}
	if (!fs::exists(out_bag_file))
	{
		std::cout << "Error: data/out.bag file does not exist" << endl;
		return -1;
	}
	if (!fs::exists(res_file))
	{
		std::cout << "Error: data/res.txt file does not exist" << endl;
		return -1;
	}
	if (!fs::exists(pf_slam_file))
	{
		std::cout << "Error: data/pf_slam.txt file does not exist" << endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudReflectors(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(map_reflector_file, *cloudReflectors); // cloud with just the reflectors
	pcl::io::loadPCDFile(map_pepperl_file, *cloudMap);					// total cloud (used for rendering)

	remove(res_file);
	// This function locates the reflectors within the map and writes into the file
	createMap(cloudReflectors, map_data_file, map_mille);

	// Read map data
	if (!read_map_data(map_data_file, map_mille))
	{
		cout << "Error: Could not open map file" << endl;
		return -1;
	}

	// Reduce the number of points in the map point cloud (for improving the performance of the rendering)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_map(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloudMap);
	vg.setLeafSize(1.0f, 1.0f, 1.0f); // Set the voxel grid
	vg.filter(*cloud_filtered_map);

	// Starts the rendering
	renderer.InitCamera(CameraAngle::XY);
	// Clear viewer
	renderer.ClearViewer();

	// Render the map and reflectors
	renderer.RenderPointCloud(cloud_filtered_map, "originalCloud", Color(0, 0, 1));
	renderer.RenderPointCloud(cloudReflectors, "reflectorCloud", Color(1, 0, 0));

	// Add the reflectors detected by the particles (you can ignore this)
	for (int i = 0; i < g_num_reflectors; i++)
		renderer.addCircle(0, 0, "reflector_id" + std::to_string(i), 0.2, 1, 1, 1);

	// map coordinates
	constexpr int map_x_min = -12;
	constexpr int map_x_max = 12;
	constexpr int map_y_min = -27;
	constexpr int map_y_max = 35;

	// Initial position of the forklift
	static constexpr double GPS_x = 2.37256;
	static constexpr double GPS_y = 1.70077;
	static constexpr double GPS_theta = -1.68385;

	// Insert one particle in the best particle set
	Particle p(GPS_x, GPS_y, GPS_theta);
	best_particles.push_back(p);

	// Init the particle filter
	// pf.init(GPS_x, GPS_y, GPS_theta, g_sigma_init, g_num_particles);
	pf.init_random(sigma_init, g_num_particles, map_x_min, map_x_max, map_y_min, map_y_max);

	assert(pf.initialized());
	std::cout << "Particle filter initialized with " << pf.particles.size() << " particles." << std::endl;

	// Render all the particles
	for (int i = 0; i < g_num_particles; i++)
	{
		pcl::PointXYZ point;
		point.x = pf.particles[i].x;
		point.y = pf.particles[i].y;
		point.z = 0;
		cloud_particles->push_back(point);
	}
	renderer.RenderPointCloud(cloud_particles, "particles", Color(1, 0, 0));

	// render the best initial guess as a circle
	renderer.addCircle(GPS_x, GPS_y, "circle_id" + std::to_string(g_num_particles + 1), 0.4, 0, 1, 1);
	renderer.SpinViewerOnce();

	// Start ROS node
	std::cout << "Map loaded, waiting for the rosbag" << std::endl;
	myfile.open(res_file, std::ios_base::app);

	ros::init(argc, argv, "Particle");
	ros::NodeHandle n;

	// Subscriber
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/auriga_id0_odom", 1, &OdomCb);
	ros::Subscriber pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/pepperl_id0_cloud", 1, &PointCloudCb);
	// To force 10hz replay use: rosbag play --clock --hz=10 out.bag
	ros::spin();
	myfile.close();
}
