#ifndef HELPER_FUNCTIONS_HPP_
#define HELPER_FUNCTIONS_HPP_

#include <vector>
#include <string>
#include <filesystem>

#include "map.hpp"

// Struct representing one position/control measurement.
struct control_s
{
	double velocity; // Velocity [m/s]
	double yawrate;	 // Yaw rate [rad/s]
};

// Struct representing one ground truth position.
struct ground_truth
{
	double x;			// Global vehicle x position [m]
	double y;			// Global vehicle y position
	double theta; // Global vehicle yaw [rad]
};

// Struct representing one landmark observation measurement.
struct LandmarkObs
{
	int id;		// Id of matching landmark in the map.
	double x; // Local (vehicle coordinates) x position of landmark observation [m]
	double y; // Local (vehicle coordinates) y position of landmark observation [m]
};

// Computes the Euclidean distance between two 2D points.
// @param (x1,y1) x and y coordinates of first point
// @param (x2,y2) x and y coordinates of second point
// @output Euclidean distance between two 2D points
double dist(double x1,
						double y1,
						double x2,
						double y2);

double *getError(double gt_x,
								 double gt_y,
								 double gt_theta,
								 double pf_x,
								 double pf_y,
								 double pf_theta);

// Reads map data from a file.
// @param filename Name of file containing map data.
// @output True if opening and reading file was successful
bool read_map_data(const std::filesystem::path &path,
									 Map &map);

// Reads control data from a file.
// @param filename Name of file containing control measurements.
// @output True if opening and reading file was successful
bool read_control_data(const std::filesystem::path &path,
											 std::vector<control_s> &position_meas);

// Reads ground truth data from a file.
// @param filename Name of file containing ground truth.
// @output True if opening and reading file was successful
bool read_gt_data(const std::filesystem::path &path,
									std::vector<ground_truth> &gt);

// Reads landmark observation data from a file.
// @param filename Name of file containing landmark observation measurements.
// @output True if opening and reading file was successful
bool read_landmark_data(const std::filesystem::path &path,
												std::vector<LandmarkObs> &observations);
#endif