#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include <vector>
#include <string>
#include "map.hpp"
#include "Box.hpp"
#include "helper_functions.hpp"

//  The Particle struct is used to represent a single particle in the particle filter algorithm.
// Each particle is a hypothesis about the forklift's position and orientation in the warehouse.
struct Particle
{
	int id;				 // The particle ID
	double x;			 // X-coordinate of the particle in the global coordinate system
	double y;			 // Y-coordinate of the particle in the global coordinate system
	double theta;	 // Orientation (angle) of the particle in radians
	double weight; // Weight of the particle (probability that it represents the correct
								 // location)

	// Vectors to store associations and sensed positions
	std::vector<int> associations; // Stores IDs of associated landmarks or features
	std::vector<double> sense_x;	 // Stores sensed X-coordinates of landmarks or features
	std::vector<double> sense_y;	 // Stores sensed Y-coordinates of landmarks or features

	Particle(double x = 0,
					 double y = 0,
					 double theta = 0)
			: x{x},
				y{y},
				theta{theta},
				id{0},
				weight{0},
				associations{},
				sense_x{},
				sense_y{}
	{
	}
};

// The ParticleFilter class implements a particle filter algorithm to estimate
// the position and orientation of a forklift in a warehouse using sensor data (LiDAR and odometry).
class ParticleFilter
{
public:
	ParticleFilter()
			: num_particles{0},
				is_initialized{false},
				particles{},
				weights{}
	{
	}
	~ParticleFilter() = default;

	// Initializes the particle filter by creating a set of particles centered around a 
	// specified position (x, y, theta) with added Gaussian noise.
	// Input:
	// 	- x, y, theta: Initial position and orientation of the forklift.
	//	- std[]: Array of standard deviations for noise in x, y, and theta.
	// 	- nParticles: Number of particles to initialize.
	void init(double x,
						double y,
						double theta,
						double std[],
						int nParticles);

	// Initializes the particle filter by randomly distributing particles across the map.
	// Input:
	// 	- std[]: Array of standard deviations for noise in x, y, and theta.
	// 	- nParticles: Number of particles to initialize.
	// 	- map_x_min, map_x_max: Minimum and maximum x-coordinates of the map.
	// 	- map_y_min, map_y_max: Minimum and maximum y-coordinates of the map.
	void init_random(double std[],
									 int nParticles,
									 int map_x_min,
									 int map_x_max,
									 int map_y_min,
									 int map_y_max);

	// Transforms an observation from the vehicle's local coordinate frame to the global map frame.
	// Input:
	// 	- obs: Observation in the vehicle's local frame.
	// 	- particle: Particle representing the vehicle's state.
	// Returns: Transformed observation in the global map frame.
	LandmarkObs transformation(const LandmarkObs &obs,
														 const Particle &particle);

	// Predicts the next state of each particle based on the motion model of the vehicle.
	// Input:
	//	- delta_t: Time elapsed between the current and the next prediction, in seconds.
	//	- std_pos[]: Array of standard deviations [x, y, theta] for process noise.
	//  - velocity: Vehicle velocity, in meters per second.
	//	- yaw_rate: Vehicle yaw rate, in radians per second.
	void prediction(double delta_t,
									double std_pos[],
									double velocity,
									double yaw_rate);

	// Updates the weight of each particle based on the likelihood of observed landmarks
	// matching the predicted landmarks.
	// Transforms observations from the vehicle's local frame to the global map frame.
	// Associates transformed observations with map landmarks using the dataAssociation method.
	// Computes the likelihood of each particle using a multivariate Gaussian distribution and updates 
	// its weight.
	// Normalizes the weights to ensure they sum to 1.
	// Input:
	//	- std_landmark[]: Array of standard deviations [range, bearing] for sensor measurements.
	//	- observations: Observed landmarks from sensors, in the vehicle's local coordinate frame.
	//	- map_landmarks: Map containing known landmark positions.
	void updateWeights(double std_landmark[],
										 const std::vector<LandmarkObs> &observations,
										 const Map &map_landmarks);

	// Associates observed landmarks (from sensors) with predicted landmarks (from the map)
	// using a nearest-neighbor approach.
	// For each observation, finds the closest predicted landmark and associates it.
	// Updates each observation with the ID of the closest predicted landmark.
	// Input:
	// 	- map_landmarks: Vector of predicted landmark positions.
	//	- transformed_observations: Vector of observed landmark positions (transformed to the global frame).
	void dataAssociation(const std::vector<LandmarkObs> &map_landmarks,
											 std::vector<LandmarkObs> &transformed_observations);

	// Resamples particles based on their weights to create a new set of particles.
	// Particles with higher weights are more likely to be selected.
	// Removes particles with low weights and focuses on particles that better represent the system's state.
	void resample();

	// Sets associations for a particle, linking observations with map landmarks.
	// Updates the particle's associations, sense_x, and sense_y fields with the provided data.
	// Input:
	//	- particle: The particle to update.
	//	- associations: List of landmark IDs associated with the particle.
	//	- sense_x, sense_y: Global coordinates of the associated observations.
	// Particle SetAssociations(const Particle &particle,
	// 												 const std::vector<int> &associations,
	// 												 const std::vector<double> &sense_x,
	// 												 const std::vector<double> &sense_y);

	// std::string getAssociations(const Particle &best);
	// std::string getSenseX(const Particle &best);
	// std::string getSenseY(const Particle &best);

	// Checks whether the particle filter has been initialized.
	// Returns: True if the particle filter is initialized, false otherwise.
	bool initialized() const { return is_initialized; }

	// Vector containing all the particles in the filter.
	std::vector<Particle> particles;

private:
	int num_particles;					 // Number of particles used in the filter.
	bool is_initialized;				 // Flag indicating whether the particle filter has been initialized.
	std::vector<double> weights; // Vector storing the weights of all particles.
};

#endif