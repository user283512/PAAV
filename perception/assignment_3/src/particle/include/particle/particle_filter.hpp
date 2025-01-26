#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include <vector>
#include <string>
#include "map.hpp"
#include "helper_functions.hpp"

struct Particle
{
	int id;
	double x;											 // denotes the longitudinal position (along the direction of travel)
	double y;											 // represents the lateral position
	double theta;									 // orientation of the vehicle (heading angle)
	double weight;								 // represents the weight/importance of the particle
	std::vector<int> associations; // stores the associations between measurements and map
	std::vector<double> sense_x;
	std::vector<double> sense_y;

	Particle(double x = 0,
					 double y = 0,
					 double theta = 0)
			: x(x), y(y), theta(theta)
	{
	}
};

class ParticleFilter
{
	// Number of particles to draw
	int num_particles;

	// Flag, if filter is initialized
	bool is_initialized;

	// Vector of weights of all particles
	std::vector<double> weights;

public:
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param M Number of particles
	ParticleFilter()
			: num_particles(0), is_initialized(false)
	{
	}

	// Destructor
	~ParticleFilter() = default;

	// init Initializes particle filter by initializing particles to Gaussian
	//   distribution around first position and all the weights to 1.
	// @param x Initial x position [m] (simulated estimate from GPS)
	// @param y Initial y position [m]
	// @param theta Initial orientation [rad]
	// @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	//   standard deviation of yaw [rad]]
	// @param nParticles Number of particles used by the algorithm
	void init(double x,
						double y,
						double theta,
						double std[],
						int nParticles);

	// init Initializes particle filter by randomly distributing the particles
	// around the map.
	// @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	//   standard deviation of yaw [rad]]
	// @param nParticles Number of particles used by the algorithm
	void init_random(double std[],
									 int nParticles);

	// prediction Predicts the state for the next time step
	//   using the process model.
	// @param delta_t Time between time step t and t+1 in measurements [s]
	// @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	//   standard deviation of yaw [rad]]
	// @param velocity Velocity of car from t to t+1 [m/s]
	// @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	void prediction(double delta_t,
									double std_pos[],
									double velocity,
									double yaw_rate);

	// dataAssociation Finds which observations correspond to which landmarks (likely by using
	//   a nearest-neighbors data association).
	// @param predicted Vector of predicted landmark observations
	// @param observations Vector of landmark observations
	void dataAssociation(const std::vector<LandmarkObs> &predicted,
											 std::vector<LandmarkObs> &observations);

	// updateWeights Updates the weights for each particle based on the likelihood of the
	//   observed measurements.
	// @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	//   standard deviation of bearing [rad]]
	// @param observations Vector of landmark observations
	// @param map Map class containing map landmarks
	void updateWeights(double std_landmark[],
										 const std::vector<LandmarkObs> &observations,
										 const Map &map_landmarks);

	// resample Resamples from the updated set of particles to form
	//   the new set of particles.
	void resample();

	// Set a particles list of associations, along with the associations calculated world x,y coordinates
	// This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	Particle SetAssociations(const Particle &particle,
													 const std::vector<int> &associations,
													 const std::vector<double> &sense_x,
													 const std::vector<double> &sense_y);

	std::string getAssociations(const Particle &best);
	std::string getSenseX(const Particle &best);
	std::string getSenseY(const Particle &best);

	// Initialized Returns whether particle filter is initialized yet or not.
	bool initialized() const { return is_initialized; }
};

#endif