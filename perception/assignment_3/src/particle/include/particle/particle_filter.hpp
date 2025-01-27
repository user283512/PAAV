#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include <vector>
#include <string>
#include "map.hpp"
#include "helper_functions.hpp"

// Each Particle represents a hypothesis about the system's (e.g., a vehicle's) state
// in terms of position (x, y), orientation (theta), and its relative likelihood (weight).
// Each particle represents a possible state, and the weights are updated during the process
// to reflect the likelihood of each hypothesis.
struct Particle
{
	int id;												 // Unique identifier for the particle.
	double x;											 // Longitudinal position of the particle (along the direction of travel).
	double y;											 // Lateral position of the particle (perpendicular to the direction of travel).
	double theta;									 // Orientation of the particle (vehicle's heading angle in radians).
	double weight;								 // Importance weight of the particle, representing its likelihood.
	std::vector<int> associations; // Stores associations between observations and landmarks (e.g., map landmarks).
	std::vector<double> sense_x;	 // Stores the x-coordinates of sensed landmarks in the world frame.
	std::vector<double> sense_y;	 // Stores the y-coordinates of sensed landmarks in the world frame.

	Particle(double x = 0,
					 double y = 0,
					 double theta = 0)
			: x(x), y(y), theta(theta)
	{
	}
};

// The ParticleFilter class implements the functionality of a particle filter, commonly used for localization.
// It uses a set of particles to represent possible states and updates them based on
// observations, motion models, and resampling.
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

	// Initializes the particle filter by creating nParticles particles centered around a
	// specified position (x, y, theta) with Gaussian noise.
	// Each particle's position and orientation is sampled from a Gaussian distribution centered on (x, y, theta).
	// All particles are initialized with equal weights (weight = 1.0).
	// Marks the particle filter as initialized (is_initialized = true).
	// Input:
	// 	- x, y, theta: Initial position and orientation
	//	- std[]: Array of standard deviations for noise in x, y, and theta.
	// 	- nParticles: Number of particles to initialize.
	void init(double x,
						double y,
						double theta,
						double std[],
						int nParticles);

	// Initializes the particle filter by randomly distributing particles across the map.
	// Randomly assigns x, y, and theta values to particles within a predefined range
	// Useful for global localization when the initial position of the system is unknown.
	// Input:
	// 	- std[]: Array of standard deviations for noise in x, y, and theta.
	// 	- nParticles: Number of particles to initialize.
	void init_random(double std[],
									 int nParticles);

	// Predicts the next state of each particle based on the motion model of the vehicle.
	// Input:
	//	- delta_t: Time elapsed between the current and the next prediction, in seconds.
	//	- std_pos[]: Array of standard deviations [x,y,theta]
	//  - velocity: Vehicle velocity, in meters per second.
	//	- yaw_rate: Vehicle yaw rate, in radians per second.
	void prediction(double delta_t,
									double std_pos[],
									double velocity,
									double yaw_rate);

	// Associates observed landmarks (from sensors) with predicted landmarks (from the map)
	// using a nearest-neighbor approach.
	// For each observation, finds the closest predicted landmark and associates it.
	// Updates each observation with the ID of the closest predicted landmark.
	// Input:
	// 	- predicted: Vector of predicted landmark positions
	//	-	observations: Vector of observed landmark positions (from sensors)
	void dataAssociation(const std::vector<LandmarkObs> &predicted,
											 std::vector<LandmarkObs> &observations);

	// Updates the weight of each particle based on the likelihood of observed landmarks
	// matching the predicted landmarks.
	// Transforms observations from the vehicle's local frame to the global map frame.
	// Associates transformed observations with map landmarks using the dataAssociation method.
	// Computes the likelihood of each particle using a multivariate Gaussian distribution and updates its weight.
	// Normalizes the weights to ensure they sum to 1.
	// Input:
	//	- std_landmark[]: Array of standard deviations [range, bearing] for sensor measurements.
	//	- observations: Observed landmarks from sensors, in the vehicle's local coordinate frame.
	//	- map_landmarks: Map containing known landmark positions.
	void updateWeights(double std_landmark[],
										 const std::vector<LandmarkObs> &observations,
										 const Map &map_landmarks);

	// Resamples particles based on their weights to create a new set of particles.
	// Particles with higher weights are more likely to be selected.
	// Removes particles with low weights and focuses on particles that better represent the system's state.
	void resample();

	// Sets associations for a particle, linking observations with map landmarks.
	// Updates the particle's associations, sense_x, and sense_y fields with the provided data.
	// Input:
	//	-	particle: The particle to update.
	//	-	associations: List of landmark IDs associated with the particle.
	//	-	sense_x, sense_y: Global coordinates of the associated observations.
	Particle SetAssociations(const Particle &particle,
													 const std::vector<int> &associations,
													 const std::vector<double> &sense_x,
													 const std::vector<double> &sense_y);

	// Retrieves a string representation of the associations (landmark IDs) for a given particle.
	std::string getAssociations(const Particle &best);
	// Retrieves a string representation of the sense_x values for a given particle.
	std::string getSenseX(const Particle &best);
	// Retrieves a string representation of the sense_y values for a given particle.
	std::string getSenseY(const Particle &best);

	// Checks whether the particle filter has been initialized.
	bool initialized() const { return is_initialized; }

	std::vector<Particle> particles; // Vector containing all the particles in the filter.

private:
	int num_particles;					 // Number of particles used in the filter.
	bool is_initialized;				 // Flag indicating whether the particle filter has been initialized.
	std::vector<double> weights; // Vector storing the weights of all particles.
};

#endif