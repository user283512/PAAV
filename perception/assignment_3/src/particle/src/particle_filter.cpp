#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
#include <sstream>

#include "particle/particle_filter.hpp"

static std::default_random_engine gen;

void ParticleFilter::init_random(double std[],
																 int nParticles)
{
	num_particles = nParticles;

	// Uniform Distribution: If you do not have a good initial position estimate and
	// want to explore the whole area uniformly.

	// Create Uniform Distributions for x, y, and theta
	std::uniform_real_distribution<double> dist_x(-100.0, 100.0); // Range arbitrario
	std::uniform_real_distribution<double> dist_y(-100.0, 100.0); // Range arbitrario
	std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

	// std::normal_distribution<double> dist_x(0, std[0]);			// Gaussian distribution for x (mean 0)
	// std::normal_distribution<double> dist_y(0, std[1]);			// Gaussian distribution for y (mean 0)
	// std::normal_distribution<double> dist_theta(0, std[2]); // Gaussian distribution for theta (mean 0)

	// Initialize particles
	particles.reserve(num_particles);
	for (int i = 0; i < num_particles; ++i)
	{
		particles.emplace_back();
		Particle &particle = particles.back();
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;
	}

	// Initialize weights
	weights.assign(num_particles, 1.0);

	is_initialized = true;
}

// TODO: complete this function
void ParticleFilter::init(double x,
													double y,
													double theta,
													double std[],
													int nParticles)
{
	num_particles = nParticles;

	// Normal Distribution: If you have a good initial estimate of position and you want the particles
	// to be concentrated around this estimate with some noise.

	// Create Normal (Gaussian) Distributions for x, y, and theta
	std::normal_distribution<double> dist_x(x, std[0]);					// Gaussian distribution for x
	std::normal_distribution<double> dist_y(y, std[1]);					// Gaussian distribution for y
	std::normal_distribution<double> dist_theta(theta, std[2]); // Gaussian distribution for theta

	// Initialize particles
	particles.reserve(num_particles);
	for (int i = 0; i < num_particles; ++i)
	{
		particles.emplace_back();
		Particle &particle = particles.back();
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;
	}

	// Initialize weights
	weights.assign(num_particles, 1.0);

	is_initialized = true;

	std::printf("Particle filter initialized with %d particles.\n", num_particles);
}

// TODO: complete this function
void ParticleFilter::prediction(double delta_t,
																double std_pos[],
																double velocity,
																double yaw_rate)
{
#if 0
	// for each particle
	double x, y, theta;
	if (fabs(yaw_rate) < 0.00001f)
	{
		// TODO
	}
	else
	{
		// TODO
	}
	std::normal_distribution<double> dist_x(0, std_pos[0]); // the random noise cannot be negative in this case
	std::normal_distribution<double> dist_y(0, std_pos[1]);
	std::normal_distribution<double> dist_theta(0, std_pos[2]);
	// TODO: add the computed noise to the current particles position (x,y,theta)
#endif
}

// TODO: complete this function
void ParticleFilter::dataAssociation(const std::vector<LandmarkObs> &mapLandmark,
																		 std::vector<LandmarkObs> &observations)
{
	// TIP: Assign to observations[i].id the id of the landmark with the smallest euclidean distance
}

// TODO: complete this function
LandmarkObs transformation(const LandmarkObs &observation,
													 const Particle &p)
{
#if 0
	LandmarkObs global;
	global.id = observation.id;
	global.x = -1; // TODO
	global.y = -1; // TODO
	return global;
#endif

	return LandmarkObs{0, 0, 0};
}

// TODO: complete this function
void ParticleFilter::updateWeights(double std_landmark[],
																	 const std::vector<LandmarkObs> &observations,
																	 const Map &map_landmarks)
{
#if 0
	// Creates a vector that stores tha map (this part can be improved)
	std::vector<LandmarkObs> mapLandmark;
	for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
	{
		mapLandmark.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f});
	}

	for (int i = 0; i < particles.size(); i++)
	{
		// Before applying the association we have to transform the observations in the global coordinates
		std::vector<LandmarkObs> transformed_observations;
		// TODO: for each observation transform it (transformation function)

		// TODO: perform the data association (associate the landmarks to the observations)

		particles[i].weight = 1.0;
		// Compute the probability
		// The particles final weight can be represented as the product of each measurement’s Multivariate-Gaussian probability density
		// We compute basically the distance between the observed landmarks and the landmarks in range from the position of the particle
		for (int k = 0; k < transformed_observations.size(); k++)
		{
			double obs_x, obs_y, l_x, l_y;
			obs_x = transformed_observations[k].x;
			obs_y = transformed_observations[k].y;
			// get the associated landmark
			for (int p = 0; p < mapLandmark.size(); p++)
			{
				if (transformed_observations[k].id == mapLandmark[p].id)
				{
					l_x = mapLandmark[p].x;
					l_y = mapLandmark[p].y;
				}
			}
			// How likely a set of landmarks measurements are, given a prediction state of the car
			double w = exp(-(pow(l_x - obs_x, 2) / (2 * pow(std_landmark[0], 2)) + pow(l_y - obs_y, 2) / (2 * pow(std_landmark[1], 2)))) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
			particles[i].weight *= w;
		}
	}
#endif
}

// TODO: complete this function
void ParticleFilter::resample()
{
#if 0
	std::uniform_int_distribution<int> dist_distribution(0, num_particles - 1);
	double beta = 0.0;
	std::vector<double> weights;
	int index = dist_distribution(gen);
	std::vector<Particle> new_particles;

	for (int i = 0; i < num_particles; i++)
		weights.push_back(particles[i].weight);

	float max_w = *max_element(weights.begin(), weights.end());
	std::uniform_real_distribution<double> uni_dist(0.0, max_w);

	// TODO write here the resampling technique (feel free to use the above variables)
#endif
}

// TODO: complete this function
Particle ParticleFilter::SetAssociations(const Particle &particle,
																				 const std::vector<int> &associations,
																				 const std::vector<double> &sense_x,
																				 const std::vector<double> &sense_y)
{
}

// TODO: complete this function
std::string ParticleFilter::getAssociations(const Particle &best)
{
	return "";
}

// TODO: complete this function
std::string ParticleFilter::getSenseX(const Particle &best)
{
	return "";
}

// TODO: complete this function
std::string ParticleFilter::getSenseY(const Particle &best)
{
	return "";
}