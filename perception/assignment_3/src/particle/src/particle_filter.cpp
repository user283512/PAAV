#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
#include <sstream>
#include <numeric>

#include "particle/particle_filter.hpp"

static std::default_random_engine gen;

void ParticleFilter::init_random(double std[],
																 int nParticles,
																 int map_x_min,
																 int map_x_max,
																 int map_y_min,
																 int map_y_max)
{
	if (nParticles <= 0)
		throw std::invalid_argument("Number of particles must be greater than 0.");
	if (std[0] < 0 || std[1] < 0 || std[2] < 0)
		throw std::invalid_argument("Standard deviations must be non-negative.");
	if (map_x_min >= map_x_max || map_y_min >= map_y_max)
		throw std::invalid_argument("Invalid map boundaries.");

	// Uniform Distribution: If you do not have a good initial position estimate and
	// want to explore the whole area uniformly.

	// Create Uniform Distributions for x, y, and theta.
	std::uniform_real_distribution<double> dist_x(map_x_min, map_x_max);
	std::uniform_real_distribution<double> dist_y(map_y_min, map_y_max);
	std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

	// Initialize all weights to 1.0
	weights.assign(nParticles, 1.0);

	// Create particles
	particles.reserve(nParticles);
	for (int i = 0; i < nParticles; ++i)
	{
		particles.emplace_back();
		Particle &particle = particles.back();
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;
	}

	num_particles = nParticles;
	is_initialized = true;
}

void ParticleFilter::init(double x,
													double y,
													double theta,
													double std[],
													int nParticles)
{
	if (nParticles <= 0)
		throw std::invalid_argument("Number of particles must be greater than 0.");
	if (std[0] < 0 || std[1] < 0 || std[2] < 0)
		throw std::invalid_argument("Standard deviations must be non-negative.");

	// Normal Distribution: If you have a good initial estimate of position and you want the particles
	// to be concentrated around this estimate with some noise.

	// Create Normal (Gaussian) Distributions for x, y, and theta
	std::normal_distribution<double> dist_x(x, std[0]);					// Gaussian distribution for x
	std::normal_distribution<double> dist_y(y, std[1]);					// Gaussian distribution for y
	std::normal_distribution<double> dist_theta(theta, std[2]); // Gaussian distribution for theta

	// Initialize all weights to 1.0
	weights.assign(nParticles, 1.0);

	// Create particles
	particles.reserve(nParticles);
	for (int i = 0; i < nParticles; ++i)
	{
		particles.emplace_back();
		Particle &particle = particles.back();
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;
	}

	num_particles = nParticles;
	is_initialized = true;

	std::printf("Particle filter initialized with %d particles.\n", num_particles);
}

void ParticleFilter::prediction(double delta_t,
																double std_pos[],
																double velocity,
																double yaw_rate)
{
	std::normal_distribution<double> dist_x(0, std_pos[0]);
	std::normal_distribution<double> dist_y(0, std_pos[1]);
	std::normal_distribution<double> dist_theta(0, std_pos[2]);

	for (auto &p : particles)
	{
		// If the yaw rate, Theta (O), is very close to zero (∣O∣ < 0.0001), it means that the forklift is not rotating.
		// The movement of the forklift is linear (it moves along a straight line).
		// In this case, we can simplify the motion model.
		if (fabs(yaw_rate) < 0.0001f)
		{
			p.x += velocity * delta_t * cos(p.theta); // Dx = v*Dt*cos(O)
			p.y += velocity * delta_t * sin(p.theta); // Dy = v*Dt*sin(O)
		}
		// The forklift is rotating, its movement is curvilinear (along an arc of a circle)
		else
		{
			// The radius of the circular trajectory (R) is given by R = V/O
			double R = velocity / yaw_rate;
			p.x += R * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta)); // Dx = R(sin(O + O'*Dt) − sin(O))
			p.y += R * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t)); // Dy = R(cos(O) − cos(O + O'*Dt))
			p.theta += yaw_rate * delta_t;																 // O' = O + O'*Dt
		}

		// Add Gaussian noise
		p.x += dist_x(gen);
		p.y += dist_y(gen);
		p.theta += dist_theta(gen);
	}
}

void ParticleFilter::updateWeights(double std_landmark[],
																	 const std::vector<LandmarkObs> &observations,
																	 const Map &map_landmarks)
{
	// Iterate on all particles
	for (auto &particle : particles)
	{
		double weight = 1.0;

		// 1. Transformation of observations: LiDAR observations are initially in local coordinates
		// (relative to the vehicle). These must be transformed into global (map) coordinates using
		// the position and orientation of the particle.
		std::vector<LandmarkObs> transformed_observations;
		transformed_observations.reserve(observations.size());
		for (const auto &obs : observations)
			transformed_observations.push_back(transformation(obs, particle));

		// 2. Landmark association: for each transformed observation, we find the nearest landmark in the map.
		// This is done by calculating the Euclidean distance between the observation and each landmark in the map.
		dataAssociation(map_landmarks.landmark_list, transformed_observations);

		// 3. Probability calculation: the probability that the observation matches the nearest landmark
		// is calculated using a two-dimensional Gaussian distribution.
		// The probability is highest when the observation is close to the landmark.
		for (const auto &obs : transformed_observations)
		{
			// Find the associated landmark
			Map::single_landmark_s nearest_landmark;
			for (const auto &landmark : map_landmarks.landmark_list)
			{
				if (landmark.id_i == obs.id)
				{
					nearest_landmark = landmark;
					break;
				}
			}

			// Calculate the probability of the observation using the Gaussian distribution:
			// P = (1 / 2*Pi*o_x*o_y) * exp( -(dx^2)/(2o_x^2) -(dy^2)/(2o_y^2))
			double std_x = std_landmark[0]; // o_x
			double std_y = std_landmark[1]; // o_y
			double dx = obs.x - nearest_landmark.x_f;
			double dy = obs.y - nearest_landmark.y_f;
			double gauss_norm = 1 / (2 * M_PI * std_x * std_y);
			double exponent = (pow(dx, 2) / (2 * pow(std_x, 2))) +
												(pow(dy, 2) / (2 * pow(std_y, 2)));
			double prob = gauss_norm * exp(-exponent);

			// 4. Weight update: the weight of the particle is updated by multiplying the probabilities
			// of all observations. This weight represents the probability that the particle is in the correct position.
			weight *= prob;
		}

		particle.weight = weight;
	}

	// 5. Normalization: after updating the weights of all particles, it is common to normalize
	// the weights so that their sum is 1.
	double weight_sum = std::reduce(particles.begin(), particles.end(), 0.0f, [&](double sum, const Particle &p)
																	{ return sum + p.weight; });
	for (auto &particle : particles)
	{
		if (weight_sum > 0.0f)
			particle.weight /= weight_sum;
		else
			particle.weight = 1.0f / particles.size();
	}
}

void ParticleFilter::dataAssociation(const std::vector<LandmarkObs> &map_landmarks,
																		 std::vector<LandmarkObs> &transformed_observations)
{
	for (auto &obs : transformed_observations)
	{
		double min_dist = std::numeric_limits<double>::max(); // Minimum initial distance
		int associated_id = -1;																// ID of associated landmark

		// Find the nearest landmark
		for (const auto &landmark : map_landmarks.landmark_list)
		{
			double dx = obs.x - landmark.x_f;
			double dy = obs.y - landmark.y_f;
			double distance = sqrt(pow(dx, 2) + pow(dy, 2)); // Distanza euclidea al quadrato
			if (distance < min_dist)
			{
				min_dist = distance;
				associated_id = landmark.id_i;
			}
		}

		// Associates the observation with the nearest landmark
		obs.id = associated_id;
	}
}

LandmarkObs ParticleFilter::transformation(const LandmarkObs &obs,
																					 const Particle &particle)
{
	// x_global​ ​= x_particle ​ + x_obs*cos(O) − y_obs​*sin(O)
	double x_global = particle.x + obs.x * cos(particle.theta) - obs.y * sin(particle.theta);
	// y_global​ ​= x_particle ​ + x_obs*sin(O) + y_obs​*cos(O)
	double y_global = particle.x + obs.x * sin(particle.theta) + obs.y * cos(particle.theta);
	return LandmarkObs(obs.id, x_global, y_global);
}

void ParticleFilter::resample()
{
	if (particles.empty())
	{
		std::cout << "particles.empty()\n";
		return;
	}

	// Get the maximum weight among all particles
	auto max_weight_particle = std::max_element(particles.begin(), particles.end(), [](const Particle &a, const Particle &b)
																							{ return a.weight < b.weight; });
	double max_weight = max_weight_particle->weight;
	// Random number generator
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dist(0.0f, 2.0f * max_weight); // Beta is in [0, 2 * max_weight]

	// Resampling wheel algorithm:
	// 1. Pick a random index
	int index = std::uniform_int_distribution<>(0, particles.size() - 1)(gen);

	// 2. Initialize beta
	double beta = 0.0;

	std::vector<Particle> new_particles;
	new_particles.reserve(particles.size());
	for (int i = 0; i < particles.size(); ++i)
	{
		// 3: Add a random value to beta
		beta += dist(gen);

		// 4: Find the particle whose weight is larger than beta
		while (beta > particles[index].weight)
		{
			beta -= particles[index].weight;				// Subtract the weight of the current particle
			index = (index + 1) % particles.size(); // Move to the next particle
		}

		// 5: Add the selected particle to the new particles list
		new_particles.push_back(particles[index]);
	}
	// 6. Replace the old particles with the new particles
	particles = new_particles;
}
