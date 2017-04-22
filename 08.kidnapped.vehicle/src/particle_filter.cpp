/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

using namespace std;

inline double bivariate_normal(double x, double y, double m_x, double m_y, double s_x, double s_y) {
	return exp(-((x - m_x)*(x - m_x) / (2 * s_x*s_x) + (y - m_y)*(y - m_y) / (2 * s_y*s_y))) / (2 * M_PI*s_x*s_y);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	default_random_engine gen; // generates pseudo-random numbers
	num_particles = 100;

	for (int i = 0; i<num_particles; i++) {
		Particle tmp_particle;
		tmp_particle.id = i;
		tmp_particle.x = dist_x(gen);
		tmp_particle.y = dist_y(gen);
		tmp_particle.theta = dist_theta(gen);
		tmp_particle.weight = 1;
		particles.push_back(tmp_particle);
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (unsigned int i = 0; i<particles.size(); i++) {
		if (yaw_rate == 0) {
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else {
			particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			particles[i].theta += yaw_rate*delta_t;
		}
		// Adding noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (unsigned int i = 0; i < observations.size(); i++) {
		double distance_matched = 1e6; // large distance for unmatched neighbor
		int nearest_neighbor = -1; // unmatched neigbor == -1

		for (unsigned int j = 0; j < predicted.size(); j++) {
			// the euclidean distance between the observation and predicted observation
			double distance_predicted = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			if (distance_predicted < distance_matched) {
				distance_matched = distance_predicted;
				nearest_neighbor = j;
			}
		}
		observations[i].id = nearest_neighbor;
		// so now observation 'i' has been associated with a landmark in range
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

	weights.clear(); // initializing weights

	// for every particle
	for (unsigned int i = 0; i < particles.size(); i++) {

		vector<LandmarkObs> landmarksInRange;

		// step 1: find the landmarks in particle's range
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			double distanceFromParticle = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
			if (distanceFromParticle <= sensor_range) {
				LandmarkObs temp_landmark;
				temp_landmark.id = map_landmarks.landmark_list[j].id_i;
				temp_landmark.x = map_landmarks.landmark_list[j].x_f;
				temp_landmark.y = map_landmarks.landmark_list[j].y_f;
				landmarksInRange.push_back(temp_landmark);
			}
		}

		// step 2: transform observation coordinates from local to global
		vector<LandmarkObs> observationsInMapCoordinates;
		for (unsigned int j = 0; j < observations.size(); ++j) {
			LandmarkObs tmp_observation;
			tmp_observation.x = particles[i].x + observations[j].x*cos(particles[i].theta) - observations[j].y*sin(particles[i].theta);
			tmp_observation.y = particles[i].y + observations[j].x*sin(particles[i].theta) + observations[j].y*cos(particles[i].theta);
			tmp_observation.id = -1; // mark it as unassociated 
			observationsInMapCoordinates.push_back(tmp_observation);
			// I now have the list of observations transformed to the map coordinates with origin the particle
		} // end of observations

		// step 3: associate the inRange Landmarks with the observations
		dataAssociation(landmarksInRange, observationsInMapCoordinates);
		// now every observation has an associated landmark id

		// step 4: calculate the weight for the particle
		double weight = 1.0;
		for (unsigned int j = 0; j < observationsInMapCoordinates.size(); j++){
			if (observationsInMapCoordinates[j].id != -1) { // assigned observation to an in range landmark
				int id = observationsInMapCoordinates[j].id; // retrieve the associated nearest landmark in range
				weight *= bivariate_normal(observationsInMapCoordinates[j].x, observationsInMapCoordinates[j].y, landmarksInRange[id].x, landmarksInRange[id].y, std_landmark[0], std_landmark[1]);
			}
			else {
				weight = 0.0; // unassociated landmark so impropable to be the correct particle.
				break;
			}
		}
		particles[i].weight = weight;
		weights.push_back(weight);
	} // next particle
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<> weightedDistribution(weights.cbegin(), weights.cend());

	vector<Particle> resampled_particles;
	resampled_particles.clear();

	for (int i = 0; i < num_particles; i++) {
		int randomPick = weightedDistribution(gen);
		resampled_particles.push_back(particles[randomPick]);
	}

	// particles = std::move(resampled_particles);
	particles = resampled_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
