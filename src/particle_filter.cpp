/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 50;  // TODO: Set the number of particles

  std::default_random_engine gen;

  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    // Initialize particle
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;
    // Add particle
    particles.push_back(p);
  }

//  for (int i = 0; i < num_particles; ++i) {
//    std::cout << "Particle " << particles[i].id << ": "
//               << particles[i].x << ", " << particles[i].y << "\n";
//  }
  ParticleFilter::is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;

  double x_p;
  double y_p;
  double theta_p;
  double theta_0;
  double theta_f;
  for (unsigned int i = 0; i < particles.size(); ++i) {
    // apply Gaussian noise to particle position
    normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
    normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
    normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);
    x_p = dist_x(gen);
    y_p = dist_y(gen);
    theta_p = dist_theta(gen);

    if (abs(yaw_rate) > 1e-8){
      theta_0 = theta_p;
      theta_f = theta_0 + yaw_rate * delta_t;

      particles[i].x = x_p + (velocity / yaw_rate) * (sin(theta_f) - sin(theta_0));
      particles[i].y = y_p + (velocity / yaw_rate) * (cos(theta_0) - cos(theta_f));
      particles[i].theta = theta_f;
    }
    else {    // model for small yaw rate to avoid dividing by 0
      theta_0 = theta_p;
      particles[i].x = x_p + velocity * delta_t * cos(theta_0);
      particles[i].y = y_p + velocity * delta_t * sin(theta_0);
      particles[i].theta = theta_0 + yaw_rate * delta_t;
    }
  }

//  for (unsigned int i = 0; i < particles.size(); ++i) {
//    std::cout << "Particle " << particles[i].id << ": "
//               << particles[i].x << ", " << particles[i].y << "\n";
//  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

  double distance;
  double min_dist;
  int nearest_landmark;
  for (unsigned int i = 0; i < observations.size(); ++i) {
    min_dist = 1e10;
    nearest_landmark = -1;
    for (unsigned int j = 0; j < predicted.size(); ++j) {
      distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (distance < min_dist) {
        min_dist = distance;
        nearest_landmark = j;
      }
    }
    observations[i].id = nearest_landmark;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double weights_sum = 0;

  for (unsigned int i=0; i < particles.size(); ++i) {
    vector<LandmarkObs> observations_mc;   // observations in map coordinates
    for (unsigned int j=0; j < observations.size(); ++j) {
      LandmarkObs obs_mc;
      obs_mc.id = observations[j].id;
      obs_mc.x = cos(particles[i].theta) * observations[j].x -
                 sin(particles[i].theta) * observations[j].y +
                 particles[i].x;
      obs_mc.y = sin(particles[i].theta) * observations[j].x +
                 cos(particles[i].theta) * observations[j].y +
                 particles[i].y;
      observations_mc.push_back(obs_mc);
    }
    // Convert map_landmarks.landmark_list to vector<LandmarkObs>
    vector<LandmarkObs> landmarks;
    for (unsigned int i=0; i<map_landmarks.landmark_list.size(); ++i) {
      LandmarkObs landmark;
      landmark.id = map_landmarks.landmark_list[i].id_i;
      landmark.x = map_landmarks.landmark_list[i].x_f;
      landmark.y = map_landmarks.landmark_list[i].y_f;
      landmarks.push_back(landmark);
    }
    // Associate the closest landmark to each observation
    dataAssociation(landmarks, observations_mc);
    //SetAssociations(particles[i], observations_mc[:].id, observations_mc.x, observations_mc.y);
    double obs_prob;
    double total_prob = 1;
    double coeff = 1/ (2 * M_PI * std_landmark[0] * std_landmark[1]);
    double mu_x, mu_y, exponent;
    double max_prob = 0;
    for (unsigned int j=0; j < observations_mc.size(); ++j) {
      mu_x = landmarks[observations_mc[j].id].x;
      mu_y = landmarks[observations_mc[j].id].y;
      exponent = -((pow(observations_mc[j].x - mu_x, 2) / (2 * pow(std_landmark[0], 2))) +
                   (pow(observations_mc[j].y - mu_y, 2) / (2 * pow(std_landmark[1], 2))) );
      obs_prob = coeff * exp(exponent);
      if (obs_prob > max_prob){
        max_prob = obs_prob;
      }
      if (obs_prob < 1e-4){
        obs_prob = 1e-4;
      }

      total_prob *= obs_prob;
    }
    particles[i].weight = total_prob;
    weights_sum += particles[i].weight;
  }
  // Normalize
  double final_prob = 0;
  for (unsigned int i=0; i < particles.size(); ++i) {
    particles[i].weight /= weights_sum;
    //std::cout << particles[i].weight << "\n";
    final_prob += particles[i].weight;
  }
  //std::cout << weights_sum << "\n\n";
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	vector<Particle> new_particles; // new particle array
	vector<double> weights;  // vector of weights, indexed by particle index

	for (int i=0; i < particles.size(); i++) {
		weights.push_back(particles[i].weight);
	}

	std::default_random_engine gen;  // RNG
	std::discrete_distribution<int> dist_weights(weights.begin(), weights.end());  // discrete distribution based on particle weights

	// Sample from discrete distribution
	for (int i=0; i < particles.size(); i++) {
		int random_idx = dist_weights(gen);
		//std::cout << random_idx << ", ";
		new_particles.push_back(particles[random_idx]);
	}
  //std::cout << "\n";
	// Replace original particle vector
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
