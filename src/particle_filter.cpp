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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle_filter.h"

using namespace std;

ParticleFilter::ParticleFilter() {
  num_particles = 500;
  is_initialized = false;
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  for (int i=0; i<num_particles; ++i) {
    Particle particle_temp;
    particle_temp.id = i;
    particle_temp.x = dist_x(gen);
    particle_temp.y = dist_y(gen);
    particle_temp.theta = dist_theta(gen);
    particle_temp.weight = 1.0;
    particles.push_back(particle_temp);
    weights.push_back(particle_temp.weight);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  double theta_0;
  double eps = 0.00001;
  double use_yaw_rate;
  if (yaw_rate>=0) {
    use_yaw_rate = max(eps, yaw_rate);
  } else {
    use_yaw_rate = min(-eps, yaw_rate);
  }
  default_random_engine gen;
  
  for (int i=0; i<particles.size(); ++i) {
    theta_0 = particles[i].theta;
    particles[i].x += velocity/use_yaw_rate*(sin(theta_0 + yaw_rate*delta_t) - sin(theta_0));
    particles[i].y += velocity/use_yaw_rate*(cos(theta_0) -  cos(theta_0 + yaw_rate*delta_t));
    particles[i].theta += yaw_rate*delta_t;
    normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
    normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
    normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
//}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  
  double obs_map_x;
  double obs_map_y;
  int nearest_landmark;
  double distance;
  double nearest_distance;
  double nearest_x = sensor_range;
  double nearest_y;
  double gauss_norm = (1/(2*M_PI*std_landmark[0]*std_landmark[1]));
  double num_x;
  double den_x = 2*std_landmark[0]*std_landmark[0];
  double num_y;
  double den_y = 2*std_landmark[1]*std_landmark[1];
  double P_;
  
  for (int i=0; i<particles.size(); ++i) {
    P_ = 1;
    particles[i].sense_x.clear();
    particles[i].sense_y.clear();
    particles[i].associations.clear();
    
    for (int j=0; j<observations.size(); ++j) {
      obs_map_x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      obs_map_y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
      
      for (int k=0; k<map_landmarks.landmark_list.size(); ++k) {
        distance = dist(obs_map_x, obs_map_y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
        if (k==0) {
          nearest_distance = distance;
          nearest_landmark = map_landmarks.landmark_list[k].id_i;
          nearest_x = map_landmarks.landmark_list[k].x_f;
          nearest_y = map_landmarks.landmark_list[k].y_f;
        }
        else if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest_landmark = map_landmarks.landmark_list[k].id_i;
          nearest_x = map_landmarks.landmark_list[k].x_f;
          nearest_y = map_landmarks.landmark_list[k].y_f;
        }
      }
      particles[i].sense_x.push_back(nearest_x);
      particles[i].sense_y.push_back(nearest_y);
      particles[i].associations.push_back(nearest_landmark);
      
      
      num_x = (obs_map_x - nearest_x)*(obs_map_x - nearest_x);
      num_y = (obs_map_y - nearest_y)*(obs_map_y - nearest_y);
      P_ *= gauss_norm * exp(-(num_x/den_x + num_y/den_y));
    }
    particles[i].weight = P_;
    weights[i] = P_;
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  default_random_engine gen;
  discrete_distribution<int> d(weights.begin(), weights.end());
  vector<Particle> new_particles;
  int index;
  for (int i=0; i<particles.size(); ++i) {
    index = d(gen);
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
}

//Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
//                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
//{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

//    particle.associations= associations;
//    particle.sense_x = sense_x;
//    particle.sense_y = sense_y;
//  return particle;
//}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
