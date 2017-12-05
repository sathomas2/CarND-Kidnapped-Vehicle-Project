/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Edited on: November 30, 2017
 * 		By: Steve Thomas
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

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {  
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
