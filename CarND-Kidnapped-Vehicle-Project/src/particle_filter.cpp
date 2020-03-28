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
#include <map>

#include "helper_functions.h"

using std::normal_distribution;
using std::string;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::vector;
static std::default_random_engine gen;

/* Some helper data */
struct single_landmark_s
{
  int id_i;  // Landmark ID
  float x_f; // Landmark x-position in the map (global coordinates)
  float y_f; // Landmark y-position in the map (global coordinates)
} single_landmark;

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y)
{
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2))) + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
  //std::cout << "w " << weight;
  if (weight <= 0.0)
  {
    weight = 0.000001;
  }

  return weight;
}
/* End of helper data */

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  if (is_initialized)
    return;
  num_particles = 20; // TODO: Set the number of particles

  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  for (int i = 0; i < num_particles; ++i)
  {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  // Make standard deviation of predict, this will be added to predict result
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; ++i)
  {
    if (fabs(yaw_rate) < 0.000001)
    {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    else
    {
      particles[i].x = particles[i].x + (velocity / yaw_rate) *
                                            (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y = particles[i].y + (velocity / yaw_rate) *
                                            (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));

      particles[i].theta = particles[i].theta + yaw_rate * delta_t;
    }
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

// Note by Sreesankar R: Implmenting this function was resulting in performance impact in my laptop. Moving the
// funtionality within updateWeights(), reduced the number of loops and helped in meeting the 100 secs.

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs> &observations)
{
  // predicted : landmarks that is in range of radar/ridar sensor
  // observations : all sensor data transformed by particles

  /**
   1. Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
   2. This function will be used in updateWeights
   **/
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{

  for (int j = 0; j < num_particles; ++j)
  {
    std::vector<LandmarkObs> obs_in_map;
    std::vector<single_landmark_s> landmarks_within_range;
    //std::cout << "Particle j " << j << " x " << particles[j].x << " y " << particles[j].y << " theta "  << particles[j].theta << std::endl;

    for (unsigned int i = 0; i < map_landmarks.landmark_list.size(); ++i)
    {

      // Filter landmarks based on sensor range
      double distance = dist(map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f, particles[j].x, particles[j].y);
      if (distance < sensor_range)
      {
        single_landmark.id_i = map_landmarks.landmark_list[i].id_i;
        single_landmark.x_f = map_landmarks.landmark_list[i].x_f;
        single_landmark.y_f = map_landmarks.landmark_list[i].y_f;
        landmarks_within_range.push_back(single_landmark);
        //std::cout << "Sensor " << map_landmarks.landmark_list[i].id_i << " " << map_landmarks.landmark_list[i].x_f << " " << map_landmarks.landmark_list[i].y_f << std::endl;
      }
    }
    //std::cout << "land marks within range " << landmarks_within_range.landmark_list.size() << std::endl;
    double weight = 1.0;
    for (unsigned int i = 0; i < observations.size(); ++i)
    {
      // Convert observations to map co-ordinates
      LandmarkObs lm_obs_map;
      lm_obs_map.x = particles[j].x + (cos(particles[j].theta) * observations[i].x) - (sin(particles[j].theta) * observations[i].y);
      lm_obs_map.y = particles[j].y + (sin(particles[j].theta) * observations[i].x) + (cos(particles[j].theta) * observations[i].y);

      // Find the landmark nearest to the observation
      double min_distance = std::numeric_limits<double>::max();
      int map_id = 0;

      LandmarkObs sel_landmark;
      for (auto iter = landmarks_within_range.begin();
           iter != landmarks_within_range.end(); ++iter)
      {
        double distance = dist(iter->x_f, iter->y_f, lm_obs_map.x, lm_obs_map.y);
        if (distance < min_distance)
        {
          min_distance = distance;
          map_id = iter->id_i;
          sel_landmark.x = iter->x_f;
          sel_landmark.y = iter->y_f;
          //std::cout << "id " << map_id << "min " << min_distance << " x1   " << iter->x_f << " y1 " << iter->y_f << " x2 " <<
          // observations[i].x << " y2 " <<  observations[i].y << std::endl;
        }
      }
      //std::cout << "sel landmark " << map_id << std::endl;
      //std::cout << "Transformed x " << lm_obs_map.x << " y " << lm_obs_map.y << std::endl;

      lm_obs_map.id = map_id;
      // Calculate the guassian prob
      //if (min_distance <= sensor_range)
      obs_in_map.push_back(lm_obs_map);

      double weight1 = multiv_prob(std_landmark[0], std_landmark[1], lm_obs_map.x, lm_obs_map.y, sel_landmark.x, sel_landmark.y);
      weight *= weight1;
      //std::cout << "1 - std x " << std_landmark[0] << " std y " << std_landmark[1] << "x 1 " << lm_obs_map.x << "y 1 " <<
      //lm_obs_map.y << "x 2 " << sel_landmark.x << " y 2 " << sel_landmark.y << " wt " << weight1 << std::endl;
    }
    particles[j].weight = weight;
  }
}

void ParticleFilter::resample()
{
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  vector<Particle> new_particles;

  // get all of the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++)
  {
    weights.push_back(particles[i].weight);
  }

  // generate random starting index for resampling wheel
  uniform_int_distribution<int> uniintdist(0, num_particles - 1);
  auto index = uniintdist(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // spin the resample wheel!
  for (int i = 0; i < num_particles; i++)
  {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index])
    {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}