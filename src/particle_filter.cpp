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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // noise arrays with gps measurement uncertainty
    std::normal_distribution<double> N_x_init(0, std[0]);
    std::normal_distribution<double> N_y_init(0, std[1]);
    std::normal_distribution<double> N_theta_init(0, std[2]);
    std::default_random_engine rnd;  //random number

    // generate #num_particles particles and init them, store in particles vector
    for (int i=0; i<num_particles; ++i){
        Particle p;  // init Particle p
        p.id = i;
        p.weight = 1;
        p.x = x+N_x_init(rnd); // add random noise to x, y, theta
        p.y = y+N_y_init(rnd);
        p.theta= theta+N_theta_init(rnd);
        particles.push_back(p); // add particle to particles
    }
    is_initialized = true;  // set initialization flag
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // noise arrays
    std::normal_distribution<double> N_x(0, std_pos[0]);
    std::normal_distribution<double> N_y(0, std_pos[1]);
    std::normal_distribution<double> N_theta(0, std_pos[2]);
    std::default_random_engine rnd;  //random number

    // update location based on velocity and yaw_rate, add normally distributed gaussian noise with 0 mean and std_pos
    for (auto p:particles){ // iterate over particles
        if(fabs(yaw_rate)<0.001){ // for zero yaw rate
            p.x += velocity*cos(p.theta)*delta_t + N_x(rnd);
            p.y += velocity*sin(p.theta)*delta_t + N_y(rnd);
            p.theta += N_theta(rnd);
        }
        else { // standard formula
            p.x += velocity/yaw_rate*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta)) + N_x(rnd);
            p.y += velocity/yaw_rate*(-1*cos(p.theta+yaw_rate*delta_t)+cos(p.theta)) + N_y(rnd);
            p.theta += yaw_rate*delta_t + N_theta(rnd);
        }
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


    for (auto obs:observations){
        int min_id = 0;
        double min_dist = 1000;

        for (auto lm:predicted){
            double distance=dist(lm.x, lm.y, obs.x, obs.y);
            if (distance < min_dist){
                min_id = lm.id;
                min_dist = distance;
            }
        obs.id = min_id;
        }
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution.

    /*
     * 1. Make list of all landmarks within sensor range of particle, call this `predicted_lm`
        2. Convert all observations from local to global frame, call this `global_obs`
        3. Perform `dataAssociation`. This will put the index of the `predicted_lm` nearest to each
        `global_obs` in the `id` field of the `global_obs` element.
        4. Loop through all the `global_obs`. Use the saved index in the `id` to find the associated
        landmark and compute the gaussian.
        5. Multiply all the gaussian values together to get total probability of particle (the weight).
     *
     */


    for (auto p:particles){

        // get landmarks in sensor range
        std::vector<LandmarkObs> pred_lm;
        for (auto lm:map_landmarks.landmark_list){
            if ((p.x-sensor_range < lm.x_f && lm.x_f < p.x+sensor_range) &&
                    (p.y-sensor_range < lm.y_f && lm.y_f < p.y+sensor_range)){
                LandmarkObs plm;
                plm.x = lm.x_f;
                plm.y = lm.y_f;
                plm.id = lm.id_i;
                pred_lm.push_back(plm);
            }
        }

        // translate observation coords in global coords
        std::vector<LandmarkObs> global_obs;
        for (auto obs:observations){
            LandmarkObs glob_obs;
            glob_obs.id = obs.id;
            glob_obs.x = obs.x*cos(p.theta)-obs.y*sin(p.theta)+p.x;
            glob_obs.y = obs.x*sin(p.theta)+obs.y*cos(p.theta)+p.y;
            global_obs.push_back(glob_obs);
        }

        // data association
        dataAssociation(pred_lm, global_obs);

        // calc weights
        double w = 1.0;
        double scale = 1.0/(2*M_PI*std_landmark[0]*std_landmark[1]);
        double two_sig_x_sq = 2*std_landmark[0]*std_landmark[0];
        double two_sig_y_sq = 2*std_landmark[1]*std_landmark[1];

        for (auto obs:global_obs){ // iterate over observations

            for (auto lm:pred_lm){  // iterate over pred landmarks

                if (lm.id == obs.id){ // if landmark is the associated landmark

                    w *= scale*exp(-1.0*( (pow(obs.x-lm.x ,2) / two_sig_x_sq) + (pow(obs.y-lm.y ,2) / two_sig_y_sq) ));
                }
            }
        }
        p.weight = w; // assign weigth to particle
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    std::vector<Particle> resampled_particles;
    resampled_particles.clear();

    std::default_random_engine rnd;  //random number generator
    // get weights vector for discrete distribution
    std::vector<double> weights;
    weights.clear();
    for (auto p:particles){
        weights.push_back(p.weight);
    }

    std::discrete_distribution<> distri (weights.begin(), weights.end());

    for (int i=0; i<num_particles; ++i){
        resampled_particles.push_back(particles[distri(rnd)]);
    }

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
