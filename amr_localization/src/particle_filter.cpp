#define _USE_MATH_DEFINES
#include <cmath>
#include <random>

#include "particle_filter.h"

ParticleFilter::ParticleFilter(double map_min_x, double map_max_x, double map_min_y, double map_max_y, ComputeParticleWeightCallback callback)
: callback_(callback)
, motion_model_(0.02, 0.01)
, random_particle_generator_(map_min_x, map_max_x, map_min_y, map_max_y)
{
}

void ParticleFilter::update(double x, double y, double yaw)
{
  //============================== YOUR CODE HERE ==============================
  // Instructions: do one complete update of the particle filter. It should
  //               generate new particle set based on the given motion
  //               parameters and the old particle set, compute the weights of
  //               the particles, resample the set, and update the private
  //               member field that holds the current best pose estimate
  //               (pose_estimate_). Note that the motion parameters provided
  //               to this function are in robot's reference frame.
  //
  // Remark: to compute the weight of a particle, use the callback_ member
  //         field:
  //
  //             Particle particle;
  //             double weight = callback_(particle);
  //
  // Remark: to generate a comletely random particle use the provided
  //         random_particle_generator_ object:
  //
  //             Particle particle = random_particle_generator_.generateParticle();
  //
  // Hint: to sample a uniformly distributed number in [min, max] range use:
  //
  //           double r = std::uniform_real_distribution<double>(min, max)(random_generator_);
  //


  //============================================================================
}

