#ifndef RANDOM_PARTICLE_GENERATOR_H
#define RANDOM_PARTICLE_GENERATOR_H

#include <random>

#include "particle.h"

/** This class generates particles with random poses.
  *
  * By default the poses are drawn from a uniform distribution across the
  * rectangle given in the constructor. Optinally a bias towards a particular
  * pose may be introduced for a limited time (see @ref setBias()). */
class RandomParticleGenerator
{

public:

  RandomParticleGenerator(double min_x, double max_x, double min_y, double max_y)
  : uniform_x_(min_x, max_x)
  , uniform_y_(min_y, max_y)
  , uniform_theta_(-M_PI, M_PI)
  , biased_particles_(0)
  { }

  /** Generate a random particle. */
  Particle generateParticle()
  {
    Particle p;
    if (biased_particles_-- > 0)
    {
      p.pose.x = normal_x_(random_generator_);
      p.pose.y = normal_y_(random_generator_);
      p.pose.theta = normal_theta_(random_generator_);
    }
    else
    {
      p.pose.x = uniform_x_(random_generator_);
      p.pose.y = uniform_y_(random_generator_);
      p.pose.theta = uniform_theta_(random_generator_);
    }
    p.weight = 0.0;
    return p;
  }

  /** Introduce a bias towards a certain point.
    *
    * Here by bias we mean that the poses will be drawn from a normal
    * distribution around the bias pose.
    *
    * @param pose : a pose around which random samples will be drawn.
    *
    * @param std : standard deviation (same for x, y, and theta).
    *
    * @param particle_count : number of particles for which the bias will have
    * effect. After this many particles have been produced, the generator
    * switches to the "uniform" mode. */
  void setBias(Pose pose, double std, int particle_count)
  {
    normal_x_ = std::normal_distribution<double>(pose.x, std);
    normal_y_ = std::normal_distribution<double>(pose.y, std);
    normal_theta_ = std::normal_distribution<double>(pose.theta, std);
    biased_particles_ = particle_count;
  }

private:

  // Required for random number generation
  std::default_random_engine random_generator_;

  // Uniform distributions from which poses for non-biased random particles are sampled
  std::uniform_real_distribution<double> uniform_x_;
  std::uniform_real_distribution<double> uniform_y_;
  std::uniform_real_distribution<double> uniform_theta_;

  // Normal distributions from which poses for biased random particles are sampled
  std::normal_distribution<double> normal_x_;
  std::normal_distribution<double> normal_y_;
  std::normal_distribution<double> normal_theta_;

  Pose bias_pose_;
  double bias_std_;
  int biased_particles_;

};

#endif /* RANDOM_PARTICLE_GENERATOR_H */

