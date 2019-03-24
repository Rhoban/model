#ifndef LEPH_RANDOMVELOCITYSPLINE_HPP
#define LEPH_RANDOMVELOCITYSPLINE_HPP

#include <vector>
#include <random>
#include "Spline/SmoothSpline.hpp"

namespace Leph
{
/**
 * RandomVelocitySpline
 *
 * Simple random exploration of a
 * multi-dimensional box with spline
 * bell shape trajectory
 */
class RandomVelocitySpline
{
public:
  /**
   * Initialization with initial
   * state and min/max box range
   * and maximum allowed velocity
   */
  RandomVelocitySpline(const Eigen::VectorXd& state, const Eigen::VectorXd& minBound, const Eigen::VectorXd& maxBound,
                       double minTime, double maxTime);

  /**
   * Update playing random trajectories
   * by given delta time step and return
   * current state position.
   */
  const Eigen::VectorXd& step(double dt);

  /**
   * Return current state position
   */
  const Eigen::VectorXd& state() const;

private:
  /**
   * Random generator
   */
  std::mt19937 _generator;

  /**
   * Trajectory playing afixe
   */
  double _t;

  /**
   * Current trajectory
   * for each dimension
   */
  std::vector<SmoothSpline> _trajs;

  /**
   * Current state position, min/max
   * bounds and time bound
   */
  Eigen::VectorXd _state;
  Eigen::VectorXd _minBound;
  Eigen::VectorXd _maxBound;
  double _minTime;
  double _maxTime;

  /**
   * Generate a new random target
   * and set up trajectory
   */
  void newRandomTarget();
};

}  // namespace Leph

#endif
