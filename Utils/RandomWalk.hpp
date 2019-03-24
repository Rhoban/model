#ifndef LEPH_RANDOMWALK_HPP
#define LEPH_RANDOMWALK_HPP

#include <Eigen/Dense>
#include <random>

namespace Leph
{
/**
 * RandomWalk
 *
 * Implement simple fully random
 * exploration of multi-dimentional space
 */
class RandomWalk
{
public:
  /**
   * Initialization with space dimention
   * (zero init position) or initial
   * state position
   */
  RandomWalk(size_t dim);
  RandomWalk(const Eigen::VectorXd& initState);

  /**
   * Update internal state to random exploration
   * and return new state position.
   * deltaMean: average delta position in case of
   * near nul inertiaRatio. Real delta average is reduce
   * in case of near 1.0 inertiaRatio.
   * InertiaRatio is between 0.0 and 1.0. 0.0 is
   * fully random walk and 1.0 is fully inertia, no
   * random.
   */
  const Eigen::VectorXd& step(double deltaMean, double inertiaRatio);

  /**
   * State update with uniformly random delta
   * between detlaLow and deltaUp bounds.
   * State is bounds between stateLow and stateUp.
   * InertiaRatio is between 0.0 and 1.0.
   * 0.0 is fully random walk and 1.0 is full inertia.
   * New state is returned.
   */
  const Eigen::VectorXd& uniformStepWithBounds(const Eigen::VectorXd& deltaLow, const Eigen::VectorXd& deltaUp,
                                               const Eigen::VectorXd& stateLow, const Eigen::VectorXd& stateUp,
                                               double inertiaRatio);

  /**
   * Access to internal state and
   * velocity
   */
  const Eigen::VectorXd& statePos() const;
  Eigen::VectorXd& statePos();
  const Eigen::VectorXd& stateVel() const;
  Eigen::VectorXd& stateVel();

private:
  /**
   * Space dimention
   */
  size_t _dim;

  /**
   * State position in space
   */
  Eigen::VectorXd _statePos;

  /**
   * State velocity normalized and
   * actual velocity (scale to asked norm)
   */
  Eigen::VectorXd _stateVelNormalized;
  Eigen::VectorXd _stateVel;

  /**
   * Random generator
   */
  std::mt19937 _generator;

  /**
   * Build random normal unit vector of
   * current dimention with mean
   * fabs value equal to 1.0
   */
  Eigen::VectorXd randomUnit();
};

}  // namespace Leph

#endif
