#ifndef LEPH_ITERATIVELEARNINGCONTROL_HPP
#define LEPH_ITERATIVELEARNINGCONTROL_HPP

#include <deque>
#include <vector>
#include "Types/VectorLabel.hpp"
#include "Spline/LinearSpline.hpp"

namespace Leph
{
/**
 * IterativeLearningControl
 *
 * Simple class implementing Iterative
 * Learning Control method in order to
 * compensate for motors errors
 */
class IterativeLearningControl
{
public:
  /**
   * Initialization with
   * the number of lag tick and learning rate
   */
  IterativeLearningControl(size_t log, double learningRate);

  /**
   * Return offset to add on all degrees of
   * freedom at given phase
   */
  VectorLabel getOffsets(double phase) const;

  /**
   * Return the summed error of last cycle
   * iteration and last update error
   */
  const VectorLabel& getSumErrors() const;
  const VectorLabel& getLastErrors() const;

  /**
   * Update the compensation spline. Current phase,
   * order send to the motor and current motor
   * read positions are given.
   */
  void update(double phase, const VectorLabel& goals, const VectorLabel& motors);

  /**
   * Direct access to convergence learning rate
   */
  const double& learningRate() const;
  double& learningRate();

private:
  /**
   * Iteration learning rate
   */
  double _learningRate;

  /**
   * The number of tick in the past
   * to consider
   */
  size_t _lag;

  /**
   * History of motor reference to compute
   * error with lag
   */
  std::deque<VectorLabel> _container;

  /**
   * Current cycle and next cycle
   * position offset for each degrees of freedom
   */
  std::vector<LinearSpline> _currentOffsets;
  std::vector<LinearSpline> _nextOffsets;

  /**
   * Error summed on last and current cycle
   */
  VectorLabel _pastSumErrors;
  VectorLabel _currentSumErrors;

  /**
   * Last update error
   */
  VectorLabel _lastErrors;
};

}  // namespace Leph

#endif
