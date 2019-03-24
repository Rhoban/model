#include <stdexcept>
#include "Utils/RandomVelocitySpline.hpp"

namespace Leph
{
RandomVelocitySpline::RandomVelocitySpline(const Eigen::VectorXd& state, const Eigen::VectorXd& minBound,
                                           const Eigen::VectorXd& maxBound, double minTime, double maxTime)
  : _generator(), _state(state), _minBound(minBound), _maxBound(maxBound), _minTime(minTime), _maxTime(maxTime)
{
  if (_state.size() != _minBound.size() || _state.size() != _maxBound.size())
  {
    throw std::logic_error("RandomVelocitySpline mismatch dimension");
  }
  for (size_t i = 0; i < (size_t)_state.size(); i++)
  {
    _trajs.push_back(SmoothSpline());
  }

  newRandomTarget();
}

const Eigen::VectorXd& RandomVelocitySpline::step(double dt)
{
  // Update playing phase
  _t += dt;
  // Create a new trajectory if the
  // current one is over
  if (_t > _trajs.front().max())
  {
    newRandomTarget();
  }

  // Evaluate current state position from splines
  for (size_t i = 0; i < (size_t)_state.size(); i++)
  {
    _state(i) = _trajs[i].pos(_t);
  }

  // Return current state
  return _state;
}

const Eigen::VectorXd& RandomVelocitySpline::state() const
{
  return _state;
}

void RandomVelocitySpline::newRandomTarget()
{
  // New random target within bounds
  Eigen::VectorXd target = _state;
  for (size_t i = 0; i < (size_t)_state.size(); i++)
  {
    std::uniform_real_distribution<double> dist(_minBound(i), _maxBound(i));
    target(i) = dist(_generator);
  }

  // Random time duration
  std::uniform_real_distribution<double> dist(_minTime, _maxTime);
  double time = dist(_generator);

  // Build new trajectory
  _t = 0.0;
  for (size_t i = 0; i < (size_t)_state.size(); i++)
  {
    _trajs[i] = SmoothSpline();
    _trajs[i].addPoint(0.0, _state(i));
    _trajs[i].addPoint(time, target(i));
  }
}

}  // namespace Leph
