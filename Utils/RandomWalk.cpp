#include <iostream>
#include <stdexcept>
#include "Utils/RandomWalk.hpp"

namespace Leph
{
RandomWalk::RandomWalk(size_t dim)
  : _dim(dim)
  , _statePos(Eigen::VectorXd::Zero(dim))
  , _stateVelNormalized(Eigen::VectorXd::Zero(dim))
  , _stateVel(Eigen::VectorXd::Zero(dim))
  , _generator(std::random_device{}())
{
}
RandomWalk::RandomWalk(const Eigen::VectorXd& initState)
  : _dim(initState.size())
  , _statePos(initState)
  , _stateVelNormalized(Eigen::VectorXd::Zero(_dim))
  , _stateVel(Eigen::VectorXd::Zero(_dim))
  , _generator()
{
}

const Eigen::VectorXd& RandomWalk::step(double deltaMean, double inertiaRatio)
{
  if (inertiaRatio > 1.0 || inertiaRatio < 0.0)
  {
    throw std::logic_error("RandomWalk invalid inertiaRatio");
  }

  // Compute normalized velocity
  Eigen::VectorXd unit = randomUnit();
  _stateVelNormalized = inertiaRatio * _stateVelNormalized + (1.0 - inertiaRatio) * unit;
  // Scale velocity to asked value
  _stateVel = deltaMean * _stateVelNormalized;
  // Position integration
  _statePos += _stateVel;

  return _statePos;
}

const Eigen::VectorXd& RandomWalk::uniformStepWithBounds(const Eigen::VectorXd& deltaLow,
                                                         const Eigen::VectorXd& deltaUp,
                                                         const Eigen::VectorXd& stateLow,
                                                         const Eigen::VectorXd& stateUp, double inertiaRatio)
{
  size_t size = _statePos.size();
  // Check inputs
  if (inertiaRatio > 1.0 || inertiaRatio < 0.0)
  {
    throw std::logic_error("RandomWalk invalid inertiaRatio");
  }
  if ((size_t)deltaLow.size() != size || (size_t)deltaUp.size() != size || (size_t)stateLow.size() != size ||
      (size_t)stateUp.size() != size)
  {
    throw std::logic_error("RandomWalk invalid input size");
  }

  // Compute uniform random delta
  // insode given bounds
  Eigen::VectorXd delta = Eigen::VectorXd::Zero(size);
  for (size_t i = 0; i < size; i++)
  {
    std::uniform_real_distribution<> dist(deltaLow(i), deltaUp(i));
    delta(i) = dist(_generator);
  }

  // Compute velocity inertia
  _stateVel = inertiaRatio * _stateVel + (1.0 - inertiaRatio) * delta;

  // Position integration
  _statePos += _stateVel;

  // Position bounds
  for (size_t i = 0; i < size; i++)
  {
    if (_statePos(i) < stateLow(i))
    {
      _statePos(i) = stateLow(i);
    }
    if (_statePos(i) > stateUp(i))
    {
      _statePos(i) = stateUp(i);
    }
  }

  return _statePos;
}

const Eigen::VectorXd& RandomWalk::statePos() const
{
  return _statePos;
}
Eigen::VectorXd& RandomWalk::statePos()
{
  return _statePos;
}
const Eigen::VectorXd& RandomWalk::stateVel() const
{
  return _stateVel;
}
Eigen::VectorXd& RandomWalk::stateVel()
{
  return _stateVel;
}

Eigen::VectorXd RandomWalk::randomUnit()
{
  Eigen::VectorXd unit(_dim);

  std::normal_distribution<double> normalRand(0.0, 1.25);
  for (size_t i = 0; i < _dim; i++)
  {
    unit(i) = normalRand(_generator);
  }

  return unit;
}

}  // namespace Leph
