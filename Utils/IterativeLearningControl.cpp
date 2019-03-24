#include <stdexcept>
#include "Utils/IterativeLearningControl.hpp"

namespace Leph
{
IterativeLearningControl::IterativeLearningControl(size_t lag, double learningRate)
  : _learningRate(learningRate)
  , _lag(lag)
  , _container()
  , _currentOffsets()
  , _nextOffsets()
  , _pastSumErrors()
  , _currentSumErrors()
  , _lastErrors()
{
}

VectorLabel IterativeLearningControl::getOffsets(double phase) const
{
  if (_container.size() == 0 && _currentOffsets.size() == 0)
  {
    return Leph::VectorLabel();
  }

  VectorLabel offsets = _container.back();
  for (size_t i = 0; i < offsets.size(); i++)
  {
    offsets(i) = _currentOffsets[i].pos(phase);
  }

  return offsets;
}

const VectorLabel& IterativeLearningControl::getSumErrors() const
{
  return _pastSumErrors;
}
const VectorLabel& IterativeLearningControl::getLastErrors() const
{
  return _lastErrors;
}

void IterativeLearningControl::update(double phase, const VectorLabel& goals, const VectorLabel& motors)
{
  // Check size
  if (motors.size() != goals.size())
  {
    throw std::logic_error("IterativeLearningControl invalid size");
  }
  // Initialize splines container
  if (_currentOffsets.size() == 0)
  {
    _pastSumErrors = motors;
    _pastSumErrors.zeroOp();
    _currentSumErrors = motors;
    _currentSumErrors.zeroOp();
    for (size_t i = 0; i < goals.size(); i++)
    {
      _currentOffsets.push_back(LinearSpline());
      _nextOffsets.push_back(LinearSpline());
      _currentOffsets.back().addPoint(0.0, 0.0);
      _currentOffsets.back().addPoint(1.0, 0.0);
    }
  }
  else if (_currentOffsets.size() != goals.size())
  {
    throw std::logic_error("IterativeLearningControl invalid size");
  }

  // Store motors goals positions
  _container.push_back(goals);
  while (_container.size() > _lag)
  {
    _container.pop_front();
  }

  // No learning if the history is not completed
  if (_container.size() != _lag)
  {
    return;
  }

  // Swap splines on phase cycling
  if (phase < _nextOffsets.front().max())
  {
    _currentOffsets = _nextOffsets;
    for (size_t i = 0; i < _nextOffsets.size(); i++)
    {
      _nextOffsets[i] = LinearSpline();
    }
    _pastSumErrors = _currentSumErrors;
    _currentSumErrors.zeroOp();
  }

  // Compute motors error
  _lastErrors = motors;
  _lastErrors.subOp(_container.front());
  for (size_t i = 0; i < _lastErrors.size(); i++)
  {
    _currentSumErrors(i) += fabs(_lastErrors(i));
  }
  Leph::VectorLabel deltas = _lastErrors;
  deltas.mulOp(-_learningRate);
  deltas.addOp(getOffsets(phase));
  for (size_t i = 0; i < deltas.size(); i++)
  {
    _nextOffsets[i].addPoint(phase, deltas(i));
  }
}

const double& IterativeLearningControl::learningRate() const
{
  return _learningRate;
}
double& IterativeLearningControl::learningRate()
{
  return _learningRate;
}

}  // namespace Leph
