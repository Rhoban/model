#include <stdexcept>
#include <string>
#include <cmath>
#include "DMP/DMP.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Utils/RungeKutta4.hpp"

namespace Leph
{
DMP::DMP()
  : _dim(0)
  , _kernelNum(0)
  , _coefDamper(0.0)
  , _coefSpring(0.0)
  , _timeLength(0.0)
  , _currentTime(0.0)
  , _state()
  , _lastStateVel()
  , _goalSplines()
  , _kernelCenters()
  , _kernelWidths()
  , _kernelWeights()
{
}

DMP::DMP(unsigned int dim, unsigned int kernelNum, double overlap)
  : _dim(dim)
  , _kernelNum(kernelNum)
  , _overlap(overlap)
  , _coefDamper(80.0)
  , _coefSpring(_coefDamper / 4.0)
  , _timeLength(0.0)
  , _currentTime(0.0)
  , _state(Eigen::VectorXd::Zero(1 + 2 * dim))
  , _lastStateVel(Eigen::VectorXd::Zero(1 + 2 * dim))
  , _goalSplines()
  , _kernelCenters()
  , _kernelWidths()
  , _kernelWeights()
{
  // Check
  if (_overlap < 0.0 || _overlap >= 1.0)
  {
    throw std::logic_error("DMP invalid overlap: " + std::to_string(_overlap));
  }
  // Zero initialization
  for (size_t i = 0; i < kernelNum; i++)
  {
    _kernelCenters.push_back(0.0);
    _kernelWidths.push_back(0.0);
  }
  for (size_t j = 0; j < dim; j++)
  {
    _kernelWeights.push_back(std::vector<double>());
    for (size_t i = 0; i < kernelNum; i++)
    {
      _kernelWeights[j].push_back(0.0);
    }
  }
  // Initialize and pre compute kernel
  // centers and widths
  computeKernels();
}

unsigned int DMP::dimension() const
{
  return _dim;
}
unsigned int DMP::kernelNum() const
{
  return _kernelNum;
}

void DMP::init(double timeLength, const Eigen::VectorXd& startPos, const Eigen::VectorXd& startVel,
               const Eigen::VectorXd& startAcc, const Eigen::VectorXd& endPos, const Eigen::VectorXd& endVel,
               const Eigen::VectorXd& endAcc)
{
  // Dimension check
  if (startPos.size() != _dim || startVel.size() != _dim || startAcc.size() != _dim || endPos.size() != _dim ||
      endVel.size() != _dim || endAcc.size() != _dim)
  {
    throw std::logic_error("DMP invalid dimension in init");
  }
  // Time length check
  if (timeLength < 1e-6)
  {
    throw std::logic_error("DMP invalid time length: " + std::to_string(timeLength));
  }

  _timeLength = timeLength;
  _currentTime = 0.0;
  // State initialization
  // Phase
  _state(0) = 0.0;
  // Position
  _state.segment(1, _dim) = startPos;
  // Velocity
  _state.segment(1 + _dim, _dim) = startVel * _timeLength;
  // Initialize differentiation
  _lastStateVel(0) = 0.0;
  _lastStateVel.segment(1, _dim) = startVel;
  _lastStateVel.segment(1 + _dim, _dim) = startAcc * _timeLength;

  // Initialize delayed goal spline
  // in phase time and taking into account
  // the movement time length.
  // Actually, this is not agreed with the literature
  // but it is working whereas the literature method is not.
  _goalSplines.clear();
  for (size_t i = 0; i < _dim; i++)
  {
    _goalSplines.push_back(SmoothSpline());
    _goalSplines[i].addPoint(0.0, startPos(i), _timeLength * startVel(i), _timeLength * _timeLength * startAcc(i));
    _goalSplines[i].addPoint(1.0, endPos(i), _timeLength * endVel(i), _timeLength * _timeLength * endAcc(i));
  }
}

Eigen::VectorXd DMP::statePos() const
{
  return _state.segment(1, _dim);
}
Eigen::VectorXd DMP::stateVel() const
{
  return _lastStateVel.segment(1, _dim);
}
Eigen::VectorXd DMP::stateAcc() const
{
  return (1.0 / _timeLength) * _lastStateVel.segment(1 + _dim, _dim);
}

double DMP::statePhase() const
{
  return _state(0);
}
double DMP::stateGating() const
{
  // Contrary with literature or Stulp,
  // we implement tricube kernel between 0.0 and 1.0 with
  // phase between 0.0 and 1.0. Maximum in 0.5.
  // This allow for acceleration continuity since
  // the forcing term is zero at trajectory start and end.
  return pow(1.0 - pow(fabs(2.0 * (_state(0) - 0.5)), 3), 3);
}

double DMP::currentTime() const
{
  return _currentTime;
}

double DMP::kernelCenter(size_t num) const
{
  if (num >= _kernelNum)
  {
    throw std::logic_error("DMP invalid kernel number: " + std::to_string(num));
  }
  return _kernelCenters[num];
}
double& DMP::kernelCenter(size_t num)
{
  if (num >= _kernelNum)
  {
    throw std::logic_error("DMP invalid kernel number: " + std::to_string(num));
  }
  return _kernelCenters[num];
}
double DMP::kernelWidth(size_t num) const
{
  if (num >= _kernelNum)
  {
    throw std::logic_error("DMP invalid kernel number: " + std::to_string(num));
  }
  return _kernelWidths[num];
}
double& DMP::kernelWidth(size_t num)
{
  if (num >= _kernelNum)
  {
    throw std::logic_error("DMP invalid kernel number: " + std::to_string(num));
  }
  return _kernelWidths[num];
}

double DMP::kernelWeight(size_t dim, size_t num) const
{
  if (num >= _kernelNum)
  {
    throw std::logic_error("DMP invalid kernel number: " + std::to_string(num));
  }
  if (dim >= _dim)
  {
    throw std::logic_error("DMP invalid dimension: " + std::to_string(dim));
  }
  return _kernelWeights[dim][num];
}
double& DMP::kernelWeight(size_t dim, size_t num)
{
  if (num >= _kernelNum)
  {
    throw std::logic_error("DMP invalid kernel number: " + std::to_string(num));
  }
  if (dim >= _dim)
  {
    throw std::logic_error("DMP invalid dimension: " + std::to_string(dim));
  }
  return _kernelWeights[dim][num];
}

void DMP::step(double dt)
{
  // Time length check
  if (_timeLength < 1e-6)
  {
    throw std::logic_error("DMP invalid time length: " + std::to_string(_timeLength));
  }

  // Build up system differential function
  auto transition = [this](const Eigen::VectorXd& state) -> Eigen::VectorXd {
    unsigned int dim = this->_dim;
    double timeLength = this->_timeLength;
    Eigen::VectorXd diff = Eigen::VectorXd::Zero(state.size());
    // Phase constant velocity
    diff(0) = 1.0 / timeLength;
    // Gating term
    double gating = stateGating();
    // Copy velocity
    diff.segment(1, dim) = state.segment(1 + dim, dim) / timeLength;
    // Compute delayed goal
    Eigen::VectorXd goalPos = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd goalVel = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd goalAcc = Eigen::VectorXd::Zero(dim);
    for (size_t i = 0; i < dim; i++)
    {
      goalPos(i) = _goalSplines[i].pos(state(0));
      goalVel(i) = _goalSplines[i].vel(state(0));
      goalAcc(i) = _goalSplines[i].acc(state(0));
    }
    // Compute acceleration
    diff.segment(1 + dim, dim) = (this->_coefDamper * (this->_coefSpring * (goalPos - state.segment(1, dim)) + goalVel -
                                                       state.segment(1 + dim, dim)) +
                                  goalAcc + forcingFunction(state(0), gating)) /
                                 timeLength;
    return diff;
  };

  // Save current differentiation
  _lastStateVel = transition(_state);
  // System integration with Runge Kutta 4 method
  _state = RungeKutta4Integration(_state, dt, transition);
  // Phase constant velocity special case
  if (_state(0) > 1.0)
  {
    _state(0) = 1.0;
  }
  // Real time integration
  _currentTime += dt;
}

Eigen::VectorXd DMP::forcingFunction(double phase, double gating) const
{
  // In literature, the radial basis function are normalized
  // but this cause major numerical issues when a very small
  // number gets divided by another. We are going to try
  // with unnormalized RBF.
  Eigen::VectorXd values = Eigen::VectorXd::Zero(_dim);
  for (size_t i = 0; i < _dim; i++)
  {
    double sumWeighted = 0.0;
    for (size_t j = 0; j < _kernelNum; j++)
    {
      double val = exp(-_kernelWidths[j] * pow(_kernelCenters[j] - phase, 2));
      sumWeighted += val * _kernelWeights[i][j];
    }
    values(i) = gating * sumWeighted;
  }

  return values;
}

Eigen::VectorXd DMP::getParameters(bool weights, bool centers, bool widths) const
{
  // Initialize the vector
  size_t size = 0;
  if (weights)
    size += _dim * _kernelNum;
  if (centers)
    size += _kernelNum;
  if (widths)
    size += _kernelNum;
  Eigen::VectorXd params(size);

  // Assign vector
  size_t index = 0;
  if (weights)
  {
    for (size_t j = 0; j < _dim; j++)
    {
      for (size_t i = 0; i < _kernelNum; i++)
      {
        params(index) = _kernelWeights[j][i];
        index++;
      }
    }
  }
  if (centers)
  {
    for (size_t i = 0; i < _kernelNum; i++)
    {
      params(index) = _kernelCenters[i];
      index++;
    }
  }
  if (widths)
  {
    for (size_t i = 0; i < _kernelNum; i++)
    {
      params(index) = _kernelWidths[i];
      index++;
    }
  }

  return params;
}
void DMP::setParameters(const Eigen::VectorXd& params, bool weights, bool centers, bool widths)
{
  // Check vector size
  size_t size = 0;
  if (weights)
    size += _dim * _kernelNum;
  if (centers)
    size += _kernelNum;
  if (widths)
    size += _kernelNum;
  if ((size_t)params.size() != size)
  {
    throw std::logic_error("DMP setParameters invalid size: " + std::to_string(params.size()));
  }

  // Assign parameters
  size_t index = 0;
  if (weights)
  {
    for (size_t j = 0; j < _dim; j++)
    {
      for (size_t i = 0; i < _kernelNum; i++)
      {
        _kernelWeights[j][i] = params(index);
        index++;
      }
    }
  }
  if (centers)
  {
    for (size_t i = 0; i < _kernelNum; i++)
    {
      if (params(index) < 0.0)
      {
        _kernelCenters[i] = 0.0;
      }
      else if (params(index) > 1.0)
      {
        _kernelCenters[i] = 1.0;
      }
      else
      {
        _kernelCenters[i] = params(index);
      }
      index++;
    }
  }
  if (widths)
  {
    for (size_t i = 0; i < _kernelNum; i++)
    {
      if (params(index) < 0.0)
      {
        _kernelWidths[i] = 0.0;
      }
      else
      {
        _kernelWidths[i] = params(index);
      }
      index++;
    }
  }
}

void DMP::computeKernels()
{
  // Initialize the kernels uniformaly in phase space
  // between 0.0 and 1.0 and compute associated kernel width
  // to fill the space with activation
  double length = 1.0 / ((double)_kernelNum);
  double width = -log(_overlap) / pow(length / 2.0, 2);
  for (size_t i = 0; i < _kernelNum; i++)
  {
    _kernelCenters[i] = (double)i * length + length / 2.0;
    _kernelWidths[i] = width;
  }
}

}  // namespace Leph
