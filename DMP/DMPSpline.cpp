#include <stdexcept>
#include <algorithm>
#include "DMP/DMPSpline.hpp"

namespace Leph
{
DMPSpline::DMPSpline() : _kernelNum(0), _overlap(0.1), _maxTimeStep(0.005), _points(), _parts()
{
}

DMPSpline::DMPSpline(unsigned int kernelNum, double overlap, double maxTimeStep)
  : _kernelNum(kernelNum), _overlap(overlap), _maxTimeStep(maxTimeStep), _points(), _parts()
{
}

void DMPSpline::addPoint(double time, double position, double velocity, double acceleration)
{
  _points.push_back({ time, position, velocity, acceleration });
  computeSplines();
}

const std::vector<DMPSpline::Point>& DMPSpline::points() const
{
  return _points;
}
std::vector<DMPSpline::Point>& DMPSpline::points()
{
  return _points;
}

const std::vector<DMPSpline::DMPPart>& DMPSpline::parts() const
{
  return _parts;
}
std::vector<DMPSpline::DMPPart>& DMPSpline::parts()
{
  return _parts;
}

size_t DMPSpline::size() const
{
  return _parts.size();
}

const DMPSpline::DMPPart& DMPSpline::part(size_t index) const
{
  return _parts.at(index);
}
DMPSpline::DMPPart& DMPSpline::part(size_t index)
{
  return _parts.at(index);
}

double DMPSpline::min() const
{
  if (_points.size() == 0)
  {
    throw std::logic_error("DMPSpline empty spline");
  }
  return _parts.front().timeBegin;
}
double DMPSpline::max() const
{
  if (_points.size() == 0)
  {
    throw std::logic_error("DMPSpline empty spline");
  }
  return _parts.back().timeEnd;
}

double DMPSpline::pos(double t)
{
  size_t index = updateInternalDMP(t);
  if (index == (size_t)-1)
  {
    if (t < _parts.front().timeBegin)
    {
      return _points.front().position;
    }
    else
    {
      return _points.back().position;
    }
  }
  else
  {
    return _parts[index].dmp.statePos()(0);
  }
}
double DMPSpline::vel(double t)
{
  size_t index = updateInternalDMP(t);
  if (index == (size_t)-1)
  {
    return 0.0;
  }
  else
  {
    return _parts[index].dmp.stateVel()(0);
  }
}
double DMPSpline::acc(double t)
{
  size_t index = updateInternalDMP(t);
  if (index == (size_t)-1)
  {
    return 0.0;
  }
  else
  {
    return _parts[index].dmp.stateAcc()(0);
  }
}

double DMPSpline::phase(double t)
{
  size_t index = updateInternalDMP(t);
  if (index == (size_t)-1)
  {
    return 0.0;
  }
  else
  {
    return _parts[index].dmp.statePhase();
  }
}
double DMPSpline::gating(double t)
{
  size_t index = updateInternalDMP(t);
  if (index == (size_t)-1)
  {
    return 0.0;
  }
  else
  {
    return _parts[index].dmp.stateGating();
  }
}
double DMPSpline::forcingFunction(double t)
{
  size_t index = updateInternalDMP(t);
  if (index == (size_t)-1)
  {
    return 0.0;
  }
  else
  {
    return _parts[index].dmp.forcingFunction(_parts[index].dmp.statePhase(), _parts[index].dmp.stateGating())(0);
  }
}
double DMPSpline::rawForcingFunction(double t)
{
  size_t index = updateInternalDMP(t);
  if (index == (size_t)-1)
  {
    return 0.0;
  }
  else
  {
    return _parts[index].dmp.forcingFunction(_parts[index].dmp.statePhase(), 1.0)(0);
  }
}

double DMPSpline::getKernelWidth(size_t index) const
{
  size_t partIndex;
  size_t kernelIndex;
  kernelIndexToDMPPart(index, partIndex, kernelIndex);
  // Compute unormalized kernel width similar to all
  // DMP time length
  double totalTimeLength = _parts.back().timeEnd - _parts.front().timeBegin;
  double partTimeLength = _parts[partIndex].timeEnd - _parts[partIndex].timeBegin;
  return _parts[partIndex].dmp.kernelWidth(kernelIndex) * pow(totalTimeLength / partTimeLength, 2);
}
Eigen::VectorXd DMPSpline::getKernelWidths() const
{
  Eigen::VectorXd vect(_kernelNum);
  size_t index = 0;
  double totalTimeLength = _parts.back().timeEnd - _parts.front().timeBegin;
  for (size_t i = 0; i < _parts.size(); i++)
  {
    double partTimeLength = _parts[i].timeEnd - _parts[i].timeBegin;
    for (size_t j = 0; j < _parts[i].dmp.kernelNum(); j++)
    {
      // Compute unnormalized kernel width to be
      // the same for all DMP length
      vect(index) = _parts[i].dmp.kernelWidth(j) * pow(totalTimeLength / partTimeLength, 2);
      index++;
    }
  }
  return vect;
}
double DMPSpline::getKernelWeight(size_t index) const
{
  size_t partIndex;
  size_t kernelIndex;
  kernelIndexToDMPPart(index, partIndex, kernelIndex);
  return _parts[partIndex].dmp.kernelWeight(0, kernelIndex);
}
Eigen::VectorXd DMPSpline::getKernelWeights() const
{
  Eigen::VectorXd vect(_kernelNum);
  size_t index = 0;
  for (size_t i = 0; i < _parts.size(); i++)
  {
    for (size_t j = 0; j < _parts[i].dmp.kernelNum(); j++)
    {
      vect(index) = _parts[i].dmp.kernelWeight(0, j);
      index++;
    }
  }
  return vect;
}
void DMPSpline::setKernelWidth(size_t index, double value)
{
  size_t partIndex;
  size_t kernelIndex;
  kernelIndexToDMPPart(index, partIndex, kernelIndex);
  // Compute unormalized kernel width similar to all
  // DMP time length
  double totalTimeLength = _parts.back().timeEnd - _parts.front().timeBegin;
  double partTimeLength = _parts[partIndex].timeEnd - _parts[partIndex].timeBegin;
  // Value bound
  if (value < 0.0)
  {
    _parts[partIndex].dmp.kernelWidth(kernelIndex) = 0.0;
  }
  else
  {
    _parts[partIndex].dmp.kernelWidth(kernelIndex) = value / pow(totalTimeLength / partTimeLength, 2);
  }
}
void DMPSpline::setKernelWidths(const Eigen::VectorXd& vect)
{
  if (vect.size() != _kernelNum)
  {
    throw std::logic_error("DMPSpline invalid vector size");
  }
  double totalTimeLength = _parts.back().timeEnd - _parts.front().timeBegin;
  size_t index = 0;
  for (size_t i = 0; i < _parts.size(); i++)
  {
    double partTimeLength = _parts[i].timeEnd - _parts[i].timeBegin;
    for (size_t j = 0; j < _parts[i].dmp.kernelNum(); j++)
    {
      // Compute unnormalized kernel width to be
      // the same for all DMP length
      // Value bound
      if (vect(index) < 0.0)
      {
        _parts[i].dmp.kernelWidth(j) = 0.0;
      }
      else
      {
        _parts[i].dmp.kernelWidth(j) = vect(index) / pow(totalTimeLength / partTimeLength, 2);
      }
      index++;
    }
  }
}
void DMPSpline::setKernelWeight(size_t index, double value)
{
  size_t partIndex;
  size_t kernelIndex;
  kernelIndexToDMPPart(index, partIndex, kernelIndex);
  _parts[partIndex].dmp.kernelWeight(0, kernelIndex) = value;
}
void DMPSpline::setKernelWeights(const Eigen::VectorXd& vect)
{
  if (vect.size() != _kernelNum)
  {
    throw std::logic_error("DMPSpline invalid vector size");
  }
  size_t index = 0;
  for (size_t i = 0; i < _parts.size(); i++)
  {
    for (size_t j = 0; j < _parts[i].dmp.kernelNum(); j++)
    {
      _parts[i].dmp.kernelWeight(0, j) = vect(index);
      index++;
    }
  }
}

void DMPSpline::computeSplines(bool noRebuild)
{
  if (!noRebuild)
  {
    // Clear existing DMP
    _parts.clear();
    if (_points.size() < 2)
    {
      return;
    }

    // Sort all points by time
    std::sort(_points.begin(), _points.end(),
              [](const Point& p1, const Point& p2) -> bool { return p1.time < p2.time; });
  }

  // Gaussian kernel center distance from each other
  double kernelLength = 1.0 / ((double)_kernelNum);
  // Complete trajectory start time and length
  double minTime = _points.front().time;
  double maxTime = _points.back().time;
  double totalTimeLength = maxTime - minTime;

  // Iterate over all points from the second one
  size_t index = (size_t)-1;
  for (size_t i = 1; i < _points.size(); i++)
  {
    // Retrieve part start and end time
    double startTime = _points[i - 1].time;
    double endTime = _points[i].time;
    double partTimeLength = endTime - startTime;
    // Skip if the requested duration is too
    // small (allow for discontinuities)
    if (partTimeLength < 1e-6)
    {
      continue;
    }
    // Phase of current part from trajectory begin
    // normalized by total time length
    double phaseStart = (startTime - minTime) / totalTimeLength;
    double phaseEnd = (endTime - minTime) / totalTimeLength;
    // Compute first and last gaussian kernel index
    // included inside the part duration
    // First kernel index after current part start time
    int firstIndex = std::floor(phaseStart / kernelLength - 0.5) + 1;
    unsigned int realFirstIndex = (unsigned int)-1;
    // Last kernel index before current part end time
    int lastIndex = std::floor(phaseEnd / kernelLength - 0.5);
    unsigned int realLastIndex = (unsigned int)-1;
    // Compute the number of included kernel
    unsigned int numKernel = 0;
    if (lastIndex >= firstIndex)
    {
      numKernel = lastIndex - firstIndex + 1;
      realFirstIndex = firstIndex;
      realLastIndex = lastIndex;
    }
    // Sanity check
    if (numKernel > 0 && ((int)realFirstIndex < 0 || (int)realFirstIndex >= (int)_kernelNum || (int)realLastIndex < 0 ||
                          (int)realLastIndex >= (int)_kernelNum))
    {
      throw std::logic_error("DMPSpline assert failed kernel index");
    }
    // Append the new part
    if (!noRebuild)
    {
      _parts.push_back(
          { _points[i - 1].time, _points[i].time, realFirstIndex, realLastIndex, DMP(1, numKernel, _overlap) });
    }
    index++;
    // Initialize the DMP with start and end state
    Eigen::VectorXd startPos(1);
    Eigen::VectorXd startVel(1);
    Eigen::VectorXd startAcc(1);
    Eigen::VectorXd endPos(1);
    Eigen::VectorXd endVel(1);
    Eigen::VectorXd endAcc(1);
    startPos(0) = _points[i - 1].position;
    startVel(0) = _points[i - 1].velocity;
    startAcc(0) = _points[i - 1].acceleration;
    endPos(0) = _points[i].position;
    endVel(0) = _points[i].velocity;
    endAcc(0) = _points[i].acceleration;
    _parts[index].dmp.init(partTimeLength, startPos, startVel, startAcc, endPos, endVel, endAcc);
    // Skip DMP update if no rebuild
    // is done
    if (noRebuild)
    {
      continue;
    }
    // Compute gaussian kernel width similar to
    // a complete DMP over the all time range
    double width = -log(_overlap) / pow(totalTimeLength * kernelLength / (2.0 * partTimeLength), 2);
    // Initialize the kernel centers in order that
    // each part have the kernel centers at the same position
    // as the unique dmp going from complete min to max time.
    // Also assign kernel width with the same purpose.
    for (size_t i = 0; i < numKernel; i++)
    {
      double kernelTimePosition = minTime + totalTimeLength * kernelLength * (realFirstIndex + i + 0.5);
      double kernelPhase = (kernelTimePosition - startTime) / partTimeLength;
      if (kernelPhase < 0.0 || kernelPhase > 1.0)
      {
        throw std::logic_error("DMPSpline assert failed kernel phase: " + std::to_string(kernelPhase));
      }
      _parts[index].dmp.kernelCenter(i) = kernelPhase;
      _parts[index].dmp.kernelWidth(i) = width;
    }
  }
}

void DMPSpline::exportData(std::ostream& os) const
{
  os << _kernelNum << " ";
  os << _points.size() << " ";
  os << std::setprecision(17) << _overlap << " ";
  os << std::setprecision(17) << _maxTimeStep << " ";
  for (size_t i = 0; i < _points.size(); i++)
  {
    os << std::setprecision(17) << _points[i].time << " ";
    os << std::setprecision(17) << _points[i].position << " ";
    os << std::setprecision(17) << _points[i].velocity << " ";
    os << std::setprecision(17) << _points[i].acceleration << " ";
  }
  Eigen::VectorXd weights = getKernelWeights();
  Eigen::VectorXd widths = getKernelWidths();
  for (size_t i = 0; i < _kernelNum; i++)
  {
    os << std::setprecision(17) << widths(i) << " ";
    os << std::setprecision(17) << weights(i) << " ";
  }
  os << std::endl;
}
void DMPSpline::importData(std::istream& is)
{
  // Cleaning
  _points.clear();
  _parts.clear();
  // Check stream validity
  while (is.peek() == ' ' || is.peek() == '\n')
  {
    is.ignore();
  }
  if (!is.good() || is.peek() == EOF)
  {
    return;
  }
  size_t pointSize = 0;
  // Load spline conf
  is >> _kernelNum;
  is >> pointSize;
  is >> _overlap;
  is >> _maxTimeStep;
  // Load via points
  for (size_t i = 0; i < pointSize; i++)
  {
    double time;
    double pos;
    double vel;
    double acc;
    is >> time;
    is >> pos;
    is >> vel;
    is >> acc;
    _points.push_back({ time, pos, vel, acc });
  }
  // Rebuilt all DMP
  computeSplines();
  // Load kernel withs and weights
  for (size_t i = 0; i < _kernelNum; i++)
  {
    double weight;
    double width;
    is >> width;
    is >> weight;
    setKernelWidth(i, width);
    setKernelWeight(i, weight);
  }
}

void DMPSpline::plot(Plot& plot, const std::string& name)
{
  double step = (max() - min()) / 200.0;
  for (double t = min(); t <= max(); t += step)
  {
    plot.add(Leph::VectorLabel("t", t, "phase:" + name, phase(t), "gating:" + name, gating(t), "forcing:" + name,
                               rawForcingFunction(t), "pos:" + name, pos(t), "vel:" + name, vel(t), "acc:" + name,
                               acc(t)));
  }
}
Plot DMPSpline::plot(const std::string& name)
{
  Plot p;
  DMPSpline::plot(p, name);
  return p;
}

size_t DMPSpline::timeToPartIndex(double t) const
{
  // Check if request time is in range
  if (t < _parts.front().timeBegin || t > _parts.back().timeEnd)
  {
    return (size_t)-1;
  }

  // Bijection spline search
  size_t indexLow = 0;
  size_t indexUp = _parts.size() - 1;
  while (indexLow != indexUp)
  {
    size_t index = (indexUp + indexLow) / 2;
    if (t < _parts[index].timeBegin)
    {
      indexUp = index - 1;
    }
    else if (t > _parts[index].timeEnd)
    {
      indexLow = index + 1;
    }
    else
    {
      indexUp = index;
      indexLow = index;
    }
  }

  return indexLow;
}

size_t DMPSpline::updateInternalDMP(double t)
{
  // Check out of bound time
  size_t index = timeToPartIndex(t);
  if (index == (size_t)-1)
  {
    return index;
  }

  // Compute current unormalized time of
  // current DMP part
  double partTime = _parts[index].timeBegin + _parts[index].dmp.currentTime();
  // Check for numerical equality
  //(no need to update)
  if (fabs(t - partTime) < 1e-9)
  {
    return index;
  }
  if (partTime > t)
  {
    // The request time is in the past.
    // We need to re roll all dynamical system
    // Clean and rebuild all DMP parts
    computeSplines(true);
    // Restart
    return updateInternalDMP(t);
  }
  else
  {
    // Else, the requested time is in future.
    // Forward update the DMP to match requested
    // time
    while (t - partTime >= _maxTimeStep)
    {
      // Update the DMP of maximum time step
      _parts[index].dmp.step(_maxTimeStep);
      // Recompute current time
      partTime = _parts[index].timeBegin + _parts[index].dmp.currentTime();
    }
    // Numerical equality check
    if (fabs(t - partTime) < 1e-9)
    {
      return index;
    }
    // Run the DMP update for the
    // remaining time step
    _parts[index].dmp.step(t - partTime);
    return index;
  }
}

void DMPSpline::kernelIndexToDMPPart(size_t kernelIndex, size_t& partIndex, size_t& dmpKernelIndex) const
{
  for (size_t i = 0; i < _parts.size(); i++)
  {
    if (kernelIndex >= _parts[i].kernelFirstIndex && kernelIndex <= _parts[i].kernelLastIndex)
    {
      partIndex = i;
      dmpKernelIndex = kernelIndex - _parts[i].kernelFirstIndex;
      return;
    }
  }
  throw std::logic_error("DMPSpline invalid kernel index: " + std::to_string(kernelIndex));
}

}  // namespace Leph
