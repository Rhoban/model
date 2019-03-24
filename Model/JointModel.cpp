#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include "Model/JointModel.hpp"
#include "Utils/Angle.h"

namespace Leph
{
JointModel::JointModel(const std::string& name)
  : _name(name)
  , _featureBacklash(true)
  , _featureFrictionStribeck(true)
  , _featureReadDiscretization(false)
  , _featureOptimizationVoltage(false)
  , _featureOptimizationResistance(true)
  , _featureOptimizationRegularization(false)
  , _featureOptimizationControlGain(false)
  , _goalTime(0.0)
  , _goalHistory()
  , _isInitialized(false)
  ,
  // Backlash initial state
  _stateBacklashIsEnabled(false)
  , _stateBacklashPosition(0.0)
  , _stateBacklashVelocity(0.0)
  ,
  // Firmware coefficients
  _coefAnglePosToPWM((4096.0 / (2.0 * M_PI)) / 3000.0)
  , _coefPWMBound(0.983)
  ,
  // Default parameters value
  // Friction static regularization coefficient
  _paramFrictionRegularization(20.0)
  ,
  // Friction velocity limit
  _paramFrictionVelLimit(0.2)
  ,
  // Joint internal gearbox inertia
  _paramInertiaIn(0.003)
  ,
  // Friction internal gearbox parameters
  //(BreakIn is a positive offset above CoulombIn)
  _paramFrictionViscousIn(0.07)
  , _paramFrictionBreakIn(0.03)
  , _paramFrictionCoulombIn(0.05)
  ,
  // Joint external gearbox inertia
  _paramInertiaOut(0.001)
  ,
  // Friction external gearbox parameters
  //(BreakOut is a positive offset above CoulombOut)
  _paramFrictionViscousOut(0.01)
  , _paramFrictionBreakOut(0.03)
  , _paramFrictionCoulombOut(0.02)
  ,
  // Electric motor voltage
  _paramElectricVoltage(10.5)
  ,
  // Electric motor ke
  _paramElectricKe(1.4)
  ,
  // Electric motor resistance
  _paramElectricResistance(4.07)
  ,
  // Motor proportional control
  _paramControlGainP(30.0)
  ,
  // Motor position discretization coefficient
  _paramControlDiscretization(2048.0)
  ,
  // Control lag in seconds
  _paramControlLag(0.030)
  ,
  // Backlash enable to disable position threshold
  //(Positive offset above activation)
  _paramBacklashThresholdDeactivation(0.01)
  ,
  // Backlash disable to enable position threshold
  _paramBacklashThresholdActivation(0.02)
  ,
  // Backlash maximum angular distance
  _paramBacklashRangeMax(0.05)
{
  // Initialize the parameters according to
  // selected feature model
  Eigen::VectorXd tmpParams = getParameters();
  setParameters(tmpParams);
}

const std::string& JointModel::getName() const
{
  return _name;
}

const Eigen::VectorXd JointModel::getParameters() const
{
  // Build parameters vector
  Eigen::VectorXd parameters;
  size_t index = 0;
  // External friction parameters
  parameters.conservativeResize(index + 3);
  parameters(index) = _paramFrictionViscousOut;
  parameters(index + 1) = _paramFrictionCoulombOut;
  parameters(index + 2) = _paramInertiaOut;
  index += 3;
  if (_featureFrictionStribeck)
  {
    parameters.conservativeResize(index + 2);
    parameters(index) = _paramFrictionVelLimit;
    parameters(index + 1) = _paramFrictionBreakOut;
    index += 2;
  }
  // Internal friction parameters
  if (_featureBacklash)
  {
    parameters.conservativeResize(index + 3);
    parameters(index) = _paramFrictionViscousIn;
    parameters(index + 1) = _paramFrictionCoulombIn;
    parameters(index + 2) = _paramInertiaIn;
    index += 3;
    if (_featureFrictionStribeck)
    {
      parameters.conservativeResize(index + 1);
      parameters(index) = _paramFrictionBreakIn;
      index++;
    }
  }
  // Backlash internal
  if (_featureBacklash)
  {
    parameters.conservativeResize(index + 3);
    parameters(index) = _paramBacklashRangeMax;
    parameters(index + 1) = _paramBacklashThresholdDeactivation;
    parameters(index + 2) = _paramBacklashThresholdActivation;
    index += 3;
  }
  // Other control and electric parameters
  parameters.conservativeResize(index + 2);
  parameters(index) = _paramControlLag;
  parameters(index + 1) = _paramElectricKe;
  index += 2;
  // Read position encoder discretization
  if (_featureReadDiscretization)
  {
    parameters.conservativeResize(index + 1);
    parameters(index) = _paramControlDiscretization;
    index++;
  }
  // Electric power voltage
  if (_featureOptimizationVoltage)
  {
    parameters.conservativeResize(index + 1);
    parameters(index) = _paramElectricVoltage;
    index++;
  }
  // Electric resistance
  if (_featureOptimizationResistance)
  {
    parameters.conservativeResize(index + 1);
    parameters(index) = _paramElectricResistance;
    index++;
  }
  // Friction force regularization coefficient
  if (_featureOptimizationRegularization)
  {
    parameters.conservativeResize(index + 1);
    parameters(index) = _paramFrictionRegularization;
    index++;
  }
  // Proportional control gain
  if (_featureOptimizationControlGain)
  {
    parameters.conservativeResize(index + 1);
    parameters(index) = _paramControlGainP;
    index++;
  }

  return parameters;
}
void JointModel::setParameters(const Eigen::VectorXd& parameters)
{
  // Shrink to positive value
  Eigen::VectorXd tmpParams = parameters;
  for (size_t i = 0; i < (size_t)tmpParams.size(); i++)
  {
    if (tmpParams(i) < 0.0)
    {
      tmpParams(i) = 0.0;
    }
  }

  // Read parameters vector
  size_t index = 0;
  // External friction parameters
  if ((size_t)tmpParams.size() < index + 3)
    goto sizeError;
  _paramFrictionViscousOut = tmpParams(index);
  _paramFrictionCoulombOut = tmpParams(index + 1);
  _paramInertiaOut = tmpParams(index + 2);
  index += 3;
  if (_featureFrictionStribeck)
  {
    if ((size_t)tmpParams.size() < index + 2)
      goto sizeError;
    _paramFrictionVelLimit = tmpParams(index);
    _paramFrictionBreakOut = tmpParams(index + 1);
    index += 2;
  }
  // Internal friction parameters
  if (_featureBacklash)
  {
    if ((size_t)tmpParams.size() < index + 3)
      goto sizeError;
    _paramFrictionViscousIn = tmpParams(index);
    _paramFrictionCoulombIn = tmpParams(index + 1);
    _paramInertiaIn = tmpParams(index + 2);
    index += 3;
    if (_featureFrictionStribeck)
    {
      if ((size_t)tmpParams.size() < index + 1)
        goto sizeError;
      _paramFrictionBreakIn = tmpParams(index);
      index++;
    }
  }
  // Backlash internal
  if (_featureBacklash)
  {
    if ((size_t)tmpParams.size() < index + 3)
      goto sizeError;
    _paramBacklashRangeMax = tmpParams(index);
    _paramBacklashThresholdDeactivation = tmpParams(index + 1);
    _paramBacklashThresholdActivation = tmpParams(index + 2);
    index += 3;
  }
  // Other control and electric parameters
  if ((size_t)tmpParams.size() < index + 2)
    goto sizeError;
  _paramControlLag = tmpParams(index);
  _paramElectricKe = tmpParams(index + 1);
  index += 2;
  // Read position encoder discretization
  if (_featureReadDiscretization)
  {
    if ((size_t)tmpParams.size() < index + 1)
      goto sizeError;
    _paramControlDiscretization = tmpParams(index);
    index++;
  }
  // Electric power voltage
  if (_featureOptimizationVoltage)
  {
    if ((size_t)tmpParams.size() < index + 1)
      goto sizeError;
    _paramElectricVoltage = tmpParams(index);
    index++;
  }
  // Electric resistance
  if (_featureOptimizationResistance)
  {
    if ((size_t)tmpParams.size() < index + 1)
      goto sizeError;
    _paramElectricResistance = tmpParams(index);
    index++;
  }
  // Friction force regularization coefficient
  if (_featureOptimizationRegularization)
  {
    if ((size_t)tmpParams.size() < index + 1)
      goto sizeError;
    _paramFrictionRegularization = tmpParams(index);
    index++;
  }
  // Propotional control gain
  if (_featureOptimizationControlGain)
  {
    if ((size_t)tmpParams.size() < index + 1)
      goto sizeError;
    _paramControlGainP = tmpParams(index);
    index++;
  }
  return;

sizeError:
  throw std::logic_error("JointModel invalid parameters size: " + std::to_string(tmpParams.size()));
}

double JointModel::getInertia() const
{
  if (_featureBacklash)
  {
    if (_stateBacklashIsEnabled)
    {
      return _paramInertiaOut;
    }
    else
    {
      return _paramInertiaIn + _paramInertiaOut;
    }
  }
  else
  {
    return _paramInertiaOut;
  }
}

void JointModel::setMaxVoltage(double volt)
{
  if (volt < 0.0)
  {
    throw std::logic_error("JointModel invalid maximum voltage: " + std::to_string(volt));
  }
  _paramElectricVoltage = volt;
}
double JointModel::getMaxVoltage() const
{
  return _paramElectricVoltage;
}

double JointModel::frictionTorque(double vel) const
{
  if (_stateBacklashIsEnabled)
  {
    return computeFrictionTorque(vel, nullptr, false, true);
  }
  else
  {
    return computeFrictionTorque(vel, nullptr, true, true);
  }
}

double JointModel::controlTorque(double pos, double vel) const
{
  if (_stateBacklashIsEnabled)
  {
    return 0.0;
  }
  else
  {
    return computeControlTorque(pos, vel);
  }
}

void JointModel::updateState(double dt, double goal, double pos, double vel)
{
  // Hidden state initialization
  if (!_isInitialized)
  {
    _goalTime = 0.0;
    while (!_goalHistory.empty())
    {
      _goalHistory.pop();
    }
    _stateBacklashIsEnabled = false;
    _stateBacklashPosition = pos;
    _stateBacklashVelocity = vel;
    _isInitialized = true;
  }

  // Append given goal
  _goalHistory.push({ _goalTime, goal });
  // Update integrated time
  _goalTime += dt;

  // Pop history to get current goal lag
  while (_goalHistory.size() >= 2 && _goalHistory.front().first < _goalTime - _paramControlLag)
  {
    _goalHistory.pop();
  }

  // Update backlash model
  if (_featureBacklash)
  {
    // Compute backlash acceleration
    double backlashControlTorque = computeControlTorque(pos, vel);
    double backlashFrictionTorque = computeFrictionTorque(vel, &backlashControlTorque, true, false);
    double backlashAcc = (backlashControlTorque + backlashFrictionTorque) / _paramInertiaIn;
    // Update backlash velocity and position
    double backlashNextVel = _stateBacklashVelocity + dt * backlashAcc;
    _stateBacklashVelocity = 0.5 * _stateBacklashVelocity + 0.5 * backlashNextVel;
    _stateBacklashPosition = _stateBacklashPosition + dt * _stateBacklashVelocity;
    // Update backlash state
    double relativePos = fabs(AngleDistance(pos, _stateBacklashPosition));
    // Used state transition thresholds
    double usedThresholdDeactivation = _paramBacklashThresholdDeactivation + _paramBacklashThresholdActivation;
    double usedThresholdActivation = _paramBacklashThresholdActivation;
    if (_stateBacklashIsEnabled && relativePos > usedThresholdDeactivation)
    {
      _stateBacklashIsEnabled = false;
    }
    else if (!_stateBacklashIsEnabled && relativePos < usedThresholdActivation)
    {
      _stateBacklashIsEnabled = true;
    }
    // Bound backlash position
    if (AngleDistance(pos, _stateBacklashPosition) >= _paramBacklashRangeMax)
    {
      _stateBacklashPosition = pos + _paramBacklashRangeMax;
      _stateBacklashVelocity = 0.0;
    }
    if (AngleDistance(pos, _stateBacklashPosition) <= -_paramBacklashRangeMax)
    {
      _stateBacklashPosition = pos - _paramBacklashRangeMax;
      _stateBacklashVelocity = 0.0;
    }
    _stateBacklashPosition = AngleBound(_stateBacklashPosition);
  }
}

double JointModel::getDelayedGoal() const
{
  if (_goalHistory.size() == 0)
  {
    return 0.0;
  }
  else
  {
    return _goalHistory.front().second;
  }
}

bool JointModel::getBacklashStateEnabled() const
{
  return _stateBacklashIsEnabled;
}
double JointModel::getBacklashStatePos() const
{
  return _stateBacklashPosition;
}
double JointModel::getBacklashStateVel() const
{
  return _stateBacklashVelocity;
}

void JointModel::resetHiddenState()
{
  _isInitialized = false;
  _goalTime = 0.0;
  while (!_goalHistory.empty())
  {
    _goalHistory.pop();
  }
  _stateBacklashIsEnabled = true;
  _stateBacklashPosition = 0.0;
  _stateBacklashVelocity = 0.0;
}

void JointModel::boundState(double& pos, double& vel)
{
  // Check numerical instability
  if (fabs(pos) > 1e10 || fabs(vel) > 1e10)
  {
    throw std::runtime_error("JointModel numerical instability:" + std::string(" name=") + _name +
                             std::string(" pos=") + std::to_string(pos) + std::string(" vel=") + std::to_string(vel));
  }
  // Bound position angle inside [-PI:PI]
  pos = AngleBound(pos);
}

double JointModel::computeElectricTension(double velGoal, double accGoal, double torqueGoal) const
{
  // Compute friction torque
  //(backlash is not considered)
  double frictionTorque = computeFrictionTorque(velGoal, &torqueGoal, true, true);

  // Compute torque from gears inertia
  //(backlash is not considered)
  double usedInertia = 0.0;
  if (!_featureBacklash)
  {
    usedInertia = _paramInertiaOut;
  }
  else
  {
    usedInertia = _paramInertiaIn + _paramInertiaOut;
  }
  double inertiaTorque = accGoal * usedInertia;

  // Compute internal torque seen by the motor
  // to produce expected motion
  double torqueInternalGoal = torqueGoal - frictionTorque + inertiaTorque;

  // Compute expected electric tension to produce
  // needed torque at motor output
  double tensionGoal =
      torqueInternalGoal * _paramElectricResistance / _paramElectricKe + velGoal * _paramElectricResistance;

  return tensionGoal;
}

double JointModel::computeFeedForward(double velGoal, double accGoal, double torqueGoal) const
{
  // Compute expected motor tension
  double tensionGoal = computeElectricTension(velGoal, accGoal, torqueGoal);

  // Compute expected control PWM ratio
  double controlRatioGoal = tensionGoal / (_paramElectricVoltage);

  // Bound control ratio to controller capability
  if (controlRatioGoal > _coefPWMBound)
  {
    tensionGoal = _coefPWMBound;
  }
  if (tensionGoal < -_coefPWMBound)
  {
    tensionGoal = -_coefPWMBound;
  }

  // Compute angular offset from tension
  double angularOffset = controlRatioGoal / (_paramControlGainP * _coefAnglePosToPWM);

  return angularOffset;
}

void JointModel::printParameters() const
{
  std::cout << "JointParameters: " << _name << " ("
            << "Backlash:" << _featureBacklash << " Stribeck:" << _featureFrictionStribeck
            << " Discretization:" << _featureReadDiscretization << " OptimVoltage:" << _featureOptimizationVoltage
            << " OptimResistance:" << _featureOptimizationResistance
            << " OptimRegularization:" << _featureOptimizationRegularization << ")" << std::endl;
  unsigned int width1 = 31;
  unsigned int width2 = 15;
  std::cout << std::setiosflags(std::ios::left);
  std::cout << std::setw(width1) << "FrictionViscousOut:" << std::setw(width2) << std::fixed << std::setprecision(10)
            << _paramFrictionViscousOut << std::endl;
  std::cout << std::setw(width1) << "FrictionCoulombOut:" << std::setw(width2) << std::fixed << std::setprecision(10)
            << _paramFrictionCoulombOut << std::endl;
  std::cout << std::setw(width1) << "InertiaOut:" << std::setw(width2) << std::fixed << std::setprecision(10)
            << _paramInertiaOut << std::endl;
  if (_featureFrictionStribeck)
  {
    std::cout << std::setw(width1) << "FrictionVelLimit:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramFrictionVelLimit << std::endl;
    std::cout << std::setw(width1) << "FrictionBreakOut:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramFrictionBreakOut << std::endl;
  }
  if (_featureBacklash)
  {
    std::cout << std::setw(width1) << "FrictionViscousIn:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramFrictionViscousIn << std::endl;
    std::cout << std::setw(width1) << "FrictionCoulombIn:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramFrictionCoulombIn << std::endl;
    std::cout << std::setw(width1) << "InertiaIn:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramInertiaIn << std::endl;
    if (_featureFrictionStribeck)
    {
      std::cout << std::setw(width1) << "FrictionBreakIn:" << std::setw(width2) << std::fixed << std::setprecision(10)
                << _paramFrictionBreakIn << std::endl;
    }
  }
  if (_featureBacklash)
  {
    std::cout << std::setw(width1) << "BacklashRangeMax:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramBacklashRangeMax << std::endl;
    std::cout << std::setw(width1) << "BacklashThresholdDeactivation:" << std::setw(width2) << std::fixed
              << std::setprecision(10) << _paramBacklashThresholdDeactivation << std::endl;
    std::cout << std::setw(width1) << "BacklashThresholdActivation:" << std::setw(width2) << std::fixed
              << std::setprecision(10) << _paramBacklashThresholdActivation << std::endl;
  }
  std::cout << std::setw(width1) << "ControlLag:" << std::setw(width2) << std::fixed << std::setprecision(10)
            << _paramControlLag << std::endl;
  std::cout << std::setw(width1) << "ElectricKe:" << std::setw(width2) << std::fixed << std::setprecision(10)
            << _paramElectricKe << std::endl;
  if (_featureReadDiscretization)
  {
    std::cout << std::setw(width1) << "ControlDiscretization:" << std::setw(width2) << std::fixed
              << std::setprecision(10) << _paramControlDiscretization << std::endl;
  }
  if (_featureOptimizationVoltage)
  {
    std::cout << std::setw(width1) << "ElectricVoltage:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramElectricVoltage << std::endl;
  }
  if (_featureOptimizationResistance)
  {
    std::cout << std::setw(width1) << "ElectricResistance:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramElectricResistance << std::endl;
  }
  if (_featureOptimizationRegularization)
  {
    std::cout << std::setw(width1) << "FrictionRegularization:" << std::setw(width2) << std::fixed
              << std::setprecision(10) << _paramFrictionRegularization << std::endl;
  }
  if (_featureOptimizationControlGain)
  {
    std::cout << std::setw(width1) << "ControlGainP:" << std::setw(width2) << std::fixed << std::setprecision(10)
              << _paramControlGainP << std::endl;
  }
  std::cout << std::setiosflags(std::ios::right);
}

double JointModel::computeFrictionTorque(double vel, double* torque, bool isInFriction, bool isOutFriction) const
{
  // Apply internal and external friction model
  double usedFrictionVelLimit = _paramFrictionVelLimit;
  double usedFrictionViscous = 0.0;
  double usedFrictionBreak = 0.0;
  double usedFrictionCoulomb = 0.0;
  if (isInFriction && _featureBacklash)
  {
    usedFrictionViscous += _paramFrictionViscousIn;
    usedFrictionBreak += _paramFrictionBreakIn + _paramFrictionCoulombIn;
    usedFrictionCoulomb += _paramFrictionCoulombIn;
  }
  if (isOutFriction)
  {
    usedFrictionViscous += _paramFrictionViscousOut;
    usedFrictionBreak += _paramFrictionBreakOut + _paramFrictionCoulombOut;
    usedFrictionCoulomb += _paramFrictionCoulombOut;
  }
  if (!_featureFrictionStribeck)
  {
    usedFrictionBreak = usedFrictionCoulomb;
  }

  // Compute friction
  double beta = exp(-fabs(vel / usedFrictionVelLimit));
  double forceViscous = -usedFrictionViscous * vel;
  double forceStatic1 = -beta * usedFrictionBreak;
  double forceStatic2 = -(1.0 - beta) * usedFrictionCoulomb;
  // Static friction regularization passing by zero
  // to prevent too stiff dynamics and allows for nice
  // continuous forces and accelerations.
  double forceStaticRegularized = (forceStatic1 + forceStatic2) * tanh(_paramFrictionRegularization * vel);

  return forceViscous + forceStaticRegularized;
}

double JointModel::computeControlTorque(double pos, double vel) const
{
  // Apply position discretization
  double discretizedPos = pos;
  if (_featureReadDiscretization)
  {
    double motorStepCoef = M_PI / _paramControlDiscretization;
    discretizedPos = std::floor(pos / motorStepCoef) * motorStepCoef;
  }

  // Retrieve delayed goal
  double delayedGoal = getDelayedGoal();

  // Angular distance in radian
  double error = AngleDistance(delayedGoal, discretizedPos);
  // Compute Motor control PWM ratio
  double controlRatio = -_paramControlGainP * error * _coefAnglePosToPWM;
  // Bound the PWM control ratio between
  //-1.0*_coefPWMBound and 1.0*_coefPWMBound
  if (controlRatio > _coefPWMBound)
  {
    controlRatio = _coefPWMBound;
  }
  else if (controlRatio < -_coefPWMBound)
  {
    controlRatio = -_coefPWMBound;
  }
  // Compute the applied tension
  // on the electric motor by H-bridge
  double tension = controlRatio * _paramElectricVoltage;

  // Compute applied electrical torque
  double torque =
      tension * _paramElectricKe / _paramElectricResistance - vel * pow(_paramElectricKe, 2) / _paramElectricResistance;

  return torque;
}

}  // namespace Leph
