#include <iostream>
#include <stdexcept>
#include "Odometry/OdometryNoiseModel.hpp"

// Parameter common bounds
// on standart deviation
constexpr double IdentityBound = 1.0;
constexpr double ConstantDistanceBound = 0.1;
constexpr double ConstantAngularBound = 5 * M_PI / 180.0;
constexpr double MixedBound = 1.0;
constexpr double DefaultIdentityBound = 0.01;
constexpr double DefaultConstantDistanceBound = 0.01;
constexpr double DefaultConstantAngularBound = 1 * M_PI / 180.0;

namespace Leph
{
OdometryNoiseModel::OdometryNoiseModel(Type type) : _type(type), _params(), _maxBounds()
{
  if (_type == NoiseDisable)
  {
    // Dummy model
    _params = Eigen::VectorXd::Zero(0);
    _maxBounds = Eigen::VectorXd::Zero(0);
  }
  else if (_type == NoiseConstant)
  {
    _params = Eigen::VectorXd::Zero(3);
    _maxBounds = Eigen::VectorXd::Zero(3);
    _params << DefaultConstantDistanceBound,  // dX = cst
        DefaultConstantDistanceBound,         // dY = cst
        DefaultConstantAngularBound;          // dA = cst
    _maxBounds << ConstantDistanceBound,      // dX = cst
        ConstantDistanceBound,                // dY = cst
        ConstantAngularBound;                 // dA = cst
  }
  else if (_type == NoiseProportional)
  {
    _params = Eigen::VectorXd::Zero(3);
    _maxBounds = Eigen::VectorXd::Zero(3);
    _params << DefaultIdentityBound,  // dX = dx
        DefaultIdentityBound,         // dY = dy
        DefaultIdentityBound;         // dA = da
    _maxBounds << IdentityBound,      // dX = dx
        IdentityBound,                // dY = dy
        IdentityBound;                // dA = da
  }
  else if (_type == NoiseLinearSimple)
  {
    _params = Eigen::VectorXd::Zero(6);
    _maxBounds = Eigen::VectorXd::Zero(6);
    _params << DefaultConstantDistanceBound,  // dX = cst
        DefaultIdentityBound,                 // dX = dx
        DefaultConstantDistanceBound,         // dY = cst
        DefaultIdentityBound,                 // dY = dy
        DefaultConstantAngularBound,          // dA = cst
        DefaultIdentityBound;                 // dA = da
    _maxBounds << ConstantDistanceBound,      // dX = cst
        IdentityBound,                        // dX = dx
        ConstantDistanceBound,                // dY = cst
        IdentityBound,                        // dY = dy
        ConstantAngularBound,                 // dA = cst
        IdentityBound;                        // dA = da
  }
  else if (_type == NoiseLinearFull)
  {
    _params = Eigen::VectorXd::Zero(12);
    _maxBounds = Eigen::VectorXd::Zero(12);
    _params << DefaultConstantDistanceBound,  // dX = cst
        DefaultIdentityBound,                 // dX = dx
        DefaultIdentityBound,                 // dX = dy
        DefaultIdentityBound,                 // dX = da
        DefaultConstantDistanceBound,         // dY = cst
        DefaultIdentityBound,                 // dY = dx
        DefaultIdentityBound,                 // dY = dy
        DefaultIdentityBound,                 // dY = da
        DefaultConstantAngularBound,          // dA = cst
        DefaultIdentityBound,                 // dA = dx
        DefaultIdentityBound,                 // dA = dy
        DefaultIdentityBound;                 // dA = da
    _maxBounds << ConstantDistanceBound,      // dX = cst
        IdentityBound,                        // dX = dx
        IdentityBound,                        // dX = dy
        IdentityBound,                        // dX = da
        ConstantDistanceBound,                // dY = cst
        IdentityBound,                        // dY = dx
        IdentityBound,                        // dY = dy
        IdentityBound,                        // dY = da
        ConstantAngularBound,                 // dA = cst
        IdentityBound,                        // dA = dx
        IdentityBound,                        // dA = dy
        IdentityBound;                        // dA = da
  }
  else
  {
    throw std::logic_error("OdometryNoiseModel invalid type");
  }
}

OdometryNoiseModel::Type OdometryNoiseModel::getType() const
{
  return _type;
}

const Eigen::VectorXd& OdometryNoiseModel::getParameters() const
{
  return _params;
}

double OdometryNoiseModel::setParameters(const Eigen::VectorXd& params)
{
  // Check size
  if (params.size() != _params.size())
  {
    throw std::logic_error("OdometryNoiseModel invalid input size");
  }

  double error = 0.0;
  // Check bounds
  for (size_t i = 0; i < (size_t)params.size(); i++)
  {
    if (params(i) <= 0.0)
    {
      error += -params(i);
    }
    if (params(i) > _maxBounds(i))
    {
      error += params(i) - _maxBounds(i);
    }
  }

  // Assign parameters if no error
  if (error == 0.0)
  {
    _params = params;
  }

  // Return distance for fitness scoring
  return error;
}

std::vector<std::string> OdometryNoiseModel::getParametersNames() const
{
  if (_type == NoiseDisable)
  {
    return {};
  }
  else if (_type == NoiseConstant)
  {
    return { "dx_bias", "dy_bias", "dz_bias" };
  }
  else if (_type == NoiseProportional)
  {
    return { "dx_from_dx", "dy_from_dy", "dz_from_z" };
  }
  else if (_type == NoiseLinearSimple)
  {
    return { "dx_bias", "dx_from_dx", "dy_bias", "dy_from_dy", "dz_bias", "dz_from_dz" };
  }
  else if (_type == NoiseLinearFull)
  {
    return { "dx_bias",    "dx_from_dx", "dx_from_dy", "dx_from_dz", "dy_bias",    "dy_from_dx",
             "dy_from_dy", "dy_from_dz", "dz_bias",    "dz_from_dx", "dz_from_dy", "dz_from_dz" };
  }
  else
  {
    throw std::logic_error("OdometryDisplacementModel invalid type");
  }
}

const Eigen::VectorXd& OdometryNoiseModel::getNormalization() const
{
  return _maxBounds;
}

Eigen::Vector3d OdometryNoiseModel::noiseGeneration(const Eigen::Vector3d& diff,
                                                    std::default_random_engine& engine) const
{
  // Normal gaussian distribution
  std::normal_distribution<double> dist(0.0, 1.0);

  Eigen::Vector3d noise = Eigen::Vector3d::Zero();
  if (_type == NoiseDisable)
  {
    noise.x() = 0.0;
    noise.y() = 0.0;
    noise.z() = 0.0;
  }
  else if (_type == NoiseConstant)
  {
    noise.x() = _params(0) * dist(engine);
    noise.y() = _params(1) * dist(engine);
    noise.z() = _params(2) * dist(engine);
  }
  else if (_type == NoiseProportional)
  {
    noise.x() = _params(0) * dist(engine) * diff.x();
    noise.y() = _params(1) * dist(engine) * diff.y();
    noise.z() = _params(2) * dist(engine) * diff.z();
  }
  else if (_type == NoiseLinearSimple)
  {
    noise.x() = _params(0) * dist(engine) + _params(1) * dist(engine) * diff.x();
    noise.y() = _params(2) * dist(engine) + _params(3) * dist(engine) * diff.y();
    noise.z() = _params(4) * dist(engine) + _params(5) * dist(engine) * diff.z();
  }
  else if (_type == NoiseLinearFull)
  {
    noise.x() = _params(0) * dist(engine) + _params(1) * dist(engine) * diff.x() +
                _params(2) * dist(engine) * diff.y() + _params(3) * dist(engine) * diff.z();
    noise.y() = _params(4) * dist(engine) + _params(5) * dist(engine) * diff.x() +
                _params(6) * dist(engine) * diff.y() + _params(7) * dist(engine) * diff.z();
    noise.z() = _params(8) * dist(engine) + _params(9) * dist(engine) * diff.x() +
                _params(10) * dist(engine) * diff.y() + _params(11) * dist(engine) * diff.z();
  }
  else
  {
    throw std::logic_error("OdometryNoiseModel invalid type");
  }

  return noise;
}

void OdometryNoiseModel::printParameters() const
{
  if (_type == NoiseDisable)
  {
    std::cout << "No noise parameter" << std::endl;
  }
  else if (_type == NoiseConstant)
  {
    std::cout << "Noise dX += cst: " << _params(0) << std::endl;
    std::cout << "Noise dY += cst: " << _params(1) << std::endl;
    std::cout << "Noise dA += cst: " << _params(2) << std::endl;
  }
  else if (_type == NoiseProportional)
  {
    std::cout << "Noise dX += dx: " << _params(0) << std::endl;
    std::cout << "Noise dY += dy: " << _params(1) << std::endl;
    std::cout << "Noise dA += da: " << _params(2) << std::endl;
  }
  else if (_type == NoiseLinearSimple)
  {
    std::cout << "Noise dX += cst: " << _params(0) << std::endl;
    std::cout << "Noise dX += dx: " << _params(1) << std::endl;
    std::cout << "Noise dY += cst: " << _params(2) << std::endl;
    std::cout << "Noise dY += dy: " << _params(3) << std::endl;
    std::cout << "Noise dA += cst: " << _params(4) << std::endl;
    std::cout << "Noise dA += da: " << _params(5) << std::endl;
  }
  else if (_type == NoiseLinearFull)
  {
    std::cout << "Noise dX += cst: " << _params(0) << std::endl;
    std::cout << "Noise dX += dx: " << _params(1) << std::endl;
    std::cout << "Noise dX += dy: " << _params(2) << std::endl;
    std::cout << "Noise dX += da: " << _params(3) << std::endl;
    std::cout << "Noise dY += cst: " << _params(4) << std::endl;
    std::cout << "Noise dY += dx: " << _params(5) << std::endl;
    std::cout << "Noise dY += dy: " << _params(6) << std::endl;
    std::cout << "Noise dY += da: " << _params(7) << std::endl;
    std::cout << "Noise dA += cst: " << _params(8) << std::endl;
    std::cout << "Noise dA += dx: " << _params(9) << std::endl;
    std::cout << "Noise dA += dy: " << _params(10) << std::endl;
    std::cout << "Noise dA += da: " << _params(11) << std::endl;
  }
  else
  {
    throw std::logic_error("OdometryNoiseModel invalid type");
  }
}

}  // namespace Leph
