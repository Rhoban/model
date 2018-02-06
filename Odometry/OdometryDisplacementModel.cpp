#include <iostream>
#include <stdexcept>
#include "Odometry/OdometryDisplacementModel.hpp"

namespace Leph {

//Parameter common bounds
constexpr double IdentityMin = 0.5;
constexpr double IdentityMax = 2.0;
constexpr double OffsetStepBound = 0.04;
constexpr double OffsetLateralBound = 0.04;
constexpr double offsetTurnBound = 0.08;
constexpr double CoefMixedBound = 1.0;

OdometryDisplacementModel::OdometryDisplacementModel(Type type) :
    _type(type),
    _params(),
    _minBounds(),
    _maxBounds()
{
    if (_type == DisplacementIdentity) {
        //Dummy correction
        _params = Eigen::VectorXd::Zero(0);
        _minBounds = Eigen::VectorXd::Zero(0);
        _maxBounds = Eigen::VectorXd::Zero(0);
    } else if (_type == DisplacementProportionalXY) {
        _params = Eigen::VectorXd::Zero(2);
        _minBounds = Eigen::VectorXd::Zero(2);
        _maxBounds = Eigen::VectorXd::Zero(2);
        _params <<
            1.0, //dX = dx
            1.0; //dY = dy
        _minBounds <<
            IdentityMin, //dX = dx
            IdentityMin; //dY = dy
        _maxBounds <<
            IdentityMax, //dX = dx
            IdentityMax; //dY = dy
    } else if (_type == DisplacementProportionalXYA) {
        _params = Eigen::VectorXd::Zero(3);
        _minBounds = Eigen::VectorXd::Zero(3);
        _maxBounds = Eigen::VectorXd::Zero(3);
        _params <<
            1.0, //dX = dx
            1.0, //dY = dy
            1.0; //dA = da
        _minBounds <<
            IdentityMin, //dX = dx
            IdentityMin, //dY = dy
            IdentityMin; //dA = da
        _maxBounds <<
            IdentityMax, //dX = dx
            IdentityMax, //dY = dy
            IdentityMax; //dA = da
    } else if (_type == DisplacementLinearSimpleXY) {
        _params = Eigen::VectorXd::Zero(4);
        _minBounds = Eigen::VectorXd::Zero(4);
        _maxBounds = Eigen::VectorXd::Zero(4);
        _params <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dY = offset
            1.0; //dY = dy
        _minBounds <<
            -OffsetStepBound,    //dX = offset
            IdentityMin,         //dX = dx
            -OffsetLateralBound, //dY = offset
            IdentityMin;         //dY = dy
        _maxBounds <<
            OffsetStepBound,    //dX = offset
            IdentityMax,        //dX = dx
            OffsetLateralBound, //dY = offset
            IdentityMax;        //dY = dy
    } else if (_type == DisplacementLinearSimpleXYA) {
        _params = Eigen::VectorXd::Zero(6);
        _minBounds = Eigen::VectorXd::Zero(6);
        _maxBounds = Eigen::VectorXd::Zero(6);
        _params <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dY = offset
            1.0, //dY = dy
            0.0, //dA = offset
            1.0; //dA = da
        _minBounds <<
            -OffsetStepBound,    //dX = offset
            IdentityMin,         //dX = dx
            -OffsetLateralBound, //dY = offset
            IdentityMin,         //dY = dy
            -offsetTurnBound,    //dA = offset
            IdentityMin;         //dA = da
        _maxBounds <<
            OffsetStepBound,    //dX = offset
            IdentityMax,        //dX = dx
            OffsetLateralBound, //dY = offset
            IdentityMax,        //dY = dy
            offsetTurnBound,    //dA = offset
            IdentityMax;        //dA = da
    } else if (_type == DisplacementLinearFullXY) {
        _params = Eigen::VectorXd::Zero(8);
        _minBounds = Eigen::VectorXd::Zero(8);
        _maxBounds = Eigen::VectorXd::Zero(8);
        _params <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dX = dy
            0.0, //dX = da
            0.0, //dY = offset
            0.0, //dY = dx
            1.0, //dY = dy
            0.0; //dY = da
        _minBounds <<
            -OffsetStepBound,    //dX = offset
            IdentityMin,         //dX = dx
            -CoefMixedBound,     //dX = dy
            -CoefMixedBound,     //dX = da
            -OffsetLateralBound, //dY = offset
            -CoefMixedBound,     //dY = dx
            IdentityMin,         //dY = dy
            -CoefMixedBound;     //dY = da
        _maxBounds <<
            OffsetStepBound,    //dX = offset
            IdentityMax,        //dX = dx
            CoefMixedBound,     //dX = dy
            CoefMixedBound,     //dX = da
            OffsetLateralBound, //dY = offset
            CoefMixedBound,     //dY = dx
            IdentityMax,        //dY = dy
            CoefMixedBound;     //dY = da
    } else if (_type == DisplacementLinearFullXYA) {
        _params = Eigen::VectorXd::Zero(12);
        _minBounds = Eigen::VectorXd::Zero(12);
        _maxBounds = Eigen::VectorXd::Zero(12);
        _params <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dX = dy
            0.0, //dX = da
            0.0, //dY = offset
            0.0, //dY = dx
            1.0, //dY = dy
            0.0, //dY = da
            0.0, //dA = offset
            0.0, //dA = dx
            0.0, //dA = dy
            1.0; //dA = da
        _minBounds <<
            -OffsetStepBound,    //dX = offset
            IdentityMin,         //dX = dx
            -CoefMixedBound,     //dX = dy
            -CoefMixedBound,     //dX = da
            -OffsetLateralBound, //dY = offset
            -CoefMixedBound,     //dY = dx
            IdentityMin,         //dY = dy
            -CoefMixedBound,     //dY = da
            -offsetTurnBound,    //dA = offset
            -CoefMixedBound,     //dA = dx
            -CoefMixedBound,     //dA = dy
            IdentityMin;         //dA = da
        _maxBounds <<
            OffsetStepBound,    //dX = offset
            IdentityMax,        //dX = dx
            CoefMixedBound,     //dX = dy
            CoefMixedBound,     //dX = da
            OffsetLateralBound, //dY = offset
            CoefMixedBound,     //dY = dx
            IdentityMax,        //dY = dy
            CoefMixedBound,     //dY = da
            offsetTurnBound,    //dA = offset
            CoefMixedBound,     //dA = dx
            CoefMixedBound,     //dA = dy
            IdentityMax;        //dA = da
    } else {
        throw std::logic_error(
            "OdometryDisplacementModel invalid type");
    }
}
        
OdometryDisplacementModel::Type OdometryDisplacementModel::getType() const
{
    return _type;
}
        
const Eigen::VectorXd& OdometryDisplacementModel::getParameters() const
{
    return _params;
}

double OdometryDisplacementModel::setParameters(
    const Eigen::VectorXd& params)
{
    //Check size
    if (params.size() != _params.size()) {
        throw std::logic_error(
            "OdometryDisplacementModel invalid input size");
    }

    double error = 0.0;
    //Check bounds
    for (size_t i=0;i<(size_t)params.size();i++) {
        if (params(i) < _minBounds(i)) {
            error += _minBounds(i) - params(i);
        }
        if (params(i) > _maxBounds(i)) {
            error += params(i) - _maxBounds(i);
        }
    }

    //Assign parameters if no error
    if (error == 0.0) {
        _params = params;
    }

    //Return distance for fitness scoring
    return error;
}

std::vector<std::string> OdometryDisplacementModel::getParametersNames() const
{
    if (_type == DisplacementIdentity) {
      return {};
    } else if (_type == DisplacementProportionalXY) {
      return {"dx_from_dx","dy_from_dy"};
    } else if (_type == DisplacementProportionalXYA) {
      return {"dx_from_dx","dy_from_dy","dz_from_z"};
    } else if (_type == DisplacementLinearSimpleXY) {
      return {"dx_bias","dx_from_dx","dy_bias","dy_from_dy"};
    } else if (_type == DisplacementLinearSimpleXYA) {
      return {"dx_bias","dx_from_dx","dy_bias","dy_from_dy", "dz_bias", "dz_from_dz"};
    } else if (_type == DisplacementLinearFullXY) {
      return {"dx_bias","dx_from_dx","dx_from_dy","dx_from_dz",
          "dy_bias","dy_from_dx","dy_from_dy","dy_from_dz"};
    } else if (_type == DisplacementLinearFullXYA) {
      return {"dx_bias","dx_from_dx","dx_from_dy","dx_from_dz",
          "dy_bias","dy_from_dx","dy_from_dy","dy_from_dz",
          "dz_bias","dz_from_dx","dz_from_dy","dz_from_dz"};
    } else {
        throw std::logic_error(
            "OdometryDisplacementModel invalid type");
    }
}
        
const Eigen::VectorXd& OdometryDisplacementModel::getNormalization() const
{
    return _maxBounds;
}

Eigen::Vector3d OdometryDisplacementModel::displacementCorrection(
    const Eigen::Vector3d& diff) const
{
    Eigen::Vector3d newDiff = diff;

    if (_type == DisplacementIdentity) {
        newDiff = diff;
    } else if (_type == DisplacementProportionalXY) {
        newDiff.x() = 
            _params(0)*diff.x();
        newDiff.y() = 
            _params(1)*diff.y();
    } else if (_type == DisplacementProportionalXYA) {
        newDiff.x() = 
            _params(0)*diff.x();
        newDiff.y() = 
            _params(1)*diff.y();
        newDiff.z() = 
            _params(2)*diff.z();
    } else if (_type == DisplacementLinearSimpleXY) {
        newDiff.x() = 
            _params(0) +
            _params(1)*diff.x();
        newDiff.y() = 
            _params(2) +
            _params(3)*diff.y();
    } else if (_type == DisplacementLinearSimpleXYA) {
        newDiff.x() = 
            _params(0) +
            _params(1)*diff.x();
        newDiff.y() = 
            _params(2) +
            _params(3)*diff.y();
        newDiff.z() = 
            _params(4) +
            _params(5)*diff.z();
    } else if (_type == DisplacementLinearFullXY) {
        newDiff.x() = 
            _params(0) +
            _params(1)*diff.x() +
            _params(2)*diff.y() +
            _params(3)*diff.z();
        newDiff.y() = 
            _params(4) +
            _params(5)*diff.x() + 
            _params(6)*diff.y() +
            _params(7)*diff.z();
    } else if (_type == DisplacementLinearFullXYA) {
        newDiff.x() = 
            _params(0) +
            _params(1)*diff.x() +
            _params(2)*diff.y() +
            _params(3)*diff.z();
        newDiff.y() = 
            _params(4) +
            _params(5)*diff.x() +
            _params(6)*diff.y() +
            _params(7)*diff.z();
        newDiff.z() = 
            _params(8) +
            _params(9)*diff.x() +
            _params(10)*diff.y() +
            _params(11)*diff.z();
    } else {
        throw std::logic_error(
            "OdometryDisplacementModel invalid type");
    }
    
    return newDiff;
}
        
void OdometryDisplacementModel::printParameters() const
{
    if (_type == DisplacementIdentity) {
        std::cout << "No displacement parameter" << std::endl;
    } else if (_type == DisplacementProportionalXY) {
        std::cout << "Displacement dX = dx: " 
            << _params(0) << std::endl;
        std::cout << "Displacement dY = dy: " 
            << _params(1) << std::endl;
    } else if (_type == DisplacementProportionalXYA) {
        std::cout << "Displacement dX = dx: " 
            << _params(0) << std::endl;
        std::cout << "Displacement dY = dy: " 
            << _params(1) << std::endl;
        std::cout << "Displacement dA = da: " 
            << _params(2) << std::endl;
    } else if (_type == DisplacementLinearSimpleXY) {
        std::cout << "Displacement dX = offset: " 
            << _params(0) << std::endl;
        std::cout << "Displacement dX = dx: " 
            << _params(1) << std::endl;
        std::cout << "Displacement dY = offset: " 
            << _params(2) << std::endl;
        std::cout << "Displacement dY = dy: " 
            << _params(3) << std::endl;
    } else if (_type == DisplacementLinearSimpleXYA) {
        std::cout << "Displacement dX = offset: " 
            << _params(0) << std::endl;
        std::cout << "Displacement dX = dx: " 
            << _params(1) << std::endl;
        std::cout << "Displacement dY = offset: " 
            << _params(2) << std::endl;
        std::cout << "Displacement dY = dy: " 
            << _params(3) << std::endl;
        std::cout << "Displacement dA = offset: " 
            << _params(4) << std::endl;
        std::cout << "Displacement dA = da: " 
            << _params(5) << std::endl;
    } else if (_type == DisplacementLinearFullXY) {
        std::cout << "Displacement dX = offset: " 
            << _params(0) << std::endl;
        std::cout << "Displacement dX = dx: " 
            << _params(1) << std::endl;
        std::cout << "Displacement dX = dy: " 
            << _params(2) << std::endl;
        std::cout << "Displacement dX = da: " 
            << _params(3) << std::endl;
        std::cout << "Displacement dY = offset: " 
            << _params(4) << std::endl;
        std::cout << "Displacement dY = dx: " 
            << _params(5) << std::endl;
        std::cout << "Displacement dY = dy: " 
            << _params(6) << std::endl;
        std::cout << "Displacement dY = da: " 
            << _params(7) << std::endl;
    } else if (_type == DisplacementLinearFullXYA) {
        std::cout << "Displacement dX = offset: " 
            << _params(0) << std::endl;
        std::cout << "Displacement dX = dx: " 
            << _params(1) << std::endl;
        std::cout << "Displacement dX = dy: " 
            << _params(2) << std::endl;
        std::cout << "Displacement dX = da: " 
            << _params(3) << std::endl;
        std::cout << "Displacement dY = offset: " 
            << _params(4) << std::endl;
        std::cout << "Displacement dY = dx: " 
            << _params(5) << std::endl;
        std::cout << "Displacement dY = dy: " 
            << _params(6) << std::endl;
        std::cout << "Displacement dY = da: " 
            << _params(7) << std::endl;
        std::cout << "Displacement dA = offset: " 
            << _params(8) << std::endl;
        std::cout << "Displacement dA = dx: " 
            << _params(9) << std::endl;
        std::cout << "Displacement dA = dy: " 
            << _params(10) << std::endl;
        std::cout << "Displacement dA = da: " 
            << _params(11) << std::endl;
    } else {
        throw std::logic_error(
            "OdometryDisplacementModel invalid type");
    }
}

}

