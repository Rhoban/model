#include <fstream>
#include <stdexcept>
#include <Utils/Angle.h>
#include <Utils/FileEigen.h>
#include "Odometry/Odometry.hpp"


namespace Leph {
Odometry::Odometry()
  : Odometry(OdometryDisplacementModel::Type::DisplacementIdentity)
{}
        
Odometry::Odometry(
    OdometryDisplacementModel::Type typeDisplacement,
    OdometryNoiseModel::Type typeNoise) :
    _modelDisplacement(typeDisplacement),
    _modelNoise(typeNoise),
    _isInitialized(false),
    _support(),
    _last(),
    _state(),
    _corrected(),
    _lastDiff()
{
    //Ask reset
    reset();
}

OdometryDisplacementModel::Type Odometry::getDisplacementType() const
{
    return _modelDisplacement.getType();
}
OdometryNoiseModel::Type Odometry::getNoiseType() const
{
    return _modelNoise.getType();
}
        
Eigen::VectorXd Odometry::getParameters() const
{
    size_t sizeDisplacement = 
        _modelDisplacement.getParameters().size();
    size_t sizeNoise = 
        _modelNoise.getParameters().size();

    Eigen::VectorXd params(
        sizeDisplacement+sizeNoise);
    if (sizeDisplacement > 0) {
        params.segment(0, sizeDisplacement) = 
            _modelDisplacement.getParameters();
    }
    if (sizeNoise > 0) {
        params.segment(sizeDisplacement, sizeNoise) = 
            _modelNoise.getParameters();
    }

    return params;
}

double Odometry::setParameters(
    const Eigen::VectorXd& params)
{
    size_t sizeDisplacement = 
        _modelDisplacement.getParameters().size();
    size_t sizeNoise = 
        _modelNoise.getParameters().size();

    if (
        (size_t)params.size() != 
        sizeDisplacement+sizeNoise
    ) {
        throw std::logic_error(
            "Odometry invalid parameters size: "
            + std::to_string(sizeDisplacement)
            + std::string("+")
            + std::to_string(sizeNoise)
            + std::string("!=")
            + std::to_string(params.size()));
    }

    double error = 0.0;
    if (sizeDisplacement > 0) {
        error += _modelDisplacement.setParameters(
            params.segment(0, sizeDisplacement));
    }
    if (sizeNoise > 0) {
        error += _modelNoise.setParameters(
            params.segment(sizeDisplacement, sizeNoise));
    }
    return error;
}

std::vector<std::string> Odometry::getParametersNames() const
{
  std::vector<std::string> displacementNames, noiseNames, result;
  displacementNames = _modelDisplacement.getParametersNames();
  noiseNames = _modelNoise.getParametersNames();
  std::cout << "Nb displacements params: " << displacementNames.size() << std::endl;
  std::cout << "Nb noise params: " << noiseNames.size() << std::endl;
  // Adding displacement names (with prefix)
  for (const std::string & name : displacementNames) {
    result.push_back("displacement_" + name);
  }
  // Adding noise names (with prefix)
  for (const std::string & name : noiseNames) {
    result.push_back("noise_" + name);
  }
  return result;
}
        
Eigen::VectorXd Odometry::getNormalization() const
{
    size_t sizeDisplacement = 
        _modelDisplacement.getNormalization().size();
    size_t sizeNoise = 
        _modelNoise.getNormalization().size();

    Eigen::VectorXd coefs(
        sizeDisplacement+sizeNoise);
    if (sizeDisplacement > 0) {
        coefs.segment(0, sizeDisplacement) = 
            _modelDisplacement.getNormalization();
    }
    if (sizeNoise > 0) {
        coefs.segment(sizeDisplacement, sizeNoise) = 
            _modelNoise.getNormalization();
    }

    return coefs;
}
        
void Odometry::printParameters() const
{
    _modelDisplacement.printParameters();
    _modelNoise.printParameters();
}

void Odometry::reset()
{
    _isInitialized = false;
    _state = Eigen::Vector3d(0.0, 0.0, 0.0);
    _corrected = Eigen::Vector3d(0.0, 0.0, 0.0);
    _lastDiff = Eigen::Vector3d(0.0, 0.0, 0.0);
}
void Odometry::reset(const Eigen::Vector3d& pose)
{
    _isInitialized = false;
    _state = pose;
    _corrected = pose;
    _lastDiff = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void Odometry::update(
    const Eigen::Vector3d& current, 
    Leph::HumanoidFixedModel::SupportFoot supportFoot,
    std::default_random_engine* engine)
{
    if (!_isInitialized) {
        _support = supportFoot;
        _last = current;
        _isInitialized = true;
    }

    //Retrieve current model state
    Leph::HumanoidFixedModel::SupportFoot lastSupport = _support;
    _support = supportFoot;
    
    //Compute displacement between current Model 
    //state and last support swap
    Eigen::Vector3d diff = odometryDiff(_last, current);
    //Apply displacement correction
    diff = _modelDisplacement.displacementCorrection(diff);

    //Update corrected odometry at support swap
    if (lastSupport != _support) {
        //Apply noise generation if available
        if (engine != nullptr) {
            diff += _modelNoise.noiseGeneration(diff, *engine);
        }
        //Save applied delta
        _lastDiff = diff;
        //Integrate the displacement at corrected odometry state
        odometryInt(diff, _state);
        //Save current Model state
        _last = current;
        //Update corrected state
        _corrected = _state;
    } else {
        _corrected = _state;
        //Integrate the displacement at corrected odometry state
        odometryInt(diff, _corrected);
    }
}
void Odometry::update(
    HumanoidFixedModel& model,
    std::default_random_engine* engine)
{
    update(
        model.get().getPose(), 
        model.getSupportFoot(),
        engine);
}
        
void Odometry::updateFullStep(
    const Eigen::Vector3d& deltaPose,
    std::default_random_engine* engine)
{
    if (!_isInitialized) {
        _support = Leph::HumanoidFixedModel::LeftSupportFoot;
        _last = _state;
        _isInitialized = true;
    }

    //Apply displacement correction
    Eigen::Vector3d diff = getDiffFullStep(deltaPose, engine);
    //Save applied delta
    _lastDiff = deltaPose;
    
    //Integrate current state with given full 
    //step relative displacement
    _last = _state;
    odometryInt(diff, _state);
    _corrected = _state;
}

Eigen::Vector3d Odometry::getDiffFullStep(
  const Eigen::Vector3d & deltaPose,
  std::default_random_engine * engine) const
{
  //Apply displacement correction
  Eigen::Vector3d diff = _modelDisplacement.displacementCorrection(deltaPose);
  //Apply noise generation if available
  if (engine != nullptr) {
    diff += _modelNoise.noiseGeneration(diff, *engine);
  }
  return diff;
}

const Eigen::Vector3d& Odometry::state() const
{
    return _corrected;
}

Eigen::Vector3d Odometry::odometryDiff(
    const Eigen::Vector3d& state1, 
    const Eigen::Vector3d& state2) const
{
    //Vector in world
    double vectX = state2.x() - state1.x();
    double vectY = state2.y() - state1.y();
    double angle = Leph::AngleDistance(state1.z(), state2.z()); 
    //Rotation to source frame
    double vectInSrcX = vectX*cos(-state1.z()) - vectY*sin(-state1.z());
    double vectInSrcY = vectX*sin(-state1.z()) + vectY*cos(-state1.z());

    return Eigen::Vector3d(vectInSrcX, vectInSrcY, angle);
}

void Odometry::odometryInt(
    const Eigen::Vector3d& diff,
    Eigen::Vector3d& state) const
{
    //Rotation to world frame
    double vectX = diff.x()*cos(state.z()) - diff.y()*sin(state.z());
    double vectY = diff.x()*sin(state.z()) + diff.y()*cos(state.z());
    //Integration
    state.x() += vectX;
    state.y() += vectY;
    state.z() += diff.z();
    //Shrink to -PI,PI
    state.z() = AngleBound(state.z());
}

void Odometry::saveToFile(const std::string & path) const
{
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + path);
    }
    file << (int)getDisplacementType() << " " 
         << (int)getNoiseType() << std::endl;
    Leph::WriteEigenVectorToStream(file, getParameters());
    file.close();
}

void Odometry::loadFromFile(const std::string & path)
{
    //Open file
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + path);
    }
    //Read data
    int tmpTypeDisplacement;
    int tmpTypeNoise;
    file >> tmpTypeDisplacement;
    file >> tmpTypeNoise;
    OdometryDisplacementModel::Type typeDisplacement = 
        (OdometryDisplacementModel::Type)tmpTypeDisplacement;
    OdometryNoiseModel::Type typeNoise = 
        (OdometryNoiseModel::Type)tmpTypeNoise;
    Eigen::VectorXd initParams = ReadEigenVectorFromStream(file);
    file.close();
    //Check bounds
    
    _modelDisplacement = OdometryDisplacementModel(typeDisplacement);
    _modelNoise = OdometryNoiseModel(typeNoise);
    reset();
    
    double isError = setParameters(initParams);
    if (isError > 0.0) {
        std::ostringstream oss;
        oss << "Odometry parameters are out of bounds: " 
            << isError << std::endl;
        throw std::runtime_error(oss.str());
    }
}

}

