#include "Utils/EncoderFilter.hpp"

namespace Leph
{
EncoderFilter::EncoderFilter()
{
  _state = Eigen::Vector3d::Zero();
  _error = Eigen::Matrix3d::Identity();
  _predictionNoise = Eigen::Matrix3d::Identity();
  _observationNoise = 1.0;
}
EncoderFilter::EncoderFilter(double pos, double vel, double acc, double var)
{
  _state = Eigen::Vector3d(pos, vel, acc);
  _error = var * Eigen::Matrix3d::Identity();
  _predictionNoise = Eigen::Matrix3d::Identity();
  _observationNoise = 1.0;
}
EncoderFilter::EncoderFilter(double pos, double vel, double acc, double posVar, double velVar, double accVar)
{
  _state = Eigen::Vector3d(pos, vel, acc);
  _error << posVar, 0.0, 0.0, 0.0, velVar, 0.0, 0.0, 0.0, accVar;
  _predictionNoise = Eigen::Matrix3d::Identity();
  _observationNoise = 1.0;
}

void EncoderFilter::setObservationVar(double noise)
{
  _observationNoise = noise;
}
void EncoderFilter::setPredictionVar(double noise)
{
  _predictionNoise = noise * Eigen::Matrix3d::Identity();
}
void EncoderFilter::setPredictionVar(const Eigen::Vector3d& noise)
{
  _predictionNoise << noise(0), 0.0, 0.0, 0.0, noise(1), 0.0, 0.0, 0.0, noise(2);
}
void EncoderFilter::setPredictionVar(const Eigen::Matrix3d& noise)
{
  _predictionNoise = noise;
}

void EncoderFilter::setPos(double val)
{
  _state(0) = val;
}
void EncoderFilter::setVel(double val)
{
  _state(1) = val;
}
void EncoderFilter::setAcc(double val)
{
  _state(2) = val;
}

double EncoderFilter::posVar() const
{
  return _error(0, 0);
}
double EncoderFilter::velVar() const
{
  return _error(1, 1);
}
double EncoderFilter::accVar() const
{
  return _error(2, 2);
}
const Eigen::Matrix3d& EncoderFilter::var() const
{
  return _error;
}

double EncoderFilter::pos() const
{
  return _state(0);
}
double EncoderFilter::vel() const
{
  return _state(1);
}
double EncoderFilter::acc() const
{
  return _state(2);
}

void EncoderFilter::update(double dt, double measurementPos)
{
  Eigen::Matrix3d predictionModel;
  predictionModel << 1.0, dt, dt * dt / 2.0, 0.0, 1.0, dt, 0.0, 0.0, 1.0;

  Eigen::Matrix<double, 1, 3> observationModel;
  observationModel << 1.0, 0.0, 0.0;

  doKalmanFiltering(predictionModel, Eigen::Matrix3d::Zero(), observationModel, measurementPos,
                    Eigen::Vector3d::Zero());
}
void EncoderFilter::update(double dt, double measurementPos, double goalVel, double goalAcc)
{
  Eigen::Matrix3d predictionModel;
  predictionModel << 1.0, dt, dt * dt / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // TODO more parameters to control the
  // merging of velocity/acceleration goal and
  // predicted information
  Eigen::Matrix3d controlModel;
  controlModel << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix<double, 1, 3> observationModel;
  observationModel << 1.0, 0.0, 0.0;

  doKalmanFiltering(predictionModel, controlModel, observationModel, measurementPos,
                    Eigen::Vector3d(0.0, goalVel, goalAcc));
}

void EncoderFilter::doKalmanFiltering(const Eigen::Matrix3d& predictionModel, const Eigen::Matrix3d& controlModel,
                                      const Eigen::Matrix<double, 1, 3>& observationModel, double measurement,
                                      const Eigen::Vector3d& control)
{
  // Prediction
  Eigen::Vector3d aprioriState = predictionModel * _state + controlModel * control;
  Eigen::Matrix3d aprioriError = predictionModel * _error * predictionModel.transpose() + _predictionNoise;
  // Update
  double residual = measurement - observationModel * aprioriState;
  double covariance = observationModel * aprioriError * observationModel.transpose() + _observationNoise;
  Eigen::Vector3d gain = (1.0 / covariance) * aprioriError * observationModel.transpose();
  _state = aprioriState + gain * residual;
  _error = aprioriError - gain * observationModel * aprioriError;
}

}  // namespace Leph
