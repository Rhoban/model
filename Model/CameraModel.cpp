#include "Model/CameraModel.hpp"

namespace Leph {

CameraModel::CameraModel()
  : focalX(-1), focalY(-1), centerX(-1), centerY(-1),
    radialCoeffs(Eigen::Vector3d::Zeros()),
    tangentialCoeffs(Eigen::Vector2d::Zeros())
{
}

cv::Mat CameraModel::getCameraMatrix() const
{
  return (Mat_<double>(3, 3) << focalX, 0, centerX, 0, focalY, centerY, 0, 0, 1);
}

cv::Mat CameraModel::getDistortionCoeffs() const
{
  return
    (Mat_<double>(5, 1) <<
     radialCoeffs(0), radialCoeffs(1),
     tangentialCoeffs(0), tangentialCoeffs(1),
     radialCoeffs(2)
    );
}

cv::Point2f CameraModel::toCorrectedImg(const cv::Point2f & imgPosUncorrected) const
{
  cv::Mat inputPoints(1,1,CV_32FC2,0.0);
  cv::Mat outputPoints(1,1,CV_32FC2,0.0);
  float * ptr;
  ptr =  inputPoints.ptr<float>(0);
  ptr[0] = imgPosUncorrected(0);
  ptr[1] = imgPosUncorrected(1);
  cv::undistortPoints(inputPoints, outputPoints, getCameraMatrix(), getDistortionCoeffs());
  ptr =  outputPoints.ptr<float>(0);
  return cv::Point2f(ptr[0], ptr[1]);
}

cv::Point2f CameraModel::toUncorrectedImg(const cv::Point2f & imgPosCorrected) const
{
  double k1, k2, k3, p1, p2;
  k1 = -1 *radialCoeffs(0);
  k2 = -1 *radialCoeffs(1);
  k3 = -1 *radialCoeffs(2);
  p1 = -1 *tangentialCoeffs(0);
  p2 = -1 *tangentialCoeffs(1);
  double x = (imgPosCorrected.x - centerX) / focalX;
  double y = (imgPosCorrected.y - centerY) / focalY;

  double r2 = x*x + y*y;

  double xDistort, yDistort;
  // Applying radial distortion
  xDistort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  yDistort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  // Applying tangential distortion
  xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
  yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

  xDistort = xDistort * focalX + centerX;
  yDistort = yDistort * focalY + centerY;

  return cv::Point2f(xDistort, yDistort);
}

}
