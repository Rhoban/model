#include "Model/CameraModel.hpp"

#include <rhoban_utils/util.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace Leph {

Eigen::Vector2d cv2Eigen(const cv::Point2f & p)
{
  return Eigen::Vector2d(p.x,p.y);
}

Eigen::Vector3d cv2Eigen(const cv::Point3f & p)
{
  return Eigen::Vector3d(p.x,p.y,p.z);
}

cv::Point2f eigen2CV(const Eigen::Vector2d & p)
{
  return cv::Point2f(p.x(),p.y());
}

cv::Point3f eigen2CV(const Eigen::Vector3d & p)
{
  return cv::Point3f(p.x(),p.y(),p.z());
}

CameraModel::CameraModel()
  : imgWidth(-1), imgHeight(-1),
    focalX(-1), focalY(-1), centerX(-1), centerY(-1),
    radialCoeffs(Eigen::Vector3d::Zero()),
    tangentialCoeffs(Eigen::Vector2d::Zero())
{
}

int CameraModel::getImgWidth() const {
  return imgWidth;
}

int CameraModel::getImgHeight() const {
  return imgHeight;
}

bool CameraModel::containsPixel(const cv::Point2f & imgPos) const {
  bool xOk = imgPos.x >= 0 && imgPos.x < imgWidth;
  bool yOk = imgPos.y >= 0 && imgPos.y <= imgHeight;
  return xOk && yOk;
}

double CameraModel::getFocalDist() const {
  return (focalX + focalY) / 2;
}


cv::Mat CameraModel::getCameraMatrix() const
{
  return (cv::Mat_<double>(3, 3) << focalX, 0, centerX, 0, focalY, centerY, 0, 0, 1);
}

cv::Mat CameraModel::getDistortionCoeffs() const
{
  return
    (cv::Mat_<double>(5, 1) <<
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
  ptr[0] = imgPosUncorrected.x;
  ptr[1] = imgPosUncorrected.y;
  cv::undistortPoints(inputPoints, outputPoints, getCameraMatrix(), getDistortionCoeffs());
  ptr =  outputPoints.ptr<float>(0);
  return cv::Point2f(ptr[0], ptr[1]);
}

cv::Point2f CameraModel::toUncorrectedImg(const cv::Point2f & imgPosCorrected) const
{
  // Convert the pixel format to something usable by 
  cv::Point3f normalized((imgPosCorrected.x - centerX) / focalX,
                         (imgPosCorrected.y - centerY) / focalY,
                         1.0);
  std::vector<cv::Point2f> distorted;
  std::vector<cv::Point3f> undistorted = {normalized};
  cv::projectPoints(undistorted,cv::Mat::zeros(3, 1, CV_64FC1),
                    cv::Mat::zeros(3, 1, CV_64FC1),
                    getCameraMatrix(), getDistortionCoeffs(), distorted);

  return distorted[0];
}

cv::Point2f CameraModel::getImgFromObject(const cv::Point3f & objectPosition,
                                          bool outputInCorrectedImg) const
{
  if (objectPosition.z <= 0.0) {
    throw std::runtime_error(DEBUG_INFO + " invalid object position: z="
                             + std::to_string(objectPosition.z));
  }

  double ratio = - objectPosition.z / getFocalDist();
  double px = ratio * objectPosition.x + centerX;
  double py = ratio * objectPosition.y + centerY;
  cv::Point2f posInCorrected(px,py);
  if (outputInCorrectedImg) {
    return posInCorrected;
  }
  return toUncorrectedImg(posInCorrected);
}

cv::Point3f CameraModel::getViewVectorFromImg(const cv::Point2f & imgPos,
                                              bool inputInCorrectedImg) const
{
  cv::Point2f correctedPos = imgPos;
  if (!inputInCorrectedImg) {
    correctedPos = toCorrectedImg(imgPos);
  }
  double dX = (centerX - correctedPos.x);
  double dY = (centerY - correctedPos.y);
  double dZ = (focalX + focalY) / 2;// Both should be rather equivalent
  double length = std::sqrt(dX * dX + dY * dY + dZ * dZ);
  return cv::Point3f(dX / length, dY / length, dZ / length);
}

//cv::Point3f CameraModel::getObjectPosFromImgAndPlan(const cv::Point2f & imgPos,
//                                                    cv::Point4f planEquation,
//                                                    bool inputInCorrectedImg) const
//{
//  throw std::logic_error(DEBUG_INFO + "not implemented");
//}

void CameraModel::fromJson(const Json::Value & v,
                           const std::string & dir_name)
{
  (void) dir_name;
  rhoban_utils::tryRead(v, "imgWidth" , &imgWidth );
  rhoban_utils::tryRead(v, "imgHeight", &imgHeight);
  rhoban_utils::tryRead(v, "focalX"   , &focalX   );
  rhoban_utils::tryRead(v, "focalY"   , &focalY   );
  rhoban_utils::tryRead(v, "centerX"  , &centerX  );
  rhoban_utils::tryRead(v, "centerY"  , &centerY  );
  rhoban_utils::tryReadEigen(v, "radialCoeffs"    , &radialCoeffs    );
  rhoban_utils::tryReadEigen(v, "tangentialCoeffs", &tangentialCoeffs);
}

Json::Value CameraModel::toJson() const
{
  Json::Value v;
  v["imgWidth" ] = imgWidth ;
  v["imgHeight"] = imgHeight;
  v["focalX"   ] = focalX   ;
  v["focalY"   ] = focalY   ;
  v["centerX"  ] = centerX  ;
  v["centerY"  ] = centerY  ;
  v["radialCoeffs"    ] = rhoban_utils::vector2Json(radialCoeffs    );
  v["tangentialCoeffs"] = rhoban_utils::vector2Json(tangentialCoeffs);
  return v;
}
std::string CameraModel::getClassName() const
{
  return "CameraModel";
}

}
