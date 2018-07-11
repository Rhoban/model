#include <Model/CameraModel.hpp>

#include <gtest/gtest.h>

using namespace std;
using namespace Leph;

string getAbsoluteTestFilePath() {
    string filePath = __FILE__;
    string currentDirPath = filePath.substr(0, filePath.rfind("/"));
    return currentDirPath + "/../Data/cameraModelTest.json";
}

/// JsonLoader will be used for 
TEST(jsonLoader, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());
  
  EXPECT_EQ(cameraModel.getImgWidth(), 800);
  EXPECT_EQ(cameraModel.getImgHeight(), 600);
  EXPECT_EQ(cameraModel.getCenterX(), 400);
  EXPECT_EQ(cameraModel.getCenterY(), 300);
  EXPECT_EQ(cameraModel.getFocalX(), 600);
  EXPECT_EQ(cameraModel.getFocalY(), 600);
}

TEST(getFOV, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  // Hand computed values
  rhoban_utils::Angle fovX = cameraModel.getFOVX();
  rhoban_utils::Angle fovY = cameraModel.getFOVY();
  EXPECT_NEAR(fovX.getSignedValue(), 67.38, 0.1);
  EXPECT_NEAR(fovY.getSignedValue(), 53.13, 0.1);
}

TEST(getCameraMatrix, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Mat cameraMatrix = cameraModel.getCameraMatrix();
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(0,0), cameraModel.getFocalX());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(0,1), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(0,2), cameraModel.getCenterX());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(1,0), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(1,1), cameraModel.getFocalY());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(1,2), cameraModel.getCenterY());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(2,0), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(2,1), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(2,2), 1.0);
}

TEST(getDistortionCoeffs, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Mat distortionCoeffs = cameraModel.getDistortionCoeffs();
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(0,0), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(1,0), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(2,0), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(3,0), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(4,0), 0.0);
}

TEST(containsPixel, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point2f in1(0,0);
  cv::Point2f in2(600,500);
  cv::Point2f out1(-2, 50);
  cv::Point2f out2(2, -50);
  cv::Point2f out3(2, 602);
  cv::Point2f out4(802, 5);
  cv::Point2f out5(802, 602);
  cv::Point2f out6(-2, -20);
  
  EXPECT_TRUE(cameraModel.containsPixel(in1));
  EXPECT_TRUE(cameraModel.containsPixel(in2));
  EXPECT_FALSE(cameraModel.containsPixel(out1));
  EXPECT_FALSE(cameraModel.containsPixel(out2));
  EXPECT_FALSE(cameraModel.containsPixel(out3));
  EXPECT_FALSE(cameraModel.containsPixel(out4));
  EXPECT_FALSE(cameraModel.containsPixel(out5));
  EXPECT_FALSE(cameraModel.containsPixel(out6));
}

TEST(toCorrectedImg, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point2f uncorrectedPos, correctedPos;
  // Case 1: center, no deformation
  uncorrectedPos.x = cameraModel.getCenterX();
  uncorrectedPos.y = cameraModel.getCenterY();
  correctedPos = cameraModel.toCorrectedImg(uncorrectedPos);
  EXPECT_FLOAT_EQ(uncorrectedPos.x, correctedPos.x);
  EXPECT_FLOAT_EQ(uncorrectedPos.y, correctedPos.y);
  // Case 2: border, no deformation
  uncorrectedPos = cv::Point2f(0,0);
  correctedPos = cameraModel.toCorrectedImg(uncorrectedPos);
  EXPECT_NEAR(uncorrectedPos.x, correctedPos.x, 0.001);
  EXPECT_NEAR(uncorrectedPos.y, correctedPos.y, 0.001);
  // Case 2: assymetric location, no deformation
  uncorrectedPos.x = cameraModel.getCenterX() * 1.5;
  uncorrectedPos.y = cameraModel.getCenterY() * 1.2;
  correctedPos = cameraModel.toCorrectedImg(uncorrectedPos);
  EXPECT_FLOAT_EQ(uncorrectedPos.x, correctedPos.x);
  EXPECT_FLOAT_EQ(uncorrectedPos.y, correctedPos.y);
}

TEST(getImgFromObject, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point3f objectPos;
  cv::Point2f imgPos;

  // First simple test: centered pixel should be at center of image
  objectPos = cv::Point3f(0,0,1);
  imgPos = cameraModel.getImgFromObject(objectPos);
  EXPECT_FLOAT_EQ(imgPos.x, cameraModel.getCenterX());
  EXPECT_FLOAT_EQ(imgPos.y, cameraModel.getCenterY());
  // Second test: zero in z throws runtime_error
  try {
    objectPos = cv::Point3f(0,0,0);
    imgPos = cameraModel.getImgFromObject(objectPos);
    EXPECT_TRUE(false);
  } catch (const std::runtime_error & exc) {
    EXPECT_TRUE(true);
  }
  // Third test: negative value in z throws runtime_error
  try {
    objectPos = cv::Point3f(0,0,-1);
    imgPos = cameraModel.getImgFromObject(objectPos);
    EXPECT_TRUE(false);
  } catch (const std::runtime_error & exc) {
    EXPECT_TRUE(true);
  }
}

TEST(getViewVectorFromImg, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point2f imgPos;
  cv::Point3f viewVector;
  // Case 1: Center of img
  imgPos = cv::Point2f(cameraModel.getCenterX(),cameraModel.getCenterY());
  viewVector = cameraModel.getViewVectorFromImg(imgPos);
  EXPECT_FLOAT_EQ(viewVector.x, 0.0);
  EXPECT_FLOAT_EQ(viewVector.y, 0.0);
  EXPECT_FLOAT_EQ(viewVector.z, 1.0);
  // Case 2: Only y value -> value corresponding to a 30 deg angle
  imgPos.x = cameraModel.getCenterX();
  imgPos.y = cameraModel.getCenterY();
  imgPos.y += cameraModel.getFocalY() * tan(M_PI / 6);
  viewVector = cameraModel.getViewVectorFromImg(imgPos);
  EXPECT_FLOAT_EQ(viewVector.x, 0.0);
  EXPECT_FLOAT_EQ(viewVector.y, 0.5);
  EXPECT_FLOAT_EQ(viewVector.z, std::sqrt(3)/2);
  // Case 3: Only x value
  imgPos.x = cameraModel.getCenterX() + cameraModel.getFocalX();
  imgPos.y = cameraModel.getCenterY();
  viewVector = cameraModel.getViewVectorFromImg(imgPos);
  EXPECT_FLOAT_EQ(viewVector.x, std::sqrt(2)/2);
  EXPECT_FLOAT_EQ(viewVector.y, 0.0);
  EXPECT_FLOAT_EQ(viewVector.z, std::sqrt(2)/2);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
