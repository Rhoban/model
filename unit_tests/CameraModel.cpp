#include <Model/CameraModel.hpp>

#include <gtest/gtest.h>

using namespace std;
using namespace Leph;

string getAbsoluteTestFilePath() {
    string filePath = __FILE__;
    string currentDirPath = filePath.substr(0, filePath.rfind("/"));
    return currentDirPath + "/../Data/cameraModel.json";
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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
