#include <Model/HumanoidModel.hpp>
#include <Model/HumanoidFloatingModel.hpp>

#include <gtest/gtest.h>

using namespace std;
using namespace Leph;

string getAbsoluteURDFFilePath() {
    string filePath = __FILE__;
    string currentDirPath = filePath.substr(0, filePath.rfind("/"));
    // This humanoid has simplified dimensions to make tests easier
    return currentDirPath + "/../Data/humanoidTest.urdf";
}

/// Depends on cameraModel test
string getAbsoluteCameraModelFilePath() {
    string filePath = __FILE__;
    string currentDirPath = filePath.substr(0, filePath.rfind("/"));
    // This humanoid has simplified dimensions to make tests easier
    return currentDirPath + "/../Data/cameraModelTest.json";
}

double trunkHeight = 0.3;//[m]
Eigen::Vector3d expectedCameraPos(0.05, 0.0, 0.6);//[m]

// Position when head_yaw = pi/2
Eigen::Vector3d expectedCameraPosLeft(0.0, 0.05, 0.6);//[m]

double tol = std::pow(10,-3);

TEST(modelLoader, testSuccess)
{
  HumanoidModel humanoidModel(getAbsoluteURDFFilePath(), RobotType::SigmabanModel, "trunk");
    /// Test default position of the camera
    Eigen::Vector3d cameraPos = humanoidModel.position("camera","origin");
    EXPECT_FLOAT_EQ(cameraPos.x(), expectedCameraPos.x());
    EXPECT_FLOAT_EQ(cameraPos.y(), expectedCameraPos.y());
    EXPECT_FLOAT_EQ(cameraPos.z(), expectedCameraPos.z() - trunkHeight);
}

TEST(putOnGround, testSuccess)
{
    HumanoidFloatingModel humanoidModel(getAbsoluteURDFFilePath(), RobotType::SigmabanModel);
    humanoidModel.putOnGround();
    /// Test default position of the camera
    Eigen::Vector3d cameraPos = humanoidModel.position("camera","origin");
    EXPECT_FLOAT_EQ(cameraPos.x(), expectedCameraPos.x());
    EXPECT_FLOAT_EQ(cameraPos.y(), expectedCameraPos.y());
    EXPECT_FLOAT_EQ(cameraPos.z(), expectedCameraPos.z());
}

TEST(cameraPixelToViewVector, testSuccess)
{
    // Declaring test setup
    HumanoidModel humanoidModel(getAbsoluteURDFFilePath(), RobotType::SigmabanModel, "trunk");
    CameraModel cameraModel;
    cameraModel.loadFile(getAbsoluteCameraModelFilePath());
    Eigen::Vector2d pixel;
    Eigen::Vector3d viewVector;
    /// Test viewVector for image center
    pixel(0) = cameraModel.getCenterX();
    pixel(1) = cameraModel.getCenterY();
    viewVector = humanoidModel.cameraPixelToViewVector(cameraModel, pixel);
    EXPECT_NEAR(viewVector.x(), 1, 0.001);
    EXPECT_NEAR(viewVector.y(), 0, 0.001);
    EXPECT_NEAR(viewVector.z(), 0, 0.001);
}

TEST(cameraWorldToPixel, testSuccess)
{
    HumanoidFloatingModel humanoidModel(getAbsoluteURDFFilePath(), RobotType::SigmabanModel);
    humanoidModel.putOnGround();
    CameraModel cameraModel;
    cameraModel.loadFile(getAbsoluteCameraModelFilePath());
    // Test variables
    Eigen::Vector3d posInWorld;
    Eigen::Vector2d posInImg;
    bool success;
    // Test in front of camera
    posInWorld = Eigen::Vector3d(expectedCameraPos.x() + 1.0,
                                 expectedCameraPos.y(),
                                 expectedCameraPos.z());
    success = humanoidModel.cameraWorldToPixel(cameraModel, posInWorld, posInImg);
    EXPECT_TRUE(success);
    EXPECT_NEAR(posInImg.x(), cameraModel.getCenterX(), 0.1);
    EXPECT_NEAR(posInImg.y(), cameraModel.getCenterY(),0.1);
    // Test offset y in image
    posInWorld = Eigen::Vector3d(expectedCameraPos.x() + cameraModel.getFocalDist(),
                                 expectedCameraPos.y(),
                                 expectedCameraPos.z() - cameraModel.getImgHeight() / 4.);
    success = humanoidModel.cameraWorldToPixel(cameraModel, posInWorld, posInImg);
    EXPECT_TRUE(success);
    EXPECT_NEAR(posInImg.x(), cameraModel.getCenterX(), 0.1);
    EXPECT_NEAR(posInImg.y(), cameraModel.getCenterY() + cameraModel.getImgHeight() / 4., 0.1);
}

TEST(cameraPixelToPanTilt, testSuccess)
{
    HumanoidFloatingModel humanoidModel(getAbsoluteURDFFilePath(), RobotType::SigmabanModel);
    humanoidModel.putOnGround();
    CameraModel cameraModel;
    cameraModel.loadFile(getAbsoluteCameraModelFilePath());
    Eigen::Vector2d pixel;
    Eigen::Vector2d panTilt;
    // Test in front of camera
    pixel = Eigen::Vector2d(cameraModel.getCenterX(), cameraModel.getCenterY());
    panTilt = humanoidModel.cameraPixelToPanTilt(cameraModel, pixel);
    EXPECT_NEAR(panTilt(0), 0, 0.01);
    EXPECT_NEAR(panTilt(1), 0, 0.01);
    // Test offset y in image -> ~45°
    pixel = Eigen::Vector2d(cameraModel.getCenterX(),
                            cameraModel.getCenterY() + cameraModel.getFocalDist());
    panTilt = humanoidModel.cameraPixelToPanTilt(cameraModel, pixel);
    EXPECT_NEAR(panTilt(0), 0, 0.01);
    EXPECT_NEAR(panTilt(1), M_PI/4, 0.01);
    // Test robot looking down°
    humanoidModel.setDOF("head_pitch", M_PI/2);
    pixel = Eigen::Vector2d(cameraModel.getCenterX(), cameraModel.getCenterY());
    panTilt = humanoidModel.cameraPixelToPanTilt(cameraModel, pixel);
    EXPECT_NEAR(panTilt(1), M_PI/2, 0.01);
    // Test robot looking down and offset in image y
    humanoidModel.setDOF("head_pitch", M_PI/2);
    pixel = Eigen::Vector2d(cameraModel.getCenterX(),
                            cameraModel.getCenterY() - cameraModel.getFocalDist());
    panTilt = humanoidModel.cameraPixelToPanTilt(cameraModel, pixel);
    EXPECT_NEAR(panTilt(0), 0, 0.01);
    EXPECT_NEAR(panTilt(1), M_PI/4, 0.01);
    // Test robot looking down and offset in image x
    humanoidModel.setDOF("head_pitch", M_PI/2);
    pixel = Eigen::Vector2d(cameraModel.getCenterX() + cameraModel.getFocalDist(),
                            cameraModel.getCenterY());
    panTilt = humanoidModel.cameraPixelToPanTilt(cameraModel, pixel);
    EXPECT_NEAR(panTilt(0), -M_PI/2, 0.01);
    EXPECT_NEAR(panTilt(1), M_PI/4, 0.01);
}

TEST(cameraViewVectorToBallWorld, testSuccess)
{
    // Declaring test setup
    HumanoidFloatingModel humanoidModel(getAbsoluteURDFFilePath(), RobotType::SigmabanModel);
    CameraModel cameraModel;
    cameraModel.loadFile(getAbsoluteCameraModelFilePath());
    humanoidModel.putOnGround();
    Eigen::Vector2d pixel;
    Eigen::Vector3d ballCenter;
    Eigen::Vector3d viewVector;
    std::vector<Eigen::Vector3d> ballBorders;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> ballBordersPix;
    // Simple test: ball in front, view down at 45 °
    double ballRadius = 0.05;// 0.1 [m] of diameter for the ball
    viewVector = Eigen::Vector3d(1,0,-1);
    humanoidModel.cameraViewVectorToBallWorld(cameraModel, viewVector, ballRadius,
                                              ballCenter, &pixel,
                                              &ballBordersPix, &ballBorders);
    EXPECT_NEAR(ballCenter.x(), expectedCameraPos.x() + expectedCameraPos.z() - ballRadius, 0.001);
    EXPECT_NEAR(ballCenter.y(), expectedCameraPos.y(), 0.001);
    EXPECT_NEAR(ballCenter.z(), ballRadius, 0.001);
    EXPECT_NEAR(pixel.x(), cameraModel.getCenterX(), 0.1);
    EXPECT_NEAR(pixel.y(), cameraModel.getCenterY() + cameraModel.getFocalDist(), 0.1);
}

TEST(getTransform, initialPosition)
{
  HumanoidFloatingModel model(getAbsoluteURDFFilePath(), RobotType::SigmabanModel);
  model.putOnGround();
  Eigen::Vector3d expectedPos, receivedPos;
  Eigen::Affine3d camera2origin = model.getTransform("origin","camera");
  // Test default position of the camera
  receivedPos = camera2origin * Eigen::Vector3d(0,0,0);
  expectedPos = expectedCameraPos;
  for (int d = 0; d < 3; d++) {
    EXPECT_NEAR(expectedPos(d), receivedPos(d), tol);
  }
  // Test another point in front of the camera
  receivedPos = camera2origin * Eigen::Vector3d(0,0,1);
  expectedPos = expectedCameraPos + Eigen::Vector3d(1,0,0);
  for (int d = 0; d < 3; d++) {
    EXPECT_NEAR(expectedPos(d), receivedPos(d), tol);
  }
}

TEST(getTransform, lookingLeft)
{
  HumanoidFloatingModel model(getAbsoluteURDFFilePath(), RobotType::SigmabanModel);
  model.putOnGround();
  model.setDOF("head_yaw", M_PI/2);
  Eigen::Vector3d expectedPos, receivedPos;
  Eigen::Affine3d camera2origin = model.getTransform("origin","camera");
  // Test default position of the camera
  receivedPos = camera2origin * Eigen::Vector3d(0,0,0);
  expectedPos = expectedCameraPosLeft;
  for (int d = 0; d < 3; d++) {
    EXPECT_NEAR(expectedPos(d), receivedPos(d), tol);
  }
  // Test another point in front of the camera
  receivedPos = camera2origin * Eigen::Vector3d(0,0,1);
  expectedPos = expectedCameraPosLeft + Eigen::Vector3d(0,1,0);
  for (int d = 0; d < 3; d++) {
    EXPECT_NEAR(expectedPos(d), receivedPos(d), tol);
  }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
