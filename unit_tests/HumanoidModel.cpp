#include <Model/HumanoidModel.hpp>

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

TEST(modelLoader, testSuccess)
{
    HumanoidModel humanoidModel(getAbsoluteURDFFilePath(), RobotType::SigmabanModel, "trunk");
    /// Test default position of the camera
    Eigen::Vector3d cameraPos = humanoidModel.position("camera","origin");
    EXPECT_FLOAT_EQ(cameraPos.x(), 0.05);//Camera is slightly in front of the pitch motor
    EXPECT_FLOAT_EQ(cameraPos.y(), 0.0);
    EXPECT_FLOAT_EQ(cameraPos.z(), 0.3);//With respect to the trunk
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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
