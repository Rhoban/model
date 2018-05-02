#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Utils/Euler.h"
#include "Utils/AxisAngle.h"

#include "Utils/Euler.h"

int main()
{
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);

    Leph::ModelViewer viewer(1200, 900);
    viewer.frameLength = 0.02;
    
    Leph::CartWalkProxy walk;
    Leph::VectorLabel params = walk.buildParams();
    params("dynamic:enabled") = 1;
    params("dynamic:step") = 15.0;
    params("dynamic:turn") = 30.0;
    
    double t = 0.0;
    double dt = std::pow(10,-2);
    while (viewer.update()) {
        t += dt;
        //CartWalk generator
        walk.exec(dt, params);
        //Adapt CartWalk convention to Model convention
        Leph::VectorLabel output = walk.lastOutputs()
            .rename("output", "");
        output("left_hip_roll") *= -1;
        output("left_ankle_pitch") *= -1;
        output("right_hip_pitch") *= -1;
        output("right_knee") *= -1;
        output("right_hip_roll") *= -1;
        //Convertion to radian
        output.mulOp(M_PI/180.0);
        //Send motor output to model
        model.get().setDOF(output);
        model.get().setDOF("head_pitch", 1.0*sin(t));

        //Update model floating base
        model.updateBase();
        //Set pitch roll reference for trunk orientation
        Eigen::Matrix3d matOrientation = 
            Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(0.8*sin(t), Eigen::Vector3d::UnitY()).toRotationMatrix();
        model.setOrientation(matOrientation, false);

        Eigen::Matrix3d mat = model.get().selfFrameOrientation("camera");
        Eigen::Vector3d vec = model.get().selfFramePosition("camera");
        viewer.drawFrame(Eigen::Vector3d(0.1, 0.1, 0) + vec, mat);
        viewer.drawFrame(Eigen::Vector3d(0.1, 0.1, 0), Eigen::Matrix3d::Identity());

        //Check self from/to frame conversion 
        Eigen::Vector3d tmp1 = model.get().frameInSelf("origin", Eigen::Vector3d(0.0, 0.0, 0.0));
        std::cout << "Frame in Self: " << tmp1.transpose() << std::endl;
        Eigen::Vector3d tmp2 = model.get().selfInFrame("origin", Eigen::Vector3d(0.0, 0.0, 0.0));
        std::cout << "Self in Frame: " << tmp2.transpose() << std::endl;
        Eigen::Vector3d tmp3 = model.get().selfInFrame("origin", model.get().frameInSelf("origin"));
        Eigen::Vector3d tmp4 = model.get().frameInSelf("origin", model.get().selfInFrame("origin"));
        if (fabs(tmp1.z()) > 0.0001 || fabs(tmp2.z()) > 0.0001 || tmp3.norm() > 0.0001 || tmp4.norm() > 0.0001) {
            std::cout << "ASSERT ERROR" << std::endl;
            return 1;
        }

        //Check IMU inputs rebuild
        Eigen::Vector3d angles = model.get().trunkSelfOrientation();
        //Orientation before check
        Eigen::Matrix3d testMat1 = model.get().orientation("trunk", "origin");
        //Build imu extrinsuc matrix orientation
        Eigen::Matrix3d imuMatrix = 
            Eigen::AngleAxisd(angles(2), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(angles(1), Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(angles(0), Eigen::Vector3d::UnitX()).toRotationMatrix();
        model.setOrientation(imuMatrix);
        //Orientation after check
        Eigen::Matrix3d testMat2 = model.get().orientation("trunk", "origin");
        //Check identity
        if ((testMat1 - testMat2).norm() > 0.0001) {
            std::cout << "ASSERT ERROR" << std::endl;
            return 1;
        }
        std::cout << "IMU angles ==> " << angles.transpose() << std::endl;

        //Display center of mass trajectory
        Eigen::Vector3d com = model.get().centerOfMass("origin");
        viewer.addTrackedPoint(com);    
        //Display model
        Leph::ModelDraw(model.get(), viewer);
    }

    return 0;
}

