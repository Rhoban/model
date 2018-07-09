#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Plot/Plot.hpp"

int main()
{
    //Initialize the Model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    Leph::ModelViewer viewer(1200, 900);

    //Camera parameters in width and height
    Leph::CameraModel camParams;//TODO: should be initialized properly

    double t = 0.0;
    Leph::Plot plot;
    while (viewer.update()) {
        t += 0.01;
        //Random trunk position and orientation
        model.get().setAutoUpdate(false);
        model.get().setDOF("base_x", 0.15*sin(t));
        model.get().setDOF("base_y", 0.1*sin(2*t));
        model.get().setDOF("base_yaw", -0.5*sin(3*t));
        model.get().setDOF("left_hip_pitch", -0.5 + 0.5*sin(t));
        model.get().setDOF("left_hip_yaw", 0.2 + 0.2*sin(2*t));
        model.get().setDOF("left_hip_roll", 0.1 + 0.4*sin(2*t));
        model.get().updateDOFPosition();
        
        //Camera world target
        Eigen::Vector3d target = model.get().selfInFrame("origin", Eigen::Vector3d(0.5, 0.5, 0.0));

        //Height pixel in camera target
        //NOTE: Using a pixelTarget different from 0 is not allowed anymore
        double pixelTarget = 0.0;

        //Compute head Inverse Kinematics
        bool isSucess = model.get().cameraLookAt(target);
        if (!isSucess) {
            std::cout << "!!! CAMERA IK LOOKAT ERROR" << std::endl;
            return 1;
        }
        //Draw target
        viewer.drawFrame(target, Eigen::Matrix3d::Identity());
        //Draw view line on pixel target
        Eigen::Vector3d groundPos;
        model.get().cameraViewVectorToWorld(
            model.get().cameraPixelToViewVector(camParams, Eigen::Vector2d(0.0, pixelTarget)), 
            groundPos);
        viewer.drawLink(model.get().position("camera", "origin"), groundPos);
        
        //Display model and view box
        Leph::ModelDraw(model.get(), viewer);
        Leph::CameraDraw(camParams, model.get(), viewer);

        //Pixel to World and World to Pixel computation checks
        Eigen::Vector2d pixel(-1.0, 0.4);
        Eigen::Vector3d ground;
        Eigen::Vector2d pixelCheck;
        bool success1 = model.get().cameraViewVectorToWorld(
            model.get().cameraPixelToViewVector(camParams, pixel), 
            ground);
        std::cout << "Pixel: " << pixel.transpose() << std::endl;
        std::cout << "Ground: " << ground.transpose() << std::endl;
        bool success2 = model.get().cameraWorldToPixel(camParams, ground, pixelCheck);
        std::cout << "PixelCheck: " << pixelCheck.transpose() << std::endl;
        if (success1 && success2 && (pixel-pixelCheck).norm() > 0.005) {
            std::cout << "!!! " << (pixel-pixelCheck).norm() << std::endl;
            std::cout << "ASSERT ERROR VIEWVECTOR TO WORLD" << std::endl;
            return 1;
        }

        //Check view vector to pan/tilt convertion
        Eigen::Vector2d pixel3(0.4, 0.5);
        Eigen::Vector3d viewVector3 = model.get().cameraPixelToViewVector(camParams, pixel3);
        Eigen::Vector2d angles3 = model.get().cameraViewVectorToPanTilt(viewVector3);
        Eigen::Vector2d angles4 = model.get().cameraPixelToPanTilt(camParams, pixel3);
        if ((angles3-angles4).norm() > 0.0001) {
            std::cout << "ASSERT ERROR VIEWVECTOR TO PANTILT" << std::endl;
        }

        //Check view vector convertion
        Eigen::Vector3d view1 = model.get().cameraPixelToViewVector(camParams, Eigen::Vector2d(0.0, 0.0));
        Eigen::Vector2d panTilt1 = model.get().cameraPixelToPanTilt(camParams, Eigen::Vector2d(0.0, 0.0));
        Eigen::Vector2d pixel1;
        model.get().cameraPanTiltToPixel(camParams, panTilt1, pixel1);
        Eigen::Vector3d view2 = model.get().cameraPanTiltToViewVector(panTilt1);
        view1.normalize();
        view2.normalize();
        if ((view1-view2).norm() > 0.0001) {
            std::cout << "ASSERT ERROR VIEW VECTOR" << std::endl;
            return 1;
        }

        //Test pixel to ball world
        std::cout << "============= Ball" << std::endl;
        Eigen::Vector3d ballCenter;
        Eigen::Vector2d ballCenterPixel;
        std::vector<Eigen::Vector2d, 
                    Eigen::aligned_allocator<Eigen::Vector2d>> bordersPixel;
        std::vector<Eigen::Vector3d> borders;
        model.get().cameraViewVectorToBallWorld(camParams, 
            model.get().cameraPixelToViewVector(camParams, Eigen::Vector2d(0.0, 0.5)), 
            0.07,
            ballCenter, &ballCenterPixel, &bordersPixel, &borders);
        viewer.drawSphere(ballCenter, 0.07);
        viewer.drawFrame(ballCenter, Eigen::Matrix3d::Identity());
        viewer.drawFrame(borders[0], Eigen::Matrix3d::Identity());
        viewer.drawFrame(borders[1], Eigen::Matrix3d::Identity());
        viewer.drawFrame(borders[2], Eigen::Matrix3d::Identity());
        viewer.drawFrame(borders[3], Eigen::Matrix3d::Identity());
        viewer.drawLink(borders[0], model.get().position("camera", "origin"));
        viewer.drawLink(borders[1], model.get().position("camera", "origin"));
        viewer.drawLink(borders[2], model.get().position("camera", "origin"));
        viewer.drawLink(borders[3], model.get().position("camera", "origin"));
        std::cout << ballCenter.transpose() << std::endl;
        std::cout << ballCenterPixel.transpose() << std::endl;
        std::cout << bordersPixel[0].transpose() << std::endl;
        std::cout << bordersPixel[1].transpose() << std::endl;
        std::cout << bordersPixel[2].transpose() << std::endl;
        std::cout << bordersPixel[3].transpose() << std::endl;
        std::cout << "-------------" << std::endl;
        
        //Pixel to PanTilt and PanTilt to Pixel computation check
        Eigen::Vector2d pixelTarget2(0.4, 0.5);
        Eigen::Vector2d angles = model.get().cameraPixelToPanTilt(
            camParams, pixelTarget2);
        std::cout << "Angles: " << angles(0)*180.0/M_PI << " " << angles(1)*180.0/M_PI << std::endl;
        Eigen::Vector2d pixelCheck2;
        model.get().cameraPanTiltToPixel(camParams, angles, pixelCheck2);
        std::cout << "Pixel2: " << pixelCheck2.transpose() << std::endl;
        if ((pixelCheck2 - pixelTarget2).norm() > 0.001) {
            std::cout << "ASSERT ERROR PANTILT/PIXEL" << std::endl;
            return 1;
        }

        //Plot head joint position
        plot.add(Leph::VectorLabel(
            "head_yaw", model.get().getDOF("head_yaw"),
            "head_pitch", model.get().getDOF("head_pitch"),
            "time", t
        ));
    }
    plot.plot("time", "all").render();

    return 0;
}

