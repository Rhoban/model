#include <iostream>
#include "Utils/Chrono.hpp"
#include "Model/HumanoidFixedModel.hpp"

void test(bool autoUpdate) 
{
    Leph::Chrono c;
    
    //Initialize the Model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    Leph::CameraModel camParams;//TODO: should be initialized properly

    size_t count = 100;
    for (double t=0.0;t<1.0;t+=0.01) {
        c.start("all");
        //Update the Model
        c.start("model update");
        for (size_t i=0;i<count;i++) {
            model.get().setDOF("base_x", 0.15*sin(t));
            model.get().setDOF("base_y", 0.1*sin(2*t));
            model.get().setDOF("base_yaw", -0.5*sin(3*t));
            model.get().setDOF("left_hip_pitch", -0.5 + 0.5*sin(t));
            model.get().setDOF("left_hip_yaw", 0.2 + 0.2*sin(2*t));
            model.get().setDOF("left_hip_roll", 0.1 + 0.4*sin(2*t));
            model.updateBase();
            model.get().setAutoUpdate(autoUpdate);
            model.get().updateDOFPosition();
        }
        c.stop("model update");
        
        //Position and orientation
        c.start("position");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Vector3d p0 = model.get().position("camera", "origin", Eigen::Vector3d(0.5, 0.5, 0.0));
            (void)p0;
        }
        c.stop("position");
        c.start("orientation");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Matrix3d m0 = model.get().orientation("camera", "origin");
            (void)m0;
        }
        c.stop("orientation");

        //Self position and orientation
        c.start("self position");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Vector3d p00 = model.get().selfFramePosition("trunk");
            (void)p00;
        }
        c.stop("self position");
        c.start("self orientation");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Matrix3d m00 = model.get().selfFrameOrientation("trunk");
            (void)m00;
        }
        c.stop("self orientation");

        //Self to frame and frame to self
        c.start("self in frame");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Vector3d p1 = model.get().selfInFrame("origin", Eigen::Vector3d(0.5, 0.5, 0.0));
            (void)p1;
        }
        c.stop("self in frame");
        c.start("frame in self");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Vector3d p2 = model.get().frameInSelf("origin", Eigen::Vector3d(0.5, 0.5, 0.0));
            (void)p2;
        }
        c.stop("frame in self");

        //Pixel and pan tilt to view vector
        c.start("pixel to view vector");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Vector3d p3 = model.get().cameraPixelToViewVector(camParams, Eigen::Vector2d(0.5, 0.5));
            (void)p3;
        }
        c.stop("pixel to view vector");
        c.start("pan/tilt to view vector");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Vector3d p4 = model.get().cameraPanTiltToViewVector(Eigen::Vector2d(0.2, 0.3));
            (void)p4;
        }
        c.stop("pan/tilt to view vector");
        
        //Pixel to pan tilt and pan tilt to pixel
        c.start("pixel to pan/tilt");
        for (size_t i=0;i<count;i++) {
            volatile Eigen::Vector2d p5 = model.get().cameraPixelToPanTilt(camParams, Eigen::Vector2d(0.5, 0.5));
            (void)p5;
        }
        c.stop("pixel to pan/tilt");
        c.start("pan/tilt to pixel");
        for (size_t i=0;i<count;i++) {
            Eigen::Vector2d p6;
            model.get().cameraPanTiltToPixel(camParams, Eigen::Vector2d(0.2, 0.3), p6);
        }
        c.stop("pan/tilt to pixel");
        
        //View vector to world
        c.start("Viewvector to world");
        for (size_t i=0;i<count;i++) {
            Eigen::Vector3d p7;
            model.get().cameraViewVectorToWorld(Eigen::Vector3d(0.5, 0.0, -0.4), p7);
        }
        c.stop("Viewvector to world");

        //World to pixel
        c.start("World to pixel");
        for (size_t i=0;i<count;i++) {
            Eigen::Vector2d p8;
            model.get().cameraWorldToPixel(camParams, Eigen::Vector3d(1.0, 0.2, 0.0), p8);
        }
        c.stop("World to pixel");
        
        //Ball info
        c.start("ball info");
        for (size_t i=0;i<count;i++) {
            Eigen::Vector3d p9;
            Eigen::Vector2d v10;
            std::vector<Eigen::Vector2d,
                        Eigen::aligned_allocator<Eigen::Vector2d>> c1;
            std::vector<Eigen::Vector3d> c2;
            model.get().cameraViewVectorToBallWorld(camParams, Eigen::Vector3d(0.4, 0.0, -0.4), 0.07, p9, &v10, &c1, &c2);
        }
        c.stop("ball info");
        
        //Head IK
        c.start("look at");
        for (size_t i=0;i<count;i++) {
            model.get().cameraLookAt(Eigen::Vector3d(1.0, 0.0, 0.0));
        }
        c.stop("look at");
        c.stop("all");
    }
    c.print();
}

int main()
{
    test(true);
    test(false);
    return 0;
}

