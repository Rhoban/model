#include "Viewer/ModelDraw.hpp"

namespace Leph {

void ModelDraw(Model& model, ModelViewer& viewer, double color)
{
    const RBDL::Model& rbdlModel = model.getRBDLModel();
    size_t originIndex = model.getFrameIndex("origin");

    //Draw tracked point trajectory
    viewer.drawTrajectory();

    //Draw all references frames
    for (size_t i=0;i<model.sizeFrame();i++) {
        Eigen::Vector3d pos = model.position(i, originIndex);
        Eigen::Matrix3d mat = model.orientation(i, originIndex);
        viewer.drawFrame(pos, mat);
        
        //Draw optional bounding box
        double sizeX;
        double sizeY;
        double sizeZ;
        Eigen::Vector3d center;
        model.boundingBox(i, sizeX, sizeY, sizeZ, center);
        viewer.drawBox(sizeX, sizeY, sizeZ, 
            pos+mat.transpose()*center, mat);
    }

    //Draw center of mass
    viewer.drawMass(model.centerOfMass("origin"), 
        RBDLMath::Matrix3d::Identity(), color);

    for (size_t i=0;i<rbdlModel.mBodies.size();i++) {
        //Virtual moby used for multi DOF are skipped
        if (rbdlModel.mBodies[i].mMass < 0.0001) {
            continue;
        }
        //Draw RBDL bodies center of mass
        size_t bodyIndex = model.bodyIdToFrameIndex(i);
        Eigen::Vector3d com = rbdlModel.mBodies[i].mCenterOfMass;
        Eigen::Vector3d pos = model.position(bodyIndex, originIndex, com);
        Eigen::Matrix3d mat = model.orientation(bodyIndex, originIndex);
        viewer.drawMass(pos, mat, color);
        Eigen::Vector3d center = model.position(bodyIndex, originIndex);
        //Draw RBDL joints if parent body is non virtual
        if (rbdlModel.mBodies[rbdlModel.lambda[i]].mMass > 0.0001) {
            RBDLMath::Vector3d jointAxis = rbdlModel.S[i].head(3);
            RBDLMath::Matrix3d transformAxis = RBDLMath::Matrix3d::Identity();
            if (
                (jointAxis-RBDLMath::Vector3d(1.0, 0.0, 0.0)).norm() > 0.001 &&
                (jointAxis-RBDLMath::Vector3d(-1.0, 0.0, 0.0)).norm() > 0.001
            ) {
                transformAxis = Eigen::AngleAxisd(-M_PI/2.0, 
                        jointAxis.cross(RBDLMath::Vector3d(1.0, 0.0, 0.0)))
                    .toRotationMatrix();
            }
            if (
                (jointAxis-RBDLMath::Vector3d(-1.0, 0.0, 0.0)).norm() < 0.001
            ) {
                transformAxis *= -1.0;
            }
            transformAxis.transposeInPlace();
            transformAxis *= mat;
            viewer.drawJoint(center, transformAxis, color);
        }
        //Draw link between body origin and center of mass
        viewer.drawLink(center, pos, color);
        //Draw link between body center of mass and its children origin
        for (size_t j=0;j<rbdlModel.mu[i].size();j++) {
            size_t childIndex = model.bodyIdToFrameIndex(rbdlModel.mu[i][j]);
            Eigen::Vector3d centerChild = model.position(childIndex, originIndex);
            viewer.drawLink(pos, centerChild, color);
        }
    }

    //Draw link between fixed reference frame and the center of 
    //mass of it movable parent
    for (size_t i=0;i<rbdlModel.mFixedBodies.size();i++) {
        size_t parentId = rbdlModel.mFixedBodies[i].mMovableParent;
        size_t parentIndex = model.bodyIdToFrameIndex(parentId);
        size_t bodyIndex = model.bodyIdToFrameIndex(i + rbdlModel.fixed_body_discriminator);
        Eigen::Vector3d com = rbdlModel.mBodies[parentId].mCenterOfMass;
        Eigen::Vector3d pos = model.position(parentIndex, originIndex, com);
        Eigen::Vector3d center = model.position(bodyIndex, originIndex);
        viewer.drawLink(pos, center, color);
    }
}

void CameraDraw(
    const CameraModel& cameraModel,
    HumanoidModel& model, 
    ModelViewer& viewer)
{
    Eigen::Vector3d groundPos1;
    Eigen::Vector3d groundPos2;
    Eigen::Vector3d groundPos3;
    Eigen::Vector3d groundPos4;
    Eigen::Vector3d groundPos5;
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        cameraModel, Eigen::Vector2d(-1.0, -1.0)),
        groundPos1);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        cameraModel, Eigen::Vector2d( 1.0, -1.0)), 
        groundPos2);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        cameraModel, Eigen::Vector2d( 1.0,  1.0)), 
        groundPos3);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        cameraModel, Eigen::Vector2d(-1.0,  1.0)), 
        groundPos4);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        cameraModel, Eigen::Vector2d( 0.0,  0.0)), 
        groundPos5);
    viewer.drawLink(model.position("camera", "origin"), groundPos1, 1.0);
    viewer.drawLink(model.position("camera", "origin"), groundPos2, 1.0);
    viewer.drawLink(model.position("camera", "origin"), groundPos3, 1.0);
    viewer.drawLink(model.position("camera", "origin"), groundPos4, 1.0);
    viewer.drawLink(model.position("camera", "origin"), groundPos5, 1.0);
    viewer.drawLink(groundPos1, groundPos2, 1.0);
    viewer.drawLink(groundPos2, groundPos3, 1.0);
    viewer.drawLink(groundPos3, groundPos4, 1.0);
    viewer.drawLink(groundPos4, groundPos1, 1.0);
}

void FieldDraw(
    const Eigen::Vector3d& center,
    double yaw,
    ModelViewer& viewer)
{
    double thick = 5.0;
    Eigen::Matrix3d rot = 
        Eigen::AngleAxisd(yaw, Eigen::Vector3d(0.0, 0.0, 1.0))
        .toRotationMatrix();
    Eigen::Vector3d p1 = center + rot*Eigen::Vector3d(-4.5, -3.0, 0.0);
    Eigen::Vector3d p2 = center + rot*Eigen::Vector3d(4.5, -3.0, 0.0);
    Eigen::Vector3d p3 = center + rot*Eigen::Vector3d(4.5, 3.0, 0.0);
    Eigen::Vector3d p4 = center + rot*Eigen::Vector3d(-4.5, 3.0, 0.0);
    Eigen::Vector3d p5 = center + rot*Eigen::Vector3d(0.0, -3.0, 0.0);
    Eigen::Vector3d p6 = center + rot*Eigen::Vector3d(0.0, 3.0, 0.0);
    Eigen::Vector3d p7 = center + rot*Eigen::Vector3d(-4.5, 3.45/2.0, 0.0);
    Eigen::Vector3d p8 = center + rot*Eigen::Vector3d(-4.5+0.6, 3.45/2.0, 0.0);
    Eigen::Vector3d p9 = center + rot*Eigen::Vector3d(-4.5+0.6, -3.45/2.0, 0.0);
    Eigen::Vector3d p10 = center + rot*Eigen::Vector3d(-4.5, -3.45/2.0, 0.0);
    Eigen::Vector3d p11 = center + rot*Eigen::Vector3d(4.5, 3.45/2.0, 0.0);
    Eigen::Vector3d p12 = center + rot*Eigen::Vector3d(4.5-0.6, 3.45/2.0, 0.0);
    Eigen::Vector3d p13 = center + rot*Eigen::Vector3d(4.5-0.6, -3.45/2.0, 0.0);
    Eigen::Vector3d p14 = center + rot*Eigen::Vector3d(4.5, -3.45/2.0, 0.0);
    Eigen::Vector3d p15 = center + rot*Eigen::Vector3d(4.5, -1.80/2.0, 0.0);
    Eigen::Vector3d p16 = center + rot*Eigen::Vector3d(4.5, 1.80/2.0, 0.0);
    Eigen::Vector3d p17 = center + rot*Eigen::Vector3d(-4.5, -1.80/2.0, 0.0);
    Eigen::Vector3d p18 = center + rot*Eigen::Vector3d(-4.5, 1.80/2.0, 0.0);
    viewer.drawLine(p1, p2, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p2, p3, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p3, p4, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p4, p1, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p5, p6, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p7, p8, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p8, p9, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p9, p10, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p11, p12, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p12, p13, thick, 1.0, 1.0, 1.0);
    viewer.drawLine(p13, p14, thick, 1.0, 1.0, 1.0);
    viewer.drawCylinder(p15, 0.1, 1.8, 1.0, 1.0, 1.0);
    viewer.drawCylinder(p16, 0.1, 1.8, 1.0, 1.0, 1.0);
    viewer.drawCylinder(p17, 0.1, 1.8, 1.0, 1.0, 1.0);
    viewer.drawCylinder(p18, 0.1, 1.8, 1.0, 1.0, 1.0);
}

void CleatsDraw(
    HumanoidSimulation& simulation,
    ModelViewer& viewer)
{
    std::vector<std::string> names = {
        "left_cleat_1", "left_cleat_2",
        "left_cleat_3", "left_cleat_4",
        "right_cleat_1", "right_cleat_2",
        "right_cleat_3", "right_cleat_4",
    };
    for (const std::string& name : names) {
        Eigen::Vector3d pos = simulation.model().position(name, "origin");
        double force = simulation.getCleatForce(name);
        if (force >= 0.0) {
            viewer.drawCylinder(
                pos, 0.002, 0.01*force, 1.0, 1.0, 1.0);
        } else {
            viewer.drawCylinder(
                pos + Eigen::Vector3d(0.0, 0.0, 0.01*force), 
                0.002, -0.01*force, 1.0, 0.0, 0.0);
        }
    }
}

}

