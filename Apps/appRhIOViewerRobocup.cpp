#include <iostream>
#include <string>
#include <mutex>
#include <map>
#include <string>
#include <sstream>
#include "Model/HumanoidFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "RhIO.hpp"
#include "RhIOClient.hpp"
#include "Model/NamesModel.h"

/**
 * Print usage info and exit
 */
void usage()
{
    std::cout 
        << "Usage: ./app robothost [goal|read|corrected] [sigmaban|sigmaban_plus|grosban]" 
        << std::endl;
    exit(1);
}

int main(int argc, char** argv)
{
    //Parse command line arguments
    if (argc != 4) {
        usage();
    }
    std::string host = std::string(argv[1]);
    std::string mode = std::string(argv[2]);
    if (
        mode != "goal" && 
        mode != "read" && 
        mode != "corrected"
    ) {
        usage();
    }
    Leph::RobotType type = Leph::SigmabanModel;
    if (std::string(argv[3]) == "sigmaban") {
        type = Leph::SigmabanModel;
    } else if (std::string(argv[3]) == "sigmaban_plus") {
        type = Leph::SigmabanPlusModel;
    } else if (std::string(argv[3]) == "grosban") {
        type = Leph::GrosbanModel;
    } else {
        usage();
    }
    std::string prefix = "model/" + mode + std::string("_");

    //Connection to RhIO server
    std::cout << "Connecting to RhIO " << host << " " << mode << " with ";
    std::cout << (type == Leph::SigmabanModel ? "Sigmaban" : "Grosban") 
        << " model" << std::endl;
    
    std::stringstream ss;
    ss << "tcp://" << host << ":" << (RhIO::ServersPortBase);
    RhIO::ClientSub clientSub(ss.str());
    ss.str("");
    ss << "tcp://" << host << ":" << (RhIO::ServersPortBase+1);
    RhIO::ClientReq clientReq(ss.str());
    
    //Initialize model instances
    Leph::HumanoidFixedModel model(type);
    Leph::ModelViewer viewer(1200, 900);
    viewer.camPosVel *= 3.0;
    viewer.maxTrajectory = 50;
    Leph::Scheduling scheduling;
    scheduling.setFrequency(50.0);

    //Mutex protecting model access
    std::mutex mutexModel;
    std::mutex mutexVar;

    //Model variables
    Eigen::Vector3d ballPos(0.0, 0.0, 0.0);
    double ballQuality;
    Eigen::Vector3d fieldPos(0.0, 0.0, 0.0);
    double fieldOrientation;
    double fieldQuality;
    double walkStep = 0.0;
    double walkLateral = 0.0;
    double walkTurn = 0.0;
    bool walkEnable = 0.0;
    std::string stateApproach;
    std::string stateRoboCup;
    std::string stateLowlevel;

    //List all requested values from RhIO
    std::map<std::string, std::function<void(double)>> streamedValues;
    for (size_t i=0;i<Leph::NamesDOFAll.size();i++) {
        streamedValues[prefix + Leph::NamesDOFAll[i]] = [&mutexModel, &model, i](double val) {
            std::lock_guard<std::mutex> lock(mutexModel);
            model.get().setDOF(Leph::NamesDOFAll[i], val);
        };
    }
    streamedValues["moves/walk/walkStep"] = [&mutexVar, &walkStep](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        walkStep = val;
    };
    streamedValues["moves/walk/walkLateral"] = [&mutexVar, &walkLateral](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        walkLateral = val;
    };
    streamedValues["moves/walk/walkTurn"] = [&mutexVar, &walkTurn](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        walkTurn = val;
    };
    streamedValues["localisation/worldBallX"] = [&mutexVar, &ballPos](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        ballPos.x() = val;
    };
    streamedValues["localisation/worldBallY"] = [&mutexVar, &ballPos](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        ballPos.y() = val;
    };
    streamedValues["localisation/worldFieldX"] = [&mutexVar, &fieldPos](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        fieldPos.x() = val;
    };
    streamedValues["localisation/worldFieldY"] = [&mutexVar, &fieldPos](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        fieldPos.y() = val;
    };
    streamedValues["localisation/fieldOrientationWorld"] = [&mutexVar, &fieldOrientation](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        fieldOrientation = -val;
    };
    streamedValues["localisation/ballQ"] = [&mutexVar, &ballQuality](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        ballQuality = val;
    };
    streamedValues["localisation/fieldQ"] = [&mutexVar, &fieldQuality](double val) {
        std::lock_guard<std::mutex> lock(mutexVar);
        fieldQuality = val;
    };

    //Setting streaming handler
    clientSub.setHandlerFloat(
        [&streamedValues]
        (const std::string name, int64_t timestamp, double val) 
    {
        (void)timestamp;
        for (auto& it : streamedValues) {
            if (name == it.first) {
                it.second(val);
            }
        }
    });
    clientSub.setHandlerInt(
        [&mutexModel, &model, &prefix]
        (const std::string name, int64_t timestamp, int val) 
    {
        if (name == prefix + "support_foot") {
            std::lock_guard<std::mutex> lock(mutexModel);
            model.setSupportFoot((Leph::HumanoidFixedModel::SupportFoot)val);
        }
    });
    clientSub.setHandlerBool(
        [&mutexVar, &walkEnable]
        (const std::string name, int64_t timestamp, bool val) 
    {
        if (name == "moves/walk/walkEnable") {
            std::lock_guard<std::mutex> lock(mutexVar);
            walkEnable = val;
        }
    });
    clientSub.setHandlerStr(
        [&mutexVar, &stateApproach, &stateRoboCup, &stateLowlevel]
        (const std::string name, int64_t timestamp, std::string val) 
    {
        if (name == "moves/approach/state") {
            std::lock_guard<std::mutex> lock(mutexVar);
            stateApproach = val;
        }
        if (name == "moves/robocup/state") {
            std::lock_guard<std::mutex> lock(mutexVar);
            stateRoboCup = val;
        }
        if (name == "model/lowlevel_state") {
            std::lock_guard<std::mutex> lock(mutexVar);
            stateLowlevel = val;
        }
    });
    
    //Fetch all requested value
    for (auto& it : streamedValues) {
        it.second(clientReq.getFloat(it.first));
    }
    model.setSupportFoot((Leph::HumanoidFixedModel::SupportFoot)
        clientReq.getInt(prefix + "support_foot"));
    walkEnable = clientReq.getBool("moves/walk/walkEnable");
    stateApproach = clientReq.getStr("moves/approach/state");
    stateRoboCup = clientReq.getStr("moves/robocup/state");
    stateLowlevel = clientReq.getStr("model/lowlevel_state");

    //Enabling value streaming
    for (auto& it : streamedValues) {
        clientReq.enableStreamingValue(it.first);
    }
    clientReq.enableStreamingValue(prefix + "support_foot");
    clientReq.enableStreamingValue("moves/walk/walkEnable");
    clientReq.enableStreamingValue("moves/approach/state");
    clientReq.enableStreamingValue("moves/robocup/state");
    clientReq.enableStreamingValue("model/lowlevel_state");
    
    //Main viewer loop
    while (viewer.update()) {
        //Waiting
        scheduling.wait();
        //Model update
        std::lock_guard<std::mutex> lockVar(mutexVar);
        std::lock_guard<std::mutex> lockModel(mutexModel);
        //Draw Robocup field
        std::stringstream ssField;
        ssField << std::setprecision(3) << fieldQuality;
        Leph::FieldDraw(fieldPos, fieldOrientation, viewer);
        viewer.drawText(
            fieldPos 
            + Eigen::AngleAxisd(fieldOrientation, Eigen::Vector3d(0.0, 0.0, 1.0))
                .toRotationMatrix()
            * Eigen::Vector3d(4.5, 0.0, 1.2), 
            25, ssField.str(), 1.0, 1.0, 1.0);
        //Draw ball
        std::stringstream ssBall;
        ssBall << std::setprecision(3) << ballQuality;
        double ballColor = 0.2 + 0.8*ballQuality;
        viewer.drawSphere(ballPos + Eigen::Vector3d(0.0, 0.0, 0.075), 
            0.075, ballColor, ballColor, ballColor);
        viewer.drawText(ballPos + Eigen::Vector3d(0.0, 0.0, 0.25), 15, 
            ssBall.str(), 1.0, 1.0, 1.0);
        //Draw walk controls
        Eigen::Vector3d pose = model.get().getPose();
        Eigen::Vector3d pos(pose.x(), pose.y(), 0.0);
        Eigen::Matrix3d rot = 
            Eigen::AngleAxisd(pose.z(), Eigen::Vector3d(0.0, 0.0, 1.0))
            .toRotationMatrix();
        viewer.drawArrow(pos, rot*Eigen::Vector3d(1.0, 0.0, 0.0), walkStep/100.0, 1.0, 0.0, 0.0);
        viewer.drawArrow(pos, rot*Eigen::Vector3d(0.0, 1.0, 0.0), walkLateral/100.0, 0.0, 1.0, 0.0);
        viewer.drawArrow(pos, Eigen::Vector3d(0.0, 0.0, 1.0), walkTurn/100.0, 0.0, 0.0, 1.0);
        if (walkEnable) {
            viewer.drawCylinder(pos + Eigen::Vector3d(0.0, 0.0, -0.02), 
                0.1, 0.02, 0.0, 1.0, 0.0);
        } else {
            viewer.drawCylinder(pos + Eigen::Vector3d(0.0, 0.0, -0.02), 
                0.1, 0.02, 1.0, 0.0, 0.0);
        }
        std::stringstream ssStep;
        ssStep << std::setprecision(3) << walkStep;
        std::stringstream ssLateral;
        ssLateral << std::setprecision(3) << walkLateral;
        std::stringstream ssTurn;
        ssTurn << std::setprecision(3) << walkTurn;
        viewer.drawText(
            pos + rot*Eigen::Vector3d(walkStep/100.0, 0.0, 0.0),
            20, ssStep.str(), 1.0, 0.0, 0.0);
        viewer.drawText(
            pos + rot*Eigen::Vector3d(0.0, walkLateral/100.0, 0.0),
            20, ssLateral.str(), 0.0, 1.0, 0.0);
        viewer.drawText(
            pos + Eigen::Vector3d(0.0, 0.0, walkTurn/100.0),
            20, ssTurn.str(), 0.0, 0.0, 1.0);
        //Draw robot state
        viewer.drawText(
            model.get().position("camera", "origin") + Eigen::Vector3d(0.0, 0.0, 0.5),
            25, stateRoboCup, 0.0, 1.0, 1.0);
        viewer.drawText(
            model.get().position("camera", "origin") + Eigen::Vector3d(0.0, 0.0, 0.35),
            25, stateApproach, 1.0, 0.0, 1.0);
        viewer.drawText(
            model.get().position("camera", "origin") + Eigen::Vector3d(0.0, 0.0, 0.2),
            25, stateLowlevel, 1.0, 1.0, 0.0);
        //Display centers of pressures trajectory
        viewer.setCamPosition(pose.x(), pose.y());
        Eigen::Vector3d com = model.get().centerOfMass("origin");
        com.z() = 0.0;
        viewer.addTrackedPoint(
            com, Leph::ModelViewer::Red);
        Leph::CameraParameters camParams = {74*3.14/180.0, 99*3.14/180.0};
        Leph::ModelDraw(model.get(), viewer);
        Leph::CameraDraw(camParams, model.get(), viewer);
    }
    //Disabling value streaming
    for (auto& it : streamedValues) {
        clientReq.disableStreamingValue(it.first);
    }
    clientReq.disableStreamingValue(prefix + "support_foot");
    clientReq.disableStreamingValue("moves/walk/walkEnable");
    clientReq.disableStreamingValue("moves/approach/state");
    clientReq.disableStreamingValue("moves/robocup/state");
    clientReq.disableStreamingValue("model/lowlevel_state");
    
    return 0;
}

