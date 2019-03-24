#include <iostream>
#include <string>
#include <mutex>
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "RhIOClient.hpp"
#include "Model/NamesModel.h"

int main(int argc, char** argv)
{
  // Parse command line arguments
  if (argc <= 1 || argc >= 5)
  {
    std::cout << "Usage: ./app robothost [rhio_prefix] [sigmaban|sigmaban_plus|grosban]" << std::endl;
    return -1;
  }
  std::string host = std::string(argv[1]);
  std::string prefix = "";
  bool isGoal = false;
  if (argc >= 3)
  {
    prefix = std::string(argv[2]);
    if (prefix == "goal")
    {
      isGoal = true;
      prefix = "";
    }
    if (prefix.size() > 0 && prefix.front() == '/')
    {
      prefix = prefix.substr(1);
    }
  }
  Leph::RobotType type = Leph::SigmabanModel;
  if (argc >= 4)
  {
    if (std::string(argv[3]) == "sigmaban")
    {
      type = Leph::SigmabanModel;
    }
    else if (std::string(argv[3]) == "sigmaban_plus")
    {
      type = Leph::SigmabanPlusModel;
    }
    else if (std::string(argv[3]) == "grosban")
    {
      type = Leph::GrosbanModel;
    }
    else
    {
      std::cout << "Usage: ./app robothost [rhio_prefix] [sigmaban|grosban]" << std::endl;
      return -1;
    }
  }

  // Connection to RhIO server
  std::cout << "Connecting to RhIO " << host << ":" << prefix << " with ";
  std::cout << (type == Leph::SigmabanModel ? "Sigmaban" : "Grosban") << " model" << std::endl;

  std::stringstream ss;
  ss << "tcp://" << host << ":" << (RhIO::ServersPortBase);
  RhIO::ClientSub clientSub(ss.str());
  ss.str("");
  ss << "tcp://" << host << ":" << (RhIO::ServersPortBase + 1);
  RhIO::ClientReq clientReq(ss.str());

  // Initialize model instances
  Leph::HumanoidFixedPressureModel model(type);
  Leph::ModelViewer viewer(1200, 900);
  viewer.maxTrajectory = 50;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(50.0);

  // Mutex protecting model access
  std::mutex mutexModel;
  std::mutex mutexVar;

  // Model variables
  double yaw;
  double pitch;
  double roll;
  double pressureLeftWeight;
  double pressureLeftX;
  double pressureLeftY;
  double pressureRightWeight;
  double pressureRightX;
  double pressureRightY;
  // Setting streaming handler
  clientSub.setHandlerFloat([&mutexVar, &mutexModel, &model, &prefix, &yaw, &pitch, &roll, &pressureLeftWeight,
                             &pressureLeftX, &pressureLeftY, &pressureRightWeight, &pressureRightX,
                             &pressureRightY](const std::string name, int64_t timestamp, double val) {
    (void)timestamp;
    if (name == "lowlevel/imu/yaw")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      yaw = val * M_PI / 180.0;
    }
    if (name == "lowlevel/imu/pitch")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      pitch = val * M_PI / 180.0;
    }
    if (name == "lowlevel/imu/roll")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      roll = val * M_PI / 180.0;
    }
    for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
    {
      if (name == "lowlevel/" + Leph::NamesDOF[i] + "/position")
      {
        std::lock_guard<std::mutex> lock(mutexModel);
        model.get().setDOF(Leph::NamesDOF[i], val * M_PI / 180.0);
      }
      if (name == "lowlevel/" + Leph::NamesDOF[i] + "/goalPosition")
      {
        std::lock_guard<std::mutex> lock(mutexModel);
        model.get().setDOF(Leph::NamesDOF[i], val * M_PI / 180.0);
      }
    }
    if (name == "lowlevel/left_pressure/weight")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      pressureLeftWeight = val;
    }
    if (name == "lowlevel/left_pressure/x")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      pressureLeftX = val;
    }
    if (name == "lowlevel/left_pressure/y")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      pressureLeftY = val;
    }
    if (name == "lowlevel/right_pressure/weight")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      pressureRightWeight = val;
    }
    if (name == "lowlevel/right_pressure/x")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      pressureRightX = val;
    }
    if (name == "lowlevel/right_pressure/y")
    {
      std::lock_guard<std::mutex> lock(mutexVar);
      pressureRightY = val;
    }
    for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
    {
      if (name == prefix + Leph::NamesDOF[i])
      {
        std::lock_guard<std::mutex> lock(mutexModel);
        model.get().setDOF(Leph::NamesDOF[i], val);
      }
    }
    if (name == prefix + "base_x")
    {
      std::lock_guard<std::mutex> lock(mutexModel);
      model.get().setDOF("base_x", val);
    }
    if (name == prefix + "base_y")
    {
      std::lock_guard<std::mutex> lock(mutexModel);
      model.get().setDOF("base_y", val);
    }
    if (name == prefix + "base_z")
    {
      std::lock_guard<std::mutex> lock(mutexModel);
      model.get().setDOF("base_z", val);
    }
    if (name == prefix + "base_yaw")
    {
      std::lock_guard<std::mutex> lock(mutexModel);
      model.get().setDOF("base_yaw", val);
    }
    if (name == prefix + "base_pitch")
    {
      std::lock_guard<std::mutex> lock(mutexModel);
      model.get().setDOF("base_pitch", val);
    }
    if (name == prefix + "base_roll")
    {
      std::lock_guard<std::mutex> lock(mutexModel);
      model.get().setDOF("base_roll", val);
    }
  });
  clientSub.setHandlerInt(
      [&mutexVar, &mutexModel, &model, &prefix](const std::string name, int64_t timestamp, int val) {
        if (name == prefix + "support_foot")
        {
          std::lock_guard<std::mutex> lock(mutexModel);
          model.setSupportFoot((Leph::HumanoidFixedModel::SupportFoot)val);
        }
      });
  // Enabling value streaming
  if (prefix == "")
  {
    clientReq.enableStreamingValue("lowlevel/imu/yaw");
    clientReq.enableStreamingValue("lowlevel/imu/pitch");
    clientReq.enableStreamingValue("lowlevel/imu/roll");
    for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
    {
      if (isGoal)
      {
        clientReq.enableStreamingValue("lowlevel/" + Leph::NamesDOF[i] + "/goalPosition");
      }
      else
      {
        clientReq.enableStreamingValue("lowlevel/" + Leph::NamesDOF[i] + "/position");
      }
    }
    clientReq.enableStreamingValue("lowlevel/left_pressure/weight");
    clientReq.enableStreamingValue("lowlevel/left_pressure/x");
    clientReq.enableStreamingValue("lowlevel/left_pressure/y");
    clientReq.enableStreamingValue("lowlevel/right_pressure/weight");
    clientReq.enableStreamingValue("lowlevel/right_pressure/x");
    clientReq.enableStreamingValue("lowlevel/right_pressure/y");
  }
  else
  {
    for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
    {
      clientReq.enableStreamingValue(prefix + Leph::NamesDOF[i]);
    }
    clientReq.enableStreamingValue(prefix + "base_x");
    clientReq.enableStreamingValue(prefix + "base_y");
    clientReq.enableStreamingValue(prefix + "base_z");
    clientReq.enableStreamingValue(prefix + "base_yaw");
    clientReq.enableStreamingValue(prefix + "base_pitch");
    clientReq.enableStreamingValue(prefix + "base_roll");
    clientReq.enableStreamingValue(prefix + "support_foot");
  }

  // Main viewer loop
  while (viewer.update())
  {
    // Waiting
    scheduling.wait();
    // Model update
    std::lock_guard<std::mutex> lockVar(mutexVar);
    std::lock_guard<std::mutex> lockModel(mutexModel);
    if (prefix == "")
    {
      if (pressureLeftWeight + pressureRightWeight > 0.0001)
      {
        model.setPressure(pressureLeftWeight + pressureRightWeight,
                          pressureLeftWeight / (pressureLeftWeight + pressureRightWeight),
                          pressureRightWeight / (pressureLeftWeight + pressureRightWeight), pressureLeftX,
                          pressureLeftY, pressureRightX, pressureRightY);
      }
      // Update base
      if (isGoal)
      {
        model.HumanoidFixedModel::updateBase();
      }
      else
      {
        model.HumanoidFixedPressureModel::updateBase();
        // Rebuilt extrinsic matrix
        Eigen::Matrix3d imuMatrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        model.setOrientation(imuMatrix);
      }
      // Model viewer
      if (!isGoal)
      {
        Eigen::Vector3d copLeft = model.centerOfPressureLeft("origin");
        Eigen::Vector3d copRight = model.centerOfPressureRight("origin");
        // Display foot pressure force
        double weightLeft = model.pressureWeight() * model.pressureLeftRatio();
        double weightRight = model.pressureWeight() * model.pressureRightRatio();
        double boxLength = 0.02;
        viewer.drawBox(0.005, 0.005, boxLength * weightLeft, copLeft + Eigen::Vector3d(0, 0, boxLength * weightLeft),
                       Eigen::Matrix3d::Identity(), 1.0, 0.0, 0.0);
        viewer.drawBox(0.005, 0.005, boxLength * weightRight, copRight + Eigen::Vector3d(0, 0, boxLength * weightRight),
                       Eigen::Matrix3d::Identity(), 0.0, 1.0, 0.0);
      }
    }
    // Display centers of pressures trajectory
    Eigen::Vector3d com = model.get().centerOfMass("origin");
    com.z() = 0.0;
    viewer.addTrackedPoint(com, Leph::ModelViewer::Red);
    Leph::CameraParameters camParams = { 74 * 3.14 / 180.0, 99 * 3.14 / 180.0 };
    Leph::ModelDraw(model.get(), viewer);
    Leph::CameraDraw(camParams, model.get(), viewer);
  }
  // Disabling value streaming
  if (prefix == "")
  {
    clientReq.disableStreamingValue("lowlevel/imu/yaw");
    clientReq.disableStreamingValue("lowlevel/imu/pitch");
    clientReq.disableStreamingValue("lowlevel/imu/roll");
    for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
    {
      if (isGoal)
      {
        clientReq.disableStreamingValue("lowlevel/" + Leph::NamesDOF[i] + "/goalPosition");
      }
      else
      {
        clientReq.disableStreamingValue("lowlevel/" + Leph::NamesDOF[i] + "/position");
      }
    }
    clientReq.disableStreamingValue("lowlevel/left_pressure/weight");
    clientReq.disableStreamingValue("lowlevel/left_pressure/x");
    clientReq.disableStreamingValue("lowlevel/left_pressure/y");
    clientReq.disableStreamingValue("lowlevel/right_pressure/weight");
    clientReq.disableStreamingValue("lowlevel/right_pressure/x");
    clientReq.disableStreamingValue("lowlevel/right_pressure/y");
  }
  else
  {
    for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
    {
      clientReq.disableStreamingValue(prefix + Leph::NamesDOF[i]);
    }
    clientReq.disableStreamingValue(prefix + "base_x");
    clientReq.disableStreamingValue(prefix + "base_y");
    clientReq.disableStreamingValue(prefix + "base_z");
    clientReq.disableStreamingValue(prefix + "base_yaw");
    clientReq.disableStreamingValue(prefix + "base_pitch");
    clientReq.disableStreamingValue(prefix + "base_roll");
    clientReq.disableStreamingValue(prefix + "support_foot");
  }

  return 0;
}
