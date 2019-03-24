#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

int main(int argc, char** argv)
{
  // Command line arguments
  if (argc != 3)
  {
    std::cout << "Usage: ./app [sigmaban|grosban] logsFile" << std::endl;
    return -1;
  }

  Leph::RobotType type;
  if (std::string(argv[1]) == "sigmaban")
  {
    type = Leph::SigmabanModel;
  }
  else if (std::string(argv[1]) == "grosban")
  {
    type = Leph::GrosbanModel;
  }
  else
  {
    std::cout << "Usage: ./app [sigmaban|grosban] logsFile" << std::endl;
    return -1;
  }
  std::string logsFile = argv[2];
  std::cout << "Loading " << logsFile << std::endl;

  // Loading data
  Leph::MatrixLabel logs;
  logs.load(logsFile);

  // Print data informations
  std::cout << "Loaded " << logs.size() << " points with " << logs.dimension() << " entries" << std::endl;
  if (logs.size() == 0)
  {
    return 0;
  }

  // Ploting curves
  std::cout << "Plotting references and motors" << std::endl;
  logs.plot().plot("time:timestamp", "output:*").plot("time:timestamp", "motor:*").render();
  std::cout << "Plotting sensors" << std::endl;
  logs.plot().plot("time:timestamp", "sensor:*").render();
  std::cout << "Plotting pressure" << std::endl;
  logs.plot().plot("index", "pressure:*").render();
  std::cout << "Plotting timming" << std::endl;
  logs.plot().plot("index", "time:*").render();
  std::cout << "Plotting walk parameters" << std::endl;
  logs.plot().plot("index", "walk:*").render();
  std::cout << "Plotting base" << std::endl;
  std::cout << "Measured walked distance: "
            << sqrt(pow(logs[0]("motor:base_x") - logs[logs.size() - 1]("motor:base_x"), 2) +
                    pow(logs[0]("motor:base_y") - logs[logs.size() - 1]("motor:base_y"), 2))
            << std::endl;
  logs.plot().plot("index", "motor:base_x").plot("index", "motor:base_y").plot("index", "motor:base_yaw").render();

  // Initialize model instances
  Leph::HumanoidFixedModel modelOutputs(type);
  Leph::HumanoidFixedModel modelMotors(type);
  Leph::HumanoidFixedModel modelWalk(type);
  Leph::ModelViewer viewer(1200, 900);
  Leph::Scheduling scheduling;

  // Initialize DOF vector
  Leph::VectorLabel outputsDOF = logs[0].extract("output").rename("output", "");
  Leph::VectorLabel motorsDOF = logs[0].extract("motor").rename("motor", "");
  Leph::VectorLabel walkParams = logs[0].extract("walk");

  // Main loop
  double freq = 50.0;
  scheduling.setFrequency(freq);
  double t = logs[0]("time:timestamp");
  size_t indexLog = 0;
  bool isPaused = false;
  int viewMode = 0;
  while (viewer.update())
  {
    // Find current log index (associated with time)
    indexLog = 0;
    while (indexLog < logs.size() - 1 && logs[indexLog]("time:timestamp") < t)
    {
      indexLog++;
    }
    // Interface control
    if (viewer.isKeyPressed(sf::Keyboard::I))
    {
      t = logs[0]("time:timestamp");
      indexLog = 0;
    }
    if (viewer.isKeyPressed(sf::Keyboard::P))
    {
      isPaused = true;
    }
    if (viewer.isKeyPressed(sf::Keyboard::C))
    {
      isPaused = false;
    }
    if (viewer.isKeyPressed(sf::Keyboard::PageUp))
    {
      t += 1000.0;
    }
    if (viewer.isKeyPressed(sf::Keyboard::PageDown))
    {
      t -= 1000.0;
    }
    if (viewer.isKeyPressed(sf::Keyboard::M))
    {
      viewMode++;
      if (viewMode > 3)
      {
        viewMode = 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Assign DOF
    outputsDOF.assignOp(logs[indexLog], "output", "");
    motorsDOF.assignOp(logs[indexLog], "motor", "");
    modelOutputs.get().setDOF(outputsDOF, false);
    modelMotors.get().setDOF(motorsDOF, false);
    // Walk model
    walkParams.assignOp(logs[indexLog], "walk");
    Leph::IKWalk::Parameters params = Leph::IKWalk::convertVectorLabel(walkParams);
    bool success = Leph::IKWalk::walk(modelWalk.get(), params, logs[indexLog]("time:phase"), 0.02);
    if (!success)
    {
      std::cout << "IKWalk error" << std::endl;
      return -1;
    }
    // Contraint the model on the ground, integrate movement
    modelOutputs.updateBase();
    modelMotors.updateBase();
    modelWalk.updateBase();
    // Set IMU data for motors real model state
    modelMotors.setOrientation(
        Eigen::AngleAxisd(logs[indexLog]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(logs[indexLog]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
        false);

    // Display captured center of mass ground projection
    if (viewMode == 1)
    {
      viewer.addTrackedPoint(modelOutputs.get().centerOfMass("origin"), Leph::ModelViewer::Yellow);
      viewer.addTrackedPoint(modelOutputs.get().position("left_foot_tip", "origin"), Leph::ModelViewer::Red);
      viewer.addTrackedPoint(modelOutputs.get().position("right_foot_tip", "origin"), Leph::ModelViewer::Green);
      viewer.addTrackedPoint(modelOutputs.get().position("trunk", "origin"), Leph::ModelViewer::Blue);
      viewer.addTrackedPoint(modelOutputs.get().position("camera", "origin"), Leph::ModelViewer::Cyan);
    }
    if (viewMode == 2)
    {
      viewer.addTrackedPoint(modelMotors.get().centerOfMass("origin"), Leph::ModelViewer::Yellow);
      viewer.addTrackedPoint(modelMotors.get().position("left_foot_tip", "origin"), Leph::ModelViewer::Red);
      viewer.addTrackedPoint(modelMotors.get().position("right_foot_tip", "origin"), Leph::ModelViewer::Green);
      viewer.addTrackedPoint(modelMotors.get().position("trunk", "origin"), Leph::ModelViewer::Blue);
      viewer.addTrackedPoint(modelMotors.get().position("camera", "origin"), Leph::ModelViewer::Cyan);
    }

    // Display models
    if (viewMode == 0)
    {
      Leph::ModelDraw(modelOutputs.get(), viewer);
      Leph::ModelDraw(modelMotors.get(), viewer);
      Leph::ModelDraw(modelWalk.get(), viewer);
    }
    if (viewMode == 1)
    {
      Leph::ModelDraw(modelOutputs.get(), viewer);
    }
    if (viewMode == 2)
    {
      Leph::ModelDraw(modelMotors.get(), viewer);
    }
    if (viewMode == 3)
    {
      Leph::ModelDraw(modelWalk.get(), viewer);
    }
    // Waiting
    scheduling.wait();
    // Phase cycling
    std::cout << "t= " << t << " index=" << indexLog << "/" << logs.size() - 1 << std::endl;
    if (!isPaused)
    {
      t += 1000.0 / freq;
    }
    if (t > logs[logs.size() - 1]("time:timestamp") + 2000.0)
    {
      t = logs[0]("time:timestamp");
      indexLog = 0;
    }
  }

  return 0;
}
