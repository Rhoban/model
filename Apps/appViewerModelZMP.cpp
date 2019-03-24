#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Utils/Differentiation.hpp"
#include "Model/HumanoidFixedPressureModel.hpp"
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

  // Pärsing robot model type
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

  // Loading data
  std::string logsFile = argv[2];
  std::cout << "Loading " << logsFile << std::endl;
  Leph::MatrixLabel logs;
  logs.load(logsFile);

  // Print data informations
  std::cout << "Loaded " << logs.size() << " points with " << logs.dimension() << " entries" << std::endl;
  if (logs.size() == 0)
  {
    return 0;
  }

  // Plotting
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

  // Initialize model instances
  Leph::HumanoidFixedPressureModel modelMotors(type);
  Leph::ModelViewer viewer(1200, 900);
  Leph::Scheduling scheduling;

  // Initialize DOF vector
  Leph::VectorLabel pos = logs[0].extract("motor").rename("motor", "");

  // Recompute base pitch and roll
  // using pressure model
  for (size_t i = 0; i < logs.size(); i++)
  {
    pos.assignOp(logs[i], "motor", "");
    modelMotors.get().setDOF(pos, false);
    modelMotors.setPressure(logs[i]("pressure:w"), logs[i]("pressure:w1") / 1000.0 / logs[i]("pressure:w"),
                            logs[i]("pressure:w2") / 1000.0 / logs[i]("pressure:w"), logs[i]("pressure:x1") / 100.0,
                            logs[i]("pressure:y1") / 100.0, logs[i]("pressure:x2") / 100.0,
                            logs[i]("pressure:y2") / 100.0);
    modelMotors.updateBase();
    modelMotors.setOrientation(
        Eigen::AngleAxisd(logs[i]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(logs[i]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
        false);
    logs[i]("motor:base_pitch") = modelMotors.get().getDOF("base_pitch");
    logs[i]("motor:base_roll") = modelMotors.get().getDOF("base_roll");
  }
  modelMotors.get().setDOF("base_x", 0.0);
  modelMotors.get().setDOF("base_y", 0.0);
  modelMotors.get().setDOF("base_z", 0.0);
  modelMotors.get().setDOF("base_yaw", 0.0);

  // Initializing DOF rolling buffer
  // for differentiation
  int bufferLen = 40;
  Leph::Differentiation buffer(bufferLen, 3);

  Leph::Plot plot;
  // Main loop
  scheduling.setFrequency(50.0);
  bool isPaused = true;
  size_t indexLog = bufferLen;
  while (viewer.update())
  {
    // Interface control
    if (viewer.isKeyPressed(sf::Keyboard::P))
    {
      isPaused = !isPaused;
    }
    // Retrieve read DOF
    pos.assignOp(logs[indexLog], "motor", "");
    // Appending to rolling buffer
    if (!isPaused)
    {
      buffer.add(logs[indexLog]("time:timestamp") / 1000.0, pos);
    }
    // Compute smooth position, velocity and acceleration
    Leph::VectorLabel vel = pos;
    Leph::VectorLabel acc = pos;
    if (buffer.isFull())
    {
      double middleT = logs[indexLog - bufferLen / 2]("time:timestamp") / 1000.0;
      pos = buffer.position(middleT);
      vel = buffer.velocity(middleT);
      acc = buffer.acceleration(middleT);
    }
    // Assigning to model
    modelMotors.get().setDOF(pos, false);
    // Assiging pressure
    // XXX Sigmaban and old pressure name convention
    // XXX is used
    modelMotors.setPressure(
        logs[indexLog - bufferLen / 2]("pressure:w"),
        logs[indexLog - bufferLen / 2]("pressure:w1") / 1000.0 / logs[indexLog - bufferLen / 2]("pressure:w"),
        logs[indexLog - bufferLen / 2]("pressure:w2") / 1000.0 / logs[indexLog - bufferLen / 2]("pressure:w"),
        logs[indexLog - bufferLen / 2]("pressure:x1") / 100.0, logs[indexLog - bufferLen / 2]("pressure:y1") / 100.0,
        logs[indexLog - bufferLen / 2]("pressure:x2") / 100.0, logs[indexLog - bufferLen / 2]("pressure:y2") / 100.0);
    // Contraint the model on the ground, integrate movement
    modelMotors.updateBase();
    // Using smoothed base pitch and roll
    modelMotors.get().setDOF("base_pitch", pos("base_pitch"));
    modelMotors.get().setDOF("base_roll", pos("base_roll"));
    // Compute ZMP point
    Eigen::Vector3d zmp /*TODO XXX API change modelMotors.zeroMomentPoint(
        "origin", vel, acc) */
        = Eigen::Vector3d(0, 0, 0);
    // Compute center of pressure form model
    // Display captured center of mass ground projection
    Eigen::Vector3d copLeft = modelMotors.centerOfPressureLeft("origin");
    Eigen::Vector3d copRight = modelMotors.centerOfPressureRight("origin");
    Eigen::Vector3d copMiddle = modelMotors.centerOfPressure("origin");
    // Adding to plot
    plot.add(Leph::VectorLabel("t", logs[indexLog - bufferLen / 2]("time:timestamp"), "cop_x", copMiddle.x(), "cop_y",
                               copMiddle.y(), "zmp_x", zmp.x(), "zmp_y", zmp.y()));
    // Display centers of pressures trajectory
    copLeft.z() = 0.0;
    copRight.z() = 0.0;
    copMiddle.z() = 0.0;
    viewer.addTrackedPoint(copMiddle, Leph::ModelViewer::Yellow);
    // Display foot pressure force
    viewer.drawBox(0.005, 0.005, 0.1 * modelMotors.pressureLeftRatio(),
                   copLeft + Eigen::Vector3d(0, 0, 0.1 * modelMotors.pressureLeftRatio()), Eigen::Matrix3d::Identity(),
                   1.0, 0.0, 0.0);
    viewer.drawBox(0.005, 0.005, 0.1 * modelMotors.pressureRightRatio(),
                   copRight + Eigen::Vector3d(0, 0, 0.1 * modelMotors.pressureRightRatio()),
                   Eigen::Matrix3d::Identity(), 0.0, 1.0, 0.0);
    // Display ZMP
    viewer.addTrackedPoint(zmp, Leph::ModelViewer::Cyan);
    // Display model
    Leph::ModelDraw(modelMotors.get(), viewer);
    // Waiting
    scheduling.wait();
    // Phase cycling
    std::cout << "t= " << logs[indexLog]("time:timestamp") << " index=" << indexLog << "/" << logs.size() - 1
              << std::endl;
    if (indexLog < logs.size() - 1)
    {
      if (!isPaused)
        indexLog++;
    }
    else
    {
      indexLog = 0;
    }
  }
  plot.plot("t", "cop_x").plot("t", "zmp_x").render();
  plot.plot("t", "cop_y").plot("t", "zmp_y").render();

  return 0;
}
