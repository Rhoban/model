#include <iostream>
#include <iomanip>
#include <thread>
#include "Types/MapSeries.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

/**
 * Assign given model with state at given time
 */
static void assignModelState(Leph::HumanoidFixedModel& model, const Leph::MapSeries& series, const std::string& prefix,
                             double t)
{
  // Assign degrees of freedom position
  for (const std::string& name : Leph::NamesDOF)
  {
    model.get().setDOF(name, series.get(prefix + ":" + name, t));
  }
  // Assign base state
  for (const std::string& name : Leph::NamesBase)
  {
    model.get().setDOF(name, series.get(prefix + ":" + name, t));
  }
}

/**
 * AppMapSeriesLogsViewer
 *
 * Laod and display a log in map series
 * format with 2016 goal/read state format
 *
 * Show goal or read data
 */
int main(int argc, char** argv)
{
  // Parse arguments
  if (argc != 3 || (std::string(argv[2]) != "goal" && std::string(argv[2]) != "read"))
  {
    std::cout << "Usage: ./app [log_file] [goal|read]" << std::endl;
    return 1;
  }
  std::string logFileName = argv[1];
  std::string prefix = argv[2];

  // Load data
  std::cout << "Loading " << logFileName << " with prefix " << prefix << std::endl;
  Leph::MapSeries series;
  series.importData(logFileName);
  // Print statistics
  std::cout << "Loaded " << series.dimension() << " series from " << series.timeMin() << "s to " << series.timeMax()
            << "s with length " << series.timeMax() - series.timeMin() << "s" << std::endl;
  std::vector<std::string> seriesNames = series.allNames();
  for (const std::string& name : seriesNames)
  {
    std::cout << "[" << name << "] " << series.size(name) << " points from " << series.timeMin(name) << "s to "
              << series.timeMax(name) << "s with length " << series.timeMax(name) - series.timeMin(name) << "s"
              << std::endl;
  }

  // Show logged time series
  series.plot().plot("time", prefix + ":*").render();

  // Compute cartesian position
  std::cout << "Compute cartesian pose" << std::endl;
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  double boundMin = series.timeMin();
  double boundMax = series.timeMax();
  for (double t = boundMin; t <= boundMax; t += 0.01)
  {
    assignModelState(model, series, prefix, t);
    Eigen::Vector3d trunkPos = model.get().position("trunk", "left_foot_tip");
    Eigen::Vector3d trunkAngles = Leph::MatrixToAxis(model.get().orientation("trunk", "left_foot_tip").transpose());
    Eigen::Vector3d footPos = model.get().position("right_foot_tip", "left_foot_tip");
    Eigen::Vector3d footAngles =
        Leph::MatrixToAxis(model.get().orientation("right_foot_tip", "left_foot_tip").transpose());
    series.append(prefix + ":trunk_x", t, trunkPos.x());
    series.append(prefix + ":trunk_y", t, trunkPos.y());
    series.append(prefix + ":trunk_z", t, trunkPos.z());
    series.append(prefix + ":trunk_roll", t, trunkAngles.x());
    series.append(prefix + ":trunk_pitch", t, trunkAngles.y());
    series.append(prefix + ":trunk_yaw", t, trunkAngles.z());
    series.append(prefix + ":foot_x", t, footPos.x());
    series.append(prefix + ":foot_y", t, footPos.y());
    series.append(prefix + ":foot_z", t, footPos.z());
    series.append(prefix + ":foot_roll", t, footAngles.x());
    series.append(prefix + ":foot_pitch", t, footAngles.y());
    series.append(prefix + ":foot_yaw", t, footAngles.z());
  }

  // Show cartesian pose
  series.plot()
      .plot("time", prefix + ":trunk_x")
      .plot("time", prefix + ":trunk_y")
      .plot("time", prefix + ":trunk_z")
      .plot("time", prefix + ":foot_x")
      .plot("time", prefix + ":foot_y")
      .plot("time", prefix + ":foot_z")
      .render();
  series.plot()
      .plot("time", prefix + ":trunk_roll")
      .plot("time", prefix + ":trunk_pitch")
      .plot("time", prefix + ":trunk_yaw")
      .plot("time", prefix + ":foot_roll")
      .plot("time", prefix + ":foot_pitch")
      .plot("time", prefix + ":foot_yaw")
      .render();
  series.plot().plot("time", "volt_power:*").render();
  series.plot().plot("time", "volt_control:*").render();

  // Display interface usage
  std::cout << "Usage:" << std::endl;
  std::cout << "N: increase time by 1s" << std::endl;
  std::cout << "P: decrease time by 1s" << std::endl;
  std::cout << "H: double time increment" << std::endl;
  std::cout << "L: half time increment" << std::endl;
  std::cout << "R: restart time to begin" << std::endl;
  std::cout << "Space: pause" << std::endl;

  Leph::ModelViewer viewer(1200, 900);
  double t = series.timeMin();
  bool isPaused = false;
  double delta = 0.01;
  while (true)
  {
    // Assign the model state
    assignModelState(model, series, prefix, t);
    // Show the model
    Leph::ModelDraw(model.get(), viewer);
    // Interface controls
    bool isContinue = viewer.update();
    if (!isContinue)
    {
      break;
    }
    if (viewer.isKeyPressed(sf::Keyboard::N))
    {
      t += 1.0;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (viewer.isKeyPressed(sf::Keyboard::P))
    {
      t -= 1.0;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (viewer.isKeyPressed(sf::Keyboard::H))
    {
      delta *= 2.0;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (viewer.isKeyPressed(sf::Keyboard::L))
    {
      delta /= 2.0;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (viewer.isKeyPressed(sf::Keyboard::R))
    {
      t = series.timeMin();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (viewer.isKeyPressed(sf::Keyboard::Space))
    {
      isPaused = !isPaused;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // Time display and update
    std::cout << '\r' << "t=" << std::setw(10) << std::setfill(' ') << t;
    std::flush(std::cout);
    if (!isPaused)
    {
      t += delta;
    }
    // Time bound
    if (t > series.timeMax())
    {
      t = series.timeMax();
    }
    if (t < series.timeMin())
    {
      t = series.timeMin();
    }
  }
  std::cout << std::endl;

  return 0;
}
