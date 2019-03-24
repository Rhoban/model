#include <iostream>
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryGeneration/TrajectoryDisplay.h"

int main(int argc, char** argv)
{
  // Check command line
  if (argc < 2)
  {
    std::cout << "Usage: ./app [Trajectories filename] [MODEL] [file.modelparams]" << std::endl;
    return 1;
  }
  std::string filename = std::string(argv[1]);
  std::string modelParamsFile;
  if (argc == 4 && std::string(argv[2]) == "MODEL")
  {
    modelParamsFile = argv[3];
    std::cout << "Loading model parameters from: " << modelParamsFile << std::endl;
  }

  // Load Trajectories
  Leph::Trajectories traj;
  traj.importData(filename);

  // Display Splines
  for (const auto& it : traj.get())
  {
    std::cout << "### SmoothSpline " << it.first << ":" << std::endl;
    for (size_t i = 0; i < it.second.size(); i++)
    {
      std::cout << "Part " << i << ": "
                << "begin=" << it.second.part(i).min << " end=" << it.second.part(i).max << std::endl
                << "    "
                << "posBegin=" << it.second.pos(it.second.part(i).min)
                << " velBegin=" << it.second.vel(it.second.part(i).min)
                << " accBegin=" << it.second.acc(it.second.part(i).min) << std::endl
                << "    "
                << "posEnd=" << it.second.pos(it.second.part(i).max)
                << " velEnd=" << it.second.vel(it.second.part(i).max)
                << " accEnd=" << it.second.acc(it.second.part(i).max) << std::endl;
    }
    const auto& points = it.second.points();
    for (size_t i = 0; i < points.size(); i++)
    {
      std::cout << "Point " << i << ": "
                << "time=" << points[i].time << " pos=" << points[i].position << " vel=" << points[i].velocity
                << " acc=" << points[i].acceleration << std::endl;
    }
  }

  // Display Trajectory
  Leph::TrajectoriesDisplay(traj, Leph::SigmabanModel, modelParamsFile);

  return 0;
}
