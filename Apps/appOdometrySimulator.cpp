#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "Model/HumanoidFixedModel.hpp"
#include "Odometry/Odometry.hpp"
#include "Odometry/OdometrySequence.hpp"
#include "Utils/Angle.h"

static Leph::OdometryType odometry_type = Leph::OdometryType::OdometryRead;

/**
 * Run simulations on the provided models, output is stored in a csv file
 * and contains:
 * - Noiseless trajectory
 * - Rollout results (terminal points)
 * - Target
 */
int main(int argc, char** argv)
{
  if (argc != 4)
  {
    std::cout << "Usage: ./app "
              << "odometry.data odometry.params output.csv" << std::endl;
    return 1;
  }

  std::string logPath = argv[1];
  std::string paramsPath = argv[2];
  std::string outputPath = argv[3];

  // Loading log sequences
  std::vector<Leph::OdometrySequence> logs;
  loadOdometryDataFromFile(logs, logPath);
  std::cout << "Loading odometry data from: " << logPath << " with " << logs.size() << " sequences" << std::endl;

  // Loading odometry
  Leph::Odometry odometry;
  odometry.loadFromFile(paramsPath);

  // Initializing random
  std::random_device rd;
  std::default_random_engine engine(rd());

  std::ofstream results(outputPath);

  // Writing header
  results << "log,type,step,x,y,theta" << std::endl;

  // Simulating deterministic trajectories
  for (size_t logId = 0; logId < logs.size(); logId++)
  {
    const Leph::OdometrySequence log = logs[logId];

    // Getting Noiseless trajectory and saving it
    std::vector<Eigen::Vector3d> trajectory;
    simulateOdometry(log, true, odometry, odometry_type, engine, &trajectory);
    for (size_t step = 0; step < trajectory.size(); step++)
    {
      results << logId << ",trajectory," << step << "," << trajectory[step](0) << "," << trajectory[step](1) << ","
              << trajectory[step](2) << std::endl;
    }

    // Sampling results
    // TODO: nb samples as a parameter?
    for (size_t sample = 0; sample < 500; sample++)
    {
      simulateOdometry(log, false, odometry, odometry_type, engine, nullptr);
      results << logId << ",sample,NA," << odometry.state()(0) << "," << odometry.state()(1) << ","
              << odometry.state()(2) << std::endl;
    }

    // Saving observation (converting hours to rad)
    double obs_theta = Leph::AngleBound(-log.targetDisplacements(2) * 2.0 * M_PI / 12);
    results << logId << ",observation,NA," << log.targetDisplacements(0) << "," << log.targetDisplacements(1) << ","
            << obs_theta << std::endl;
  }

  results.close();

  return 0;
}
