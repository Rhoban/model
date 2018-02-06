#pragma once

#include "Odometry/Odometry.hpp"
#include "Model/HumanoidFixedModel.hpp"

#include <Eigen/StdVector>

namespace Leph {

/**
 * Odometry computation type
 */
enum OdometryType {
  //No model. Use walk order as input
  OdometryOrder,
  //Use goal model state as input
  OdometryGoal,
  //Use read model state as input
  OdometryRead,
};

/**
 * Odometry data for one 
 * recorded sequence
 */
class OdometrySequence {
public:
  //Model read and goal model cartesian 
  //pose in origin at each logged point
  //(X,Y,Theta).
  std::vector<double> timestamps;
  std::vector<int> stepIndices;
  std::vector<Eigen::Vector3d> readTrajsPose;
  std::vector<HumanoidFixedModel::SupportFoot> readTrajsSupport;
  std::vector<Eigen::Vector3d> goalTrajsPose;
  std::vector<HumanoidFixedModel::SupportFoot> goalTrajsSupport;
  //Walk step, lateral, turn and phase 
  //walk order at each logged point 
  std::vector<Eigen::Vector4d,
              Eigen::aligned_allocator<Eigen::Vector4d>> walkTrajsOrder;
  std::vector<double> walkTrajsPhase;
  //Target observed cartesian 
  //displacement  (X,Y,Theta)
  Eigen::Vector3d targetDisplacements;

  void pushEntry(double timestamp,
                 int stepIndex,
                 Leph::HumanoidFixedModel & readModel,
                 Leph::HumanoidFixedModel & goalModel,
                 const Eigen::Vector4d & walkOrder,
                 double walkPhase);

  size_t getNbRows() const;
};

void dumpOdometryDataToFile(const std::vector<OdometrySequence> & data, 
                            const std::string & filename);

/**
 * Append data loaded from fiven filename to
 * given OdometrySequence container
 */
void loadOdometryDataFromFile(
  std::vector<OdometrySequence>& data, 
  const std::string& filename);

/**
 * Compute and return the estimation
 * (try to predict observation) from
 * given data and using given parameter
 * if positions is not null, push all the successive
 * positions of the robot inside the vector
 */
Eigen::VectorXd simulateOdometry(
  const OdometrySequence& data,
  bool noRandom,
  Odometry& odometry,
  OdometryType odometry_type,
  std::default_random_engine& engine,
  std::vector<Eigen::Vector3d> * positions);

}
