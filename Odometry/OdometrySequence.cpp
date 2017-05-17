#include "Odometry/OdometrySequence.hpp"

#include <fstream>

namespace Leph
{


void loadOdometryDataFromFile(
  std::vector<OdometrySequence>& data, 
  const std::string& filename)
{
  //Open file
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error(
      "Unable to open log file: " + filename);
  }

  //Loop over file entries
  size_t lastSeq = -1;
  while (file.good() && file.peek() != EOF) {
    //Skip end of line
    while (file.peek() == ' ' || file.peek() == '\n') {
      file.ignore();
    }
    if (!file.good() || file.peek() == EOF) {
      break;
    }
    //Retrieve one file line
    size_t seq;
    size_t index;
    double readPoseX;
    double readPoseY;
    double readPoseYaw;
    int readSupportFoot;
    double goalPoseX;
    double goalPoseY;
    double goalPoseYaw;
    int goalSupportFoot;
    double walkOrderX;
    double walkOrderY;
    double walkOrderTheta;
    double walkOrderEnabled;
    double walkPhase;
    double targetX;
    double targetY;
    double targetA;
    file >> seq;
    file >> index;
    file >> readPoseX;
    file >> readPoseY;
    file >> readPoseYaw;
    file >> readSupportFoot;
    file >> goalPoseX;
    file >> goalPoseY;
    file >> goalPoseYaw;
    file >> goalSupportFoot;
    file >> walkOrderX;
    file >> walkOrderY;
    file >> walkOrderTheta;
    file >> walkOrderEnabled;
    file >> walkPhase;
    file >> targetX;
    file >> targetY;
    file >> targetA;
    if (lastSeq != seq) {
      //Start a new sequence
      data.push_back(OdometrySequence());
      data.back().targetDisplacements = 
        Eigen::Vector3d(targetX, targetY, targetA);
    }
    lastSeq = seq;
    data.back().readTrajsPose.push_back(
      Eigen::Vector3d(readPoseX, readPoseY, readPoseYaw));
    data.back().readTrajsSupport.push_back(
      (HumanoidFixedModel::SupportFoot)readSupportFoot);
    data.back().goalTrajsPose.push_back(
      Eigen::Vector3d(goalPoseX, goalPoseY, goalPoseYaw));
    data.back().goalTrajsSupport.push_back(
      (HumanoidFixedModel::SupportFoot)goalSupportFoot);
    data.back().walkTrajsOrder.push_back(
      Eigen::Vector4d(walkOrderX, walkOrderY, walkOrderTheta, walkOrderEnabled));
    data.back().walkTrajsPhase.push_back(
      walkPhase);
  }
  file.close();
}

Eigen::VectorXd simulateOdometry(
  const OdometrySequence& data,
  bool noRandom,
  Odometry& odometry,
  OdometryType odometry_type,
  std::default_random_engine& engine,
  std::vector<Eigen::Vector3d> * positions)
{
  odometry.reset();

  std::default_random_engine* usedEngine = &engine;
  if (noRandom) {
    //Disable the random engine to disable
    //the noise model in odometry
    usedEngine = nullptr;
  }

  //Iterate over recorded point inside the sequence
  double lastPhase = data.walkTrajsPhase.front();
  for (size_t i=0;i<data.readTrajsPose.size();i++) {
    double phase = data.walkTrajsPhase[i];
    bool skipStep = false;
    //Use given odometry type
    if (odometry_type == OdometryOrder) {
      if (lastPhase > 0.8 && phase < 0.2) {
        double enableGain = data.walkTrajsOrder[i](3);
        odometry.updateFullStep(
          2.0*data.walkTrajsOrder[i].segment(0, 3)
          *enableGain,
          usedEngine);
      }
      else {
        skipStep = true;
      }
    } else if (odometry_type == OdometryGoal) {
      odometry.update(
        data.goalTrajsPose[i], 
        data.goalTrajsSupport[i],
        usedEngine);
    } else if (odometry_type == OdometryRead) {
      odometry.update(
        data.readTrajsPose[i], 
        data.readTrajsSupport[i],
        usedEngine);
    } else {
      throw std::logic_error("Invalid odometry type");
    }
    lastPhase = phase;
    if (positions != nullptr && !skipStep) {
      positions->push_back(odometry.state());
    }
  }

  //Final integrated state
  return odometry.state();
}

}
