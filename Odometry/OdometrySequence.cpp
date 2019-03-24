#include "Odometry/OdometrySequence.hpp"

#include <fstream>

namespace Leph
{
void OdometrySequence::pushEntry(double timestamp, int stepIndex, Leph::HumanoidFixedModel& readModel,
                                 Leph::HumanoidFixedModel& goalModel, const Eigen::Vector4d& walkOrder,
                                 double walkPhase)
{
  timestamps.push_back(timestamp);
  stepIndices.push_back(stepIndex);
  readTrajsPose.push_back(readModel.get().getPose());
  readTrajsSupport.push_back(readModel.getSupportFoot());
  goalTrajsPose.push_back(goalModel.get().getPose());
  goalTrajsSupport.push_back(goalModel.getSupportFoot());
  walkTrajsOrder.push_back(walkOrder);
  walkTrajsPhase.push_back(walkPhase);
}

size_t OdometrySequence::getNbRows() const
{
  return timestamps.size();
}

static std::string odometryDataHeader()
{
  std::ostringstream oss;
  oss << "seqId "
      << "rowId "
      << "timestamp "
      << "stepIndex "
      << "readTrajPoseX "
      << "readTrajPoseY "
      << "readTrajPoseZ "
      << "readTrajSupport "
      << "goalTrajPoseX "
      << "goalTrajPoseY "
      << "goalTrajPoseZ "
      << "goalTrajSupport "
      << "walkOrderX "
      << "walkOrderY "
      << "walkOrderZ "
      << "walkSmoothing "
      << "walkPhase "
      << "targetX "
      << "targetY "
      << "targetZ";
  return oss.str();
}

void dumpOdometryDataToFile(const std::vector<OdometrySequence>& data, const std::string& filename)
{
  std::ofstream file(filename);
  file << odometryDataHeader() << std::endl;
  for (size_t seqId = 0; seqId < data.size(); seqId++)
  {
    const OdometrySequence& seq = data[seqId];
    for (size_t row = 0; row < seq.readTrajsPose.size(); row++)
    {
      file << seqId << " " << row << " " << seq.timestamps[row] << " " << seq.stepIndices[row] << " "
           << seq.readTrajsPose[row].x() << " " << seq.readTrajsPose[row].y() << " " << seq.readTrajsPose[row].z()
           << " " << (int)seq.readTrajsSupport[row] << " " << seq.goalTrajsPose[row].x() << " "
           << seq.goalTrajsPose[row].y() << " " << seq.goalTrajsPose[row].z() << " " << (int)seq.goalTrajsSupport[row]
           << " " << seq.walkTrajsOrder[row].x() << " " << seq.walkTrajsOrder[row].y() << " "
           << seq.walkTrajsOrder[row].z() << " " << seq.walkTrajsOrder[row](3) << " " << seq.walkTrajsPhase[row] << " "
           << seq.targetDisplacements.x() << " " << seq.targetDisplacements.y() << " " << seq.targetDisplacements.z()
           << std::endl;
    }
  }
  file.close();
}

void loadOdometryDataFromFile(std::vector<OdometrySequence>& data, const std::string& filename)
{
  // Open file
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("Unable to open log file: " + filename);
  }

  std::string line;
  std::getline(file, line);
  if (line != odometryDataHeader())
  {
    throw std::logic_error("Invalid header for OdometrySequence: '" + line + "'");
  }

  // Loop over file entries
  size_t lastSeq = -1;
  while (file.good() && file.peek() != EOF)
  {
    // Skip end of line
    while (file.peek() == ' ' || file.peek() == '\n')
    {
      file.ignore();
    }
    if (!file.good() || file.peek() == EOF)
    {
      break;
    }
    // Retrieve one file line
    size_t seq;
    size_t index;
    double timestamp;
    int stepIndex;
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
    file >> timestamp;
    file >> stepIndex;
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
    if (lastSeq != seq)
    {
      // Start a new sequence
      data.push_back(OdometrySequence());
      data.back().targetDisplacements = Eigen::Vector3d(targetX, targetY, targetA);
    }
    lastSeq = seq;
    data.back().timestamps.push_back(timestamp);
    data.back().stepIndices.push_back(stepIndex);
    data.back().readTrajsPose.push_back(Eigen::Vector3d(readPoseX, readPoseY, readPoseYaw));
    data.back().readTrajsSupport.push_back((HumanoidFixedModel::SupportFoot)readSupportFoot);
    data.back().goalTrajsPose.push_back(Eigen::Vector3d(goalPoseX, goalPoseY, goalPoseYaw));
    data.back().goalTrajsSupport.push_back((HumanoidFixedModel::SupportFoot)goalSupportFoot);
    data.back().walkTrajsOrder.push_back(Eigen::Vector4d(walkOrderX, walkOrderY, walkOrderTheta, walkOrderEnabled));
    data.back().walkTrajsPhase.push_back(walkPhase);
  }
  file.close();
}

Eigen::VectorXd simulateOdometry(const OdometrySequence& data, bool noRandom, Odometry& odometry,
                                 OdometryType odometry_type, std::default_random_engine& engine,
                                 std::vector<Eigen::Vector3d>* positions)
{
  odometry.reset();

  std::default_random_engine* usedEngine = &engine;
  if (noRandom)
  {
    // Disable the random engine to disable
    // the noise model in odometry
    usedEngine = nullptr;
  }

  // Iterate over recorded point inside the sequence
  int lastStep = -1;
  int lastSwapIndex = -1;
  for (size_t i = 0; i < data.readTrajsPose.size(); i++)
  {
    int stepIndex = data.stepIndices[i];
    // Update only on step changes
    if (stepIndex > lastStep)
    {
      // Use given odometry type to compute rawDelta (before correction
      Eigen::Vector3d rawDelta = Eigen::Vector3d::Zero();
      if (odometry_type == OdometryOrder)
      {
        Eigen::Vector3d walkOrders = data.walkTrajsOrder[i].segment(0, 3);
        double enableGain = data.walkTrajsOrder[i](3);
        rawDelta = walkOrders * enableGain;
      }
      else if (odometry_type == OdometryGoal)
      {
        if (lastSwapIndex >= 0)
        {
          rawDelta = odometry.odometryDiff(data.goalTrajsPose[lastSwapIndex], data.goalTrajsPose[i]);
        }
      }
      else if (odometry_type == OdometryRead)
      {
        if (lastSwapIndex >= 0)
        {
          rawDelta = odometry.odometryDiff(data.readTrajsPose[lastSwapIndex], data.readTrajsPose[i]);
        }
      }
      else
      {
        throw std::logic_error("Invalid odometry type");
      }
      odometry.updateFullStep(rawDelta, usedEngine);
      // If user asked positions provide them
      if (positions != nullptr)
      {
        positions->push_back(odometry.state());
      }
      // Update lastSwapIndex
      lastSwapIndex = i;
    }
    lastStep = stepIndex;
  }

  // Final integrated state
  return odometry.state();
}

}  // namespace Leph
