#include "Odometry/Odometry.hpp"
#include "Odometry/OdometrySequence.hpp"

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cout << "Usage: ./app "
              << "input.data odometry.conf output.data" 
              << std::endl;
    return 1;
  }

  std::string inputPath = argv[1];
  std::string odomPath = argv[2];
  std::string outPath = argv[3];

  // Reading logs
  std::vector<Leph::OdometrySequence> logs;
  loadOdometryDataFromFile(logs, inputPath);
  std::cout << "Loading odometry data from: " 
            << inputPath << " with " 
            << logs.size()
            << " sequences" << std::endl;

  //Loading Odometry config
  Leph::Odometry odom;
  odom.loadFromFile(odomPath);

  // Unused, but still required from 
  std::default_random_engine engine;

  std::vector<Leph::OdometrySequence> output_logs;
  for (const Leph::OdometrySequence & seq : logs) {
    // Simulating odometry without noise
    Eigen::VectorXd final_pos;
    std::vector<Eigen::Vector3d> positions;
    //TODO: change OdometryGoal to OdometryRead
    final_pos  = simulateOdometry(seq, true, odom,
                                  Leph::OdometryType::OdometryGoal,
                                  engine, &positions);
    // Writing only rows for step changes
    Leph::OdometrySequence newSeq;
    int last_step = -1;
    for (size_t row = 0; row < seq.getNbRows(); row++) {
      int step = seq.stepIndices[row];
      if (step != last_step) {
        //TODO: Still requiring to create sub-logs
        newSeq.timestamps.push_back      (seq.timestamps[row]      );
        newSeq.stepIndices.push_back     (seq.stepIndices[row]     );
        newSeq.readTrajsPose.push_back   (seq.readTrajsPose[row]   );
        newSeq.readTrajsSupport.push_back(seq.readTrajsSupport[row]);
        newSeq.goalTrajsPose.push_back   (seq.goalTrajsPose[row]   );
        newSeq.goalTrajsSupport.push_back(seq.goalTrajsSupport[row]);
        newSeq.walkTrajsOrder.push_back  (seq.walkTrajsOrder[row]  );
        newSeq.walkTrajsPhase.push_back  (seq.walkTrajsPhase[row]  );
        last_step = step;
      }
    }
    newSeq.targetDisplacements = final_pos;
    output_logs.push_back(newSeq);
  }

  dumpOdometryDataToFile(output_logs, outPath);
}
