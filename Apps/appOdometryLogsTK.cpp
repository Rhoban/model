#include "Odometry/Odometry.hpp"
#include "Odometry/OdometrySequence.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cout << "Usage: ./app <output_file> <all_logs>"
              << std::endl;
    return 1;
  }

  std::string outputPath = argv[1];
  std::vector<std::string> logsPaths;

  for (int argNo = 2; argNo < argc; argNo++) {
    logsPaths.push_back(std::string(argv[argNo]));
  }

  // Reading logs
  std::vector<Leph::OdometrySequence> logs;
  for (const std::string & path : logsPaths) {
    std::vector<Leph::OdometrySequence> tmpLogs;
    loadOdometryDataFromFile(tmpLogs, path);
    std::cout << "Loading odometry data from: " 
              << path << " with " 
              << tmpLogs.size()
              << " sequences" << std::endl;
    logs.insert(logs.end(), tmpLogs.begin(), tmpLogs.end());
  }
  std::cout << "Total number of sequences: " << logs.size() << std::endl;

  std::vector<Leph::OdometrySequence> output_logs;
  for (const Leph::OdometrySequence & seq : logs) {
    // Writing only rows with step changes
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
    newSeq.targetDisplacements = seq.targetDisplacements;
    output_logs.push_back(newSeq);
  }

  dumpOdometryDataToFile(output_logs, outputPath);
}
