#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>
#include "Types/VectorLabel.hpp"
#include "Types/MatrixLabel.hpp"
#include "Plot/Plot.hpp"

int main(int argc, char** argv)
{
  // Usage
  if (argc != 2)
  {
    std::cout << "Usage: ./app <log-file.csv>" << std::endl;
    return 1;
  }

  // Data loading
  unsigned long dataLines = 0;
  Leph::Plot plot;
  Leph::VectorLabel labels;
  std::ifstream logFile(argv[1]);
  if (!logFile.is_open())
  {
    throw std::runtime_error("Unable to open file: " + std::string(argv[1]));
  }
  std::cout << "Loading data..." << std::endl;
  bool isEnd = false;
  while (!isEnd)
  {
    Leph::VectorLabel tmp;
    isEnd = !tmp.readFromCSV(logFile);
    dataLines++;
    labels.mergeUnion(tmp);
    plot.add(tmp);
  }
  logFile.close();

  std::cout << "Loaded " << dataLines << std::endl;
  std::cout << "Labels:" << std::endl;
  for (size_t i = 0; i < labels.size(); i++)
  {
    std::cout << labels.getLabel(i) << std::endl;
  }

  plot.plot("index", "fitness:lateral", Leph::Plot::LinesPoints)
      .plot("index", "fitness:step", Leph::Plot::LinesPoints)
      .plot("index", "fitness:turn", Leph::Plot::LinesPoints)
      .render();

  plot.plot("static:swingGain", "static:swingPhase", "static:timeGain", Leph::Plot::Points, "fitness:lateral")
      .plot("state:swingGain", "state:swingPhase", "state:timeGain", Leph::Plot::LinesPoints)
      .render();

  plot.plot("static:hipOffset", "static:xOffset", "fitness:lateral", Leph::Plot::Points)
      //.plot("state:hipOffset", "state:xOffset", "ZERO", Leph::Plot::LinesPoints)
      .render();
  plot.plot("static:hipOffset", "static:xOffset", "fitness:step", Leph::Plot::Points).render();

  return 0;
}
