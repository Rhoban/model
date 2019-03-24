#include <iostream>
#include "Types/MapSeries.hpp"

int main(int argc, char** argv)
{
  if (argc != 5)
  {
    std::cout << "Usage: ./app original.mapseries "
              << "timeBegin timeEnd output.mapseries" << std::endl;
    return 1;
  }
  std::string inputFile = argv[1];
  double timeBegin = std::stod(argv[2]);
  double timeEnd = std::stod(argv[3]);
  std::string outputFile = argv[4];

  std::cout << "Cutting mapseries: " << inputFile << " to [" << timeBegin << ":" << timeEnd << "] --> " << outputFile
            << std::endl;
  Leph::MapSeries mapOriginal;
  mapOriginal.importData(inputFile);
  Leph::MapSeries mapCut = mapOriginal.sliceTimeRange(timeBegin, timeEnd);
  mapCut.exportData(outputFile);

  return 0;
}
