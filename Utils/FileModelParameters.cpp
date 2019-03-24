#include <iostream>
#include <fstream>
#include <iomanip>
#include "Utils/FileModelParameters.h"
#include "Utils/FileMap.h"
#include "Utils/FileEigen.h"

namespace Leph
{
void WriteModelParameters(const std::string& filename, const Eigen::MatrixXd& jointData,
                          const std::map<std::string, size_t>& jointName, const Eigen::MatrixXd& inertiaData,
                          const std::map<std::string, size_t>& inertiaName, const Eigen::MatrixXd& geometryData,
                          const std::map<std::string, size_t>& geometryName)
{
  // Open file
  std::ofstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("WriteModelParameters unable to open file: " + filename);
  }
  // Write joint
  WriteEigenMatrixToStream(file, jointData);
  WriteMapToStream(file, jointName);
  // Write inertia
  WriteEigenMatrixToStream(file, inertiaData);
  WriteMapToStream(file, inertiaName);
  // Write geometry
  WriteEigenMatrixToStream(file, geometryData);
  WriteMapToStream(file, geometryName);
  file.close();
}

void ReadModelParameters(const std::string& filename, Eigen::MatrixXd& jointData,
                         std::map<std::string, size_t>& jointName, Eigen::MatrixXd& inertiaData,
                         std::map<std::string, size_t>& inertiaName, Eigen::MatrixXd& geometryData,
                         std::map<std::string, size_t>& geometryName)
{
  // Open file
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("ReadModelParameters unable to open file: " + filename);
  }
  // Read joint
  jointData = ReadEigenMatrixFromStream(file);
  jointName = ReadMapFromStream<std::string, size_t>(file);
  // Read inertia
  inertiaData = ReadEigenMatrixFromStream(file);
  inertiaName = ReadMapFromStream<std::string, size_t>(file);
  // Read geometry
  geometryData = ReadEigenMatrixFromStream(file);
  geometryName = ReadMapFromStream<std::string, size_t>(file);
  file.close();
}

}  // namespace Leph
