#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <Eigen/Dense>
#include "Model/HumanoidModel.hpp"
#include "Model/JointModel.hpp"
#include "Utils/FileModelParameters.h"
#include "Model/NamesModel.h"

/**
 * Print a formated line with given
 * label and two values
 */
void printLine(const std::string& label1, const std::string& label2, double val1, double val2)
{
  std::cout << std::setw(40) << label1 << " " << std::setw(40) << label2 << " " << std::setw(15) << std::fixed
            << std::setprecision(10) << val1 << " " << std::setw(15) << std::fixed << std::setprecision(10) << val2
            << " " << std::setw(10) << std::fixed << std::setprecision(3) << val1 / val2 << " " << std::endl;
}

/**
 * Display and compare given two
 * inertia parameters with given name
 */
void printInertiaParams(const std::string& name, const Eigen::VectorXd& data1, const Eigen::VectorXd& data2)
{
  printLine(name, "inertia_mass", data1(0), data2(0));
  printLine(name, "inertia_x", data1(1), data2(1));
  printLine(name, "inertia_y", data1(2), data2(2));
  printLine(name, "inertia_z", data1(3), data2(3));
  printLine(name, "inertia_ixx", data1(4), data2(4));
  printLine(name, "inertia_ixy", data1(5), data2(5));
  printLine(name, "inertia_ixz", data1(6), data2(6));
  printLine(name, "inertia_iyy", data1(7), data2(7));
  printLine(name, "inertia_iyz", data1(8), data2(8));
  printLine(name, "inertia_izz", data1(9), data2(9));
}

/**
 * Print given model parameters from file
 * and compare with default value
 */
int main(int argc, char** argv)
{
  // Parse user input
  if (argc != 2)
  {
    std::cout << "Usage: ./app file.modelparams" << std::endl;
    return 1;
  }
  std::string filename = argv[1];

  // Inertia default data and name
  Eigen::MatrixXd defaultInertiaData;
  std::map<std::string, size_t> defaultInertiaName;
  // Geometry default data and name
  Eigen::MatrixXd defaultGeometryData;
  std::map<std::string, size_t> defaultGeometryName;
  // Load default inertia and geometry data from model
  Leph::HumanoidModel tmpModel(Leph::SigmabanModel, "left_foot_tip", false);
  defaultInertiaData = tmpModel.getInertiaData();
  defaultInertiaName = tmpModel.getInertiaName();
  defaultGeometryData = tmpModel.getGeometryData();
  defaultGeometryName = tmpModel.getGeometryName();
  // Load default joint model parameters
  Leph::JointModel tmpJoint;
  Eigen::VectorXd defaultJointParams = tmpJoint.getParameters();
  // Build default joint parameters matrix
  Eigen::MatrixXd defaultJointData(Leph::NamesDOF.size(), defaultJointParams.size());
  std::map<std::string, size_t> defaultJointName;
  size_t tmpIndex = 0;
  for (const std::string& name : Leph::NamesDOF)
  {
    defaultJointName[name] = tmpIndex;
    defaultJointData.block(tmpIndex, 0, 1, defaultJointParams.size()) = defaultJointParams.transpose();
    tmpIndex++;
  }

  // Loading model parameters
  std::cout << "Loading model parameters: " << filename << std::endl;
  Eigen::MatrixXd inertiaData;
  std::map<std::string, size_t> inertiaName;
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  Eigen::MatrixXd jointData;
  std::map<std::string, size_t> jointName;
  Leph::ReadModelParameters(filename, jointData, jointName, inertiaData, inertiaName, geometryData, geometryName);

  // Display
  for (const auto& it : jointName)
  {
    std::string name = it.first;
    size_t index1 = it.second;
    tmpJoint.setParameters(jointData.row(index1).transpose());
    tmpJoint.printParameters();
  }
  for (const auto& it : inertiaName)
  {
    std::string name = it.first;
    size_t index1 = it.second;
    size_t index2 = defaultInertiaName.at(name);
    printInertiaParams(name, inertiaData.row(index1).transpose(), defaultInertiaData.row(index2).transpose());
  }

  return 0;
}
