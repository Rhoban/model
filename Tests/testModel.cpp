#include <iostream>
#include "Model/Model.hpp"
#include "Utils/FileModelParameters.h"

int main()
{
  Leph::Model model("sigmaban.urdf");
  std::cout << "DOF " << model.sizeDOF() << std::endl;
  Leph::VectorLabel dof = model.getDOF();
  std::cout << dof << std::endl;
  model.setDOF(dof);

  std::cout << "Frame " << model.sizeFrame() << std::endl;
  for (size_t i = 0; i < model.sizeFrame(); i++)
  {
    std::cout << "index=" << i << " name=" << model.getFrameName(i)
              << " index=" << model.getFrameIndex(model.getFrameName(i)) << " id=" << model.frameIndexToBodyId(i)
              << std::endl;
  }

  std::cout << "Inertia Data:" << std::endl;
  for (const auto& it : model.getInertiaName())
  {
    std::cout << it.second << " " << it.first << ": ";
    for (size_t i = 0; i < (size_t)model.getInertiaData().cols(); i++)
    {
      std::cout << model.getInertiaData()(it.second, i) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  std::cout << "Geometry Data:" << std::endl;
  for (const auto& it : model.getGeometryName())
  {
    std::cout << it.second << " " << it.first << ": ";
    for (size_t i = 0; i < (size_t)model.getGeometryData().cols(); i++)
    {
      std::cout << model.getGeometryData()(it.second, i) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  for (size_t i = 0; i < model.sizeFrame(); i++)
  {
    std::cout << model.getFrameName(i) << ": " << model.position(i, model.getFrameIndex("trunk")).transpose()
              << std::endl;
    std::cout << model.orientation(i, model.getFrameIndex("origin")) << std::endl;
  }

  std::cout << "Mass " << model.sumMass() << std::endl;
  std::cout << "CoM " << model.centerOfMass("origin").transpose() << std::endl;

  Eigen::MatrixXd jointData(1, 18);
  std::map<std::string, size_t> jointName = { { "all", 0 } };
  Eigen::MatrixXd inertiaData = model.getInertiaData();
  std::map<std::string, size_t> inertiaName = model.getInertiaName();
  Eigen::MatrixXd geometryData = model.getGeometryData();
  std::map<std::string, size_t> geometryName = model.getGeometryName();
  Leph::WriteModelParameters("/tmp/testModel1.params", jointData, jointName, inertiaData, inertiaName, geometryData,
                             geometryName);
  Leph::ReadModelParameters("/tmp/testModel1.params", jointData, jointName, inertiaData, inertiaName, geometryData,
                            geometryName);
  Leph::WriteModelParameters("/tmp/testModel2.params", jointData, jointName, inertiaData, inertiaName, geometryData,
                             geometryName);

  return 0;
}
