#include <iostream>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Model/RBDLRootUpdate.h"
#include "Model/Model.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

namespace RBDL = RigidBodyDynamics;

int main()
{
  // URDF loading
  RBDL::Model modelOld;
  if (!RBDL::Addons::URDFReadFromFile("sigmaban.urdf", &modelOld, false))
  {
    std::runtime_error("Model unable to load URDF file");
  }

  // Build new model with root located at right foot roll frame
  // with floating base added
  std::cout << RBDL::Utils::GetModelDOFOverview(modelOld) << std::endl;
  std::cout << RBDL::Utils::GetModelHierarchy(modelOld) << std::endl;
  RBDL::Model modelNew = Leph::RBDLRootUpdate(modelOld, 2147483649, true);
  std::cout << RBDL::Utils::GetModelDOFOverview(modelNew) << std::endl;
  std::cout << RBDL::Utils::GetModelHierarchy(modelNew) << std::endl;

  // Load new built model into wrapping class
  Leph::Model wrappedModelNew(modelNew);
  std::cout << wrappedModelNew.getDOF() << std::endl;

  // Viewer loop
  Leph::ModelViewer viewer(1200, 900);
  while (viewer.update())
  {
    wrappedModelNew.setDOF("base_z", wrappedModelNew.getDOF("base_z") + 0.0005);
    Leph::ModelDraw(wrappedModelNew, viewer);
  }

  return 0;
}
