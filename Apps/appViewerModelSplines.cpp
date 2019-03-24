#include <iostream>
#include "Model/HumanoidFloatingModel.hpp"
#include "Spline/SplineContainer.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"

/**
 * Load given splines file on command argument
 * and run and display it with Sigmaban model
 * in the graphical viewer
 */
int main(int argc, char** argv)
{
  // Command line arguments
  if (argc != 2)
  {
    std::cout << "Usage: ./app splinesFile" << std::endl;
    return -1;
  }
  std::string splinesFile = argv[1];
  std::cout << "Loading " << splinesFile << std::endl;

  // Initialize instance
  Leph::HumanoidFloatingModel model(Leph::SigmabanModel);
  Leph::ModelViewer viewer(1200, 900);
  Leph::Scheduling scheduling;
  Leph::SplineContainer<Leph::Spline> splines;

  // Load splines
  splines.importData(splinesFile);
  // Print spline names
  std::cout << "Loaded splines " << splines.size() << std::endl;
  for (const auto& sp : splines.get())
  {
    std::cout << sp.first << std::endl;
  }

  // Retrieve splines bounds
  double timeMin = splines.min();
  double timeMax = splines.max();
  std::cout << "Spline time from " << timeMin << " to " << timeMax << std::endl;

  double freq = 50.0;
  scheduling.setFrequency(freq);
  double t = timeMin;
  while (viewer.update())
  {
    // Compute and assign DOF
    for (const auto& sp : splines.get())
    {
      model.setDOF(sp.first, sp.second.pos(t) * M_PI / 180.0);
    }
    // Contraint the model on the ground
    model.putOnGround();
    // Display model
    Leph::ModelDraw(model, viewer);
    // Waiting
    scheduling.wait();
    // Phase cycling
    t += 1.0 / freq;
    if (t >= timeMax)
      t = timeMin;
  }

  return 0;
}
