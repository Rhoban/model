#include <iostream>
#include "StaticWalk/StaticWalk.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::HumanoidFloatingModel model(Leph::SigmabanModel);
  model.putOnGround();

  Leph::ModelViewer viewer(1200, 900);
  viewer.frameLength = 0.02;

  Leph::StaticWalk walk(Leph::SigmabanModel);
  Leph::VectorLabel params = walk.buildParams();
  Leph::VectorLabel outputs = walk.initPose(params);
  model.setDOF(outputs);

  double freq = 50.0;
  Leph::Plot plot;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(freq);
  while (viewer.update())
  {
    // StaticWalk generator
    Leph::VectorLabel outputs = walk.exec(1.0 / freq, params);

    // Send motor output to model
    model.setDOF(outputs);
    plot.add(outputs);
    // Contraint the model on the ground
    model.putOnGround();

    // Display center of mass trajectory
    Eigen::Vector3d com = model.centerOfMass("origin");
    com.z() = 0.0;
    viewer.addTrackedPoint(com);
    // Display model
    Leph::ModelDraw(model, viewer);
    // Waiting
    scheduling.wait();
  }
  plot.plot("index", "all").render();

  return 0;
}
