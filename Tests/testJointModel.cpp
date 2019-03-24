#include <iostream>
#include <cassert>
#include <cmath>
#include "Model/JointModel.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::Plot plot;
  Leph::JointModel joint("test");
  assert(joint.getName() == "test");

  // Test parameters get/set
  Eigen::VectorXd params1 = joint.getParameters();
  std::cout << "Size Parameters: " << params1.size() << std::endl;
  joint.setParameters(params1);
  Eigen::VectorXd params2 = joint.getParameters();
  assert((param1 - param2).norm() < 1e-9);
  joint.printParameters();

  // Test goal lag implementation
  plot.clear();
  std::cout << "Zero friction torque: " << joint.frictionTorque(0.0) << std::endl;
  for (double t = 0.0; t < 2.0; t += 0.005)
  {
    double goal = sin(t * 2.0 * 3.14 * 3.0) * sin(t * 2.0 * 3.14 * 0.2) + sin(t * 2.0 * 3.14 * 2.0);
    joint.updateState(0.005, goal, 0.0, 0.0);
    plot.add(Leph::VectorLabel("t", t, "goal", goal, "delayedGoal", joint.getDelayedGoal()));
  }
  plot.plot("t", "all").render();

  // Test friction torque model
  plot.clear();
  for (double vel = -1.5; vel < 1.5; vel += 0.001)
  {
    plot.add(Leph::VectorLabel("vel", vel, "friction", joint.frictionTorque(vel)));
  }
  plot.plot("vel", "friction").render();

  return 0;
}
