#include <iostream>
#include "Utils/RandomWalk.hpp"
#include "Plot/Plot.hpp"

void testUniformBounds()
{
  Leph::RandomWalk randomWalk(3);
  Eigen::VectorXd deltaLow(3);
  Eigen::VectorXd deltaUp(3);
  Eigen::VectorXd stateLow(3);
  Eigen::VectorXd stateUp(3);
  deltaLow << -0.2, -0.2, -0.2;
  deltaUp << 0.2, 0.2, 0.2;
  stateLow << -2.0, -2.0, -2.0;
  stateUp << 2.0, 2.0, 2.0;
  double inertiaRatio = 0.1;

  Leph::Plot plot;
  for (size_t k = 0; k < 1000; k++)
  {
    randomWalk.uniformStepWithBounds(deltaLow, deltaUp, stateLow, stateUp, inertiaRatio);
    plot.add(Leph::VectorLabel("t", k, "x", randomWalk.statePos()(0), "y", randomWalk.statePos()(1), "z",
                               randomWalk.statePos()(2)));
  }
  plot.plot("x", "y", "z", Leph::Plot::LinesPoints, "t").render();
}

void testGaussian()
{
  Leph::RandomWalk randomWalk(3);

  Leph::Plot plot;
  double sum = 0.0;
  double count = 0.0;
  for (size_t k = 0; k < 1000; k++)
  {
    randomWalk.step(1.0, 0.9);
    plot.add(Leph::VectorLabel("t", k, "x", randomWalk.statePos()(0), "y", randomWalk.statePos()(1), "z",
                               randomWalk.statePos()(2)));
    sum += fabs(randomWalk.stateVel()(0));
    count++;
  }
  std::cout << "Mean delta: " << sum / count << std::endl;
  plot.plot("x", "y", "z", Leph::Plot::LinesPoints, "t").render();
}

int main()
{
  std::cout << "Gaussian" << std::endl;
  testGaussian();
  std::cout << "Uniform" << std::endl;
  testUniformBounds();
  return 0;
}
