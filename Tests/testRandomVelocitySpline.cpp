#include <iostream>
#include "Utils/RandomVelocitySpline.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Eigen::VectorXd state(3);
  Eigen::VectorXd minBound(3);
  Eigen::VectorXd maxBound(3);
  double minTime = 0.5;
  double maxTime = 2.0;

  state << 0.0, 0.0, 0.0;
  minBound << -1.0, -1.0, -1.0;
  maxBound << 1.0, 1.0, 1.0;
  Leph::RandomVelocitySpline exploration(state, minBound, maxBound, minTime, maxTime);

  Leph::Plot plot;
  for (int k = 0; k < 1000; k++)
  {
    exploration.step(0.05);
    plot.add(Leph::VectorLabel("k", k, "x", exploration.state()(0), "y", exploration.state()(1), "z",
                               exploration.state()(2)));
  }
  plot.plot("x", "y", "z", Leph::Plot::LinesPoints, "k").render();

  return 0;
}
