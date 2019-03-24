#include <iostream>
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::CartWalkProxy walk;
  Leph::VectorLabel outputs = walk.buildOutputs();
  Leph::VectorLabel params = walk.buildParams();

  params("dynamic:enabled") = 1;
  params("dynamic:step") = 10;

  std::cout << outputs << std::endl;
  std::cout << params << std::endl;

  Leph::Plot plot;
  for (size_t i = 0; i < 50 * 5; i++)
  {
    walk.exec(0.02, params);
    plot.add(walk.lastOutputs());
  }
  plot.plot("index", "all").render();

  std::cout << "CartWalk params typical deltas" << std::endl;
  std::cout << walk.buildParamsDelta() << std::endl;

  return 0;
}
