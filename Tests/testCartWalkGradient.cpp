#include <iostream>
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "CartWalk/CartWalkGradient.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::CartWalkProxy walk;
  Leph::CartWalkGradient gradient;

  Leph::VectorLabel params = walk.buildParams();

  params("dynamic:enabled") = 1;
  params("dynamic:step") = 2.0;

  Leph::Matrix jacobian = gradient.differentiation(0.9, params);
  std::cout << walk.buildOutputs() << std::endl;
  std::cout << params << std::endl;
  std::cout << jacobian << std::endl;

  return 0;
}
