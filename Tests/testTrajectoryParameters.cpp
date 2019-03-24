#include <iostream>
#include <Eigen/Dense>
#include "TrajectoryGeneration/TrajectoryParameters.hpp"

int main()
{
  Leph::TrajectoryParameters params;
  params.add("test1", 42.0);
  params.add("test2", 43.0, true);
  params.add("test3", 44.0, true);
  params.add("test_x", 1.0, true);
  params.add("test_y", 2.0, true);
  params.add("test_z", 3.0, true);
  params.cpy("alias", "test_z");
  params.print();
  Eigen::VectorXd vect = params.buildVector();
  std::cout << vect.transpose() << std::endl;
  std::cout << params.getVect("test", vect).transpose() << std::endl;
  std::cout << params.get("alias", vect) << std::endl;
  params.set("alias") = 4.0;
  std::cout << params.get("alias") << std::endl;
  std::cout << params.get("test_z") << std::endl;
  vect.setZero();
  std::cout << params.get("test1", vect) << std::endl;
  std::cout << params.get("test3", vect) << std::endl;

  return 0;
}
