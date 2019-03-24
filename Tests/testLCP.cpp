#include <iostream>
#include "LCPMobyDrake/LCPSolver.hpp"

/**
 * Test Linear Complimentary Problem
 * solver extracted from
 * Drake and Moby project
 */
int main()
{
  std::cout << "Test Linear Complimentary Problem" << std::endl;

  // Problem initialization
  Eigen::MatrixXd M(2, 2);
  Eigen::VectorXd q(2);
  Eigen::VectorXd z(2);
  M << 1, 5, 3, 1;
  q << -16, -15;

  // Compute solution
  Drake::MobyLCPSolver solver;
  bool isSuccess = solver.SolveLcpLemkeRegularized(M, q, &z);
  Eigen::VectorXd w = M * z + q;

  // Display
  std::cout << "M:" << std::endl;
  std::cout << M << std::endl;
  std::cout << "q: " << q.transpose() << std::endl;
  std::cout << "z: " << z.transpose() << std::endl;
  std::cout << "w: " << w.transpose() << std::endl;
  std::cout << "isSuccess: " << isSuccess << std::endl;

  // Check solution property
  for (size_t i = 0; i < (size_t)z.size(); i++)
  {
    if (fabs(z(i) * w(i)) > 1e-8)
    {
      std::cout << "Error product not zero" << std::endl;
    }
    if (z(i) < 0.0)
    {
      std::cout << "Error z not positive" << std::endl;
    }
    if (w(i) < 0.0)
    {
      std::cout << "Error z not positive" << std::endl;
    }
  }

  return 0;
}
