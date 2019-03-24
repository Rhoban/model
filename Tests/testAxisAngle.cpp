#include <iostream>
#include "Utils/AxisAngle.h"

static void test(const Eigen::Vector3d& vect)
{
  Eigen::Matrix3d mat = Leph::AxisToMatrix(vect);
  Eigen::Vector3d result = Leph::MatrixToAxis(mat);
  if ((vect - result).norm() > 0.0001)
  {
    std::cout << "ERROR: " << std::endl;
    std::cout << vect.transpose() << std::endl;
    std::cout << result.transpose() << std::endl;
  }
}

int main()
{
  test(Eigen::Vector3d(0.0, 0.0, 0.0));
  test(Eigen::Vector3d(1.0, 0.0, 0.0));
  test(Eigen::Vector3d(0.0, 1.0, 0.0));
  test(Eigen::Vector3d(0.0, 0.0, 1.0));
  test(Eigen::Vector3d(1.0, 0.5, 0.0));
  test(Eigen::Vector3d(0.0, 1.0, 0.5));
  test(Eigen::Vector3d(1.0, 0.0, 0.5));
  test(Eigen::Vector3d(0.1, 0.2, 0.3));

  try
  {
    test(Eigen::Vector3d(0.1, 0.2, 2.0));
    std::cout << "ERROR" << std::endl;
  }
  catch (...)
  {
  };

  return 0;
}
