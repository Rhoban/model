#include <iostream>
#include <cassert>
#include "Utils/Euler.h"

void test(const Eigen::Vector3d& angles)
{
  if (!Leph::CheckEulerBounds(angles))
  {
    std::cout << "ERROR Range" << std::endl;
    std::cout << "init:  " << angles.transpose() << std::endl;
  }
  {
    Eigen::Matrix3d mat = Leph::EulerToMatrix(angles, Leph::EulerYawPitchRoll);
    Eigen::Vector3d result = Leph::MatrixToEuler(mat, Leph::EulerYawPitchRoll);
    if ((angles - result).norm() > 0.0001)
    {
      std::cout << "ERROR YawPitchRoll" << std::endl;
      std::cout << "init:  " << angles.transpose() << std::endl;
      std::cout << "final: " << result.transpose() << std::endl;
    }
  }
  {
    Eigen::Matrix3d mat = Leph::EulerToMatrix(angles, Leph::EulerPitchRollYaw);
    Eigen::Vector3d result = Leph::MatrixToEuler(mat, Leph::EulerPitchRollYaw);
    if ((angles - result).norm() > 0.0001)
    {
      std::cout << "ERROR PitchRollYaw" << std::endl;
      std::cout << "init:  " << angles.transpose() << std::endl;
      std::cout << "final: " << result.transpose() << std::endl;
    }
  }
  {
    Eigen::Matrix3d mat = Leph::EulerToMatrix(angles, Leph::EulerRollPitchYaw);
    Eigen::Vector3d result = Leph::MatrixToEuler(mat, Leph::EulerRollPitchYaw);
    if ((angles - result).norm() > 0.0001)
    {
      std::cout << "ERROR RollPitchYaw" << std::endl;
      std::cout << "init:  " << angles.transpose() << std::endl;
      std::cout << "final: " << result.transpose() << std::endl;
    }
  }
  {
    Eigen::Matrix3d mat = Leph::EulerToMatrix(angles, Leph::EulerYawRollPitch);
    Eigen::Vector3d result = Leph::MatrixToEuler(mat, Leph::EulerYawRollPitch);
    if ((angles - result).norm() > 0.0001)
    {
      std::cout << "ERROR YawRollPitch" << std::endl;
      std::cout << "init:  " << angles.transpose() << std::endl;
      std::cout << "final: " << result.transpose() << std::endl;
    }
  }
  {
    Eigen::Matrix3d mat = Leph::EulerToMatrix(angles, Leph::EulerRollYawPitch);
    Eigen::Vector3d result = Leph::MatrixToEuler(mat, Leph::EulerRollYawPitch);
    if ((angles - result).norm() > 0.0001)
    {
      std::cout << "ERROR RollYawPitch" << std::endl;
      std::cout << "init:  " << angles.transpose() << std::endl;
      std::cout << "final: " << result.transpose() << std::endl;
    }
  }
  {
    Eigen::Matrix3d mat = Leph::EulerToMatrix(angles, Leph::EulerPitchYawRoll);
    Eigen::Vector3d result = Leph::MatrixToEuler(mat, Leph::EulerPitchYawRoll);
    if ((angles - result).norm() > 0.0001)
    {
      std::cout << "ERROR PitchYawRoll" << std::endl;
      std::cout << "init:  " << angles.transpose() << std::endl;
      std::cout << "final: " << result.transpose() << std::endl;
    }
  }
}

int main()
{
  /**
   * Valid Euler angles (0, 1, 2) range
   * are (bound included):
   * (0): -M_PI : M_PI
   * (1): -M_PI/2.0 : M_PI/2.0
   * (2): 0.0 : M_PI
   *
   * For angle (1), bounds are reached if others
   * angles (0), (2) are zero.
   */

  test(Eigen::Vector3d(-M_PI, 0.2, 0.3));
  test(Eigen::Vector3d(M_PI, 0.2, 0.3));

  test(Eigen::Vector3d(0.0, -M_PI / 2.0, 0.0));
  test(Eigen::Vector3d(0.0, M_PI / 2.0, 0.0));

  test(Eigen::Vector3d(0.1, -M_PI / 2.0 + 0.1, 0.3));
  test(Eigen::Vector3d(0.1, M_PI / 2.0 - 0.1, 0.3));

  test(Eigen::Vector3d(0.1, 0.2, 0.0));
  test(Eigen::Vector3d(0.1, 0.2, M_PI));

  if (Leph::CheckEulerBounds(Eigen::Vector3d(-M_PI - 0.1, 0.0, 0.0)) ||
      !Leph::CheckEulerBounds(Eigen::Vector3d(0.0, M_PI / 2.0, 0.0)) ||
      Leph::CheckEulerBounds(Eigen::Vector3d(0.1, M_PI / 2.0, 0.0)) ||
      !Leph::CheckEulerBounds(Eigen::Vector3d(0.1, M_PI / 2.0 - 0.1, 0.1)) ||
      Leph::CheckEulerBounds(Eigen::Vector3d(0.1, 0.1, -0.1)) ||
      Leph::CheckEulerBounds(Eigen::Vector3d(0.1, 0.1, M_PI + 0.1)))
  {
    std::cout << "ERROR Check" << std::endl;
  }

  return 0;
}
