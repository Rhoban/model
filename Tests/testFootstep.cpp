#include <iostream>
#include "QuinticWalk/Footstep.hpp"

/**
 * Print given current footstep state
 */
void printStatePretty(const Leph::Footstep& footstep)
{
  std::cout << "Support foot: " << (footstep.isLeftSupport() ? "left" : "right") << std::endl;
  std::cout << "Support to last: " << footstep.getLast().x() << " " << footstep.getLast().y() << " "
            << footstep.getLast().z() * 180.0 / M_PI << std::endl;
  std::cout << "Support to next: " << footstep.getNext().x() << " " << footstep.getNext().y() << " "
            << footstep.getNext().z() * 180.0 / M_PI << std::endl;
  std::cout << "World to left: " << footstep.getLeft().x() << " " << footstep.getLeft().y() << " "
            << footstep.getLeft().z() * 180.0 / M_PI << std::endl;
  std::cout << "World to right: " << footstep.getRight().x() << " " << footstep.getRight().y() << " "
            << footstep.getRight().z() * 180.0 / M_PI << std::endl;
}
void printStateRaw(const Leph::Footstep& footstep)
{
  std::cout << (footstep.isLeftSupport() ? "1" : "0") << " " << footstep.getLast().x() << " " << footstep.getLast().y()
            << " " << footstep.getLast().z() * 180.0 / M_PI << " " << footstep.getNext().x() << " "
            << footstep.getNext().y() << " " << footstep.getNext().z() * 180.0 / M_PI << " " << footstep.getLeft().x()
            << " " << footstep.getLeft().y() << " " << footstep.getLeft().z() << " " << footstep.getRight().x() << " "
            << footstep.getRight().y() << " " << footstep.getRight().z() << std::endl;
}

int main()
{
  Leph::Footstep footstep(0.14);

  // printStatePretty(footstep);

  printStateRaw(footstep);
  for (int k = 0; k < 15; k++)
  {
    footstep.stepFromOrders(Eigen::Vector3d(0.1, 0.0, 0.0));
    printStateRaw(footstep);
  }
  for (int k = 0; k < 10; k++)
  {
    footstep.stepFromOrders(Eigen::Vector3d(0.2, -0.0, 0.4));
    printStateRaw(footstep);
  }
  footstep.stepFromOrders(Eigen::Vector3d(0.0, 0.0, 0.0));
  printStateRaw(footstep);

  return 0;
}
