#include <iostream>
#include <stdexcept>
#include "Model/NullSpace.hpp"

namespace Leph
{
NullSpace::NullSpace(InverseKinematics& inv) : _inverseModel(&inv)
{
}

void NullSpace::addConstraint(ConstraintFunc func)
{
  _customConstraints.push_back(func);
}

Eigen::MatrixXd NullSpace::computeKernel(const Eigen::VectorXd& state)
{
  if ((size_t)state.size() != _inverseModel->sizeDOF())
  {
    throw std::logic_error("NullSpace invalid DOF size");
  }

  // Retrieve jacobian matrix
  Eigen::MatrixXd fjac = Eigen::MatrixXd::Zero(_inverseModel->values(), _inverseModel->inputs());
  _inverseModel->df(state, fjac);
  // Compute kernel basis
  Eigen::FullPivLU<Eigen::MatrixXd> lu(fjac);
  lu.setThreshold(Eigen::Default);
  return lu.kernel();
}

void NullSpace::exploreKernelDiscretized(const Eigen::VectorXd& startingPoint,
                                         std::vector<Eigen::VectorXd>& exploredContainer, double distanceThreshold,
                                         unsigned int maxExploredPoints, bool isQuiet)
{
  if ((size_t)startingPoint.size() != _inverseModel->sizeDOF())
  {
    throw std::logic_error("NullSpace invalid DOF size");
  }

  // Initialize container for point to be explored
  // and points for which exploration has failed
  std::vector<Eigen::VectorXd> toExploreContainer;
  std::vector<Eigen::VectorXd> failedExploredContainer;

  // Add starting point
  toExploreContainer.push_back(startingPoint);
  // Refine starting point
  if (!refinePoint(toExploreContainer.front()))
  {
    throw std::runtime_error("NullSpace starting point invalid");
  }

  // Kernel exploration step
  double distanceStep = 2.0 * distanceThreshold;

  // Stop exploration if all points have been processed
  while (toExploreContainer.size() != 0 && exploredContainer.size() < maxExploredPoints)
  {
    // Retrieve next point
    Eigen::MatrixXd point = toExploreContainer.back();
    // Delete explored point
    toExploreContainer.pop_back();
    // Add to explored points
    exploredContainer.push_back(point);
    // Compute kernel basis
    Eigen::MatrixXd ker = computeKernel(point);
    // Debug information
    if (!isQuiet)
    {
      std::cout << "NullSpace Exploration"
                << " explored=" << exploredContainer.size() << " queue=" << toExploreContainer.size()
                << " failed=" << failedExploredContainer.size() << " dofsDim=" << ker.rows()
                << " kernelDim=" << ker.cols() << std::endl;
    }
    // Check if kernel is no zero
    if (ker.cols() == 1 && ker.col(0).norm() < 0.00001)
    {
      continue;
    }
    // Iterate throuth all kernel basis directions
    for (size_t i = 0; i < (size_t)ker.cols(); i++)
    {
      // Adding to queue next point to explore
      // while following unit vector in both directions
      Eigen::VectorXd newPointUp = point + distanceStep * ker.col(i).normalized();
      Eigen::VectorXd newPointDown = point - distanceStep * ker.col(i).normalized();
      // Check if new point lies inside constraints
      // and have not already been tried
      if (checkConstraints(newPointUp) && checkDistance(newPointUp, failedExploredContainer, distanceThreshold / 2.0))
      {
        // Run InverseKinematics to stay in nullspace
        if (refinePoint(newPointUp))
        {
          // Check if point is not already explored
          // or set to be explored
          if (checkDistance(newPointUp, toExploreContainer, distanceThreshold) &&
              checkDistance(newPointUp, exploredContainer, distanceThreshold))
          {
            // Add it to exploration
            toExploreContainer.push_back(newPointUp);
          }
          else
          {
            // Save it as already explored
            failedExploredContainer.push_back(newPointUp);
          }
        }
        else
        {
          // Mark point as fail if no convergence
          failedExploredContainer.push_back(newPointUp);
        }
      }
      // Check if new point lies inside constraints
      // and have not already been tried
      if (checkConstraints(newPointDown) &&
          checkDistance(newPointDown, failedExploredContainer, distanceThreshold / 2.0))
      {
        // Run InverseKinematics to stay in nullspace
        if (refinePoint(newPointDown))
        {
          // Check if point is not already explored
          // or set to be explored
          if (checkDistance(newPointDown, toExploreContainer, distanceThreshold) &&
              checkDistance(newPointDown, exploredContainer, distanceThreshold))
          {
            // Add it to exploration
            toExploreContainer.push_back(newPointDown);
          }
          else
          {
            // Save it as already explored
            failedExploredContainer.push_back(newPointDown);
          }
        }
        else
        {
          // Mark point as fail if no convergence
          failedExploredContainer.push_back(newPointDown);
        }
      }
    }
  }
}

bool NullSpace::refinePoint(Eigen::VectorXd& state)
{
  if ((size_t)state.size() != _inverseModel->sizeDOF())
  {
    throw std::logic_error("NullSpace invalid DOF size");
  }

  // Refine position in order to meet geometric target
  // using inverse kinematics
  _inverseModel->setDOFSubset(state);
  double ampl = 0.01;
  for (int k = 0; k < 100; k++)
  {
    _inverseModel->run(0.001, 100);
    if (_inverseModel->errorSum() <= 0.001)
    {
      break;
    }
    if (k % 5 == 0)
    {
      ampl *= 2.0;
    }
    // Random DOF perturbation to go away from
    // singular position
    _inverseModel->randomDOFNoise(ampl);
  }

  // Check convergence error
  if (_inverseModel->errorSum() <= 0.001)
  {
    // Update found point
    state = _inverseModel->getDOFSubset();
    return true;
  }
  else
  {
    return false;
  }
}

bool NullSpace::checkConstraints(const Eigen::VectorXd& state)
{
  // Check if point lies in DOF limits
  for (size_t i = 0; i < (size_t)state.size(); i++)
  {
    if ((_inverseModel->getIsLowerBounds()[i] && _inverseModel->getLowerBounds()[i] > state(i)) ||
        (_inverseModel->getIsUpperBounds()[i] && _inverseModel->getUpperBounds()[i] < state(i)))
    {
      return false;
    }
  }
  // Check custom constraint
  for (size_t i = 0; i < _customConstraints.size(); i++)
  {
    if (!_customConstraints[i](state))
    {
      return false;
    }
  }
  return true;
}

bool NullSpace::checkDistance(const Eigen::VectorXd& state, const std::vector<Eigen::VectorXd>& container,
                              double distanceThreshold)
{
  for (size_t i = 0; i < container.size(); i++)
  {
    // Compute l-infinity distance
    double dist = (container[i] - state).lpNorm<Eigen::Infinity>();
    if (dist < distanceThreshold)
    {
      return false;
    }
  }
  return true;
}

}  // namespace Leph
