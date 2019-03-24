#ifndef LEPH_NULLSPACE_HPP
#define LEPH_NULLSPACE_HPP

#include <vector>
#include <functional>
#include "Model/InverseKinematics.hpp"

namespace Leph
{
/**
 * NullSpace
 *
 * Utility class to compute kernel
 * basis and explore with simple grid
 * discretization the NullSpace associated with
 * InverseKinematics degrees of freedom and constraints
 */
class NullSpace
{
public:
  /**
   * Typedef for custom constraint functions
   * Return false when constrained is break
   */
  typedef std::function<bool(const Eigen::VectorXd&)> ConstraintFunc;

  /**
   * Initialization with
   * InverseKinematics instance reference
   */
  NullSpace(InverseKinematics& inv);

  /**
   * Add a custom constraint taking state (DOF subset)
   * as input and returning false when the constraint
   * is break
   */
  void addConstraint(ConstraintFunc func);

  /**
   * Compute and return the kernel basis at
   * given degree of freedom subset state
   */
  Eigen::MatrixXd computeKernel(const Eigen::VectorXd& state);

  /**
   * Explore and discretize NullSpace from given starting
   * point of degrees of freedom subset
   * Found DOF points are stored into exploredContainer.
   *
   * distanceThreshold is the infinity norm distance
   * threshold for two points to be in the same discretization bin.
   * maxExploredPoints is the maximum number of point to explore
   * before exploration stop
   * If isQuiet is true, debug information is printed on stdout
   */
  void exploreKernelDiscretized(const Eigen::VectorXd& startingPoint, std::vector<Eigen::VectorXd>& exploredContainer,
                                double distanceThreshold, unsigned int maxExploredPoints, bool isQuiet = true);

  /**
   * Run Inverse Kinematics and update given degree of
   * freedom subset state to meet model targets
   * False is returned if convergence failed
   */
  bool refinePoint(Eigen::VectorXd& state);

  /**
   * Check if given degree of freedom subset state is
   * meeting limit bounds and declared constraints.
   * Return false if constraints is broken
   */
  bool checkConstraints(const Eigen::VectorXd& state);

private:
  /**
   * InverseKinematics with DOF subset and
   * joint limit informations
   */
  InverseKinematics* _inverseModel;

  /**
   * Container of custom constraint functions
   */
  std::vector<ConstraintFunc> _customConstraints;

  /**
   * Check if given degree of freedom subset state
   * minimum distance from points in given point container
   * is below given threshold.
   * False is return if distanceThreshold constraints is broken.
   */
  bool checkDistance(const Eigen::VectorXd& state, const std::vector<Eigen::VectorXd>& container,
                     double distanceThreshold);

  /**
   * Return false if given DOF points is not valid
   * (allowed range, already explored) to be explored
   */
  bool isPointValidForExploration(const Eigen::VectorXd& point, const std::vector<Eigen::VectorXd>& exploredContainer,
                                  const std::vector<Eigen::VectorXd>& toExploreContainer,
                                  const std::vector<Eigen::VectorXd>& failedExploredContainer,
                                  double distanceThreshold);
};

}  // namespace Leph

#endif
