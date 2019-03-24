#ifndef LEPH_DMP_HPP
#define LEPH_DMP_HPP

#include <Eigen/Dense>
#include <vector>
#include "Spline/SmoothSpline.hpp"

namespace Leph
{
/**
 * DMP
 *
 * Implementation of Dynamic Movement Primitive
 * for multi dimensional DOFs.
 * Based on the tutorial of Stulp (github.com/stulp/dmpbbo),
 * B-Human implementation (github.com/bhuman/BHumanCodeRelease)
 * and the paper "Kick Motions for the NAO Robot using
 * Dynamic Movement Primitives" using target goal velocity
 * and acceleration
 */
class DMP
{
public:
  /**
   * Uninitialized default
   */
  DMP();

  /**
   * Initialize the DMP with given dimension
   * and given number of gaussian kernel.
   * Optional gaussian kernal overlap
   * can be overload
   */
  DMP(unsigned int dim, unsigned int kernelNum, double overlap = 0.4);

  /**
   * Return current DMP dimension and
   * forcing term gaussian kernel dimension
   */
  unsigned int dimension() const;
  unsigned int kernelNum() const;

  /**
   * Initialize the DMP with given
   * movement time length,
   * start and target position,
   * velocity and acceleration
   */
  void init(double timeLength, const Eigen::VectorXd& startPos, const Eigen::VectorXd& startVel,
            const Eigen::VectorXd& startAcc, const Eigen::VectorXd& endPos, const Eigen::VectorXd& endVel,
            const Eigen::VectorXd& endAcc);

  /**
   * Read access to state position,
   * velocity and acceleration
   */
  Eigen::VectorXd statePos() const;
  Eigen::VectorXd stateVel() const;
  Eigen::VectorXd stateAcc() const;

  /**
   * Read access to phase state
   */
  double statePhase() const;

  /**
   * Return current value for gating
   * term computed from current phase state
   * (between 0.0 and 1.0)
   */
  double stateGating() const;

  /**
   * Return current integrated time
   */
  double currentTime() const;

  /**
   * Read/Write access to kernel centers for given
   * kernel number and kernel width
   */
  double kernelCenter(size_t num) const;
  double& kernelCenter(size_t num);
  double kernelWidth(size_t num) const;
  double& kernelWidth(size_t num);

  /**
   * Read/Write access to kernel parameters
   * for given dimention and kernel number
   */
  double kernelWeight(size_t dim, size_t num) const;
  double& kernelWeight(size_t dim, size_t num);

  /**
   * Compute and integrate one step of the
   * internal dynamical system with given
   * time step
   */
  void step(double dt);

  /**
   * Return the forcing function value
   * for each dimension at given affixe
   * state between 1.0 and 0.0 and multiplied
   * with giben gating term.
   */
  Eigen::VectorXd forcingFunction(double phase, double gating) const;

  /**
   * Get and set the DMP parameters using Eigen
   * Vector format.
   * If weights is true, gaussian kernel weights
   * are used as parameters.
   * If centers is true, gaussian kernel centers
   * are used as parameters.
   * If widths is true, gaussian kernel widths
   * are used as parameters.
   */
  Eigen::VectorXd getParameters(bool weights, bool centers, bool widths) const;
  void setParameters(const Eigen::VectorXd& params, bool weights, bool centers, bool widths);

private:
  /**
   * DMP dimension and gaussian kernel
   * approximator dimension.
   */
  unsigned int _dim;
  unsigned int _kernelNum;

  /**
   * Forcing function gaussian value
   * (between 0.0 and 1.0) at the middle of
   * two following kernel
   */
  double _overlap;

  /**
   * Dynamical system coeficients
   */
  double _coefDamper;
  double _coefSpring;
  double _timeLength;

  /**
   * Integrated real time (dt) since
   * initialization
   */
  double _currentTime;

  /**
   * Current dynamical system state
   * of size 1+2*dim
   */
  Eigen::VectorXd _state;

  /**
   * Last computed state differential
   * of size 1+2*dim
   */
  Eigen::VectorXd _lastStateVel;

  /**
   * Delayed goal 5th order smooth
   * polynomial indexed by the phase
   */
  std::vector<SmoothSpline> _goalSplines;

  /**
   * Pre computed kernel center in phase
   * (between 0.0 and 1.0) for each approximator
   * gaussian kernels and associated kernel width
   */
  std::vector<double> _kernelCenters;
  std::vector<double> _kernelWidths;

  /**
   * Approximator gaussian kernel weights
   * for each dimension and gaussian kernels.
   * [dimension][kernelCount].
   */
  std::vector<std::vector<double>> _kernelWeights;

  /**
   * Recompute and initialize kernel
   * parameters and centers
   */
  void computeKernels();
};

}  // namespace Leph

#endif
