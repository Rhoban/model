#ifndef LEPH_DMPSPLINE_HPP
#define LEPH_DMPSPLINE_HPP

#include <vector>
#include <iostream>
#include "DMP/DMP.hpp"
#include "Plot/Plot.hpp"

namespace Leph
{
/**
 * DMPSpline
 *
 * Joint together multiple unidimensional DMP
 * in order to possibly add via point with
 * constraints position, velocity and acceleration
 */
class DMPSpline
{
public:
  /**
   * Simple point structure
   */
  struct Point
  {
    double time;
    double position;
    double velocity;
    double acceleration;
  };

  /**
   * Structure for a DMP part
   */
  struct DMPPart
  {
    double timeBegin;
    double timeEnd;
    unsigned int kernelFirstIndex;
    unsigned int kernelLastIndex;
    DMP dmp;
  };

  /**
   * Dumb default initalization
   * (the smpline is untinizialized)
   */
  DMPSpline();

  /**
   * Initialization with the total number
   * of forcing gaussian kernels and optionnaly
   * the overlap value between 0.0 and 1.0.
   */
  DMPSpline(unsigned int kernelNum, double overlap = 0.1, double maxTimeStep = 0.005);

  /**
   * Set given gaussian kernel number and
   */

  /**
   * Add the via point with given time,
   * position, velocity and acceleration
   */
  void addPoint(double time, double position, double velocity = 0.0, double acceleration = 0.0);

  /**
   * Access to points container
   */
  const std::vector<Point>& points() const;
  std::vector<Point>& points();

  /**
   * Access to DMP part container
   */
  const std::vector<DMPPart>& parts() const;
  std::vector<DMPPart>& parts();

  /**
   * Return the number of internal DMP part
   */
  size_t size() const;

  /**
   * Access to given part by its index
   */
  const DMPPart& part(size_t index) const;
  DMPPart& part(size_t index);

  /**
   * Return minimum and maximum abscisse
   * value for which spline is defined
   */
  double min() const;
  double max() const;

  /**
   * Return DMP spline value
   * at given t. Compute spline value,
   * its first and second derivative.
   * Random time request have very poor
   * performance. Time request shall be
   * increasing.
   */
  double pos(double t);
  double vel(double t);
  double acc(double t);

  /**
   * DMP specific state at given time
   */
  double phase(double t);
  double gating(double t);
  double forcingFunction(double t);
  double rawForcingFunction(double t);

  /**
   * Get and set all DMP forcing
   * gaussian kernel weights and
   * widths in Vector Eigen format
   * or a single component
   */
  double getKernelWidth(size_t index) const;
  Eigen::VectorXd getKernelWidths() const;
  double getKernelWeight(size_t index) const;
  Eigen::VectorXd getKernelWeights() const;
  void setKernelWidth(size_t index, double value);
  void setKernelWidths(const Eigen::VectorXd& vect);
  void setKernelWeight(size_t index, double value);
  void setKernelWeights(const Eigen::VectorXd& vect);

  /**
   * Recompute DMP parts. DMP are just
   * re-initialized (save weights and widths
   * parameters) if noRebuild is true.
   */
  void computeSplines(bool noRebuild = false);

  /*
   * Write and read splines data into given
   * iostream in ascii format
   */
  void exportData(std::ostream& os) const;
  void importData(std::istream& is);

  /**
   * Return or update given a Plot instance
   * with spline pos/vel/acc data with given
   * optional name.
   */
  void plot(Leph::Plot& plot, const std::string& name = "value");
  Leph::Plot plot(const std::string& name = "value");

private:
  /**
   * Common number of forcing
   * term gaussian kernels
   */
  unsigned int _kernelNum;

  /**
   * Gaussian forcing kernel
   * overlap value (between 0.0 and 1.0)
   */
  double _overlap;

  /**
   * Maximum DMP integration step
   */
  double _maxTimeStep;

  /**
   * Added points container
   */
  std::vector<Point> _points;

  /**
   * DMP spline parts computed from
   * added points list
   */
  std::vector<DMPPart> _parts;

  /**
   * Return the DMP part index associated with
   * given time. -1 is returned if given time
   * is outside spline range
   */
  size_t timeToPartIndex(double t) const;

  /**
   * Update the internal DMP state
   * to match given time.
   * Current DMP part index is returned.
   */
  size_t updateInternalDMP(double t);

  /**
   * Compute and assign from given global kernel
   * index the associated DMP part index and the
   * internal kernel index for this DMP.
   */
  void kernelIndexToDMPPart(size_t kernelIndex, size_t& partIndex, size_t& dmpKernelIndex) const;
};

}  // namespace Leph

#endif
