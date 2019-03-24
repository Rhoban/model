#ifndef LEPH_SPLINE_HPP
#define LEPH_SPLINE_HPP

#include <vector>
#include <iostream>
#include "Spline/Polynom.hpp"
#include "Plot/Plot.hpp"

namespace Leph
{
/**
 * Spline
 *
 * Generic one dimentional
 * polynomial spline generator
 */
class Spline
{
public:
  /**
   * Internal spline part structure
   * with a polynom valid on an interval
   */
  struct Spline_t
  {
    Polynom polynom;
    double min;
    double max;
  };

  /**
   * Return spline interpolation
   * at given t. Compute spline value,
   * its first, second and third derivative
   */
  double pos(double t) const;
  double vel(double t) const;
  double acc(double t) const;
  double jerk(double t) const;

  /**
   * Return spline interpolation
   * value, first, second and third derivative
   * with given t bound between 0 and 1
   */
  double posMod(double t) const;
  double velMod(double t) const;
  double accMod(double t) const;
  double jerkMod(double t) const;

  /**
   * Return minimum and maximum abscisse
   * value for which spline is defined
   */
  double min() const;
  double max() const;

  /**
   * Return or update given a Plot instance
   * with spline pos/vel/acc data with given
   * optional name.
   */
  void plot(Leph::Plot& plot, const std::string& name = "value") const;
  Leph::Plot plot(const std::string& name = "value") const;

  /**
   * Write and read splines data into given
   * iostream in ascii format
   */
  void exportData(std::ostream& os) const;
  void importData(std::istream& is);

  /**
   * Return the number of internal polynom
   */
  size_t size() const;

  /**
   * Access to given by its index
   */
  const Spline_t& part(size_t index) const;

  /**
   * Add a part with given polynom
   * and min/max time range
   */
  void addPart(const Polynom& poly, double min, double max);

  /**
   * Replace this spline part with the
   * internal data of given spline
   */
  void copyData(const Spline& sp);

protected:
  /**
   * Spline part container
   */
  std::vector<Spline_t> _splines;

  /**
   * Possible override callback
   * after importation
   */
  virtual void importCallBack();

private:
  /**
   * Return spline interpolation of given value and
   * used given polynom evaluation function
   * (member function pointer)
   */
  double interpolation(double x, double (Polynom::*func)(double) const) const;

  /**
   * Return interpolation with x
   * bound between 0 and 1
   */
  double interpolationMod(double x, double (Polynom::*func)(double) const) const;
};

}  // namespace Leph

#endif
