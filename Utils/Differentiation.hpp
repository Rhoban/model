#ifndef LEPH_DIFFERENTIATION_HPP
#define LEPH_DIFFERENTIATION_HPP

#include <list>
#include "Types/VectorLabel.hpp"

namespace Leph
{
/**
 * Differentiation
 *
 * Usefull rolling VectorLabel buffer
 * and position, vepolity and acceleration
 * computation.
 * Do not call position, velocity or acceleration
 * if the buffer is not full.
 */
class Differentiation
{
public:
  /**
   * Initialization with rolling buffer size and
   * interpolating polynom degree
   */
  Differentiation(unsigned int size = 3, unsigned int degree = 2);

  /**
   * Add given VectorLabel with given
   * associated time (preferably in seconds)
   */
  void add(double time, const VectorLabel& vect);

  /**
   * Return true when enought points
   * has been added to the buffer
   */
  bool isFull() const;

  /**
   * Return minimum and maximum
   * stored points time
   */
  double minTime() const;
  double maxTime() const;

  /**
   * Compute and return position, velocity or
   * acceleration of given values interpolated at
   * given time
   */
  VectorLabel position(double t);
  VectorLabel velocity(double t);
  VectorLabel acceleration(double t);

private:
  /**
   * Typedef for timestamped VectorLabel
   */
  typedef std::pair<double, VectorLabel> Point;

  /**
   * Rolling buffer size
   */
  unsigned int _size;

  /**
   * Polynom fitting degree
   */
  unsigned int _degree;

  /**
   * Registered points container
   */
  std::list<Point> _container;
};

}  // namespace Leph

#endif
