#ifndef LEPH_CIRCULARBUFFER_HPP
#define LEPH_CIRCULARBUFFER_HPP

#include <vector>
#include "Types/VectorLabel.hpp"

namespace Leph
{
/**
 * CircularBuffer
 *
 * Implement simple rolling buffer for VectorLabel
 * and provide mean and variance values computation
 */
class CircularBuffer
{
public:
  /**
   * Initialization with maximum
   * circular buffer size
   */
  CircularBuffer(size_t size);

  /**
   * Return maximum buffer size
   */
  size_t size() const;

  /**
   * Return the number of currently
   * registered vectors
   * (between 0 and size-1)
   */
  size_t count() const;

  /**
   * Return the dimention of registered vectors
   */
  size_t dimension() const;

  /**
   * Access contained vector
   * (No bounds checks)
   */
  const VectorLabel& operator[](size_t index) const;

  /**
   * Add a vector to the buffer
   */
  void add(const VectorLabel& vect);

  /**
   * Compute and return the buffer mean
   * and variance
   */
  VectorLabel mean() const;
  VectorLabel variance() const;

private:
  /**
   * Added vector circular container
   */
  std::vector<VectorLabel> _buffer;

  /**
   * Begin and End buffer indexes
   * begin : older written slot
   * end : next slot to be written
   */
  size_t _begin;
  size_t _end;
};

}  // namespace Leph

#endif
