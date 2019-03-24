#ifndef LEPH_METASERIES_HPP
#define LEPH_METASERIES_HPP

#include <stdexcept>
#include <limits>
#include <cmath>

namespace Leph
{
/**
 * MetaSeries
 *
 * Online compute meta information on
 * given time serie stream
 */
class MetaSeries
{
public:
  /**
   * Initialization
   */
  MetaSeries()
    : _count(0), _min(std::numeric_limits<double>::quiet_NaN()), _max(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /**
   * Clear internal meta data
   */
  inline void metaClear()
  {
    _count = 0;
    _min = std::numeric_limits<double>::quiet_NaN();
    _max = std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * Update meta data with given point
   */
  inline void metaUpdate(double time, double value)
  {
    if (std::isnan(time) || std::isnan(value))
    {
      throw std::logic_error("MetaSeries NaN");
    }

    // Update count
    _count++;
    // Update global min and max
    if (std::isnan(_min) || value < _min)
    {
      _min = value;
    }
    if (std::isnan(_max) || value > _max)
    {
      _max = value;
    }
  }

  /**
   * Return the total number updated
   */
  inline size_t metaCount() const
  {
    return _count;
  }

  /**
   * Return the minimum and maximum
   * value from all time
   */
  inline double metaMin() const
  {
    if (std::isnan(_min))
    {
      throw std::logic_error("MetaSeries empty");
    }
    return _min;
  }
  inline double metaMax() const
  {
    if (std::isnan(_max))
    {
      throw std::logic_error("MetaSeries empty");
    }
    return _max;
  }

private:
  /**
   * Total number of added points
   */
  size_t _count;

  /**
   * Global minimum and maximum
   * point value
   */
  double _min;
  double _max;
};

/**
 * Print operator
 */
inline std::ostream& operator<<(std::ostream& os, const MetaSeries& ts)
{
  os << "Meta count=" << ts.metaCount();
  if (ts.metaCount() != 0)
  {
    os << " min=" << ts.metaMin() << " max=" << ts.metaMax();
  }

  return os;
}

}  // namespace Leph

#endif
