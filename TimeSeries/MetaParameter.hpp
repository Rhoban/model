#ifndef LEPH_METAPARAMETER_HPP
#define LEPH_METAPARAMETER_HPP

#include <string>
#include <limits>
#include <cmath>
#include <stdexcept>

namespace Leph
{
/**
 * MetaParameter
 *
 * Simple structure holding Concepts
 * and Regression parameters to be optimized
 */
class MetaParameter
{
public:
  /**
   * Constructors empty and default
   * value initialization
   */
  MetaParameter()
    : _name("")
    , _hasMinimum(false)
    , _hasMaximum(false)
    , _minimum(std::numeric_limits<double>::quiet_NaN())
    , _maximum(std::numeric_limits<double>::quiet_NaN())
    , _value(std::numeric_limits<double>::quiet_NaN())
  {
  }
  MetaParameter(const std::string& str, double val)
    : _name(str)
    , _hasMinimum(false)
    , _hasMaximum(false)
    , _minimum(std::numeric_limits<double>::quiet_NaN())
    , _maximum(std::numeric_limits<double>::quiet_NaN())
    , _value(val)
  {
  }

  /**
   * Return parameter name
   */
  inline const std::string& name() const
  {
    return _name;
  }

  /**
   * Return parameter value
   */
  inline double value() const
  {
    return _value;
  }

  /**
   * Set minimum and maximum bounds
   */
  inline void setMinimum(double min)
  {
    _hasMinimum = true;
    _minimum = min;
  }
  inline void setMaximum(double max)
  {
    _hasMaximum = true;
    _maximum = max;
  }

  /**
   * Assign parameter to given value.
   * If given value is out min or max bound,
   * the value is trimmed and false is returned.
   */
  inline bool setValue(double val)
  {
    bool isBounded = true;

    if (_hasMinimum && val < _minimum)
    {
      val = _minimum;
      isBounded = false;
    }
    if (_hasMaximum && val > _maximum)
    {
      val = _maximum;
      isBounded = false;
    }
    _value = val;

    return isBounded;
  }

  /**
   * Access to minimum and maximum values
   */
  inline bool hasMinimum() const
  {
    return _hasMinimum;
  }
  inline bool hasMaximum() const
  {
    return _hasMaximum;
  }
  inline double getMinimum() const
  {
    if (!_hasMinimum)
    {
      throw std::logic_error("MetaParameter has no minimum");
    }
    return _minimum;
  }
  inline double getMaximum() const
  {
    if (!_hasMaximum)
    {
      throw std::logic_error("MetaParameter has no maximum");
    }
    return _maximum;
  }

private:
  /**
   * Textual name
   */
  std::string _name;

  /**
   * Is minimum and maximum
   * relevent
   */
  bool _hasMinimum;
  bool _hasMaximum;

  /**
   * Minimun and maximum bounds
   */
  double _minimum;
  double _maximum;

  /**
   * Current value
   */
  double _value;
};

/**
 * Print operator
 */
inline std::ostream& operator<<(std::ostream& os, const MetaParameter& p)
{
  os << "[Parameter " << p.name() << "] = ";
  os << p.value();
  if (p.hasMinimum())
  {
    os << " min=" << p.getMinimum();
  }
  if (p.hasMaximum())
  {
    os << " max=" << p.getMaximum();
  }

  return os;
}

}  // namespace Leph

#endif
