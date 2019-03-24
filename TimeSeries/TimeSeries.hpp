#ifndef LEPH_TIMESERIES_HPP
#define LEPH_TIMESERIES_HPP

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <stdexcept>
#include <limits>
#include <cmath>
#include "Utils/Angle.h"
#include "TimeSeries/MetaSeries.hpp"

namespace Leph
{
/**
 * Numeric epsilon constant for time
 * (set to be < 1ms if time is in seconds)
 */
const double TIME_EPSILON = 0.0001;

/**
 * TimeSeries
 *
 * Simple time series
 * contained associated with a name.
 * Internal container is implemented
 * as a rolling buffer to save memory.
 * Future data point can also be inserted
 * and clear.
 * Meta data are computed by sub class.
 */
class TimeSeries : public MetaSeries
{
public:
  /**
   * Structure for timed value
   * time is in second
   */
  struct Point
  {
    double time;
    double value;
  };

  /**
   * Initialization with empty or
   * given name
   * maxSize is maximum size of internal rolling buffer.
   * Infinite size is assumed if maxSize is -1
   */
  TimeSeries() : MetaSeries(), _name(""), _maxSize(-1), _indexEnd(0), _data(), _isFutureMode(false), _dataFuture()
  {
  }
  TimeSeries(const std::string& name, size_t maxSize = -1)
    : MetaSeries(), _name(name), _maxSize(maxSize), _indexEnd(0), _data(), _isFutureMode(false), _dataFuture()
  {
  }

  /**
   * Return TimeSeries name
   */
  inline const std::string& name() const
  {
    return _name;
  }

  /**
   * Return the number of currently
   * stored data points
   */
  inline size_t size() const
  {
    if (_maxSize == (size_t)-1 || _data.size() < _maxSize)
    {
      if (_isFutureMode)
      {
        return _data.size() + sizeFuture();
      }
      else
      {
        return _data.size();
      }
    }
    else
    {
      if (_isFutureMode)
      {
        return _maxSize + sizeFuture();
      }
      else
      {
        return _maxSize;
      }
    }
  }

  /**
   * Reset the series data to empty
   */
  inline void clear()
  {
    _indexEnd = 0;
    _data.clear();
    _isFutureMode = false;
    _dataFuture.clear();
  }

  /**
   * Return newest and oldest
   * inserted point time
   */
  inline double timeMin() const
  {
    return at(size() - 1).time;
  }
  inline double timeMax() const
  {
    return at(0).time;
  }

  /**
   * Return true if given time is bounded between
   * minimum and maximum series time
   */
  inline bool isTimeValid(double time) const
  {
    if (size() < 2)
    {
      return false;
    }
    else
    {
      return (time >= timeMin() && time <= timeMax());
    }
  }

  /**
   * Compute and do linear interpolation
   * of series values at given time.
   */
  inline double get(double time) const
  {
    size_t indexLow;
    size_t indexUp;
    bisectionSearch(time, indexLow, indexUp);

    // Linear interpolation
    double d1 = at(indexLow).time - at(indexUp).time;
    double d2 = time - at(indexUp).time;
    return at(indexUp).value * (d1 - d2) / d1 + at(indexLow).value * d2 / d1;
  }

  /**
   * Compute and do linear interpolation
   * of series values at given time specialized
   * for angular values
   */
  inline double getAngular(double time) const
  {
    size_t indexLow;
    size_t indexUp;
    bisectionSearch(time, indexLow, indexUp);

    // Linear interpolation
    double d1 = at(indexLow).time - at(indexUp).time;
    double d2 = time - at(indexUp).time;
    return AngleWeightedMean(d1 - d2, at(indexUp).value, d2, at(indexLow).value);
  }

  /**
   * Find the value whose timestamp is lower and
   * upper given time and return its index in
   * internal (time resersed order) container
   */
  inline size_t getLowerIndex(double time) const
  {
    size_t indexLow;
    size_t indexUp;
    bisectionSearch(time, indexLow, indexUp);

    return indexLow;
  }
  inline size_t getUpperIndex(double time) const
  {
    size_t indexLow;
    size_t indexUp;
    bisectionSearch(time, indexLow, indexUp);

    return indexUp;
  }

  /**
   * Find the closest index from given
   * associated time
   */
  inline size_t getClosestIndex(double time) const
  {
    size_t indexLow;
    size_t indexUp;
    bisectionSearch(time, indexLow, indexUp);

    double distLow = fabs(time - at(indexLow).time);
    double distUp = fabs(time - at(indexUp).time);
    if (distLow < distUp)
    {
      return indexLow;
    }
    else
    {
      return indexUp;
    }
  }

  /**
   * Direct access to Points container
   * from index 0 to size()-1.
   * Indexes goes from newest inserted point
   * to oldest inserted point
   */
  inline const Point& operator[](size_t index) const
  {
    if (index >= size())
    {
      throw std::logic_error("TimeSeries index out of bounds: index=" + std::to_string(index) + " name=" + name());
    }
    if (_isFutureMode && index < sizeFuture())
    {
      return atFuture(index);
    }
    else if (_isFutureMode)
    {
      index -= sizeFuture();
    }
    if (_maxSize == (size_t)-1 || _data.size() < _maxSize)
    {
      return _data.at(_data.size() - 1 - index);
    }
    else
    {
      int i = _indexEnd - 1 - index;
      if (i < 0)
      {
        i += _data.size();
      }
      return _data.at(i);
    }
  }
  inline const Point& at(size_t index) const
  {
    return operator[](index);
  }

  /**
   * Direct access to last added point
   * time and value
   */
  inline double lastTime() const
  {
    return at(0).time;
  }
  inline double lastValue() const
  {
    return at(0).value;
  }

  /**
   * Append given point to the series.
   * Throw error if time is not strictly upper
   * than last stored time
   */
  inline void append(double time, double value)
  {
    if (std::isnan(time) || std::isnan(value))
    {
      throw std::logic_error("TimeSeries adding NaN");
    }
    if (_data.size() > 0 && time <= lastTime())
    {
      throw std::logic_error("TimeSeries adding past time");
    }

    // Insert the point
    if (_isFutureMode)
    {
      appendFuture(time, value);
    }
    else
    {
      if (_maxSize == (size_t)-1 || _data.size() < _maxSize)
      {
        _data.push_back({ time, value });
      }
      else
      {
        _data[_indexEnd] = { time, value };
      }
      _indexEnd++;
      if (_maxSize != (size_t)-1 && _indexEnd >= _maxSize)
      {
        _indexEnd = 0;
      }
      // Update meta data
      MetaSeries::metaUpdate(time, value);
    }
  }

  /**
   * Load from given input stream.
   * True is returned on sucess.
   * False is returned if given stream is empty.
   * Throw error on malformed input.
   */
  inline bool load(std::istream& is)
  {
    // Load is forbiden in future mode
    if (_isFutureMode)
    {
      throw std::logic_error("TimeSeries invalid load in future mode");
    }

    // Extract one line
    std::string line;
    while (is.good() && is.peek() != '\n' && is.peek() != EOF)
    {
      line += is.get();
    }
    is.ignore();
    if (line.length() == 0)
    {
      return false;
    }

    // Extract name
    size_t pos = 0;
    pos = line.find_first_of(" ", pos);
    if (pos == std::string::npos)
    {
      throw std::runtime_error("TimeSeries parsing error: " + line);
    }
    _name = line.substr(0, pos);
    line = line.substr(pos + 1);

    // Extract values
    while (line.length() > 0)
    {
      // Extract time
      double time = std::stod(line, &pos);
      if (pos >= line.length() || line[pos] != ' ')
      {
        throw std::runtime_error("TimeSeries parsing error: " + line);
      }
      line = line.substr(pos + 1);
      // Extract value
      double value = std::stod(line, &pos);
      if (pos >= line.length() || line[pos] != ' ')
      {
        throw std::runtime_error("TimeSeries parsing error: " + line);
      }
      line = line.substr(pos + 1);
      // Add to series
      append(time, value);
    }

    return true;
  }

  /**
   * Dump the time series into given
   * output stream
   */
  inline void save(std::ostream& os) const
  {
    bool state = _isFutureMode;
    _isFutureMode = false;
    os << _name << " ";
    for (int i = size() - 1; i >= 0; i--)
    {
      os << std::setprecision(17) << at(i).time << " " << std::setprecision(17) << at(i).value << " ";
    }
    os << std::endl;
    _isFutureMode = state;
  }

  /**
   * Retrieve and set current future mode
   */
  inline bool isFutureMode() const
  {
    return _isFutureMode;
  }
  inline void enableFutureMode()
  {
    _isFutureMode = false;
    // Check that past and future data series
    // does not overlap
    if (sizeFuture() > 0 && size() > 0 && lastTime() >= _dataFuture.front().time)
    {
      throw std::logic_error("TimeSeries past and future overlapping");
    }

    _isFutureMode = true;
  }
  inline void disableFutureMode()
  {
    _isFutureMode = false;
  }

  /**
   * Return the size of currenlty stored
   * future data points
   */
  inline size_t sizeFuture() const
  {
    return _dataFuture.size();
  }

  /**
   * Clear future data points
   */
  inline void clearFuture()
  {
    _dataFuture.clear();
  }

  /**
   * Direct access to appended future points.
   * 0 index is last appended point.
   * sizeFuture()-1 is first appended point.
   */
  inline const Point& atFuture(size_t index) const
  {
    if (index >= sizeFuture())
    {
      throw std::logic_error("TimeSeries index out of bounds (future)");
    }

    return _dataFuture.at(_dataFuture.size() - 1 - index);
  }

  /**
   * Append the given point to future data series.
   * Throw error if time is not strictly upper
   * than last stored time
   */
  inline void appendFuture(double time, double value)
  {
    // Check NaN
    if (std::isnan(time) || std::isnan(value))
    {
      throw std::logic_error("TimeSeries adding NaN (future)");
    }
    // Check past insertion
    if (_dataFuture.size() > 0 && _dataFuture.back().time >= time)
    {
      throw std::logic_error("TimeSeries adding past time (future)");
    }
    // Check that no future point is added
    // in past time
    bool state = _isFutureMode;
    _isFutureMode = false;
    if (size() > 0 && lastTime() >= time)
    {
      throw std::logic_error("TimeSeries inserting past future point");
    }
    _isFutureMode = state;

    // Insert the point
    _dataFuture.push_back({ time, value });
  }

private:
  /**
   * Series name
   */
  std::string _name;

  /**
   * Rolling buffer max size
   * -1 is infinite size
   */
  size_t _maxSize;

  /**
   * Rolling buffer end index
   */
  size_t _indexEnd;

  /**
   * Data points container
   */
  std::vector<Point> _data;

  /**
   * Is Future mode enabled.
   * When future mode is enabled all time series
   * operation (get, at, size, append) use temporary
   * future data trajectory.
   */
  mutable bool _isFutureMode;

  /**
   * Future data points container
   */
  std::vector<Point> _dataFuture;

  /**
   * Compute a bijection search onto data internal
   * values container for upper index and lower index
   * between given time
   */
  inline void bisectionSearch(double time, size_t& indexLow, size_t& indexUp) const
  {
    if (size() < 2)
    {
      throw std::logic_error("TimeSeries not enough points: time=" + std::to_string(time) + " name:" + name());
    }
    if (time < timeMin() || time > timeMax())
    {
      throw std::logic_error("TimeSeries out of bound: time=" + std::to_string(time) + " name:" + name());
    }

    // Bijection search
    indexLow = 0;
    indexUp = size() - 1;
    while (indexUp - indexLow != 1)
    {
      size_t i = (indexLow + indexUp) / 2;
      if (at(i).time <= time)
      {
        indexUp = i;
      }
      else
      {
        indexLow = i;
      }
    }
  }
};

/**
 * Print operator
 */
inline std::ostream& operator<<(std::ostream& os, const TimeSeries& ts)
{
  os << "[" << ts.name() << "] ";
  if (ts.size() > 0)
  {
    os << "size=" << ts.size() << " ";
    os << "tMin=" << ts.timeMin() << " tMax=" << ts.timeMax();
    os << " val=" << ts.lastValue();
  }
  else
  {
    os << "empty";
  }
  os << " (modeFuture=";
  if (ts.isFutureMode())
  {
    os << "true";
  }
  else
  {
    os << "false";
  }
  os << ")";

  // Print Meta data
  os << " " << (MetaSeries)ts;

  return os;
}

}  // namespace Leph

#endif
