#ifndef LEPH_MAPSERIES_HPP
#define LEPH_MAPSERIES_HPP

#include <string>
#include <map>
#include <vector>
#include <stdexcept>
#include <fstream>
#include "Plot/Plot.hpp"

namespace Leph
{
/**
 * MapSeries
 *
 * Simple container for time
 * series indexed by their name
 */
class MapSeries
{
public:
  /**
   * Structure for data point.
   * time should not be assigned.
   */
  struct Point
  {
    double time;
    double value;
  };

  /**
   * Initialization
   */
  inline MapSeries() : _data()
  {
  }

  /**
   * Return true if given key name
   * series is contained
   */
  inline bool exist(const std::string& name) const
  {
    return (_data.count(name) > 0);
  }

  /**
   * Return the number of contained data points
   * for given series name
   */
  inline size_t size(const std::string& name) const
  {
    checkSeries(name);
    return _data.at(name).size();
  }

  /**
   * Return the number of contained series
   */
  inline size_t dimension() const
  {
    return _data.size();
  }

  /**
   * Clear all contained points
   */
  inline void clear()
  {
    _data.clear();
  }

  /**
   * Append to the MapSeries
   * with given name, time and value.
   * Throw a std::logic_error if given time
   * is lower to already inserted time for the same key.
   */
  inline void append(const std::string& name, double time, double value)
  {
    if (_data.count(name) == 0)
    {
      _data[name] = std::vector<Point>();
    }
    if (_data.at(name).size() > 0 && _data.at(name).back().time > time)
    {
      throw std::logic_error("MapSeries append time too low: " + name);
    }
    _data.at(name).push_back({ time, value });
  }

  /**
   * Return the minimum and maximum inserted time
   * for given series name
   */
  inline double timeMin(const std::string& name) const
  {
    checkSeries(name);
    return _data.at(name).front().time;
  }
  inline double timeMax(const std::string& name) const
  {
    checkSeries(name);
    return _data.at(name).back().time;
  }

  /**
   * Return the minimum and maximum of
   * inserted time valid for all contained series.
   * (maximum of minimum time and minimum of maximum time).
   */
  inline double timeMin() const
  {
    bool isInit = false;
    double minT = 0.0;
    for (const auto& it : _data)
    {
      if (!isInit || timeMin(it.first) > minT)
      {
        minT = timeMin(it.first);
        isInit = true;
      }
    }
    return minT;
  }
  inline double timeMax() const
  {
    bool isInit = false;
    double maxT = 0.0;
    for (const auto& it : _data)
    {
      if (!isInit || timeMax(it.first) < maxT)
      {
        maxT = timeMax(it.first);
        isInit = true;
      }
    }
    return maxT;
  }

  /**
   * Return the first point for given series name
   */
  inline const Point& front(const std::string& name) const
  {
    checkSeries(name);
    return _data.at(name).front();
  }
  inline Point& front(const std::string& name)
  {
    checkSeries(name);
    return _data.at(name).front();
  }

  /**
   * Return the last point for given series name
   */
  inline const Point& back(const std::string& name) const
  {
    checkSeries(name);
    return _data.at(name).back();
  }
  inline Point& back(const std::string& name)
  {
    checkSeries(name);
    return _data.at(name).back();
  }

  /**
   * Return the point at given index for given series name
   */
  inline const Point& at(const std::string& name, size_t index) const
  {
    checkSeries(name);
    if (_data.at(name).size() <= index)
    {
      throw std::logic_error("MapSeries unbound index: " + name + std::string(" index ") + std::to_string(index));
    }
    return _data.at(name)[index];
  }
  inline Point& at(const std::string& name, size_t index)
  {
    checkSeries(name);
    if (_data.at(name).size() <= index)
    {
      throw std::logic_error("MapSeries unbound index: " + name + std::string(" index ") + std::to_string(index));
    }
    return _data.at(name)[index];
  }

  /**
   * Return the closest data index from given
   * time point for given series name.
   */
  inline size_t getIndex(const std::string& name, double time) const
  {
    checkSeries(name);
    const std::vector<Point>& vect = _data.at(name);
    if (time <= vect.front().time)
    {
      return 0;
    }
    if (time >= vect.back().time)
    {
      return vect.size() - 1;
    }
    if (vect.size() == 1)
    {
      return 0;
    }

    size_t indexLow = 0;
    size_t indexUp = vect.size() - 1;
    while (indexUp - indexLow != 1)
    {
      size_t indexMiddle = (indexLow + indexUp) / 2;
      if (vect[indexMiddle].time <= time)
      {
        indexLow = indexMiddle;
      }
      else
      {
        indexUp = indexMiddle;
      }
    }

    if (fabs(time - vect[indexLow].time) <= fabs(time - vect[indexUp].time))
    {
      return indexLow;
    }
    else
    {
      return indexUp;
    }
  }

  /**
   * Interpolate (linear) given series name to given time.
   * If given time is out of bounds, mininum or
   * maximum value is returned.
   * Il not null optionaly assign low and up index and Point
   * used for interpolation.
   */
  inline double get(const std::string& name, double time, size_t* ptrIndexLow = nullptr, size_t* ptrIndexUp = nullptr,
                    Point* ptrPtLow = nullptr, Point* ptrPtUp = nullptr) const
  {
    checkSeries(name);
    const std::vector<Point>& vect = _data.at(name);

    // Degererate failback cases
    if (vect.size() == 0)
    {
      return 0.0;
    }
    if (vect.size() == 1)
    {
      return vect.front().value;
    }
    if (time <= vect.front().time)
    {
      return vect.front().value;
    }
    if (time >= vect.back().time)
    {
      return vect.back().value;
    }

    // Bijection search
    size_t indexLow = 0;
    size_t indexUp = vect.size() - 1;
    while (indexUp - indexLow > 1)
    {
      size_t indexMiddle = (indexLow + indexUp) / 2;
      if (vect[indexMiddle].time <= time)
      {
        indexLow = indexMiddle;
      }
      else
      {
        indexUp = indexMiddle;
      }
    }

    Point ptLow = vect[indexLow];
    Point ptUp = vect[indexUp];

    if (ptrIndexLow != nullptr)
    {
      *ptrIndexLow = indexLow;
    }
    if (ptrIndexUp != nullptr)
    {
      *ptrIndexUp = indexUp;
    }
    if (ptrPtLow != nullptr)
    {
      *ptrPtLow = ptLow;
    }
    if (ptrPtUp != nullptr)
    {
      *ptrPtUp = ptUp;
    }

    double ratio = ptUp.time - ptLow.time;
    return (ptUp.time - time) / ratio * ptLow.value + (time - ptLow.time) / ratio * ptUp.value;
  }

  /**
   * Return an initialized plot instance
   * with all contained data points
   */
  inline Leph::Plot plot() const
  {
    Leph::Plot plot;
    for (const auto& it : _data)
    {
      for (size_t i = 0; i < it.second.size(); i++)
      {
        plot.add(Leph::VectorLabel("time", it.second[i].time, it.first, it.second[i].value));
      }
    }
    return plot;
  }

  /**
   * Import and export internal data
   * from and to given filename
   */
  inline void importData(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open())
    {
      throw std::runtime_error("MapSeries unable to read file: " + filename);
    }

    std::string name = "";
    while (!file.eof())
    {
      while (file.peek() == ' ' || file.peek() == '\n')
      {
        file.ignore();
      }
      if (file.peek() == '#')
      {
        file.ignore();
        file >> name;
        continue;
      }
      double time;
      double value;
      file >> time >> value;
      if (!file.good())
      {
        break;
      }
      if (name == "")
      {
        throw std::runtime_error("MapSeries malformed file");
      }
      append(name, time, value);
    }

    file.close();
  }
  inline void exportData(const std::string& filename) const
  {
    std::ofstream file(filename);
    if (!file.is_open())
    {
      throw std::runtime_error("MapSeries unable to write file: " + filename);
    }

    for (const auto& it : _data)
    {
      file << "#" << it.first << std::endl;
      for (size_t i = 0; i < it.second.size(); i++)
      {
        file << std::setprecision(17) << it.second[i].time << " " << std::setprecision(17) << it.second[i].value
             << std::endl;
      }
    }

    file.close();
  }

  /**
   * Return all contained series names
   */
  std::vector<std::string> allNames() const
  {
    std::vector<std::string> container;
    for (const auto& it : _data)
    {
      container.push_back(it.first);
    }
    return container;
  }

  /**
   * Return a MapSeries which data are the same as this
   * but containts only the data points whose time
   * is between given time range.
   */
  MapSeries sliceTimeRange(double timeBegin, double timeEnd) const
  {
    if (timeBegin > timeEnd)
    {
      throw std::logic_error("MapSeries invalid time range");
    }
    MapSeries cutMap;
    for (const auto& it : _data)
    {
      cutMap._data[it.first] = std::vector<Point>();
      for (size_t i = 0; i < it.second.size(); i++)
      {
        if (it.second[i].time >= timeBegin && it.second[i].time <= timeEnd)
        {
          cutMap._data[it.first].push_back(it.second[i]);
        }
      }
      if (cutMap._data[it.first].size() == 0)
      {
        cutMap._data.erase(it.first);
      }
    }
    return cutMap;
  }

private:
  /**
   * Data Points container indexed by their name
   */
  std::map<std::string, std::vector<Point>> _data;

  /**
   * Throw a logic_error exception if given
   * series name is not contained
   */
  void checkSeries(const std::string& name) const
  {
    if (_data.count(name) == 0)
    {
      throw std::logic_error("MapSeries unknown series: " + name);
    }
  }
};

}  // namespace Leph

#endif
