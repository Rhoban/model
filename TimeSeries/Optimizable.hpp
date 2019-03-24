#ifndef LEPH_OPTIMIZABLE_HPP
#define LEPH_OPTIMIZABLE_HPP

#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdexcept>
#include <vector>
#include "TimeSeries/MetaParameter.hpp"

namespace Leph
{
/**
 * Optimizable
 *
 * Base interface for class
 * containing optimizable meta
 * parameters.
 * resetParameters() have to be called to
 * initialize the parameters container.
 */
class Optimizable
{
public:
  /**
   * Initialization
   */
  Optimizable() : _parameters()
  {
  }

  /**
   * Virtual destructor
   */
  virtual ~Optimizable()
  {
  }

  /**
   * Recreate parameters container with
   * default values
   */
  inline void resetParameters()
  {
    // Clear parameters
    _parameters.clear();
    // Initialize the parameters
    // container with default values
    size_t size = parameterSize();
    for (size_t i = 0; i < size; i++)
    {
      _parameters.push_back(defaultParameter(i));
    }
  }

  /**
   * Read access to meta parameter
   * at given index
   */
  inline const MetaParameter& getParameter(size_t index) const
  {
    size_t paramSize = parameterSize();
    if (paramSize != _parameters.size())
    {
      throw std::logic_error("Optimizable parameters not resetted: " + std::to_string(paramSize) +
                             "!=" + std::to_string(_parameters.size()));
    }
    if (index >= paramSize)
    {
      throw std::logic_error("Optimizable invalid parameter index");
    }

    return _parameters[index];
  }

  /**
   * Update the internal meta parameter with
   * given index and given value.
   * False is returned if given value is trimmed to
   * fit parameter bounds
   */
  inline bool setParameter(size_t index, double value)
  {
    size_t paramSize = parameterSize();
    if (paramSize != _parameters.size())
    {
      resetParameters();
    }
    if (index >= paramSize)
    {
      throw std::logic_error("Optimizable invalid parameter index");
    }

    return _parameters[index].setValue(value);
  }

  /**
   * Return the number of used
   * meta parameters
   * (Must be implemented)
   */
  virtual size_t parameterSize() const = 0;

  /**
   * Return the default initialization
   * (name, min, max, value) of given meta
   * parameter index.
   * (Must be implemented)
   */
  virtual MetaParameter defaultParameter(size_t index) const = 0;

  /**
   * Display on given stream parameters
   */
  inline void parameterPrint(std::ostream& os = std::cout)
  {
    for (size_t i = 0; i < _parameters.size(); i++)
    {
      os << "[" << i << "] " << _parameters[i] << std::endl;
    }
  }

  /**
   * Write and read in given file path
   * all contained parameters
   */
  inline void parameterSave(const std::string& filename) const
  {
    std::ofstream file(filename);
    if (!file.is_open())
    {
      throw std::logic_error("Optimizable unable to write: " + filename);
    }

    for (size_t i = 0; i < _parameters.size(); i++)
    {
      file << _parameters[i].name() << " ";
      if (_parameters[i].hasMinimum())
      {
        file << "1 " << std::setprecision(10) << _parameters[i].getMinimum() << " ";
      }
      else
      {
        file << "0 0 ";
      }
      if (_parameters[i].hasMaximum())
      {
        file << "1 " << std::setprecision(10) << _parameters[i].getMaximum() << " ";
      }
      else
      {
        file << "0 0 ";
      }
      file << std::setprecision(10) << _parameters[i].value() << std::endl;
    }

    file.close();
  }
  inline void parameterLoad(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open())
    {
      throw std::logic_error("Optimizable unable to read: " + filename);
    }

    size_t index = 0;
    while (file.good() && file.peek() != '\n' && file.peek() != EOF)
    {
      std::string name;
      bool hasMin;
      double min;
      bool hasMax;
      double max;
      double val;
      file >> name;
      file >> hasMin;
      file >> min;
      file >> hasMax;
      file >> max;
      file >> val;
      if (index >= parameterSize())
      {
        throw std::logic_error("Optimizable loading too much parameters");
      }
      if (name != _parameters[index].name())
      {
        throw std::logic_error("Optimizable loading invalid parameter name");
      }
      _parameters[index] = MetaParameter(name, val);
      if (hasMin)
      {
        _parameters[index].setMinimum(min);
      }
      if (hasMax)
      {
        _parameters[index].setMaximum(max);
      }
      file.ignore();
      index++;
    }

    file.close();
  }

private:
  /**
   * Meta parameters container
   * of size parameterSize()
   */
  std::vector<MetaParameter> _parameters;
};

}  // namespace Leph

#endif
