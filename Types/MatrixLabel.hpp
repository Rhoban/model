#ifndef LEPH_MATRIXLABEL_HPP
#define LEPH_MATRIXLABEL_HPP

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <fstream>
#include "Types/VectorLabel.hpp"
#include "Plot/Plot.hpp"

namespace Leph
{
/**
 * MatrixLabel
 *
 * Represent a time serie
 * of VectorLabel
 */
class MatrixLabel
{
public:
  /**
   * Initialization empty
   */
  MatrixLabel() : _container(), _indexBegin(0), _indexEnd(-1)
  {
    _container = std::make_shared<Container>();
  }
  MatrixLabel(size_t size) : _container(), _indexBegin(0), _indexEnd(size - 1)
  {
    _container = std::make_shared<Container>(size);
  }

  /**
   * Explicitely forbid assignement
   */
  MatrixLabel& operator=(const MatrixLabel&) = delete;

  /**
   * Copy and return a same MatrixLabel instance
   */
  inline MatrixLabel copy() const
  {
    return MatrixLabel(std::make_shared<Container>(*_container), _indexBegin, _indexEnd);
  }

  /**
   * Empty and reset the container
   */
  inline void clear()
  {
    _container->clear();
    _indexBegin = 0;
    _indexEnd = (size_t)-1;
  }

  /**
   * Write and save given MatrixLabel view into
   * given file path name
   */
  inline void save(const std::string& fileName) const
  {
    if (size() == 0)
    {
      throw std::logic_error("MatrixLabel view empty");
    }

    // Open data file
    std::ofstream logFile(fileName);
    if (!logFile.is_open())
    {
      throw std::runtime_error("MatrixLabel unable to write file: " + fileName);
    }

    // Write first label line
    logFile << "# ";
    for (size_t j = 0; j < _container->front().size(); j++)
    {
      logFile << "'" << _container->front().getLabel(j) << "' ";
    }
    logFile << std::endl;
    // Write data
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      for (size_t j = 0; j < _container->front().size(); j++)
      {
        logFile << std::setprecision(17) << _container->at(i)(j) << " ";
      }
      logFile << std::endl;
    }

    logFile.close();
  }

  /**
   * Read and append data from given file path name
   */
  inline void load(const std::string& fileName)
  {
    // Open data file
    std::ifstream logFile(fileName);
    if (!logFile.is_open())
    {
      throw std::runtime_error("MatrixLabel unable to read file: " + fileName);
    }

    // Read first two data lines
    Leph::VectorLabel tmp;
    tmp.readFromCSV(logFile);
    append(tmp);

    // Read all others
    while (logFile.good())
    {
      // Skip labels line
      if (logFile.peek() == '#')
      {
        while (logFile.peek() != '\n')
        {
          if (!logFile.good())
          {
            throw std::runtime_error("MatrixLabel invalid format (1)");
          }
          logFile.ignore();
        }
        logFile.ignore();
      }
      else if (logFile.peek() == EOF)
      {
        break;
      }
      if (!logFile.good())
      {
        throw std::runtime_error("MatrixLabel invalid format (3)");
      }
      // Create a new points by copy
      append(_container->front());
      // Parse line values
      // All lines are supposed to have
      // same labels and order
      // than first line
      for (size_t i = 0; i < dimension(); i++)
      {
        double val;
        logFile >> val;
        _container->back()(i) = val;
        while (logFile.peek() == ' ')
        {
          if (!logFile.good())
          {
            throw std::runtime_error("MatrixLabel invalid format (4)");
          }
          logFile.ignore();
        }
        if (!logFile.good())
        {
          throw std::runtime_error("MatrixLabel invalid format (5)");
        }
      }
      if (logFile.peek() != '\n')
      {
        throw std::runtime_error("MatrixLabel invalid format (6)");
      }
      logFile.ignore();
    }
    logFile.close();
  }

  /**
   * Return time serie (view) size and contained
   * Vector dimension
   */
  inline size_t size() const
  {
    if (_container->size() == 0)
    {
      return 0;
    }
    else
    {
      return _indexEnd - _indexBegin + 1;
    }
  }
  inline size_t dimension() const
  {
    if (size() == 0)
    {
      return 0;
    }
    else
    {
      return _container->front().size();
    }
  }

  /**
   * Return a sliced view of this MatrixLabel
   */
  inline MatrixLabel range(size_t indexBegin, size_t indexEnd) const
  {
    return MatrixLabel(_container, indexBegin, indexEnd);
  }

  /**
   * Append given VectorLabel at the end of the serie.
   */
  inline void append(const VectorLabel& vect)
  {
    if (size() != 0 && dimension() != vect.size())
    {
      throw std::logic_error("MatrixLabel invalid dimension");
    }

    appendForce(vect);
  }

  /**
   * Append given VectorLabel at the end of the serie.
   * No error is thrown if given vector
   * has invalid dimension but export/import are expected
   * to failed !!!
   */
  inline void appendForce(const VectorLabel& vect)
  {
    if (_indexEnd == (size_t)-1 || _indexEnd == _container->size() - 1)
    {
      _indexEnd++;
    }
    _container->push_back(vect);
  }

  /**
   * Element access
   */
  inline const VectorLabel& operator[](size_t index) const
  {
    size_t realIndex = index + _indexBegin;
    if (realIndex < _indexBegin || realIndex > _indexEnd)
    {
      throw std::logic_error("MatrixLabel invalid index access: " + std::to_string(index));
    }

    return _container->at(realIndex);
  }
  inline VectorLabel& operator[](size_t index)
  {
    size_t realIndex = index + _indexBegin;
    if (realIndex < _indexBegin || realIndex > _indexEnd)
    {
      throw std::logic_error("MatrixLabel invalid index access: " + std::to_string(index));
    }

    return _container->at(realIndex);
  }

  /**
   * VectorLabel coefficient wise operation
   * apply on all contained points
   * (Beware if used VectorLabel vect is part
   * of the container itself because of reference aliasing)
   */
  inline void addOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).addOp(vect, filterSrc, filterDst);
    }
  }
  inline void subOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).subOp(vect, filterSrc, filterDst);
    }
  }
  inline void mulOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).mulOp(vect, filterSrc, filterDst);
    }
  }
  inline void divOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).divOp(vect, filterSrc, filterDst);
    }
  }
  inline void assignOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).assignOp(vect, filterSrc, filterDst);
    }
  }
  inline void addOp(double val, const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).addOp(val, filter);
    }
  }
  inline void subOp(double val, const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).subOp(val, filter);
    }
  }
  inline void mulOp(double val, const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).mulOp(val, filter);
    }
  }
  inline void divOp(double val, const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).divOp(val, filter);
    }
  }
  inline void powerOp(double val, const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).powerOp(val, filter);
    }
  }
  inline void squareOp(const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).squareOp(filter);
    }
  }
  inline void sqrtOp(const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).sqrtOp(filter);
    }
  }
  inline void zeroOp(const std::string& filter = "#")
  {
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).zeroOp(filter);
    }
  }

  /**
   * Compute the mean, variance and standard
   * deviation of this MatrixLabel view
   */
  inline VectorLabel mean() const
  {
    if (size() == 0)
    {
      throw std::logic_error("MatrixLabel empty");
    }

    VectorLabel sum = _container->front();
    sum.zeroOp();
    size_t count = 0;
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      sum.addOp(_container->at(i));
      count++;
    }

    if (count > 0)
    {
      sum.divOp(count);
    }
    return sum;
  }
  inline VectorLabel variance() const
  {
    if (size() == 0)
    {
      throw std::logic_error("MatrixLabel empty");
    }

    VectorLabel sum = _container->front();
    VectorLabel sum2 = _container->front();
    sum.zeroOp();
    sum2.zeroOp();
    size_t count = 0;
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      sum.addOp(_container->at(i));
      VectorLabel tmp = _container->at(i);
      tmp.squareOp();
      sum2.addOp(tmp);
      count++;
    }

    if (count > 0)
    {
      sum.divOp(count);
      sum2.divOp(count);
    }
    sum.squareOp();
    sum2.subOp(sum);
    return sum2;
  }
  inline VectorLabel stdDev() const
  {
    VectorLabel var = variance();
    var.sqrtOp();
    return var;
  }

  /**
   * Normalize the current view
   * by mean and standard deviation
   */
  inline void normalize()
  {
    VectorLabel m = mean();
    VectorLabel dev = stdDev();
    for (size_t i = 0; i < dev.size(); i++)
    {
      if (fabs(dev(i)) < 0.0001)
      {
        dev(i) = 1.0;
      }
    }

    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      _container->at(i).subOp(m);
      _container->at(i).divOp(dev);
    }
  }

  /**
   * Return a Plot object with data
   * of this MatrixLabel view added
   */
  inline Plot plot() const
  {
    Plot plot;
    for (size_t i = _indexBegin; i <= _indexEnd; i++)
    {
      plot.add(_container->at(i));
    }

    return plot;
  }

private:
  /**
   * VectorLabel container typedef
   */
  typedef std::vector<VectorLabel> Container;

  /**
   * The vector container
   */
  std::shared_ptr<Container> _container;

  /**
   * Global container index bound
   * (use to create view)
   */
  size_t _indexBegin;
  size_t _indexEnd;

  /**
   * Initialization of Matrix view
   * with existing container and given bounds
   */
  MatrixLabel(std::shared_ptr<Container> container, size_t indexBegin, size_t indexEnd)
    : _container(container), _indexBegin(indexBegin), _indexEnd(indexEnd)
  {
  }
};

}  // namespace Leph

#endif
