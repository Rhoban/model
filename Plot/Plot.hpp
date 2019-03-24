#ifndef LEPH_PLOT_HPP
#define LEPH_PLOT_HPP

#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include "Utils/time.h"
#include "Types/VectorLabel.hpp"
#include "TimeSeries/TimeSeries.hpp"

namespace Leph
{
/**
 * Plot
 *
 * Programming interface with
 * the Linux ploting utility
 * Gnuplot using VectorLabel
 */
class Plot
{
public:
  /**
   * Plot style enumeration
   */
  enum Style
  {
    Points,
    Lines,
    LinesPoints,
    ErrorsPoints,
    ErrorsLines,
    None,
  };

  /**
   * Initialization
   */
  Plot()
    : _database()
    , _plots2D()
    , _plots3D()
    , _multiplotLayoutRow(1)
    , _multiplotLayoutCol(1)
    , _multiplotIndexes()
    , _rangeMinX(1.0)
    , _rangeMaxX(0.0)
    , _rangeMinY(1.0)
    , _rangeMaxY(0.0)
    , _rangeMinZ(1.0)
    , _rangeMaxZ(0.0)
    , _rangeUniform(false)
    , _pipeFd(-1)
  {
  }

  /**
   * Add association name -> value to
   * internal container.
   */
  inline void add(const std::map<std::string, double>& list)
  {
    VectorLabel vect;
    for (const auto& it : list)
    {
      vect.setOrAppend(it.first, it.second);
    }
    _database.push_back(vect);
  }

  /**
   * Add a (vector) point
   */
  inline void add(const VectorLabel& vect)
  {
    _database.push_back(vect);
  }

  /**
   * Merge the given plot data base into
   * this.
   */
  inline void merge(const Plot plot)
  {
    for (size_t i = 0; i < plot._database.size(); i++)
    {
      add(plot._database[i]);
    }
  }

  /**
   * Add given TimeSeries with optionally
   * an other name
   */
  inline void add(const TimeSeries& series)
  {
    for (size_t i = 0; i < series.size(); i++)
    {
      _database.push_back(VectorLabel("time", series[i].time, series.name(), series[i].value));
    }
  }
  inline void add(const std::string& name, const TimeSeries& series)
  {
    for (size_t i = 0; i < series.size(); i++)
    {
      _database.push_back(VectorLabel("time", series[i].time, name, series[i].value));
    }
  }

  /**
   * Request a 2D plot with X and Y axis
   * X axis could be "index"
   * Y axis could be "all"
   * Y axis could be "filter:*"
   */
  inline Plot& plot(const std::string& xAxis, const std::string& yAxis, const Style style = LinesPoints,
                    const std::string& palette = "")
  {
    for (size_t i = 0; i < _plots2D.size(); i++)
    {
      if (_plots2D[i].xAxis == xAxis && _plots2D[i].yAxis == yAxis)
      {
        _plots2D[i].xAxis = xAxis;
        _plots2D[i].yAxis = yAxis;
        _plots2D[i].style = style;
        _plots2D[i].palette = palette;
        return *this;
      }
    }

    if (yAxis == "all")
    {
      for (size_t i = 0; i < _database.size(); i++)
      {
        for (const auto& label : _database[i].labels())
        {
          if (label.first != xAxis)
          {
            plot(xAxis, label.first, style, palette);
          }
        }
      }
      return *this;
    }
    else if (yAxis.size() > 2 && yAxis[yAxis.size() - 1] == '*' && yAxis[yAxis.size() - 2] == ':')
    {
      std::string filter = yAxis.substr(0, yAxis.find_first_of(':'));
      for (size_t i = 0; i < _database.size(); i++)
      {
        for (const auto& label : _database[i].labels())
        {
          if (label.first != xAxis && VectorLabel::toSection(label.first) == filter)
          {
            plot(xAxis, label.first, style, palette);
          }
        }
      }
      return *this;
    }
    else
    {
      if ((style == ErrorsLines || style == ErrorsPoints) && palette == "")
      {
        throw std::logic_error("Plot errors need third column");
      }
      Plot2D request;
      request.xAxis = xAxis;
      request.yAxis = yAxis;
      request.style = style;
      request.palette = palette;
      _plots2D.push_back(request);
      return *this;
    }
  }

  /**
   * Request a 3D plot with X, Y and Z axis
   * X axis could be "index"
   * Y axis could be "index"
   * Z axis could be "all"
   * Z axis could be "filter:*"
   * Z axis could be "ZERO"
   */
  inline Plot& plot(const std::string& xAxis, const std::string& yAxis, const std::string& zAxis,
                    const Style style = Points, const std::string& palette = "")
  {
    for (size_t i = 0; i < _plots3D.size(); i++)
    {
      if (_plots3D[i].xAxis == xAxis && _plots3D[i].yAxis == yAxis && _plots3D[i].zAxis == zAxis)
      {
        _plots3D[i].xAxis = xAxis;
        _plots3D[i].yAxis = yAxis;
        _plots3D[i].zAxis = zAxis;
        _plots3D[i].style = style;
        _plots3D[i].palette = palette;
        return *this;
      }
    }

    if (zAxis == "all")
    {
      for (size_t i = 0; i < _database.size(); i++)
      {
        for (const auto& label : _database[i].labels())
        {
          if (label.first != xAxis && label.first != yAxis)
          {
            plot(xAxis, yAxis, label.first, style, palette);
          }
        }
      }
      return *this;
    }
    else if (zAxis.size() > 2 && zAxis[zAxis.size() - 1] == '*' && zAxis[zAxis.size() - 2] == ':')
    {
      std::string filter = zAxis.substr(0, zAxis.find_first_of(':'));
      for (size_t i = 0; i < _database.size(); i++)
      {
        for (const auto& label : _database[i].labels())
        {
          if (label.first != xAxis && VectorLabel::toSection(label.first) == filter)
          {
            plot(xAxis, yAxis, label.first, style, palette);
          }
        }
      }
      return *this;
    }
    else
    {
      if (style == ErrorsLines || style == ErrorsPoints)
      {
        throw std::logic_error("Plot errors not implemented in 3D");
      }
      Plot3D request;
      request.xAxis = xAxis;
      request.yAxis = yAxis;
      request.zAxis = zAxis;
      request.style = style;
      request.palette = palette;
      _plots3D.push_back(request);
      return *this;
    }
  }

  /**
   * Set x,y,z ploting range
   */
  inline Plot& rangeX(double min, double max)
  {
    _rangeMinX = min;
    _rangeMaxX = max;
    _rangeUniform = false;

    return *this;
  }
  inline Plot& rangeY(double min, double max)
  {
    _rangeMinY = min;
    _rangeMaxY = max;
    _rangeUniform = false;

    return *this;
  }
  inline Plot& rangeZ(double min, double max)
  {
    _rangeMinZ = min;
    _rangeMaxZ = max;
    _rangeUniform = false;

    return *this;
  }

  /**
   * Enable uniform scaling
   */
  inline Plot& rangeUniform(bool value = true)
  {
    _rangeMinX = 1.0;
    _rangeMaxX = 0.0;
    _rangeMinY = 1.0;
    _rangeMaxY = 0.0;
    _rangeMinZ = 1.0;
    _rangeMaxZ = 0.0;
    _rangeUniform = value;

    return *this;
  }

  /**
   * Set multiplot layout Row and Col
   */
  inline Plot& multiplot(unsigned int row, unsigned int col)
  {
    if (_plots3D.size() != 0)
    {
      throw std::logic_error("Plot 3D plots invalid with multiplot");
    }
    _multiplotLayoutRow = row;
    _multiplotLayoutCol = col;

    return *this;
  }

  /**
   * Following plot calls will be plot
   * in next multiplot layout
   */
  inline Plot& nextPlot()
  {
    if (_plots3D.size() != 0)
    {
      throw std::logic_error("Plot 3D plots invalid with multiplot");
    }
    if (_plots2D.size() > 0)
    {
      _multiplotIndexes.push_back(_plots2D.size());
    }

    size_t countAvailabled = _multiplotLayoutRow * _multiplotLayoutCol;
    size_t countNeeded = _multiplotIndexes.size() + 1;
    if (countAvailabled < countNeeded)
    {
      if (_multiplotLayoutRow == _multiplotLayoutCol)
      {
        _multiplotLayoutCol++;
      }
      else
      {
        _multiplotLayoutRow++;
      }
    }

    return *this;
  }

  /**
   * Render the plot
   * Wait until plot window is closed
   */
  inline void render(bool waitExit = true, const std::string& exportFile = "")
  {
    if (_plots2D.size() > 0 && _plots3D.size() > 0)
    {
      throw std::logic_error("Plot error 2d and 3d request");
    }
    if (_plots2D.size() == 0 && _plots3D.size() == 0)
    {
      return;
    }

    std::string commands = generatePlotting();

    // Dump gnuplot script to file
    std::ofstream file;
    if (exportFile == "")
    {
      file.open("/tmp/plot-" + Leph::currentDate() + ".plot");
    }
    else
    {
      file.open(exportFile);
    }
    file << commands;
    file.close();

    // Send to gnuplot instance
    if (exportFile == "")
    {
      createGnuplotInstance();
      if (_pipeFd <= 0)
      {
        throw std::logic_error("Plot closed pipe fd");
      }
      int written = write(_pipeFd, commands.c_str(), commands.length());
      if (written != (int)commands.length())
      {
        std::cerr << "Plot failed to write in pipe: " << std::to_string(written) << std::endl;
      }
      if (waitExit)
      {
        waitCloseGnuplotInstance();
      }
    }

    clearPlots();
  }

  /**
   * Reset points database, plot
   * requests and both
   */
  inline void clearPoints()
  {
    _database.clear();
  }
  inline void clearPlots()
  {
    _plots2D.clear();
    _plots3D.clear();
    _multiplotLayoutRow = 1;
    _multiplotLayoutCol = 1;
    _multiplotIndexes.clear();
    _rangeMinX = 1.0;
    _rangeMaxX = 0.0;
    _rangeMinY = 1.0;
    _rangeMaxY = 0.0;
    _rangeMinZ = 1.0;
    _rangeMaxZ = 0.0;
    _rangeUniform = false;
  }
  inline void clear()
  {
    clearPoints();
    clearPlots();
  }

private:
  /**
   * Plot request 2D and 3D
   * structures
   */
  struct Plot2D
  {
    std::string xAxis;
    std::string yAxis;
    Style style;
    std::string palette;
  };
  struct Plot3D
  {
    std::string xAxis;
    std::string yAxis;
    std::string zAxis;
    Style style;
    std::string palette;
  };

  /**
   * Vector labeled points database
   */
  std::vector<VectorLabel> _database;

  /**
   * Plot request container
   */
  std::vector<Plot2D> _plots2D;
  std::vector<Plot3D> _plots3D;

  /**
   * Multiplot row and col layout
   * and _multiplotIndexes hold the split
   * index in plot2D
   */
  unsigned int _multiplotLayoutRow;
  unsigned int _multiplotLayoutCol;
  std::vector<size_t> _multiplotIndexes;

  /**
   * Plot x,y,z min,max range
   * If max > min, auto scaling is used
   */
  double _rangeMinX;
  double _rangeMaxX;
  double _rangeMinY;
  double _rangeMaxY;
  double _rangeMinZ;
  double _rangeMaxZ;

  /**
   * If true, uniform scaling is enabled
   */
  bool _rangeUniform;

  /**
   * Gnuplot pipe file descriptor
   */
  int _pipeFd;

  /**
   * Fork current process to create a new GnuPlot window
   */
  inline void createGnuplotInstance()
  {
    // Creating communication pipe
    int pipefd[2];
    if (pipe(pipefd) == -1)
    {
      throw std::runtime_error("Plot failed to create pipe");
    }

    // Forkink current process
    pid_t pid = fork();
    if (pid > 0)
    {
      // Closing reading pipe end
      close(pipefd[0]);
      // Saving pipe fd
      _pipeFd = pipefd[1];
    }
    else if (pid == 0)
    {
      // Closing writting pipe end
      close(pipefd[1]);
      // Redirecting reading pipe end to standart input
      if (dup2(pipefd[0], STDIN_FILENO) == -1)
      {
        throw std::runtime_error("Plot failed to dup2");
      }
      // Closing output and err
      int null = open("/dev/null", O_WRONLY);
      if (dup2(null, STDOUT_FILENO) == -1)
      {
        throw std::runtime_error("Plot failed to dup2");
      }
      if (dup2(null, STDERR_FILENO) == -1)
      {
        throw std::runtime_error("Plot failed to dup2");
      }
      // Calling Gnuplot
      execlp("gnuplot", "gnuplot", "-", NULL);
    }
    else
    {
      throw std::runtime_error("Plot failed to fork");
    }
  }

  /**
   * Wait for end of gnuplot session and
   * close the openned pipe to GnuPlot window instance
   */
  inline void waitCloseGnuplotInstance()
  {
    waitpid(-1, NULL, 0);
    if (_pipeFd != -1)
    {
      close(_pipeFd);
      _pipeFd = -1;
    }
  }

  /**
   * Generate and return Gnuplot commands and data
   */
  inline std::string generatePlotting()
  {
    std::string commandsSum;
    commandsSum += "set terminal qt size 1200,800\n";

    bool isMultiplot = false;
    if (_multiplotLayoutRow != 1 || _multiplotLayoutCol != 1)
    {
      isMultiplot = true;
    }

    if (isMultiplot)
    {
      commandsSum += "set multiplot layout ";
      commandsSum += std::to_string(_multiplotLayoutRow);
      commandsSum += ", ";
      commandsSum += std::to_string(_multiplotLayoutCol);
      commandsSum += ";\n";
    }
    int indexSplitMultiplot = -1;
    while (true)
    {
      std::string commands;
      std::string data;
      if (indexSplitMultiplot >= (int)_multiplotIndexes.size())
      {
        break;
      }
      size_t startIndex2D = 0;
      size_t endIndex2D = _plots2D.size() - 1;
      if (isMultiplot)
      {
        if (indexSplitMultiplot >= 0)
        {
          startIndex2D = _multiplotIndexes[indexSplitMultiplot];
        }
        else
        {
          startIndex2D = 0;
        }
        if (indexSplitMultiplot < (int)_multiplotIndexes.size() - 1)
        {
          endIndex2D = _multiplotIndexes[indexSplitMultiplot + 1] - 1;
        }
        else
        {
          endIndex2D = _plots2D.size() - 1;
        }
        indexSplitMultiplot++;
      }
      commands += "set grid;\n";
      if (_rangeUniform)
      {
        commands += "set size ratio -1;\n";
      }
      if (_rangeMinX < _rangeMaxX)
      {
        std::ostringstream oss;
        oss << "set xrange[" << std::setprecision(10) << _rangeMinX;
        oss << ":" << std::setprecision(10) << _rangeMaxX << "];\n";
        commands += oss.str();
      }
      if (_rangeMinY < _rangeMaxY)
      {
        std::ostringstream oss;
        oss << "set yrange[" << std::setprecision(10) << _rangeMinY;
        oss << ":" << std::setprecision(10) << _rangeMaxY << "];\n";
        commands += oss.str();
      }
      if (_rangeMinZ < _rangeMaxZ)
      {
        std::ostringstream oss;
        oss << "set zrange[" << std::setprecision(10) << _rangeMinZ;
        oss << ":" << std::setprecision(10) << _rangeMaxZ << "];\n";
        commands += oss.str();
      }

      // Create commands and data to send to gnuplot
      if (_plots2D.size() != 0)
      {
        commands += "plot ";
      }
      else
      {
        commands += "splot ";
      }

      bool isFirst = true;
      for (size_t i = startIndex2D; _plots2D.size() > 0 && i <= endIndex2D; i++)
      {
        if (_plots2D[i].style == None)
        {
          continue;
        }
        bool isPalette = _plots2D[i].palette != "";
        if (!isFirst)
        {
          commands += ", ";
        }
        isFirst = false;
        if (isPalette)
        {
          if (_plots2D[i].style == ErrorsLines || _plots2D[i].style == ErrorsPoints)
          {
            commands += "'-' using 1:2:3 with ";
          }
          else
          {
            commands += "'-' using 1:2:3 palette with ";
          }
        }
        else
        {
          commands += "'-' using 1:2 with ";
        }
        if (_plots2D[i].style == Points)
        {
          commands += "points";
        }
        if (_plots2D[i].style == Lines)
        {
          commands += "lines";
        }
        if (_plots2D[i].style == LinesPoints)
        {
          commands += "linespoints";
        }
        if (_plots2D[i].style == ErrorsPoints)
        {
          commands += "yerrorpoints";
        }
        if (_plots2D[i].style == ErrorsLines)
        {
          commands += "yerrorlines";
        }
        if (isPalette)
        {
          commands +=
              " title '" + _plots2D[i].xAxis + " --> " + _plots2D[i].yAxis + " // " + _plots2D[i].palette + "' ";
        }
        else
        {
          commands += " title '" + _plots2D[i].xAxis + " --> " + _plots2D[i].yAxis + "' ";
        }
        for (size_t j = 0; j < _database.size(); j++)
        {
          if ((_plots2D[i].xAxis != "index" && !_database[j].exist(_plots2D[i].xAxis)) ||
              !_database[j].exist(_plots2D[i].yAxis) ||
              (isPalette && _plots2D[i].palette != "index" && !_database[j].exist(_plots2D[i].palette)))
          {
            continue;
          }
          std::ostringstream oss;
          if (_plots2D[i].xAxis == "index")
          {
            oss << j << " " << std::setprecision(10) << _database[j](_plots2D[i].yAxis);
          }
          else
          {
            oss << std::setprecision(10) << _database[j](_plots2D[i].xAxis) << " " << std::setprecision(10)
                << _database[j](_plots2D[i].yAxis);
          }
          if (isPalette)
          {
            if (_plots2D[i].palette == "index")
            {
              oss << " " << j;
            }
            else
            {
              oss << " " << std::setprecision(10) << _database[j](_plots2D[i].palette);
            }
          }
          data += oss.str() + "\n";
        }
        data += "end\n";
      }
      for (size_t i = 0; i < _plots3D.size(); i++)
      {
        if (_plots3D[i].style == None)
        {
          continue;
        }
        bool isPalette = _plots3D[i].palette != "";
        if (!isFirst)
        {
          commands += ", ";
        }
        isFirst = false;
        if (isPalette)
        {
          commands += "'-' using 1:2:3:4 palette with ";
        }
        else
        {
          commands += "'-' using 1:2:3 with ";
        }
        if (_plots3D[i].style == Points)
        {
          commands += "points";
        }
        if (_plots3D[i].style == Lines)
        {
          commands += "lines";
        }
        if (_plots3D[i].style == LinesPoints)
        {
          commands += "linespoints";
        }
        if (isPalette)
        {
          commands += " title '" + _plots3D[i].xAxis + "," + _plots3D[i].yAxis + " --> " + _plots3D[i].zAxis + " // " +
                      _plots3D[i].palette + "' ";
        }
        else
        {
          commands += " title '" + _plots3D[i].xAxis + "," + _plots3D[i].yAxis + " --> " + _plots3D[i].zAxis + "' ";
        }
        for (size_t j = 0; j < _database.size(); j++)
        {
          if ((_plots3D[i].xAxis != "index" && !_database[j].exist(_plots3D[i].xAxis)) ||
              (_plots3D[i].yAxis != "index" && !_database[j].exist(_plots3D[i].yAxis)) ||
              (_plots3D[i].zAxis != "ZERO" && !_database[j].exist(_plots3D[i].zAxis)) ||
              (isPalette && _plots3D[i].palette != "index" && !_database[j].exist(_plots3D[i].palette)))
          {
            continue;
          }
          std::ostringstream oss;
          if (_plots3D[i].xAxis == "index")
          {
            oss << j << " ";
          }
          else
          {
            oss << std::setprecision(10) << _database[j](_plots3D[i].xAxis) << " ";
          }
          if (_plots3D[i].yAxis == "index")
          {
            oss << j << " ";
          }
          else
          {
            oss << std::setprecision(10) << _database[j](_plots3D[i].yAxis) << " ";
          }
          if (_plots3D[i].zAxis == "ZERO")
          {
            oss << "0.0";
          }
          else
          {
            oss << std::setprecision(10) << _database[j](_plots3D[i].zAxis);
          }
          if (isPalette)
          {
            if (_plots3D[i].palette == "index")
            {
              oss << " " << j;
            }
            else
            {
              oss << " " << std::setprecision(10) << _database[j](_plots3D[i].palette);
            }
          }
          data += oss.str() + "\n";
        }
        data += "end\n";
      }
      commands += ";\n";
      commandsSum += commands + data;
      if (!isMultiplot)
      {
        break;
      }
    }
    if (isMultiplot)
    {
      commandsSum += "unset multiplot;\n";
    }
    commandsSum += "pause mouse close;\nquit;\nquit;\n";

    return commandsSum;
  }
};

}  // namespace Leph

#endif
