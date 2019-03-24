#ifndef LEPH_CONCEPT_HPP
#define LEPH_CONCEPT_HPP

#include <string>
#include <vector>
#include <stdexcept>
#include <limits>
#include <cmath>
#include "TimeSeries/TimeSeries.hpp"
#include "TimeSeries/Optimizable.hpp"

namespace Leph
{
/**
 * Concept
 *
 * Abstract base class for
 * Repressentation linking input to
 * output series into a graph.
 * It could also have optimizable
 * meta parameters.
 */
class Concept : public Optimizable
{
public:
  /**
   * Initialization
   */
  Concept() : Optimizable(), _inputSeries(), _outputSeries()
  {
  }

  /**
   * Empty virtual destructor
   */
  virtual ~Concept()
  {
  }

  /**
   * Return Concept name.
   * (Must be implemented)
   */
  virtual std::string name() const = 0;

  /**
   * Return the number of inputs
   * and outputs time series
   * (Must be implemented)
   */
  virtual size_t inputSize() const = 0;
  virtual size_t outputSize() const = 0;

  /**
   * Read inputs values, compute and append
   * outputs values at given time.
   * False is returned if input series values are
   * not available and no output update is performed.
   */
  inline bool computeAtTime(double time)
  {
    // Check inputs outputs initialization
    if (_inputSeries.size() == 0)
    {
      throw std::logic_error("Concept not initialized");
    }

    // Call real concept implementation
    return doCompute(time);
  }

  /**
   * Find in input series values not
   * propagated to output series. An
   * output value is computed for each
   * new input value.
   * False is returned if input series values are
   * not available and no output update is performed.
   */
  inline bool computePropagate()
  {
    // Check inputs outputs initialization
    if (_inputSeries.size() == 0 || _outputSeries.size() == 0)
    {
      throw std::logic_error("Concept not initialized");
    }

    // Check if all input are non empty
    for (size_t i = 0; i < _inputSeries.size(); i++)
    {
      if (_inputSeries[i]->size() < 2)
      {
        return false;
      }
    }

    // Check if there is one non empty output
    bool isOutputsEmpty = true;
    for (size_t i = 0; i < _outputSeries.size(); i++)
    {
      if (_outputSeries[i]->size() != 0)
      {
        isOutputsEmpty = false;
      }
    }

    // Look for maximum of input minimum time and minimum
    // of maximum
    double inputsTimeMin = _inputSeries.front()->timeMin();
    double inputsTimeMax = _inputSeries.front()->timeMax();
    for (size_t i = 0; i < _inputSeries.size(); i++)
    {
      if (_inputSeries[i]->timeMin() > inputsTimeMin)
      {
        inputsTimeMin = _inputSeries[i]->timeMin();
      }
      if (_inputSeries[i]->timeMax() < inputsTimeMax)
      {
        inputsTimeMax = _inputSeries[i]->timeMax();
      }
    }

    // Find propagate starting time
    double currentTime;
    if (isOutputsEmpty)
    {
      // If no output already exists, we have to
      // run propagation from inputs begining
      currentTime = inputsTimeMin;
    }
    else
    {
      // Look for maximum output time
      double outputsTimeMax = _outputSeries.front()->timeMax();
      for (size_t i = 0; i < _outputSeries.size(); i++)
      {
        if (_outputSeries[i]->timeMax() > outputsTimeMax)
        {
          outputsTimeMax = _outputSeries[i]->timeMax();
        }
      }
      if (outputsTimeMax <= inputsTimeMax && outputsTimeMax >= inputsTimeMin)
      {
        // We fill values from output
        // maximum values if fresh input values
        // are available
        currentTime = outputsTimeMax;
      }
      else
      {
        // Else, nothing to do
        return false;
      }
    }

    bool isUpdated = false;
    while (true)
    {
      // Find the time associated with the
      // first next updated input
      currentTime += TIME_EPSILON;
      double nextTime = std::numeric_limits<double>::quiet_NaN();
      bool isFinished = false;
      for (size_t i = 0; i < _inputSeries.size(); i++)
      {
        if (currentTime <= _inputSeries[i]->timeMax())
        {
          size_t indexLow = _inputSeries[i]->getLowerIndex(currentTime);
          double timeLow = _inputSeries[i]->at(indexLow).time;
          if (std::isnan(nextTime) || timeLow < nextTime)
          {
            nextTime = timeLow;
          }
        }
        else
        {
          isFinished = true;
        }
      }
      // End if one input has reached an end
      if (isFinished)
      {
        break;
      }
      if (std::isnan(nextTime))
      {
        throw std::logic_error("Concept error nan");
      }
      // Try to run concept and update
      // output values
      if (doCompute(nextTime))
      {
        isUpdated = true;
      }
      // Go to next point
      currentTime = nextTime;
    }

    return isUpdated;
  }

  /**
   * Direct access to internal
   * input and output time series pointer.
   */
  inline const TimeSeries* getInput(size_t index) const
  {
    if (_inputSeries.size() == 0)
    {
      throw std::logic_error("Concept not initialized");
    }
    if (index >= inputSize())
    {
      throw std::logic_error("Concept invalid index");
    }
    return _inputSeries[index];
  }
  inline const TimeSeries* getOutput(size_t index) const
  {
    if (_outputSeries.size() == 0)
    {
      throw std::logic_error("Concept not initialized");
    }
    if (index >= outputSize())
    {
      throw std::logic_error("Concept invalid index");
    }
    return _outputSeries[index];
  }
  inline TimeSeries* getOutput(size_t index)
  {
    if (_outputSeries.size() == 0)
    {
      throw std::logic_error("Concept not initialized");
    }
    if (index >= outputSize())
    {
      throw std::logic_error("Concept invalid index");
    }
    return _outputSeries[index];
  }

  /**
   * Set input and output time series pointer
   * at given index.
   * (pointer are intialized to nullptr and
   * must be defined)
   */
  inline void setInput(size_t index, const TimeSeries* ptr)
  {
    // Check pointer
    if (ptr == nullptr)
    {
      throw std::logic_error("Concept invalid null pointer");
    }
    if (index >= inputSize())
    {
      throw std::logic_error("Concept invalid index");
    }
    // Initialization if needed
    checkInit();
    // Assign
    _inputSeries[index] = ptr;
  }
  inline void setOutput(size_t index, TimeSeries* ptr)
  {
    // Check pointer
    if (ptr == nullptr)
    {
      throw std::logic_error("Concept invalid null pointer");
    }
    if (index >= outputSize())
    {
      throw std::logic_error("Concept invalid index");
    }
    // Initialization if needed
    checkInit();
    // Assign
    _outputSeries[index] = ptr;
  }

protected:
  /**
   * Implementation of Concept computation
   * at given time. Read inputs and generate
   * outputs values.
   * False is returned if input series values are
   * not available and no output update is performed.
   * (Must be implemented)
   */
  virtual bool doCompute(double time) = 0;

private:
  /**
   * Input and output time series container
   * of size inputSize() and outputSize().
   */
  std::vector<const TimeSeries*> _inputSeries;
  std::vector<TimeSeries*> _outputSeries;

  /**
   * Initialize input and output series
   * pointer container if they are still empty
   */
  inline void checkInit()
  {
    if (_inputSeries.size() == 0 || _outputSeries.size() == 0)
    {
      size_t inSize = inputSize();
      size_t outSize = outputSize();
      // Check implemetation
      if (inSize == 0 || outSize == 0)
      {
        throw std::logic_error("Concept null input or output size");
      }
      // Pointer initialization
      for (size_t i = 0; i < inSize; i++)
      {
        _inputSeries.push_back(nullptr);
      }
      for (size_t i = 0; i < outSize; i++)
      {
        _outputSeries.push_back(nullptr);
      }
    }
  }
};

}  // namespace Leph

#endif
