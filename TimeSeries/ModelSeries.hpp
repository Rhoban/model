#ifndef LEPH_MODELSERIES_HPP
#define LEPH_MODELSERIES_HPP

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <limits>
#include "Utils/LWPRUtils.h"
#include "TimeSeries/TimeSeries.hpp"
#include "TimeSeries/Concept.hpp"
#include "TimeSeries/RegressionLWPR.hpp"

namespace Leph
{
/**
 * ModelSeries
 *
 * Main container for delared time series,
 * Concepts and Regressions between series.
 */
class ModelSeries
{
public:
  /**
   * Initialization
   */
  ModelSeries() : _series(), _concepts(), _regressions()
  {
  }

  /**
   * Deallocating Series,
   * Concepts and Regressions
   */
  ~ModelSeries()
  {
    for (auto& series : _series)
    {
      delete series.second;
    }
    for (size_t i = 0; i < _concepts.size(); i++)
    {
      delete _concepts[i];
    }
    for (auto& model : _regressions)
    {
      delete model.second;
    }
  }

  /**
   * Access to given existing series name
   */
  inline const TimeSeries& series(const std::string& name) const
  {
    if (_series.count(name) == 0)
    {
      throw std::logic_error("ModelSeries invalid name: " + name);
    }
    return *_series.at(name);
  }
  inline TimeSeries& series(const std::string& name)
  {
    if (_series.count(name) == 0)
    {
      throw std::logic_error("ModelSeries invalid name: " + name);
    }
    return *_series.at(name);
  }

  /**
   * Create a new TimeSeries with given name
   * and given history length (-1 is infinite)
   */
  inline void addSeries(const std::string& name, size_t maxSize = -1)
  {
    if (_series.count(name) != 0)
    {
      throw std::logic_error("ModelSeries name exists: " + name);
    }

    _series[name] = new TimeSeries(name, maxSize);
  }

  /**
   * Clear all future data and enable or
   * disable future mode on all TimeSeries.
   */
  inline void enableFutureMode()
  {
    for (auto& series : _series)
    {
      series.second->clearFuture();
      series.second->enableFutureMode();
    }
  }
  inline void disableFutureMode()
  {
    for (auto& series : _series)
    {
      series.second->disableFutureMode();
    }
  }

  /**
   * Display all TimeSeries state
   */
  inline void printSeries() const
  {
    for (auto& series : _series)
    {
      std::cout << *(series.second) << std::endl;
    }
  }

  /**
   * Add the given newly allocated Concept to
   * the model with given inputs and outputs series names.
   * Parameters are resetted.
   */
  inline void addConcept(Concept* concept, const std::vector<std::string>& inputs,
                         const std::vector<std::string>& outputs)
  {
    _concepts.push_back(concept);
    // Reset meta parameters
    _concepts.back()->resetParameters();
    // Check inputs and ouputs numbers
    if (inputs.size() != _concepts.back()->inputSize())
    {
      throw std::logic_error("ModelSeries invalid input number for concept: " + concept->name());
    }
    if (outputs.size() != _concepts.back()->outputSize())
    {
      throw std::logic_error("ModelSeries invalid output number for concept: " + concept->name());
    }
    // Register inputs and outputs series
    for (size_t i = 0; i < inputs.size(); i++)
    {
      _concepts.back()->setInput(i, &series(inputs[i]));
    }
    for (size_t i = 0; i < outputs.size(); i++)
    {
      _concepts.back()->setOutput(i, &series(outputs[i]));
    }
  }

  /**
   * Propagate all values thought concepts
   */
  inline void propagateConcepts()
  {
    bool isUpdate = true;
    while (isUpdate)
    {
      isUpdate = false;
      for (size_t i = 0; i < _concepts.size(); i++)
      {
        if (_concepts[i]->computePropagate() == true)
        {
          isUpdate = true;
        }
      }
    }
  }

  /**
   * Propagate all regressions predictions
   * (Future mode has to be enabled)
   */
  inline void propagateRegressions()
  {
    bool isUpdate = true;
    while (isUpdate)
    {
      isUpdate = false;
      for (auto& model : _regressions)
      {
        if (model.second->computePropagate() == true)
        {
          isUpdate = true;
        }
      }
    }
  }

  /**
   * Create a new LWPR regression model with given
   * name, given output TimeSeries
   */
  inline void addRegression(const std::string& name, const std::string& output)
  {
    // Initializing Regression model
    RegressionLWPR* model = new RegressionLWPR();
    model->setOutput(&series(output));
    // Inserting to container
    if (_regressions.count(name) != 0)
    {
      delete model;
      throw std::logic_error("ModelSeries regression name exists: " + name);
    }
    _regressions[name] = model;
  }

  /**
   * Add given input TimeSeries to given named
   * regression
   */
  inline void regressionAddInputDeltaTime(const std::string& name, const std::string& input, double deltaTime = 0.0)
  {
    regression(name).addInputDeltaTime(&series(input), deltaTime);
  }
  inline void regressionAddInputDeltaIndex(const std::string& name, const std::string& input, size_t deltaIndex = 0)
  {
    regression(name).addInputDeltaIndex(&series(input), deltaIndex);
  }

  /**
   * Access to given named LWPR regression
   */
  inline const RegressionLWPR& regression(const std::string& name) const
  {
    if (_regressions.count(name) == 0)
    {
      throw std::logic_error("ModelSeries invalid regression name: " + name);
    }

    return *_regressions.at(name);
  }
  inline RegressionLWPR& regression(const std::string& name)
  {
    if (_regressions.count(name) == 0)
    {
      throw std::logic_error("ModelSeries invalid regression name: " + name);
    }

    return *_regressions.at(name);
  }

  /**
   * Run parameters optimization on all registered
   * regressions with given learn and test bound time and
   * maximum of iterations.
   * The optimization is run retry times and then found
   * parameters are saved in given folder path
   * (with trailling '/').
   */
  inline void regressionsOptimizeParameters(double beginLearnTime, double endLearnTime, double beginTestTime,
                                            double endTestTime, int maxIteration, bool isQuiet, int retry = 1,
                                            const std::string& folderPath = "/tmp/")
  {
    for (auto& model : _regressions)
    {
      std::cout << "Optimizing regression " << model.first << std::endl;
      std::string filepath = folderPath + model.first + ".params";
      double minError = -1.0;
      for (int k = 1; k <= retry; k++)
      {
        model.second->optimizeParameters(beginLearnTime, endLearnTime, beginTestTime, endTestTime, maxIteration,
                                         isQuiet);
        double error = model.second->rangeMSE(beginTestTime, endTestTime);
        std::cout << "CurrentMSE: " << error << " BestMSE: " << minError << " retry=" << k << std::endl;
        if (minError < 0.0 || minError > error)
        {
          minError = error;
          model.second->parameterSave(filepath);
        }
      }
    }
  }

  /**
   * Learn all regressions using all possible
   * data points between given time or
   * whose time are above given time threshold.
   * True is returned if some regressions have
   * been updated
   */
  inline bool regressionsLearn(double beginTime, double endTime)
  {
    bool isUpdate = false;
    for (auto& model : _regressions)
    {
      if (model.second->rangeLearn(beginTime, endTime))
      {
        isUpdate = true;
      }
    }

    return isUpdate;
  }
  inline bool regressionsLearn(double time)
  {
    return regressionsLearn(time - TIME_EPSILON, std::numeric_limits<double>::max());
  }

  /**
   * Display on standart output all registered regression
   * prediction error between given times
   */
  inline void regressionsPrintMSE(double beginTime, double endTime) const
  {
    for (auto& model : _regressions)
    {
      std::cout << model.first << ": " << model.second->rangeMSE(beginTime, endTime) << std::endl;
    }
  }

  /**
   * Write all registered regresssions into given
   * folder path (with trailling "/").
   */
  inline void regressionsSave(const std::string& folderPath) const
  {
    for (auto& model : _regressions)
    {
      std::string filepath = folderPath + model.first + ".bin";
      model.second->save(filepath);
    }
  }

  /**
   * Read all registered regressions from gien
   * folder path (with trailling "/").
   */
  inline void regressionsLoad(const std::string& folderPath)
  {
    for (auto& model : _regressions)
    {
      std::string filepath = folderPath + model.first + ".bin";
      model.second->load(filepath);
    }
  }

  /**
   * Write and read all registered regresssions meta parameter
   * into given folder path (with trailling "/").
   */
  inline void regressionsParameterSave(const std::string& folderPath) const
  {
    for (auto& model : _regressions)
    {
      std::string filepath = folderPath + model.first + ".params";
      model.second->parameterSave(filepath);
    }
  }
  inline void regressionsParameterLoad(const std::string& folderPath)
  {
    for (auto& model : _regressions)
    {
      std::string filepath = folderPath + model.first + ".params";
      model.second->parameterLoad(filepath);
      model.second->resetRegression();
    }
  }

  /**
   * Write all contained TimeSeries
   * into given file name
   */
  inline void saveTimeSeries(const std::string& filepath) const
  {
    // Open data file
    std::ofstream file(filepath);
    if (!file.is_open())
    {
      throw std::runtime_error("ModelSeries unable to write file: " + filepath);
    }

    // Save contained TimeSeries
    for (auto& series : _series)
    {
      series.second->save(file);
    }

    file.close();
  }

  /**
   * Read and load all TimeSeries into
   * given file name
   */
  inline void loadTimesSeries(const std::string& filepath)
  {
    // Open data file
    std::ifstream file(filepath);
    if (!file.is_open())
    {
      throw std::runtime_error("ModelSeries unable to read file: " + filepath);
    }

    // Read all saved TimeSeries
    while (file.good())
    {
      // Load TimeSeries
      TimeSeries series;
      series.load(file);
      addSeries(series.name());
      *(_series.at(series.name())) = series;
      // Skip end line
      while (file.peek() == ' ' || file.peek() == '\n')
      {
        if (!file.good())
          break;
        file.ignore();
      }
    }

    file.close();
  }

private:
  /**
   * Series container
   */
  std::map<std::string, TimeSeries*> _series;

  /**
   * Concepts container
   */
  std::vector<Concept*> _concepts;

  /**
   * LWPR regression container
   */
  std::map<std::string, RegressionLWPR*> _regressions;
};

}  // namespace Leph

#endif
