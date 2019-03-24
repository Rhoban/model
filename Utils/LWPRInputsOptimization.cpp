#include <iostream>
#include <stdexcept>
#include <libcmaes/cmaes.h>
#include "Utils/LWPRUtils.h"
#include "Utils/LWPRInputsOptimization.hpp"

namespace Leph
{
LWPRInputsOptimization::LWPRInputsOptimization(double testingRatio, size_t dataStepLearn)
  : _testingRatio(testingRatio), _dataStepLearn(dataStepLearn), _outputName(), _inputNames(), _sequences()
{
  if (_testingRatio <= 0.0 || _testingRatio >= 1.0)
  {
    throw std::logic_error("LWPRInputsOptimization invalid testingRatio");
  }
  if (_dataStepLearn == 0)
  {
    throw std::logic_error("LWPRInputsOptimization invalid dataStepLearn");
  }
}

void LWPRInputsOptimization::setOuput(const std::string& outputName)
{
  _outputName = outputName;
}

void LWPRInputsOptimization::setInputs(const std::vector<std::string>& inputNames)
{
  _inputNames = inputNames;
}

void LWPRInputsOptimization::addSequence(const MatrixLabel& seq)
{
  if (seq.size() == 0)
  {
    throw std::logic_error("LWPRInputsOptimization empty sequence");
  }
  _sequences.push_back(seq);
}

void LWPRInputsOptimization::optimize()
{
  checkInitialization();

  // Forward greedy selection
  InputSet set;
  for (size_t k = 0; k < _inputNames.size(); k++)
  {
    double bestScore = -1.0;
    std::string bestInput = "";
    for (size_t i = 0; i < _inputNames.size(); i++)
    {
      bool isIn = false;
      for (size_t j = 0; j < set.size(); j++)
      {
        if (set[j].name == _inputNames[i])
        {
          isIn = true;
        }
      }
      if (isIn)
      {
        continue;
      }
      InputSet testSet = set;
      testSet.push_back({ _inputNames[i], -1.0, -1.0 });
      double score = optimizeInputSet(testSet);
      if (bestScore < 0.0 || score < bestScore)
      {
        bestScore = score;
        bestInput = _inputNames[i];
      }
    }
    set.push_back({ bestInput, -1.0, -1.0 });
  }
  // Backward greedy selection
  for (size_t k = 0; k < _inputNames.size() - 1; k++)
  {
    double bestScore = -1.0;
    std::string bestInput = "";
    for (size_t i = 0; i < _inputNames.size(); i++)
    {
      bool isIn = false;
      for (size_t j = 0; j < set.size(); j++)
      {
        if (set[j].name == _inputNames[i])
        {
          isIn = true;
        }
      }
      if (!isIn)
      {
        continue;
      }
      InputSet testSet;
      for (size_t j = 0; j < set.size(); j++)
      {
        if (set[j].name != _inputNames[i])
        {
          testSet.push_back(set[j]);
        }
      }
      double score = optimizeInputSet(testSet);
      if (bestScore < 0.0 || score < bestScore)
      {
        bestScore = score;
        bestInput = _inputNames[i];
      }
    }
    InputSet tmpSet = set;
    set.clear();
    for (size_t j = 0; j < tmpSet.size(); j++)
    {
      if (tmpSet[j].name != bestInput)
      {
        set.push_back(tmpSet[j]);
      }
    }
  }

  // Forward greedy selection
  set.clear();
  for (size_t k = 0; k < _inputNames.size(); k++)
  {
    double bestScore = -1.0;
    std::string bestInput = "";
    for (size_t i = 0; i < _inputNames.size(); i++)
    {
      bool isIn = false;
      for (size_t j = 0; j < set.size(); j++)
      {
        if (set[j].name == _inputNames[i])
        {
          isIn = true;
        }
      }
      if (isIn)
      {
        continue;
      }
      InputSet testSet = set;
      testSet.push_back({ _inputNames[i], 1.0, 1.0 });
      double score = optimizeInputSet(testSet);
      if (bestScore < 0.0 || score < bestScore)
      {
        bestScore = score;
        bestInput = _inputNames[i];
      }
    }
    set.push_back({ bestInput, 1.0, 1.0 });
  }
  // Backward greedy selection
  for (size_t k = 0; k < _inputNames.size() - 1; k++)
  {
    double bestScore = -1.0;
    std::string bestInput = "";
    for (size_t i = 0; i < _inputNames.size(); i++)
    {
      bool isIn = false;
      for (size_t j = 0; j < set.size(); j++)
      {
        if (set[j].name == _inputNames[i])
        {
          isIn = true;
        }
      }
      if (!isIn)
      {
        continue;
      }
      InputSet testSet;
      for (size_t j = 0; j < set.size(); j++)
      {
        if (set[j].name != _inputNames[i])
        {
          testSet.push_back(set[j]);
        }
      }
      double score = optimizeInputSet(testSet);
      if (bestScore < 0.0 || score < bestScore)
      {
        bestScore = score;
        bestInput = _inputNames[i];
      }
    }
    InputSet tmpSet = set;
    set.clear();
    for (size_t j = 0; j < tmpSet.size(); j++)
    {
      if (tmpSet[j].name != bestInput)
      {
        set.push_back(tmpSet[j]);
      }
    }
  }
}

void LWPRInputsOptimization::checkInitialization() const
{
  if (_sequences.size() == 0)
  {
    throw std::logic_error("LWPRInputsOptimization empty sequences");
  }
  if (!_sequences.front()[0].exist(_outputName))
  {
    throw std::logic_error("LWPRInputsOptimization invalid output name:" + _outputName);
  }
  if (_inputNames.size() == 0)
  {
    throw std::logic_error("LWPRInputsOptimization empty inputs");
  }
  for (size_t i = 0; i < _inputNames.size(); i++)
  {
    if (!_sequences.front()[0].exist(_inputNames[i]))
    {
      throw std::logic_error("LWPRInputsOptimization invalid input name: " + _inputNames[i]);
    }
  }
}

void LWPRInputsOptimization::learnModel(LWPR_Object& model, const InputSet& set)
{
  // Get learning stop index
  size_t sequenceIndex;
  size_t dataIndex;
  testingStartIndex(sequenceIndex, dataIndex);
  // Do learning
  for (size_t i = 0; i < _sequences.size(); i++)
  {
    for (size_t j = 0; j < _sequences[i].size(); j += _dataStepLearn)
    {
      // Stopping index
      if (sequenceIndex == i && dataIndex == j)
      {
        return;
      }
      // Learning
      Eigen::VectorXd in = retrieveInputs(_sequences[i], j, set);
      if (in.size() > 0)
      {
        model.update(in, _sequences[i][j](_outputName));
      }
    }
  }
}

double LWPRInputsOptimization::testModel(LWPR_Object& model, const InputSet& set)
{
  // Get testing start index
  size_t sequenceIndex;
  size_t dataIndex;
  testingStartIndex(sequenceIndex, dataIndex);
  // Do testing
  double sumSquareError = 0.0;
  long countPoint = 0;
  for (size_t i = sequenceIndex; i < _sequences.size(); i++)
  {
    size_t start = 0;
    if (i == sequenceIndex)
      start = dataIndex;
    for (size_t j = start; j < _sequences[i].size(); j++)
    {
      // Prediction
      Eigen::VectorXd in = retrieveInputs(_sequences[i], j, set);
      if (in.size() > 0)
      {
        double yp = model.predict(in, 0.0)(0);
        // Compute prediction error
        double error = yp - _sequences[i][j](_outputName);
        sumSquareError += pow(error, 2);
        countPoint++;
      }
    }
  }

  if (countPoint > 0)
  {
    return sumSquareError / (double)countPoint;
  }
  else
  {
    return -1.0;
  }
}

size_t LWPRInputsOptimization::inputSetDim(const InputSet& set) const
{
  size_t dim = 0;
  for (size_t i = 0; i < set.size(); i++)
  {
    dim++;
    if (set[i].delta >= 0.0)
    {
      dim++;
    }
  }

  return dim;
}

Eigen::VectorXd LWPRInputsOptimization::initParamsFromInputSet(const InputSet& set) const
{
  size_t paramSize = 0;
  for (size_t i = 0; i < set.size(); i++)
  {
    if (set[i].lag >= 0.0)
    {
      paramSize++;
    }
    if (set[i].delta >= 0.0)
    {
      paramSize++;
    }
  }
  Eigen::VectorXd params(paramSize);
  paramSize = 0;
  for (size_t i = 0; i < set.size(); i++)
  {
    if (set[i].lag >= 0.0)
    {
      params(paramSize) = 1.0;
      paramSize++;
    }
    if (set[i].delta >= 0.0)
    {
      params(paramSize) = 1.0;
      paramSize++;
    }
  }

  return params;
}

void LWPRInputsOptimization::applyParamsToInputSet(const Eigen::VectorXd& params, InputSet& set) const
{
  size_t paramSize = 0;
  for (size_t i = 0; i < set.size(); i++)
  {
    if (set[i].lag >= 0.0)
    {
      if (params(paramSize) < 0.0)
      {
        set[i].lag = 0.0;
      }
      else
      {
        set[i].lag = params(paramSize);
      }
      paramSize++;
    }
    if (set[i].delta >= 0.0)
    {
      if (params(paramSize) < 0.0)
      {
        set[i].delta = 0.0;
      }
      else
      {
        set[i].delta = params(paramSize);
      }
      paramSize++;
    }
  }
}

Eigen::VectorXd LWPRInputsOptimization::retrieveInputs(const MatrixLabel& data, size_t index, const InputSet& set) const
{
  Eigen::VectorXd input(inputSetDim(set));
  size_t dim = 0;
  for (size_t i = 0; i < set.size(); i++)
  {
    double lag = set[i].lag;
    if (lag < 0.0)
    {
      lag = 0.0;
    }
    long lagLow = std::floor(lag);
    long lagUp = std::ceil(lag);
    double lagLength = lag - (double)lagLow;
    double delta = set[i].delta;
    size_t deltaLow = std::floor(delta);
    size_t deltaUp = std::ceil(delta);
    double deltaLength = delta - (double)deltaLow;
    if ((long)index - (long)lagUp < 0)
    {
      return Eigen::VectorXd();
    }
    double middle1 =
        (1.0 - lagLength) * data[index - lagLow](set[i].name) + lagLength * data[index - lagUp](set[i].name);
    input(dim) = middle1;
    dim++;
    if (set[i].delta >= 0.0)
    {
      if ((long)index - (long)lagLow - (long)deltaUp < 0)
      {
        return Eigen::VectorXd();
      }
      double middle2 = (1.0 - deltaLength) * data[index - lagLow - deltaLow](set[i].name) +
                       deltaLength * data[index - lagLow - deltaUp](set[i].name);
      input(dim) = middle2;
      dim++;
    }
  }

  return input;
}

void LWPRInputsOptimization::testingStartIndex(size_t& sequenceIndex, size_t& dataIndex) const
{
  // Sum total point size
  size_t totalSize = 0;
  for (size_t i = 0; i < _sequences.size(); i++)
  {
    totalSize += _sequences[i].size();
  }
  // Compute learning start index
  size_t index = (size_t)(_testingRatio * (double)(totalSize));
  // Find sequence and data index
  for (size_t i = 0; i < _sequences.size(); i++)
  {
    if (index < _sequences[i].size())
    {
      sequenceIndex = i;
      dataIndex = index;
      return;
    }
    else
    {
      index -= _sequences[i].size();
    }
  }
  throw std::logic_error("LWPRInputsOptimization index error");
}

double LWPRInputsOptimization::optimizeInputSet(const InputSet& set)
{
  // Parameter initialization
  Eigen::VectorXd paramsSet = initParamsFromInputSet(set);
  Eigen::VectorXd paramsLWPR = initParamsLWPR(set);
  size_t sizeParamsSet = paramsSet.size();
  size_t sizeParamsLWPR = paramsLWPR.size();

  // Starting point conversion to vector
  std::vector<double> x0(sizeParamsLWPR + sizeParamsSet);
  for (size_t i = 0; i < sizeParamsLWPR; i++)
  {
    x0[i] = paramsLWPR(i);
  }
  for (size_t i = 0; i < sizeParamsSet; i++)
  {
    x0[sizeParamsLWPR + i] = paramsSet(i);
  }

  // Optimization init
  libcmaes::CMAParameters<> cmaparams(x0, 0.1);
  cmaparams.set_quiet(true);
  cmaparams.set_mt_feval(true);
  cmaparams.set_str_algo("acmaes");
  cmaparams.set_max_iter(2000);

  // Fitness function
  libcmaes::FitFunc fitness = [this, sizeParamsLWPR, sizeParamsSet, &set](const double* x, const int N) {
    (void)N;
    // Conversion to Eigen Vector format
    InputSet tmpSet = set;
    Eigen::VectorXd paramsLWPR(sizeParamsLWPR);
    Eigen::VectorXd paramsSet(sizeParamsSet);
    for (size_t i = 0; i < sizeParamsLWPR; i++)
    {
      paramsLWPR(i) = x[i];
    }
    for (size_t i = 0; i < sizeParamsSet; i++)
    {
      paramsSet(i) = x[i + sizeParamsLWPR];
    }
    // Penality for unbound parameters
    double cost = 0.0;
    cost += boundParamsLWPR(set, paramsLWPR);
    cost += boundParamsSet(set, paramsSet);
    // Model init
    LWPR_Object model = this->initModel(set, paramsLWPR);
    this->applyParamsToInputSet(paramsSet, tmpSet);
    // Learning
    this->learnModel(model, tmpSet);
    // Testing
    double mse = this->testModel(model, tmpSet);
    // Return fitness reward
    return mse + cost;
  };

  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
  Eigen::VectorXd bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
  double score = cmasols.get_best_seen_candidate().get_fvalue();

  // Display optimization stats on stanraft output
  std::cout << "Score=" << score << " ";
  std::cout << "Set={";
  for (size_t i = 0; i < set.size(); i++)
  {
    std::cout << set[i].name;
    if (set[i].lag >= 0.0)
    {
      std::cout << " lag";
    }
    if (set[i].delta >= 0.0)
    {
      std::cout << " delta";
    }
    if (i != set.size() - 1)
    {
      std::cout << ", ";
    }
  }
  std::cout << "} ";
  Eigen::VectorXd bestParamsLWPR(sizeParamsLWPR);
  Eigen::VectorXd bestParamsSet(sizeParamsSet);
  for (size_t i = 0; i < sizeParamsLWPR; i++)
  {
    bestParamsLWPR(i) = bestParams(i);
  }
  for (size_t i = 0; i < sizeParamsSet; i++)
  {
    bestParamsSet(i) = bestParams(i + sizeParamsLWPR);
  }
  std::cout << "ParamSet={" << bestParamsSet.transpose() << "} ";
  std::cout << "Params={";
  std::cout << bestParams.transpose() << "} ";
  LWPR_Object model = this->initModel(set, bestParamsLWPR);
  InputSet tmpSet = set;
  applyParamsToInputSet(bestParamsSet, tmpSet);
  learnModel(model, tmpSet);
  std::cout << "RF=" << LWPRRFTrustworthy(model) << " ";
  std::cout << "MinRFReg=" << LWPRMinRFReg(model) << " ";
  std::cout << "MaxRFReg=" << LWPRMaxRFReg(model) << " ";
  std::cout << "DataCount=" << model.nData() << " ";
  std::cout << std::endl;

  return score;
}

Eigen::VectorXd LWPRInputsOptimization::initParamsLWPR(const InputSet& set) const
{
  size_t sizeIn = inputSetDim(set);
  Eigen::VectorXd params(sizeIn + 2);

  // NormIn
  for (size_t i = 0; i < sizeIn; i++)
  {
    params(i) = 1.0;
  }
  // Penalty
  params(sizeIn) = 1e-6;
  // InitD
  params(sizeIn + 1) = 1.0;

  return params;
}

double LWPRInputsOptimization::boundParamsLWPR(const InputSet& set, Eigen::VectorXd& params) const
{
  size_t sizeIn = inputSetDim(set);
  double cost = 0.0;

  // NormIn
  for (size_t i = 0; i < sizeIn; i++)
  {
    if (params(i) < 0.01)
    {
      cost += 100.0 + 1000.0 * fabs(params(i) - 0.01);
      params(i) = 0.01;
    }
  }
  // Penalty
  if (params(sizeIn) < 0.0)
  {
    cost += 100.0 + 1000.0 * fabs(params(sizeIn) - 0.0);
    params(sizeIn) = 0.0;
  }
  // InitD
  if (params(sizeIn + 1) < 0.01)
  {
    cost += 100.0 + 1000.0 * fabs(params(sizeIn + 1) - 0.01);
    params(sizeIn + 1) = 0.01;
  }

  return cost;
}
double LWPRInputsOptimization::boundParamsSet(const InputSet& set, Eigen::VectorXd& params) const
{
  double cost = 0.0;

  size_t index = 0;
  for (size_t i = 0; i < set.size(); i++)
  {
    if (set[i].lag >= 0.0)
    {
      if (params(index) < 0.0)
      {
        cost += 100.0 + 100.0 * fabs(params(index));
        params(index) = 0.0;
      }
      index++;
    }
    if (set[i].delta >= 0.0)
    {
      if (params(index) < 1.0)
      {
        cost += 100.0 + 100.0 * fabs(params(index) - 1.0);
        params(index) = 1.0;
      }
      index++;
    }
  }

  return cost;
}

LWPR_Object LWPRInputsOptimization::initModel(const InputSet& set, const Eigen::VectorXd& params) const
{
  size_t sizeIn = inputSetDim(set);
  LWPR_Object model(sizeIn);
  try
  {
    // Normalisation value for each input dimension (>0)
    model.normIn(params.head(sizeIn));
    // Use only diagonal matrix. Big speed up in high dimension
    // but lower performance in complex learning.
    model.diagOnly(false);
    // Automatic tunning of distance metric
    model.useMeta(true);
    // Larger value enforce wider receptive field (>0)
    model.penalty(params(sizeIn));
    // Set diagonal (input_dim) or complet (input_dim*input_dim)
    // initial distance matrix (>0)
    model.setInitD(params(sizeIn + 1));
  }
  catch (const LWPR_Exception& e)
  {
    throw std::logic_error("LWPR_Exception LWPR parameters exception: " + std::string(e.getString()));
  }

  return model;
}

}  // namespace Leph
