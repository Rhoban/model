#ifndef LEPH_LWPRINPUTSOPTIMIZATION_HPP
#define LEPH_LWPRINPUTSOPTIMIZATION_HPP

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <lwpr_eigen.hpp>
#include "Types/VectorLabel.hpp"
#include "Types/MatrixLabel.hpp"

namespace Leph
{
/**
 * LWPRInputsOptimization
 *
 * Try to find the better fitting
 * inputs to predict selected output
 */
class LWPRInputsOptimization
{
public:
  /**
   * Initialization
   */
  LWPRInputsOptimization(double testingRatio = 0.5, size_t dataStepLearn = 1);

  /**
   * Set output name
   */
  void setOuput(const std::string& outputName);

  /**
   * Set used input names
   */
  void setInputs(const std::vector<std::string>& inputNames);

  /**
   * Add given learning sequence
   * as MatrixLabel
   */
  void addSequence(const MatrixLabel& seq);

  /**
   * TODO
   */
  void optimize();

private:
  /**
   * Item data for one
   * regression input.
   * Lag and delta offset are
   * disable if negative.
   */
  struct InputItem
  {
    std::string name;
    double lag;
    double delta;
  };

  /**
   * Typedef for regression inputs
   * item set
   */
  typedef std::vector<InputItem> InputSet;

  /**
   * Configuration variable
   */
  double _testingRatio;
  size_t _dataStepLearn;

  /**
   * Output data name
   */
  std::string _outputName;

  /**
   * Input data names container
   */
  std::vector<std::string> _inputNames;

  /**
   * Learning sequences data
   */
  std::vector<MatrixLabel> _sequences;

  /**
   * Check if the class is properly
   * initialized
   */
  void checkInitialization() const;

  /**
   * Learn given model and input set
   * with internal data
   */
  void learnModel(LWPR_Object& model, const InputSet& set);

  /**
   * Compute and return the mean square
   * error (MSE) of given model and given input set
   */
  double testModel(LWPR_Object& model, const InputSet& set);

  /**
   * Return the input dimention of given
   * input set
   */
  size_t inputSetDim(const InputSet& set) const;

  /**
   * Return given InputSet parameters
   * into Eigen Vector format
   */
  Eigen::VectorXd initParamsFromInputSet(const InputSet& set) const;

  /**
   * Assign given parameters in Eigen VectorXd
   * to given InputSet
   */
  void applyParamsToInputSet(const Eigen::VectorXd& params, InputSet& set) const;

  /**
   * Build input vector using given data MatrixLabel
   * and given index with given InputSet.
   * Empty Vector is returned if this point is
   * not possible.
   */
  Eigen::VectorXd retrieveInputs(const MatrixLabel& data, size_t index, const InputSet& set) const;

  /**
   * Compute and assign sequence index and
   * data index limit between learning and
   * testing set
   */
  void testingStartIndex(size_t& sequenceIndex, size_t& dataIndex) const;

  /**
   * Optimize LWPR and given InputSet
   * parameters to get the better fitting
   * using CMA-ES and print result on
   * standart output
   * Best fitting score is returned
   */
  double optimizeInputSet(const InputSet& set);

  /**
   * Return initial LWPR parameters for
   * given InputSet
   */
  Eigen::VectorXd initParamsLWPR(const InputSet& set) const;

  /**
   * Bound LWPR and InputSet parameters to acceptable
   * range. Penality fitness is returned
   */
  double boundParamsLWPR(const InputSet& set, Eigen::VectorXd& params) const;
  double boundParamsSet(const InputSet& set, Eigen::VectorXd& params) const;

  /**
   * Initialize with given LWPR parameters
   * and return the created empty model
   */
  LWPR_Object initModel(const InputSet& set, const Eigen::VectorXd& params) const;
};

}  // namespace Leph

#endif
