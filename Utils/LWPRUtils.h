#ifndef LEPH_LWPRUTILS_HPP
#define LEPH_LWPRUTILS_HPP

#include <vector>
#include <Eigen/Dense>
#include <lwpr_eigen.hpp>

namespace Leph
{
/**
 * Return default LWPR meta parameters for given
 * input dimmension
 */
Eigen::VectorXd LWPRInitParameters(size_t inputDim);

/**
 * Create empty and initialize an LWPR model with given
 * input dimention and meta parameters
 */
LWPR_Object LWPRInit(size_t inputDim, const Eigen::VectorXd& params);

/**
 * Optimize given LWPR meta parameters with given
 * train and test inputs/outputs data. The maximum
 * number of CMAES iteration is given. Best found
 * meta parameters is returnned.
 */
Eigen::VectorXd LWPROptimizeParameters(size_t inputDim, const Eigen::VectorXd& params,
                                       const std::vector<Eigen::VectorXd> trainInputs,
                                       const std::vector<double> trainOutputs,
                                       const std::vector<Eigen::VectorXd> testInputs,
                                       const std::vector<double> testOutputs, unsigned int maxIteration,
                                       bool isQuiet = false);

/**
 * Print on std::cout given LWMR model
 * information summary.
 */
void LWPRPrint(const LWPR_Object& model);

/**
 * Return the number of trustworthy receptive field
 * min and max of RF regressions
 */
size_t LWPRRFTrustworthy(const LWPR_Object& model);
size_t LWPRMinRFReg(const LWPR_Object& model);
size_t LWPRMaxRFReg(const LWPR_Object& model);

/**
 * Print on std::cout given LWPR
 * meta parameters
 */
void LWPRPrintParameters(size_t inputDim, const Eigen::VectorXd& params);

}  // namespace Leph

#endif
