#include "CartWalk/CartWalkGradient.hpp"

namespace Leph
{
Matrix CartWalkGradient::differentiation(double phase, const VectorLabel& params, double diffStep)
{
  size_t rows = _walk.buildOutputs().size();
  size_t cols = _walk.buildParams().size();
  Matrix jacobian(rows, cols);

  VectorLabel tmpParams;
  for (size_t i = 0; i < cols; i++)
  {
    tmpParams = params;
    tmpParams(i) += diffStep;
    _walk.setPhase(phase);
    VectorLabel tmpOutput1 = _walk.exec(0.0, tmpParams);

    tmpParams = params;
    tmpParams(i) -= diffStep;
    _walk.setPhase(phase);
    VectorLabel tmpOutput2 = _walk.exec(0.0, tmpParams);

    for (size_t j = 0; j < rows; j++)
    {
      jacobian(j, i) = (tmpOutput1(j) - tmpOutput2(j)) / (2.0 * diffStep);
    }
  }

  return jacobian;
}

}  // namespace Leph
