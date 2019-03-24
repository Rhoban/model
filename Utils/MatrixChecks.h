#ifndef LEPH_MATRIX_CHECKS_H
#define LEPH_MATRIX_CHECKS_H

#include <cmath>

namespace Leph
{
/**
 * Return true if given matrix is
 * square and symmetric
 */
inline bool IsMatrixSymmetric(const Eigen::MatrixXd& mat, double epsilon = 1e-9)
{
  // Check matrix square
  if (mat.rows() == 0 || mat.cols() != mat.rows())
  {
    return false;
  }
  // Check matrix symmetric
  size_t size = mat.rows();
  for (size_t i = 0; i < size; i++)
  {
    for (size_t j = i + 1; j < size; j++)
    {
      if (fabs(mat(i, j) - mat(j, i)) > epsilon)
      {
        return false;
      }
    }
  }

  return true;
}

/**
 * Return true if given matrix is
 * square, symmetric and definite positive.
 */
inline bool IsMatrixSymmetricDefinitePositive(const Eigen::MatrixXd& mat)
{
  // Check matrix square
  if (mat.rows() == 0 || mat.cols() != mat.rows())
  {
    return false;
  }
  // Compute the Cholesky decomposition
  Eigen::LLT<Eigen::MatrixXd> llt(mat);
  if (llt.info() == Eigen::NumericalIssue)
  {
    return false;
  }
  else
  {
    return true;
  }
}

}  // namespace Leph

#endif
