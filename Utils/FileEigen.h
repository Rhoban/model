#ifndef LEPH_FILEEIGEN_H
#define LEPH_FILEEIGEN_H

#include <iostream>
#include <Eigen/Dense>

namespace Leph
{
/**
 * Write the given Eigen Vector
 * into given output stream
 */
void WriteEigenVectorToStream(std::ostream& os, const Eigen::VectorXd& vect);

/**
 * Write the given Eigen Vector
 * into given filename
 */
void WriteEigenVector(const std::string& filename, const Eigen::VectorXd& vect);

/**
 * Read from given input stream
 * an Eigen Vector and return it
 */
Eigen::VectorXd ReadEigenVectorFromStream(std::istream& is);

/**
 * Read from given filename an
 * Eigen Vector and return it
 */
Eigen::VectorXd ReadEigenVector(const std::string& filename);

/**
 * Write the given Eigen Matrix
 * into given output stream
 */
void WriteEigenMatrixToStream(std::ostream& os, const Eigen::MatrixXd& mat);

/**
 * Write the given Eigen Matrix
 * into given filename
 */
void WriteEigenMatrix(const std::string& filename, const Eigen::MatrixXd& mat);

/**
 * Read from given input stream
 * an Eigen Matrix and return it
 */
Eigen::MatrixXd ReadEigenMatrixFromStream(std::istream& is);

/**
 * Read from given filename an
 * Eigen Matrix and return it
 */
Eigen::MatrixXd ReadEigenMatrix(const std::string& filename);

}  // namespace Leph

#endif
