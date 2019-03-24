#include <fstream>
#include <iomanip>
#include <stdexcept>
#include "Utils/FileEigen.h"

namespace Leph
{
void WriteEigenVectorToStream(std::ostream& os, const Eigen::VectorXd& vect)
{
  os << vect.size();
  for (size_t i = 0; i < (size_t)vect.size(); i++)
  {
    os << " " << std::setprecision(17) << vect(i);
  }
  os << std::endl;
}

void WriteEigenVector(const std::string& filename, const Eigen::VectorXd& vect)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("WriteEigenVector unable to open file: " + filename);
  }
  WriteEigenVectorToStream(file, vect);
  file.close();
}

Eigen::VectorXd ReadEigenVectorFromStream(std::istream& is)
{
  size_t size;
  is >> size;
  Eigen::VectorXd vect(size);
  for (size_t i = 0; i < size; i++)
  {
    double value;
    is >> value;
    vect(i) = value;
  }
  while (is.peek() == ' ' || is.peek() == '\n')
  {
    is.ignore();
  }

  return vect;
}

Eigen::VectorXd ReadEigenVector(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("ReadEigenVector unable to open file: " + filename);
  }
  Eigen::VectorXd vect = ReadEigenVectorFromStream(file);
  file.close();
  return vect;
}

void WriteEigenMatrixToStream(std::ostream& os, const Eigen::MatrixXd& mat)
{
  os << mat.rows() << " " << mat.cols();
  for (size_t i = 0; i < (size_t)mat.rows(); i++)
  {
    for (size_t j = 0; j < (size_t)mat.cols(); j++)
    {
      os << " " << std::setprecision(17) << mat(i, j);
    }
  }
  os << std::endl;
}

void WriteEigenMatrix(const std::string& filename, const Eigen::MatrixXd& mat)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("WriteEigenMatrix unable to open file: " + filename);
  }
  WriteEigenMatrixToStream(file, mat);
  file.close();
}

Eigen::MatrixXd ReadEigenMatrixFromStream(std::istream& is)
{
  size_t sizeRows;
  size_t sizeCols;
  is >> sizeRows;
  is >> sizeCols;
  Eigen::MatrixXd mat(sizeRows, sizeCols);
  for (size_t i = 0; i < sizeRows; i++)
  {
    for (size_t j = 0; j < sizeCols; j++)
    {
      double value;
      is >> value;
      mat(i, j) = value;
    }
  }
  while (is.peek() == ' ' || is.peek() == '\n')
  {
    is.ignore();
  }

  return mat;
}

Eigen::MatrixXd ReadEigenMatrix(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("ReadEigenMatrix unable to open file: " + filename);
  }
  Eigen::MatrixXd mat = ReadEigenMatrixFromStream(file);
  file.close();
  return mat;
}

}  // namespace Leph
