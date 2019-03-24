#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <cassert>
#include "Utils/FileVector.h"
#include "Utils/FileEigen.h"
#include "Utils/FileMap.h"

int main()
{
  std::vector<double> vect1 = { 42.5, 43.5, 44.5 };
  std::vector<double> vect2;
  Leph::WriteVector("/tmp/testFileDump_vector1.data", vect1);
  vect2 = Leph::ReadVector<double>("/tmp/testFileDump_vector1.data");
  assert(vect1.size() == vect2.size());
  for (size_t i = 0; i < vect1.size(); i++)
  {
    assert(vect1[i] == vect2[i]);
  }

  std::vector<double> vect3 = { 2.5, 3.5, 4.5 };
  std::ofstream file1("/tmp/testFileDump_vector2.data");
  assert(file1.is_open());
  Leph::WriteVectorToStream(file1, vect1);
  Leph::WriteVectorToStream(file1, vect3);
  file1.close();
  std::ifstream file2("/tmp/testFileDump_vector2.data");
  assert(file2.is_open());
  std::vector<double> vect4 = Leph::ReadVectorFromStream<double>(file2);
  std::vector<double> vect5 = Leph::ReadVectorFromStream<double>(file2);
  file2.close();
  assert(vect1.size() == vect4.size());
  for (size_t i = 0; i < vect1.size(); i++)
  {
    assert(vect1[i] == vect4[i]);
  }
  assert(vect3.size() == vect5.size());
  for (size_t i = 0; i < vect3.size(); i++)
  {
    assert(vect3[i] == vect5[i]);
  }

  std::map<std::string, double> map1 = { { "aa", 42.5 }, { "bb", 43.5 }, { "cc", 44.5 } };
  std::map<std::string, double> map2;
  Leph::WriteMap("/tmp/testFileDump_map1.data", map1);
  map2 = Leph::ReadMap<std::string, double>("/tmp/testFileDump_map1.data");
  assert(map1.size() == map2.size());
  for (const auto& it : map1)
  {
    assert(map2.count(it.first) > 0);
    assert(map2[it.first] == it.second);
  }

  std::map<std::string, double> map3 = { { "test1", 2.5 }, { "test2", 3.5 }, { "test3", 4.5 } };
  std::ofstream file3("/tmp/testFileDump_map2.data");
  assert(file3.is_open());
  Leph::WriteMapToStream(file3, map1);
  Leph::WriteMapToStream(file3, map3);
  file3.close();
  std::ifstream file4("/tmp/testFileDump_map2.data");
  assert(file4.is_open());
  std::map<std::string, double> map4 = Leph::ReadMapFromStream<std::string, double>(file4);
  std::map<std::string, double> map5 = Leph::ReadMapFromStream<std::string, double>(file4);
  file4.close();
  assert(map1.size() == map4.size());
  for (const auto& it : map1)
  {
    assert(map4.count(it.first) > 0);
    assert(map4[it.first] == it.second);
  }
  assert(map3.size() == map5.size());
  for (const auto& it : map3)
  {
    assert(map5.count(it.first) > 0);
    assert(map5[it.first] == it.second);
  }

  Eigen::MatrixXd mat1(2, 3);
  mat1 << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;
  Eigen::MatrixXd mat2;
  Leph::WriteEigenMatrix("/tmp/testFileDump_eigen1.data", mat1);
  mat2 = Leph::ReadEigenMatrix("/tmp/testFileDump_eigen1.data");
  assert(mat1.rows() == mat2.rows());
  assert(mat1.cols() == mat2.cols());
  for (int i = 0; i < mat1.rows(); i++)
  {
    for (int j = 0; j < mat1.cols(); j++)
    {
      assert(mat1(i, j) == mat2(i, j));
    }
  }

  Eigen::MatrixXd mat3(2, 2);
  mat3 << 10.1, 20.2, 40.4, 50.5;
  std::ofstream file5("/tmp/testFileDump_eigen2.data");
  assert(file5.is_open());
  Leph::WriteEigenMatrixToStream(file5, mat1);
  Leph::WriteEigenMatrixToStream(file5, mat3);
  file5.close();
  std::ifstream file6("/tmp/testFileDump_eigen2.data");
  assert(file6.is_open());
  Eigen::MatrixXd mat4 = Leph::ReadEigenMatrixFromStream(file6);
  Eigen::MatrixXd mat5 = Leph::ReadEigenMatrixFromStream(file6);
  file6.close();
  assert(mat1.rows() == mat4.rows());
  assert(mat1.cols() == mat4.cols());
  for (int i = 0; i < mat1.rows(); i++)
  {
    for (int j = 0; j < mat1.cols(); j++)
    {
      assert(mat1(i, j) == mat4(i, j));
    }
  }
  assert(mat3.rows() == mat5.rows());
  assert(mat3.cols() == mat5.cols());
  for (int i = 0; i < mat3.rows(); i++)
  {
    for (int j = 0; j < mat3.cols(); j++)
    {
      assert(mat3(i, j) == mat5(i, j));
    }
  }

  return 0;
}
