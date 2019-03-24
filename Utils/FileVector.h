#ifndef LEPH_FILEVECTOR_H
#define LEPH_FILEVECTOR_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

namespace Leph
{
/**
 * Write the given vector into
 * given output stream
 */
template <typename T>
void WriteVectorToStream(std::ostream& os, const std::vector<T>& vect)
{
  if (vect.size() == 0)
  {
    throw std::logic_error("WriteVectorToStream empty vector");
  }
  for (size_t i = 0; i < vect.size(); i++)
  {
    os << std::setprecision(17) << vect[i] << " ";
  }
  os << std::endl;
}

/**
 * Write the given vector into
 * given filename
 */
template <typename T>
void WriteVector(const std::string& filename, const std::vector<T>& vect)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("WriteVector unable to open file: " + filename);
  }
  WriteVectorToStream(file, vect);
  file.close();
}

/**
 * Read from given input stream
 * a vector and return it
 */
template <typename T>
std::vector<T> ReadVectorFromStream(std::istream& is)
{
  std::vector<T> vect;
  while (true)
  {
    while (is.peek() == ' ')
    {
      is.ignore();
    }
    if (is.peek() == '\n' || is.peek() == EOF || is.bad())
    {
      is.ignore();
      return vect;
    }
    T val;
    is >> val;
    vect.push_back(val);
  }
}

/**
 * Read from given filename a
 * vector and return it
 */
template <typename T>
std::vector<T> ReadVector(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("ReadVector unable to open file: " + filename);
  }
  std::vector<T> vect = ReadVectorFromStream<T>(file);
  file.close();
  return vect;
}

}  // namespace Leph

#endif
