#ifndef LEPH_FILEMAP_H
#define LEPH_FILEMAP_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>

namespace Leph
{
/**
 * Write the given map into
 * given output stream
 */
template <typename T, typename U>
void WriteMapToStream(std::ostream& os, const std::map<T, U>& map)
{
  if (map.size() == 0)
  {
    throw std::logic_error("WriteMapToStream empty map");
  }
  for (const auto& it : map)
  {
    os << it.first << " " << std::setprecision(17) << it.second << " ";
  }
  os << std::endl;
}

/**
 * Write the given map into
 * given filename
 */
template <typename T, typename U>
void WriteMap(const std::string& filename, const std::map<T, U>& map)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("WriteMap unable to open file: " + filename);
  }
  WriteMapToStream(file, map);
  file.close();
}

/**
 * Read from given input stream
 * a map and return it
 */
template <typename T, typename U>
std::map<T, U> ReadMapFromStream(std::istream& is)
{
  std::map<T, U> map;
  while (true)
  {
    while (is.peek() == ' ')
    {
      is.ignore();
    }
    if (is.peek() == '\n' || is.peek() == EOF || is.bad())
    {
      is.ignore();
      return map;
    }
    T key;
    U val;
    is >> key;
    is >> val;
    map[key] = val;
  }
}

/**
 * Read from given filename a map
 * and return it
 */
template <typename T, typename U>
std::map<T, U> ReadMap(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("ReadMap unable to open file: " + filename);
  }
  std::map<T, U> map = ReadMapFromStream<T, U>(file);
  file.close();
  return map;
}

}  // namespace Leph

#endif
