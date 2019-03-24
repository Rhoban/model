#ifndef LEPH_TIME_HPP
#define LEPH_TIME_HPP

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <ctime>

namespace Leph
{
/**
 * Return the current date formated string
 */
inline std::string currentDate()
{
  std::ostringstream oss;
  time_t t = time(0);
  struct tm* now = localtime(&t);
  oss << now->tm_year + 1900 << "-";
  oss << std::setfill('0') << std::setw(2);
  oss << now->tm_mon + 1 << "-";
  oss << std::setfill('0') << std::setw(2);
  oss << now->tm_mday << "-";
  oss << std::setfill('0') << std::setw(2);
  oss << now->tm_hour << "-";
  oss << std::setfill('0') << std::setw(2);
  oss << now->tm_min << "-";
  oss << std::setfill('0') << std::setw(2);
  oss << now->tm_sec;

  return oss.str();
}

}  // namespace Leph

#endif
