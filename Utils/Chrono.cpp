#include <stdexcept>
#include <algorithm>
#include "Utils/Chrono.hpp"

namespace Leph
{
Chrono::Chrono() : _durations()
{
}

void Chrono::clear()
{
  _durations.clear();
}

void Chrono::start(const std::string& name)
{
  if (_durations.count(name) == 0)
  {
    _durations[name] = SequenceContainer();
  }

  _durations[name].push_back(PairTimePoint(std::chrono::system_clock::now(), std::chrono::system_clock::now()));
}

void Chrono::stop(const std::string& name)
{
  if (_durations.count(name) == 0)
  {
    throw std::logic_error("Chrono invalid stop");
  }

  _durations[name].back().second = std::chrono::system_clock::now();
}

void Chrono::print(std::ostream& os) const
{
  // Look for the first duration
  std::string firstName = "";
  TimePoint firstTime;
  for (const auto& d : _durations)
  {
    if (firstName == "" || firstTime > d.second.front().first)
    {
      firstName = d.first;
      firstTime = d.second.front().first;
    }
  }
  // Display the hierarchy
  if (firstName != "")
  {
    printHierarchy(os, firstName, 0, NULL, "");
  }
}

double Chrono::mean(const std::string& name) const
{
  if (_durations.count(name) == 0)
  {
    throw std::logic_error("Chrono unknown name: " + name);
  }

  double tmpSum = sum(name);
  return tmpSum / (double)(_durations.at(name).size());
}
double Chrono::max(const std::string& name) const
{
  if (_durations.count(name) == 0)
  {
    throw std::logic_error("Chrono unknown name: " + name);
  }

  double tmpMax = 0.0;
  for (size_t i = 0; i < _durations.at(name).size(); i++)
  {
    Duration d = _durations.at(name)[i].second - _durations.at(name)[i].first;
    double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(d).count();
    if (time > tmpMax)
    {
      tmpMax = time;
    }
  }

  return tmpMax;
}
double Chrono::sum(const std::string& name) const
{
  if (_durations.count(name) == 0)
  {
    throw std::logic_error("Chrono unknown name: " + name);
  }

  double tmpSum = 0.0;
  for (size_t i = 0; i < _durations.at(name).size(); i++)
  {
    Duration d = _durations.at(name)[i].second - _durations.at(name)[i].first;
    tmpSum += std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(d).count();
  }
  if (tmpSum < 0.0)
  {
    tmpSum = 0.0;
  }

  return tmpSum;
}

void Chrono::printDuration(std::ostream& os, const std::string& name, unsigned int level,
                           const std::string& father) const
{
  for (size_t i = 0; i < level; i++)
  {
    os << "    ";
  }
  os << "[" << name << "]";
  os << " --- mean=" << mean(name);
  os << " --- count=" << _durations.at(name).size();
  os << " --- sum=" << sum(name);
  os << " --- max=" << max(name);
  if (father != "")
  {
    os << " --- ratio=" << 100.0 * sum(name) / sum(father) << "%";
  }
  os << std::endl;
}

void Chrono::printHierarchy(std::ostream& os, const std::string& name, unsigned int level, const TimePoint* endTime,
                            const std::string& father) const
{
  printDuration(os, name, level, father);
  // Look for the first children
  std::string firstName = "";
  TimePoint firstTime;
  for (const auto& d : _durations)
  {
    if (d.first != name && d.second.front().first > _durations.at(name).front().first &&
        d.second.front().second < _durations.at(name).front().second)
    {
      if (firstName == "" || firstTime > d.second.front().first)
      {
        firstName = d.first;
        firstTime = d.second.front().first;
      }
    }
  }
  if (firstName != "")
  {
    printHierarchy(os, firstName, level + 1, &(_durations.at(name).front().second), name);
  }

  // Look for the next duration
  firstName = "";
  for (const auto& d : _durations)
  {
    if (d.first != name && d.second.front().first > _durations.at(name).front().second &&
        (endTime == NULL || d.second.front().second < *endTime))
    {
      if (firstName == "" || firstTime > d.second.front().first)
      {
        firstName = d.first;
        firstTime = d.second.front().first;
      }
    }
  }
  // Display the following hierarchy
  if (firstName != "")
  {
    printHierarchy(os, firstName, level, endTime, father);
  }
}

}  // namespace Leph
