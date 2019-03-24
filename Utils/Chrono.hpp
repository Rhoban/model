#ifndef LEPH_CHRONO_HPP
#define LEPH_CHRONO_HPP

#include <iostream>
#include <chrono>
#include <string>
#include <map>
#include <vector>

namespace Leph
{
/**
 * Chrono
 *
 * Simple C++11 tree chrono wrapper for
 * test and benchmarking
 */
class Chrono
{
public:
  /**
   * Initialization empty
   */
  Chrono();

  /**
   * Reset chrono instance to empty
   */
  void clear();

  /**
   * Create and start a named duration
   */
  void start(const std::string& name);

  /**
   * Stop and save a named duration
   */
  void stop(const std::string& name);

  /**
   * Write on given stream an overview of chrono state
   */
  void print(std::ostream& os = std::cout) const;

  /**
   * Compute the mean, max and sum
   * duration in milliseconds of given duration
   * name
   */
  double mean(const std::string& name) const;
  double max(const std::string& name) const;
  double sum(const std::string& name) const;

private:
  /**
   * Typedef shortcuts
   */
  typedef std::chrono::duration<double> Duration;
  typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
  typedef std::pair<TimePoint, TimePoint> PairTimePoint;
  typedef std::vector<PairTimePoint> SequenceContainer;
  typedef std::map<std::string, SequenceContainer> DurationContainer;

  /**
   * Sequences of starting and stopping time point
   * indexed by their name
   */
  DurationContainer _durations;

  /**
   * Print the given name on given stream
   * with given indent level
   */
  void printDuration(std::ostream& os, const std::string& name, unsigned int level, const std::string& father) const;

  /**
   * Print the hierarchy with given level using given
   * name as root
   */
  void printHierarchy(std::ostream& os, const std::string& name, unsigned int level, const TimePoint* endTime,
                      const std::string& father) const;
};

}  // namespace Leph

#endif
