#ifndef LEPH_SCHEDULING_HPP
#define LEPH_SCHEDULING_HPP

#include <chrono>
#include <thread>

namespace Leph
{
/**
 * Scheduling
 *
 * Simple loop scheduling
 * at fixed frequency
 */
class Scheduling
{
public:
  /**
   * Initialization with
   * initial frequency
   */
  Scheduling();
  Scheduling(double frequency);

  /**
   * Set and get Scheduling frequency
   * in Hertz
   */
  double getFrequency() const;
  void setFrequency(double frequency);

  /**
   * Return last iteration time duration
   * in milliseconds
   */
  double duration() const;

  /**
   * Return if last iteration have failed
   * to meet fixed frequency (iteration too long)
   * in milliseconds
   */
  bool isError() const;
  double timeError() const;

  /**
   * Return a timestamp since class initialization
   * in milliseconds
   */
  double timestamp() const;

  /**
   * Wait for scheduling
   */
  void wait();

private:
  /**
   * Timepoint  typedef
   */
  typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
  typedef std::chrono::duration<double> Duration;

  /**
   * Asked frequency
   */
  double _frequency;

  /**
   * Last and current loop time point
   */
  TimePoint _timeOld;
  TimePoint _timeNew;

  /**
   * Last iteration scheduling time
   */
  double _lastTime;

  /*
   * Last iteration error
   * boolean and added time
   */
  bool _isLastError;
  double _lastError;

  /**
   * Instance initialization TimePoint
   */
  TimePoint _timestamp;
};

}  // namespace Leph

#endif
