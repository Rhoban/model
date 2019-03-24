#include <stdexcept>
#include "Utils/Scheduling.hpp"

namespace Leph
{
Scheduling::Scheduling() : _frequency(0.0), _timeOld(), _timeNew(), _isLastError(false), _lastError(0.0), _timestamp()
{
  _timestamp = std::chrono::system_clock::now();
  _timeOld = std::chrono::system_clock::now();
  _timeNew = std::chrono::system_clock::now();
}
Scheduling::Scheduling(double frequency)
  : _frequency(0.0), _timeOld(), _timeNew(), _isLastError(false), _lastError(0.0), _timestamp()
{
  _timestamp = std::chrono::system_clock::now();
  _timeOld = std::chrono::system_clock::now();
  _timeNew = std::chrono::system_clock::now();

  setFrequency(frequency);
}

double Scheduling::getFrequency() const
{
  return _frequency;
}
void Scheduling::setFrequency(double frequency)
{
  if (frequency < 0.0)
  {
    throw std::logic_error("Scheduling negative frequency");
  }
  _frequency = frequency;
}

double Scheduling::duration() const
{
  return _lastTime;
}

bool Scheduling::isError() const
{
  return _isLastError;
}
double Scheduling::timeError() const
{
  return _lastError;
}

double Scheduling::timestamp() const
{
  TimePoint now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(now - _timestamp).count();
}

void Scheduling::wait()
{
  _timeNew = std::chrono::system_clock::now();
  Duration d = _timeNew - _timeOld;
  _lastTime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(d).count();
  double delay = 1000.0 / _frequency - _lastTime;
  if (delay > 0.0)
  {
    std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1, 1000>>(delay));
    _isLastError = false;
  }
  else
  {
    _isLastError = true;
    _lastError = -delay;
  }
  _timeOld = std::chrono::system_clock::now();
}

}  // namespace Leph
