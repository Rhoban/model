#include <cmath>
#include "Concepts/FallDetectorConcept.hpp"

namespace Leph
{
std::string FallDetectorConcept::name() const
{
  return "FallDetectorConcept";
}

size_t FallDetectorConcept::inputSize() const
{
  return 2;
}
size_t FallDetectorConcept::outputSize() const
{
  return 1;
}

size_t FallDetectorConcept::parameterSize() const
{
  return 0;
}
Leph::MetaParameter FallDetectorConcept::defaultParameter(size_t index) const
{
  (void)index;
  return MetaParameter();
}

bool FallDetectorConcept::doCompute(double time)
{
  if (!Concept::getInput(0)->isTimeValid(time) || !Concept::getInput(1)->isTimeValid(time))
  {
    return false;
  }
  if (Concept::getOutput(0)->size() > 0 && Concept::getOutput(0)->timeMax() >= time)
  {
    return false;
  }

  bool is_fallen =
      (fabs(Concept::getInput(0)->get(time)) > M_PI / 4) || (fabs(Concept::getInput(1)->get(time)) > M_PI / 4);
  Concept::getOutput(0)->append(time, is_fallen ? 1.0 : 0.0);

  return true;
}

}  // namespace Leph
