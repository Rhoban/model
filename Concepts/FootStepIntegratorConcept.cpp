#include <cmath>
#include "Concepts/FootStepIntegratorConcept.hpp"

namespace Leph
{
std::string FootStepIntegratorConcept::name() const
{
  return "FootStepIntegratorConcept";
}
size_t FootStepIntegratorConcept::inputSize() const
{
  return 4;
}
size_t FootStepIntegratorConcept::outputSize() const
{
  return 3;
}

size_t FootStepIntegratorConcept::parameterSize() const
{
  return 0;
}
Leph::MetaParameter FootStepIntegratorConcept::defaultParameter(size_t index) const
{
  (void)index;
  return MetaParameter();
}

bool FootStepIntegratorConcept::doCompute(double time)
{
  // Check that inputs are available
  for (size_t i = 0; i < inputSize(); i++)
  {
    if (!Concept::getInput(i)->isTimeValid(time))
    {
      return false;
    }
  }

  // Retrieve last outputs value
  double stateX = 0.0;
  double stateY = 0.0;
  double stateTheta = 0.0;
  if (Concept::getOutput(0)->size() != 0)
  {
    stateX = Concept::getOutput(0)->lastValue();
  }
  if (Concept::getOutput(1)->size() != 0)
  {
    stateY = Concept::getOutput(1)->lastValue();
  }
  if (Concept::getOutput(2)->size() != 0)
  {
    stateTheta = Concept::getOutput(2)->lastValue();
  }

  // Find the last support foot
  size_t index = Concept::getInput(0)->getClosestIndex(time);
  double t = Concept::getInput(0)->at(index).time;

  // Retrieve delta
  if (fabs(t - time) > TIME_EPSILON)
  {
    return false;
  }
  double deltaX = Concept::getInput(1)->get(time);
  double deltaY = Concept::getInput(2)->get(time);
  double deltaTheta = Concept::getInput(3)->getAngular(time);

  // Check that outputs are available
  for (size_t i = 0; i < outputSize(); i++)
  {
    if (Concept::getOutput(i)->size() > 0 && Concept::getOutput(i)->timeMax() > t)
    {
      return false;
    }
  }

  // Compute integration
  double tmpX = deltaX * cos(stateTheta) - deltaY * sin(stateTheta);
  double tmpY = deltaX * sin(stateTheta) + deltaY * cos(stateTheta);
  stateX += tmpX;
  stateY += tmpY;
  stateTheta += deltaTheta;

  // Write new integrated state
  Concept::getOutput(0)->append(t, stateX);
  Concept::getOutput(1)->append(t, stateY);
  Concept::getOutput(2)->append(t, stateTheta);

  return true;
}

}  // namespace Leph
