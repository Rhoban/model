#include "Utils/Angle.h"
#include "FootStepDifferentiatorConcept.hpp"

namespace Leph
{
std::string FootStepDifferentiatorConcept::name() const
{
  return "FootStepDifferentiatorConcept";
}
size_t FootStepDifferentiatorConcept::inputSize() const
{
  return 5;
}
size_t FootStepDifferentiatorConcept::outputSize() const
{
  return 3;
}

size_t FootStepDifferentiatorConcept::parameterSize() const
{
  return 0;
}
Leph::MetaParameter FootStepDifferentiatorConcept::defaultParameter(size_t index) const
{
  (void)index;
  return MetaParameter();
}

bool FootStepDifferentiatorConcept::doCompute(double time)
{
  // Check that inputs are available
  for (size_t i = 0; i < inputSize(); i++)
  {
    if (!Concept::getInput(i)->isTimeValid(time))
    {
      return false;
    }
  }

  // Retrieve last and second last support foot swap
  size_t indexSupport1 = Concept::getInput(0)->getClosestIndex(time);
  double timeSupport1 = Concept::getInput(0)->at(indexSupport1).time;
  if (fabs(time - timeSupport1) > TIME_EPSILON)
  {
    return false;
  }
  if (indexSupport1 + 1 >= Concept::getInput(0)->size())
  {
    return false;
  }
  double timeSupport2 = Concept::getInput(0)->at(indexSupport1 + 1).time;
  // Check that inputs are available on last step
  for (size_t i = 0; i < inputSize(); i++)
  {
    if (!Concept::getInput(i)->isTimeValid(timeSupport1) || !Concept::getInput(i)->isTimeValid(timeSupport2))
    {
      return false;
    }
  }
  // Check that input differentiation is valid
  if (Concept::getInput(4)->get(timeSupport1) < 0.5 || Concept::getInput(4)->get(timeSupport2) < 0.5)
  {
    return false;
  }

  // Retrieving pose
  double x1 = Concept::getInput(1)->get(timeSupport1);
  double x2 = Concept::getInput(1)->get(timeSupport2);
  double y1 = Concept::getInput(2)->get(timeSupport1);
  double y2 = Concept::getInput(2)->get(timeSupport2);
  double theta1 = Concept::getInput(3)->getAngular(timeSupport1);
  double theta2 = Concept::getInput(3)->getAngular(timeSupport2);

  // Check that outputs are available for writing
  for (size_t i = 0; i < outputSize(); i++)
  {
    if (Concept::getOutput(i)->size() > 0 && Concept::getOutput(i)->timeMax() >= timeSupport1)
    {
      return false;
    }
  }

  // Compute the differentiation
  double deltaTheta = AngleDistance(theta2, theta1);
  double tmpX = x1 - x2;
  double tmpY = y1 - y2;
  double deltaX = cos(-theta2) * tmpX - sin(-theta2) * tmpY;
  double deltaY = sin(-theta2) * tmpX + cos(-theta2) * tmpY;

  // Write to outputs
  Concept::getOutput(0)->append(timeSupport1, deltaX);
  Concept::getOutput(1)->append(timeSupport1, deltaY);
  Concept::getOutput(2)->append(timeSupport1, deltaTheta);

  return true;
}

}  // namespace Leph
