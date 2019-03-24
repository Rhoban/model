#include <iostream>
#include "Utils/IterativeLearningControl.hpp"
#include "Plot/Plot.hpp"
#include "Types/VectorLabel.hpp"

double goalFunc(double phase, double offset = 0.0)
{
  return sin(2.0 * 3.14 * phase) + sin(3.0 * 3.14 * phase) + offset;
}
double motorFunc(double phase, double offset = 0.0)
{
  return 0.6 * goalFunc(phase - 0.02 * 6, offset) + 0.4;
}

int main()
{
  Leph::IterativeLearningControl ilc(6, 0.1);

  for (size_t k = 0; k <= 50; k++)
  {
    Leph::Plot plot;
    std::cout << "Iteration " << k << std::endl;
    for (double t = 0; t <= 1.0; t += 0.02)
    {
      double offset = 0.0;
      if (k > 0)
      {
        offset = ilc.getOffsets(t)(0);
      }
      // Generate servo goal and read position
      double goal = goalFunc(t);
      double pos = motorFunc(t, offset);
      // Convertion to VectorLabel
      Leph::VectorLabel goalVect("servo", goal);
      Leph::VectorLabel posVect("servo", pos);
      // Learn iterativement the compensation
      ilc.update(t, goalVect, posVect);
      // Plot
      plot.add(Leph::VectorLabel("t", t, "goal", goal, "offset", offset, "error",
                                 ilc.getLastErrors().size() > 0 ? ilc.getLastErrors()(0) : 0.0, "motor", pos));
    }
    std::cout << "Error: " << ilc.getSumErrors()(0) << std::endl;
    if (k % 5 == 0)
    {
      plot.plot("t", "all").render();
    }
  }

  return 0;
}
