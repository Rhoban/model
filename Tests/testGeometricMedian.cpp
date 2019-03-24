#include <iostream>
#include <cstdlib>
#include "Utils/GeometricMedian.hpp"
#include "Types/types.h"
#include "Plot/Plot.hpp"

int main()
{
  Leph::Plot plot;
  Leph::GeometricMedian median;

  for (int k = 0; k < 6; k++)
  {
    Leph::Vector pt(2);
    pt << rand() % 100, rand() % 100;
    median.add(pt);
    plot.add(Leph::VectorLabel("x", pt(0), "y", pt(1)));
  }

  for (int k = 0; k < 10; k++)
  {
    Leph::Vector med = median.median(-1, k);
    plot.add(Leph::VectorLabel("x", med(0), "median", med(1)));
  }

  plot.rangeX(0, 100)
      .rangeY(0, 100)
      .plot("x", "median", Leph::Plot::LinesPoints, "index")
      .plot("x", "y", Leph::Plot::Points)
      .render();

  return 0;
}
