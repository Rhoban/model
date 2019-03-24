#include <iostream>
#include <cstdlib>
#include "Types/types.h"
#include "Types/VectorLabel.hpp"
#include "Gradient/FiniteDifferenceGradient.hpp"
#include "Plot/Plot.hpp"

double randFloat(double length)
{
  return length * (double)((rand() % 2000) - 1000) / 1000.0;
}

double function(double x, double y)
{
  return x * x + y * y;
}
double noisedFunction(double x, double y, double length)
{
  return function(x, y) + randFloat(length);
}
double diffFunction(double x, double y)
{
  return 2.0 * x + 2.0 * y;
}

int main()
{
  double noiseLevel = 50.0;
  double deltaLen = 3.0;
  double learningRate = 0.2;

  Leph::Plot plot;
  for (double x = -10.0; x <= 10.0; x += 0.5)
  {
    for (double y = -10.0; y <= 10.0; y += 0.5)
    {
      plot.add(Leph::VectorLabel("x", x, "y", y, "fitness", function(x, y)));
    }
  }

  Leph::Vector point(2);
  point(0) = 5.0;
  point(1) = 4.0;

  Leph::Vector delta(2);
  double fitness;

  plot.add(Leph::VectorLabel("x", point(0), "y", point(1), "state", function(point(0), point(1))));
  for (int l = 0; l < 5; l++)
  {
    Leph::FiniteDifferenceGradient gradient;
    double fitnessMean = 0.0;
    int count = 0;
    for (int k = 0; k < 15; k++)
    {
      while (true)
      {
        delta(0) = randFloat(deltaLen);
        delta(1) = randFloat(deltaLen);
        double len = delta.norm();
        if (len <= deltaLen)
          break;
      }
      std::cout << "    Delta: " << delta.transpose() << std::endl;
      double x = point(0) + delta(0);
      double y = point(1) + delta(1);
      fitness = noisedFunction(x, y, noiseLevel);
      fitnessMean += fitness;
      count++;
      std::cout << "    Sample: " << (point + delta).transpose() << " --> " << fitness << std::endl;
      plot.add(Leph::VectorLabel("x", x, "y", y, "loop", l, "sample", fitness));
      gradient.addExperiment(delta, fitness);
    }
    point -= learningRate * gradient.gradient();
    plot.add(Leph::VectorLabel("x", point(0), "y", point(1), "state", function(point(0), point(1))));
    std::cout << "State: " << point.transpose() << std::endl;
    std::cout << "Mean: " << fitnessMean / count << std::endl;
  }
  plot.plot("x", "y", "fitness")
      .plot("x", "y", "state", Leph::Plot::LinesPoints)
      .plot("x", "y", "sample", Leph::Plot::Points, "loop")
      .render();

  return 0;
}
