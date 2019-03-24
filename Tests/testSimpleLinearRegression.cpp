#include <iostream>
#include "LinearRegression/SimpleLinearRegression.hpp"
#include "Plot/Plot.hpp"

double function(double t)
{
  return 10.0 * t - 20.0;
}

int main()
{
  Leph::Plot plot;
  Leph::SimpleLinearRegression reg;

  for (int i = 0; i < 20; i++)
  {
    double x = rand() % 100;
    double y = function(x);
    double yy = y + 4.0 * (rand() % 100 - 50);
    Leph::Vector input(2);
    input << 1.0, x;
    reg.add(input, yy, 1.0);
    plot.add(Leph::VectorLabel("x", x, "points", yy));
  }
  Leph::Vector parameters = reg.regression();
  for (double x = -50; x < 150; x += 2.0)
  {
    Leph::Vector input(2);
    input << 1.0, x;
    plot.add(Leph::VectorLabel("x", x, "found", reg.prediction(input), "boundsUpperNoise",
                               reg.prediction(input) + reg.predictionBound(input), "boundsLowerNoise",
                               reg.prediction(input) - reg.predictionBound(input), "boundsUpper",
                               reg.prediction(input) + reg.predictionBound(input, false), "boundsLower",
                               reg.prediction(input) - reg.predictionBound(input, false), "target", function(x)));
  }

  reg.print();

  plot.plot("x", "all", Leph::Plot::Points)
      .plot("x", "boundsUpperNoise", Leph::Plot::Lines)
      .plot("x", "boundsLowerNoise", Leph::Plot::Lines)
      .plot("x", "boundsUpper", Leph::Plot::Lines)
      .plot("x", "boundsLower", Leph::Plot::Lines)
      .render();

  return 0;
}
