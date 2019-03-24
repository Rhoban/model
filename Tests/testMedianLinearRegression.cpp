#include <iostream>
#include "LinearRegression/MedianLinearRegression.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"
#include "Plot/Plot.hpp"

double function(double t)
{
  return 10.0 * t - 20.0;
}

int main()
{
  Leph::Plot plot;
  Leph::SimpleLinearRegression regLinear;
  Leph::MedianLinearRegression regMedian;

  for (int i = 0; i < 20; i++)
  {
    double x = rand() % 100;
    double y = function(x);
    double yy = y + 0.5 * (rand() % 100 - 50);
    Leph::Vector input(2);
    input << 1.0, x;
    regLinear.add(input, yy);
    regMedian.add(input, yy);
    plot.add(Leph::VectorLabel("x", x, "points", yy));
  }
  /*
  for (int i=0;i<5;i++) {
      double x = 140;
      double y = rand()%150;
      Leph::Vector input(2);
      input << 1.0, x;
      regLinear.add(input, y);
      regMedian.add(input, y);
      plot.add(Leph::VectorLabel(
          "x", x,
          "points", y));
  }
  */
  Leph::Vector paramsLinear = regLinear.regression();
  Leph::Vector paramsMedian = regMedian.regression();

  for (double x = -50; x < 150; x += 2.0)
  {
    Leph::Vector input(2);
    input << 1.0, x;
    plot.add(Leph::VectorLabel("x", x, "linear", paramsLinear.dot(input), "median", paramsMedian.dot(input), "target",
                               function(x)));
  }

  plot.plot("x", "points", Leph::Plot::Points)
      //.plot("x", "all", Leph::Plot::Points)
      .render();

  return 0;
}
