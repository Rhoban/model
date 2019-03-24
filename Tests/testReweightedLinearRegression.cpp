#include <iostream>
#include <vector>
#include <random>
#include "Types/types.h"
#include "Types/VectorLabel.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"
#include "Plot/Plot.hpp"

const double noiseLevel = 40.0;
const double deltaLen = 1.0;
const int reweightIterations = 2;
const int sampleCount = 20;
const double kernelSize = 20.0;

std::mt19937 generator;
std::uniform_real_distribution<double> uniformFuncDist(-noiseLevel, noiseLevel);
std::normal_distribution<double> normalFuncDist(0.0, noiseLevel / 2.0);
std::uniform_real_distribution<double> uniformDeltaDist(-deltaLen, deltaLen);
std::normal_distribution<double> normalDeltaDist(0.0, deltaLen / 2.0);

double function(double x, double y)
{
  return x * x + y * y;
}
double functionDiff(double x, double y, double ptx, double pty)
{
  return 2.0 * ptx * x + 2.0 * pty * y - (ptx * ptx + pty * pty);
}
double noisedFunction(double x, double y)
{
  return function(x, y) + normalFuncDist(generator);
  // return function(x, y) + uniformFuncDist(generator);
}

int main()
{
  Leph::Plot plot;

  for (double x = -10.0; x <= 10.0; x += 0.5)
  {
    for (double y = -10.0; y <= 10.0; y += 0.5)
    {
      plot.add(Leph::VectorLabel("x", x, "y", y, "noisedTarget", noisedFunction(x, y), "target", function(x, y)));
    }
  }
  plot.plot("x", "y", "noisedTarget").render();

  Leph::Vector point(2);
  point << 5.0, 4.0;

  std::vector<Leph::Vector> inputs;
  std::vector<double> outputs;
  for (int k = 0; k < sampleCount; k++)
  {
    Leph::Vector delta(2);
    /*
    delta <<
        uniformDeltaDist(generator),
        uniformDeltaDist(generator);
    */
    delta << normalDeltaDist(generator), normalDeltaDist(generator);
    Leph::Vector samplePt = point + delta;
    double sampleVal = noisedFunction(samplePt(0), samplePt(1));
    plot.add(Leph::VectorLabel("x", samplePt(0), "y", samplePt(1), "sample", sampleVal));
    samplePt.conservativeResize(3, Eigen::NoChange_t());
    samplePt(2) = 1.0;
    inputs.push_back(samplePt);
    outputs.push_back(sampleVal);
  }

  Leph::SimpleLinearRegression oldRegression;
  Leph::SimpleLinearRegression regression;
  for (size_t k = 0; k < reweightIterations; k++)
  {
    regression.clear();
    for (size_t i = 0; i < inputs.size(); i++)
    {
      double weight = 1.0;
      if (k != 0)
      {
        double dist = fabs(outputs[i] - oldRegression.prediction(inputs[i]));
        weight = exp(-dist / kernelSize);
        std::cout << "Dist=" << dist << " weight=" << weight << std::endl;
      }
      regression.add(inputs[i], outputs[i], weight);
    }
    regression.regression();
    regression.print();
    Leph::Vector answer(3);
    answer << 2.0 * point(0), 2.0 * point(1), -function(point(0), point(1));
    std::cout << "ERROR=" << (regression.parameters() - answer).norm() << std::endl;
    oldRegression = regression;
  }

  for (double x = -10.0; x <= 10.0; x += 1.0)
  {
    for (double y = -10.0; y <= 10.0; y += 1.0)
    {
      Leph::Vector input(3);
      input << x, y, 1.0;
      plot.add(Leph::VectorLabel("x", x, "y", y, "real", functionDiff(x, y, point(0), point(1)), "fitted",
                                 regression.prediction(input)));
    }
  }
  plot.plot("x", "y", "target").plot("x", "y", "fitted").plot("x", "y", "sample").plot("x", "y", "real").render();

  return 0;
}
