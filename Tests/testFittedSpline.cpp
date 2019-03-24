#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include "Spline/FittedSpline.hpp"
#include "Types/MatrixLabel.hpp"
#include "Plot/Plot.hpp"

std::default_random_engine generator;
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

double noise(double gain)
{
  return gain * uniform(generator);
}

double function(double t)
{
  return sin(5.0 * t * sin(8.0 * t * t)) - exp(-t);
}

void testGeneratedData()
{
  Leph::FittedSpline spline;
  Leph::Plot plot;

  // Generates noised data
  for (double t = 0; t < 1.0; t += 0.02)
  {
    double f = function(t);
    spline.addPoint(t, f);
    plot.add(Leph::VectorLabel("t", t, "target", f));
  }

  // Do fitting
  spline.fittingPieces(0.01, false);

  // Display fitted splines
  for (double t = 0; t < 1.0; t += 0.01)
  {
    plot.add(Leph::VectorLabel("t", t, "fitted", spline.pos(t)));
  }
  plot.plot("t", "all").render();

  // Tests data dump and load
  std::ofstream fileOut("/tmp/testSpline.csv");
  spline.exportData(fileOut);
  fileOut.close();

  Leph::Spline splineImport;
  std::ifstream fileIn("/tmp/testSpline.csv");
  splineImport.importData(fileIn);
  fileIn.close();
  splineImport.exportData(std::cout);
}

void testGeneratedDataNoise()
{
  Leph::FittedSpline spline;
  Leph::Plot plot;

  // Generates noised data
  for (double t = 0; t < 1.0; t += 0.02)
  {
    double tt = t;
    if (noise(1.0) > 0.5)
    {
      tt -= 0.02;
    }
    double f = function(tt);
    double ff = f + noise(0.15);
    spline.addPoint(tt, ff);
    plot.add(Leph::VectorLabel("t", tt, "target", ff));
  }

  // Do fitting
  spline.fittingGlobal(4, 6);

  // Display fitted splines
  for (double t = 0; t < 1.0; t += 0.01)
  {
    plot.add(Leph::VectorLabel("t", t, "fitted", spline.pos(t), "real", function(t)));
  }
  plot.plot("t", "all").render();
}

void testCapturedData()
{
  Leph::FittedSpline spline;
  Leph::Plot plot;

  // Load logged data
  Leph::MatrixLabel logs;
  logs.load("../../These/Data/logs-2015-05-16/model_2015-05-16-18-47-31.log");
  // Print data informations
  std::cout << "Loaded " << logs.size() << " points with " << logs.dimension() << " entries" << std::endl;
  // Add target points
  double tMin = logs[0]("time:timestamp");
  double tMax = logs[logs.size() - 1]("time:timestamp");
  for (size_t i = 0; i < logs.size(); i++)
  {
    plot.add(Leph::VectorLabel("t", logs[i]("time:timestamp"), "target", logs[i]("motor:left_knee")));
    spline.addPoint(logs[i]("time:timestamp"), logs[i]("motor:left_knee"));
  }

  // Do fitting
  spline.fittingGlobal(1, 4);

  // Display fitted splines
  for (double t = tMin; t <= tMax; t += (tMax - tMin) / 3000.0)
  {
    plot.add(Leph::VectorLabel("t", t, "fitted", spline.pos(t), "fitted acc", 1000.0 * spline.acc(t)));
  }
  plot.plot("t", "all").render();
}

void testCapturedDataWindow()
{
  Leph::Plot plot;

  // Load logged data
  Leph::MatrixLabel logs;
  logs.load("../../These/Data/logs-2015-05-16/model_2015-05-16-18-47-31.log");
  // Print data informations
  std::cout << "Loaded " << logs.size() << " points with " << logs.dimension() << " entries" << std::endl;
  // Add target points
  double tMin = logs[0]("time:timestamp");
  double tMax = logs[logs.size() - 1]("time:timestamp");
  for (size_t i = 0; i < logs.size(); i++)
  {
    plot.add(Leph::VectorLabel("t", logs[i]("time:timestamp"), "target", logs[i]("motor:left_knee")));
  }

  // Display fitted splines
  for (double t = tMin; t <= tMax; t += (tMax - tMin) / 3000.0)
  {
    size_t indexBegin = 0;
    size_t indexEnd = 0;
    while (indexBegin < logs.size() && logs[indexBegin]("time:timestamp") < t - 300.0)
      indexBegin++;
    while (indexEnd < logs.size() && logs[indexEnd]("time:timestamp") < t + 300.0)
      indexEnd++;
    if (indexBegin >= logs.size())
      indexBegin = logs.size() - 1;
    if (indexEnd >= logs.size())
      indexEnd = logs.size() - 1;
    Leph::FittedSpline spline;
    std::cout << "t=" << t << " nb=" << indexEnd - indexBegin << " " << indexBegin << " " << indexEnd << " --- "
              << logs[indexBegin]("time:timestamp") << " " << logs[indexEnd]("time:timestamp") << std::endl;
    for (size_t i = indexBegin; i <= indexEnd; i++)
    {
      spline.addPoint(logs[i]("time:timestamp"), logs[i]("motor:left_knee"));
    }
    // Do fitting
    spline.fittingGlobal(3, 5);
    // Compute spline value
    plot.add(Leph::VectorLabel("t", t, "fitted", spline.pos(t), "fitted acc", 1000.0 * spline.acc(t)));
  }
  plot.plot("t", "all").render();
}

void testFittingCubic()
{
  Leph::FittedSpline spline;
  Leph::Plot plot;

  // Generates noiseless data
  for (double t = 0; t < 1.0; t += 0.01)
  {
    double f = function(t);
    spline.addPoint(t, f);
    plot.add(Leph::VectorLabel("t", t, "target", f));
  }

  // Do fitting
  spline.fittingCubic(8);

  // Display fitted splines
  for (double t = 0; t < 1.0; t += 0.005)
  {
    plot.add(Leph::VectorLabel("t", t, "fitted", spline.pos(t)));
  }
  plot.plot("t", "all").render();
}

void testFittingSmooth()
{
  Leph::FittedSpline spline;
  Leph::Plot plot;

  // Generates noiseless data
  for (double t = 0; t < 1.0; t += 0.01)
  {
    double f = function(t);
    spline.addPoint(t, f);
    plot.add(Leph::VectorLabel("t", t, "target", f));
  }

  // Do fitting
  spline.fittingSmooth(6);

  // Display fitted splines
  for (double t = 0; t < 1.0; t += 0.005)
  {
    plot.add(Leph::VectorLabel("t", t, "fitted", spline.pos(t)));
  }
  plot.plot("t", "all").render();
}

void testFittingCMAES()
{
  Leph::FittedSpline spline;
  Leph::Plot plot;

  // Generates noisy data
  for (int k = 0; k < 5; k++)
  {
    for (double t = 0; t < 1.0; t += 0.01)
    {
      double f = function(t);
      double p = f + noise(0.3);
      spline.addPoint(t, p);
      plot.add(Leph::VectorLabel("t", t, "target", f, "data", p));
    }
  }

  // Do fitting
  spline.fittingSmoothCMAES(3, false, 0.0, 1.0, 5000, 5);

  // Display fitted splines
  for (double t = 0; t < 1.0; t += 0.005)
  {
    plot.add(Leph::VectorLabel("t", t, "fitted", spline.pos(t)));
  }
  plot.plot("t", "target").plot("t", "fitted").plot("t", "data", Leph::Plot::Points).render();
}

int main()
{
  testGeneratedData();
  testGeneratedDataNoise();
  testCapturedData();
  testCapturedDataWindow();
  testFittingCubic();
  testFittingSmooth();
  testFittingCMAES();

  return 0;
}
