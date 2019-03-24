#include <iostream>
#include <random>
#include <lwpr_eigen.hpp>
#include "Utils/LWPRUtils.h"
#include "Plot/Plot.hpp"
#include "Spline/CubicSpline.hpp"

std::default_random_engine generator;
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

double dRand()
{
  return uniform(generator);
}

double nonConvexFunction(double t, int mode)
{
  Leph::CubicSpline spline1;
  spline1.addPoint(0.0, 0.0);
  spline1.addPoint(0.1, 0.5);
  spline1.addPoint(0.3, -0.5);
  spline1.addPoint(0.5, 0.0);
  spline1.addPoint(0.6, -0.4);
  spline1.addPoint(0.8, 0.5);
  spline1.addPoint(1.0, 0.0);
  Leph::CubicSpline spline2;
  spline2.addPoint(0.0, 0.0);
  spline2.addPoint(0.1, 1.0);
  spline2.addPoint(0.3, -0.5);
  spline2.addPoint(0.5, 0.0);
  spline2.addPoint(0.6, 0.4);
  spline2.addPoint(0.8, -0.2);
  spline2.addPoint(1.0, 0.0);
  if (t < 0.5)
  {
    return spline1.pos(t);
  }
  else if (mode == 1)
  {
    return spline1.pos(t);
  }
  else if (mode == 2)
  {
    return spline2.pos(t);
  }
  else
  {
    return 0.0;
  }
}

double goalFunction(double t)
{
  return sin(2.0 * 3.14 * t) + 0.4 * sin(3.0 * 3.14 * t);
}
double readFunction(double t)
{
  return 0.8 * goalFunction(t - 0.1) + 0.5 + 0.6 * sin(3.14 * 2.0 * t);
}

/**
 * Try to learn non convex function
 */
void testNonConvex()
{
  size_t inputDim = 2;
  Eigen::VectorXd params = Leph::LWPRInitParameters(inputDim);
  std::vector<Eigen::VectorXd> trainInputs;
  std::vector<double> trainOutputs;

  Leph::Plot plot;
  for (size_t k = 0; k < 10; k++)
  {
    int mode = rand() % 2 + 1;
    for (double t = 0; t <= 1.0; t += 0.01)
    {
      double y = nonConvexFunction(t, mode) + 0.01 * dRand();
      double y1 = nonConvexFunction(t - 0.02, mode) + 0.01 * dRand();
      plot.add(Leph::VectorLabel("t", t, "target", y, "mode", mode));
      Eigen::VectorXd x(inputDim);
      x << t, y1;
      trainInputs.push_back(x);
      trainOutputs.push_back(y);
    }
  }
  plot.plot("t", "target", Leph::Plot::Points, "mode").render();

  params =
      Leph::LWPROptimizeParameters(inputDim, params, trainInputs, trainOutputs, trainInputs, trainOutputs, 500, false);

  LWPR_Object model = Leph::LWPRInit(inputDim, params);
  for (size_t i = 0; i < trainInputs.size(); i++)
  {
    Eigen::VectorXd y(1);
    y << trainOutputs[i];
    model.update(trainInputs[i], y);
  }

  plot.clear();
  for (int mode = 1; mode <= 2; mode++)
  {
    for (double t = 0; t <= 1.0; t += 0.01)
    {
      double y = nonConvexFunction(t, mode);
      Eigen::VectorXd confidence(1);
      Eigen::VectorXd x(inputDim);
      x << t, nonConvexFunction(t - 0.02, mode);
      Eigen::VectorXd yp = model.predict(x, confidence);
      if (confidence(0) > 2.0)
        confidence(0) = 2.0;
      plot.add(Leph::VectorLabel("t", t, "diff", t - nonConvexFunction(t - 0.02, mode), "real", y, "mode", mode,
                                 "fitted", yp(0), "confidence", confidence(0)));
    }
  }
  plot.plot("t", "real", Leph::Plot::Points)
      .plot("t", "fitted", Leph::Plot::Points)
      .plot("t", "confidence", Leph::Plot::Points)
      .render();
  plot.plot("t", "diff", "fitted", Leph::Plot::Points).render();
}

/**
 * Try to optimize and find lag
 */
double testLag(double lag, double delta)
{
  size_t inputDim = 2;
  Eigen::VectorXd params = Leph::LWPRInitParameters(inputDim);
  std::vector<Eigen::VectorXd> trainInputs;
  std::vector<double> trainOutputs;

  Leph::Plot plot;
  for (double t = 0; t < 40.0; t += 0.02)
  {
    double goal = goalFunction(t);
    double read = readFunction(t) + 0.3 * dRand();
    plot.add(Leph::VectorLabel("t", t, "goal", goal, "read", read));

    Eigen::VectorXd x(inputDim);
    x << goalFunction(t - lag), goalFunction(t - lag - delta);
    trainInputs.push_back(x);
    trainOutputs.push_back(read);
  }
  plot.plot("t", "all").render();

  params =
      Leph::LWPROptimizeParameters(inputDim, params, trainInputs, trainOutputs, trainInputs, trainOutputs, 100, true);

  LWPR_Object model = Leph::LWPRInit(inputDim, params);
  for (size_t i = 0; i < trainInputs.size(); i++)
  {
    Eigen::VectorXd y(1);
    y << trainOutputs[i];
    model.update(trainInputs[i], y);
  }

  plot.clear();
  double mse = 0.0;
  for (double t = 0; t <= 6.0; t += 0.02)
  {
    Eigen::VectorXd x(inputDim);
    x << goalFunction(t - lag), goalFunction(t - lag - delta);
    Eigen::VectorXd yp = model.predict(x);
    plot.add(Leph::VectorLabel("t", t, "x0", goalFunction(t - lag), "x1", goalFunction(t - lag - delta), "read",
                               readFunction(t), "fitted", yp(0)));
    mse += pow(yp(0) - readFunction(t), 2);
  }
  plot.plot("t", "all").render();
  plot.plot("x0", "x1", "fitted", Leph::Plot::Points).plot("x0", "x1", "read", Leph::Plot::Points).render();

  return mse;
}

int main()
{
  /*
  for (double d=0.0;d<0.3;d+=0.02) {
      for (double l=0.0;l<0.3;l+=0.02) {
          std::cout << l << " " << d << " " << testLag(l, d) << std::endl;
      }
  }
  */
  testLag(0.12, 0.12);
  return 0;
  testNonConvex();
  return 0;
}
