#include <iostream>
#include <Eigen/Dense>
#include "Plot/Plot.hpp"
#include "gp.h"
#include "rprop.h"
#include <libcmaes/cmaes.h>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"

std::default_random_engine generator;
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

double noise(double gain)
{
  return gain * uniform(generator);
}

double function(double x, double y)
{
  return sin(10 * x * x) * y + 0.5 * y * x + x * x - sin(6 * y);
}

void testLogs()
{
  Leph::Plot plot;

  // Load logged data
  Leph::MatrixLabel logs;
  logs.load(
      //"../../These/Data/logs-2015-05-16/model_2015-05-16-18-47-31.log");
      "../../These/Data/logs-2015-05-16/model_2015-05-16-18-49-57.log");

  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  for (size_t i = 10; i < logs.size(); i++)
  {
    model.get().setDOF(logs[i].extract("motor").rename("motor", ""), false);
    /*
    model.updateBase();
    model.setOrientation(
        logs[i]("sensor:pitch"),
        -logs[i]("sensor:roll"));
    */
    Leph::VectorLabel vel;
    Leph::VectorLabel torques = model.get().inverseDynamics(vel, vel);

    Leph::VectorLabel output = logs[i].extract("output");
    Leph::VectorLabel motor = logs[i].extract("motor");
    Leph::VectorLabel error = output.rename("output", "error");
    error.subOp(motor, "motor", "error");

    Leph::VectorLabel pt;
    pt.mergeUnion(error);
    pt.mergeUnion(logs[i]);
    pt.mergeUnion(torques.rename("", "torque"));

    // plot.add(pt);
    plot.add(Leph::VectorLabel("time", i, "motor", logs[i]("motor:left_ankle_roll"), "output",
                               logs[i]("output:left_ankle_roll"), "torque", torques("left_ankle_roll"), "outputold",
                               logs[i]("output:left_ankle_roll") - logs[i - 7]("output:left_ankle_roll")
                               //"outputold", logs[i-7]("output:left foot roll")
                               ));
  }
  plot
      //.plot("time", "motor", Leph::Plot::LinesPoints)
      //.plot("time", "output", Leph::Plot::LinesPoints)
      //.plot("output", "outputold", "motor", Leph::Plot::Points, "time")
      .plot("torque", "outputold", "motor", Leph::Plot::Points, "time")
      .render();
  return;
  plot.plot("time:timestamp", "error:left_ankle_roll", Leph::Plot::LinesPoints)
      .plot("time:timestamp", "motor:left_ankle_roll", Leph::Plot::LinesPoints)
      .plot("time:timestamp", "output:left_ankle_roll", Leph::Plot::LinesPoints)
      // plot.plot("time:timestamp", "error:left knee", Leph::Plot::Points)
      // plot.plot("time:timestamp", "torque:left knee", Leph::Plot::Points)
      // plot.plot("time:phase", "time:timestamp", "error:left knee", Leph::Plot::Points)
      .render();
  // logs.plot().plot("motor:left knee", "output:left knee", Leph::Plot::Points, "time:timestamp").render();
  // logs.plot().plot("time:phase", "motor:left knee", "output:left knee", Leph::Plot::Points,
  // "time:timestamp").render();
  return;

  libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
  for (size_t i = 0; i < logs.size(); i++)
  {
    Leph::VectorLabel output = logs[i].extract("output");
    Leph::VectorLabel motor = logs[i].extract("motor");
    Leph::VectorLabel error = output.rename("output", "error");
    error.subOp(motor, "motor", "error");
    double input[] = { /*
                       logs[i]("output:left foot roll"),
                       logs[i]("output:left foot pitch"),
                       logs[i]("output:left knee"),
                       logs[i]("output:left hip pitch"),
                       logs[i]("output:left hip roll"),
                       logs[i]("output:left hip yaw"),
                       */
                       logs[i]("time:phase"), logs[i]("time:timestamp")
    };
    gp.add_pattern(input, error("error:left_knee"));
  }
  // Optimize hyper parameter
  libgp::RProp rprop;
  rprop.init();
  rprop.maximize(&gp, 50, true);

  double sumError = 0.0;
  plot.clear();
  for (size_t i = 0; i < logs.size(); i++)
  {
    Leph::VectorLabel output = logs[i].extract("output");
    Leph::VectorLabel motor = logs[i].extract("motor");
    Leph::VectorLabel error = output.rename("output", "error");
    error.subOp(motor, "motor", "error");
    double input[] = { logs[i]("time:phase"), logs[i]("time:timestamp") };
    sumError += gp.f(input);
  }
}

void testSequence()
{
  Leph::Plot plot;
  std::vector<double> series;
  double tt = -1.0;
  for (double t = 0; t < 50.0; t += 0.1)
  {
    double value = noise(1.0);
    // double value = 0.8*sin(t) + sin(0.4*t) + noise(1.0);
    if (tt < 0.0 && series.size() > 0 && series.back() >= -0.05 && series.back() <= 0.05)
    {
      tt = 1.0;
    }

    if (tt > 0.0 && tt < 0.5)
    {
      series.push_back(-0.4 * tt + 0.8);
      // series.push_back(0.2*sin(2.0*tt) + 0.5);
      tt -= 0.1;
    }
    else if (tt >= 0.5)
    {
      series.push_back(0.4 * tt + 0.5);
      // series.push_back(0.2*sin(2.0*tt) + 0.5);
      tt -= 0.1;
    }
    else
    {
      series.push_back(value);
    }
  }

  // Initialize Gaussian Process
  libgp::GaussianProcess gp(3, "CovSum ( CovSEiso, CovNoise)");
  // libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");

  tt = -1.0;
  for (size_t i = 5; i < series.size(); i++)
  {
    bool enabled = false;
    if (tt < 0.0 && series[i - 1] >= -0.05 && series[i - 1] <= 0.05)
    {
      tt = 1.0;
    }
    if (tt > 0.0)
    {
      enabled = true;
      tt -= 0.1;
    }
    else
    {
      enabled = false;
    }
    plot.add(Leph::VectorLabel("t", i, "v0", series[i - 0], "v1", series[i - 1], "v2", series[i - 1] - series[i - 2],
                               "v3", series[i - 3], "enabled", !enabled));

    double input[] = { series[i - 1], series[i - 2], series[i - 3], series[i - 4], series[i - 5] };
    // double input[] = {series[i-1], enabled};
    gp.add_pattern(input, series[i - 0]);
  }
  plot.plot("t", "v0").render();
  plot.plot("v1", "v2", "v3", Leph::Plot::Points, "enabled").render();
  plot.plot("v1", "v2", "v0", Leph::Plot::Points, "v3").render();

  // Optimize hyper parameter
  libgp::RProp rprop;
  rprop.init();
  rprop.maximize(&gp, 50, true);

  plot.clear();
  for (size_t i = 3; i < series.size(); i++)
  {
    bool enabled = false;
    if (tt < 0.0 && series[i - 1] >= -0.05 && series[i - 1] <= 0.05)
    {
      tt = 1.0;
    }
    if (tt > 0.0)
    {
      enabled = true;
      tt -= 0.1;
    }
    else
    {
      enabled = false;
    }
    // double input[] = {series[i-1], enabled};
    double input[] = { series[i - 1], series[i - 2], series[i - 3], series[i - 4], series[i - 5] };
    plot.add(Leph::VectorLabel("t", i, "target", series[i - 0], "found", gp.f(input), "var", gp.var(input), "v0",
                               series[i - 0], "v1", series[i - 1], "v2", series[i - 2], "enabled", enabled, "v3",
                               series[i - 3]));
  }
  plot.plot("t", "target").plot("t", "found").plot("t", "var").render();
  plot.plot("v1", "v2", "found", Leph::Plot::Points, "v3")
      .plot("v1", "v2", "v0", Leph::Plot::Points, "v3")
      .plot("v1", "v2", "var", Leph::Plot::Points, "v3")
      .render();
  plot.plot("v1", "v0", Leph::Plot::Points).render();
}

void testSequenceCMAES()
{
  Leph::Plot plot;
  std::vector<double> series;
  double tt = -1.0;
  for (double t = 0; t < 50.0; t += 0.1)
  {
    double value = 0.8 * sin(t) + sin(0.4 * t) + noise(1.0);
    if (tt < 0.0 && series.size() > 0 && series.back() >= -0.05 && series.back() <= 0.05)
    {
      tt = 1.0;
    }
    if (tt > 0.0 && tt < 0.5)
    {
      series.push_back(0.4 * tt + 0.4);
      tt -= 0.1;
    }
    else if (tt > 0.5)
    {
      series.push_back(-0.4 * tt + 0.8);
      tt -= 0.1;
    }
    else
    {
      series.push_back(value);
    }
  }

  for (size_t i = 5; i < series.size(); i++)
  {
    plot.add(Leph::VectorLabel("t", i, "v0", series[i - 0], "v1", series[i - 1], "v2", series[i - 2]));
  }
  plot.plot("t", "v0").render();
  plot.plot("v1", "v0").render();
  plot.plot("v1", "v2", "v0").render();

  bool doPlot = false;
  libcmaes::FitFunc fitness = [&series, &doPlot](const double* x, const int N) {
    std::function<double(const double* x, double)> feature = [](const double* x, double x0) -> double {
      static double mem = 0.0;

      /*
      if (fabs(x0-x[0]) <= x[1]) {
          mem = x[2]*mem + x[3]*x0 + x[4];
      }

      double ret = 0.0;
      if (fabs(mem-x[5]) <= x[6]) {
          ret += mem;
      }
      */
      double ret = mem;
      mem = mem * x[0] + x0 * (1.0 - x[0]);
      return ret;

      /*
      if (fabs(x0) < 0.05) {
          mem = 1.0;
      }
      mem -= 0.1;

      if (mem >= 0.0) {
          return 1.0;
      } else {
          return 0.0;
      }
      */
    };
    Leph::Plot plot;
    // Initialize Gaussian Process
    libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
    for (size_t i = 3; i < series.size(); i++)
    {
      double input[] = { series[i - 1], series[i - 2] };
      gp.add_pattern(input, series[i]);
      plot.add(Leph::VectorLabel("t", i, "y", series[i], "x0", series[i - 1], "x1", feature(x, series[i - 1])));
    }
    // Optimize hyper parameter
    libgp::RProp rprop;
    rprop.init();
    rprop.maximize(&gp, 20, doPlot);
    // Compute error
    double error = 0.0;
    for (size_t i = 3; i < series.size(); i++)
    {
      plot.clear();
      double state = series[i - 1];
      for (size_t j = 0; j <= 10 && i + j < series.size(); j++)
      {
        double input[] = { state, feature(x, state) };
        double fitted = gp.f(input);
        error += pow(fitted - series[i + j], 2);
        state = fitted;
        if (doPlot)
        {
          plot.add(Leph::VectorLabel("t", i + j, "fitted", fitted, "y", series[i + j], "x0", fitted, "x1",
                                     feature(x, state)));
        }
      }
      /*
      double input[] = {series[i-1], feature(x, series[i-1])};
      double fitted = gp.f(input);
      double var = gp.var(input);
      plot.add(Leph::VectorLabel(
          "t", i,
          "fitted", fitted,
          "var", var,
          "x0", series[i-1],
          "x1", feature(x, series[i-1])
      ));
      error += pow(fitted-series[i], 2);
      */
      if (doPlot)
      {
        plot.plot("t", "y").plot("t", "fitted").render();
        plot.plot("x0", "x1", "y").plot("x0", "x1", "fitted").render();
        i += 20;
      }
    }
    std::cout << "ERROR: " << error << std::endl;
    for (size_t i = 0; i < (size_t)N; i++)
    {
      std::cout << x[i] << " ";
    }
    std::cout << std::endl;

    return error;
  };

  // Initializing CMA-ES starting point
  std::vector<double> x0;
  x0.push_back(0.5);
  /*
  x0.push_back(0.1);
  x0.push_back(0.1);
  x0.push_back(0.0);
  x0.push_back(0.1);
  x0.push_back(0.1);
  x0.push_back(0.1);
  */
  // Initialize CMAES algo
  libcmaes::CMAParameters<> cmaparams(x0, 0.1);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(true);
  cmaparams.set_str_algo("acmaes");
  cmaparams.set_max_iter(2);
  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
  // Get result
  std::cout << "RESULT: " << cmasols.best_candidate().get_x_dvec().transpose() << std::endl;
  double result[] = { cmasols.best_candidate().get_x_dvec()(0) };

  doPlot = true;
  fitness(result, 0);
}

void testSequence2()
{
  Leph::Plot plot;
  std::vector<double> series;
  // int state = 2;
  for (size_t k = 0; k < 1000; k++)
  {
    /*
    double t = 0.0;
    if (state == 0) t = 1.0;
    if (state == 1) t = 1.25;
    if (state == 2) t = 1.5;
    if (state == 3) t = 1.75;
    while (t >= 0.0) {
        double value;
        if (state == 0) value = -exp(-0.5*t*t) + 1.0;
        if (state == 1) value = 0.2*t;
        if (state == 2) value = 0.4*(t-0.8)*(t-0.8) - 0.25*t;
        if (state == 3) value = 0.2*sin(3.0*t);
        series.push_back(value);
        t -= 0.02;
    }
    state = rand() % 4;
    */
    static double t = 0.0;
    t += 0.02;
    series.push_back(sin(2.0 * t) + sin(3.0 * t));
  }

  libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
  for (size_t i = 8; i < series.size() - 5; i++)
  {
    plot.add(Leph::VectorLabel("t", i, "y0", series[i], "y1", series[i - 1], "y2", series[i - 2], "y3", series[i - 3]));
    double input[] = { series[i - 1], series[i - 2], series[i - 3], series[i - 4], series[i - 5], series[i - 6] };
    gp.add_pattern(input, series[i]);
  }
  plot.plot("t", "y0").render();
  plot.plot("y1", "y0").render();
  plot.plot("y1", "y2", "y0").render();

  // Optimize hyper parameter
  libgp::RProp rprop;
  rprop.init();
  rprop.maximize(&gp, 50, true);

  plot.clear();
  for (size_t i = 8; i < series.size() - 5; i++)
  {
    double input[] = { series[i - 1], series[i - 2], series[i - 3], series[i - 4], series[i - 5], series[i - 6] };
    double yp = gp.f(input);
    plot.add(Leph::VectorLabel("t", i, "y0", series[i], "yp", yp));
  }
  plot.plot("t", "all").render();

  size_t beginIndex = 100;
  plot.clear();
  double input[] = { series[beginIndex - 1], series[beginIndex - 2], series[beginIndex - 3],
                     series[beginIndex - 4], series[beginIndex - 5], series[beginIndex - 6] };
  for (size_t i = beginIndex; i < series.size() - 5; i++)
  {
    double yp = gp.f(input);
    input[5] = input[4];
    input[4] = input[3];
    input[3] = input[2];
    input[2] = input[1];
    input[1] = input[0];
    input[0] = yp;
    plot.add(Leph::VectorLabel("t", i, "y0", series[i], "yp", yp));
  }
  plot.plot("t", "all").render();
}

int main()
{
  testSequence2();
  return 0;
  testSequenceCMAES();
  return 0;
  testSequence();
  return 0;
  testLogs();
  return 0;

  // Plot real target function
  Leph::Plot plot;
  for (double x = 0.0; x <= 1.0; x += 0.03)
  {
    for (double y = 0.0; y <= 1.0; y += 0.03)
    {
      plot.add(Leph::VectorLabel("x", x, "y", y, "real", function(x, y)));
    }
  }

  // Initialize Gaussian Process
  libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");

  // Adding noised data point to model
  for (size_t k = 0; k < 100; k++)
  {
    double x = 0.5 * uniform(generator) + 0.5;
    double y = 0.5 * uniform(generator) + 0.5;
    double f = function(x, y) + noise(1.0);
    plot.add(Leph::VectorLabel("x", x, "y", y, "target", f));
    double input[] = { x, y };
    gp.add_pattern(input, f);
  }

  // Optimize hyper parameter
  libgp::RProp rprop;
  rprop.init();
  rprop.maximize(&gp, 50, true);

  // Predict fitted model
  for (double x = 0.0; x <= 1.0; x += 0.04)
  {
    for (double y = 0.0; y <= 1.0; y += 0.04)
    {
      double input[] = { x, y };
      plot.add(Leph::VectorLabel("x", x, "y", y, "fitted", gp.f(input)));
    }
  }

  // Display plot
  plot.plot("x", "y", "all").render();

  return 0;
}
