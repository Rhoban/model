#include <iostream>
#include <cassert>
#include <cmath>
#include "TimeSeries/TimeSeries.hpp"
#include "TimeSeries/RegressionLWPR.hpp"
#include "Plot/Plot.hpp"
#include "Utils/LWPRUtils.h"

double function(double x, double y)
{
  return x * x + y * y + 20.0 * sin(x);
}

int main()
{
  Leph::TimeSeries input1("in1");
  Leph::TimeSeries input2("in2");
  Leph::TimeSeries output("out");

  double time = 0.0;
  for (double y = -10.0; y <= 10.0; y += 1.0)
  {
    for (double x = -10.0; x <= 10.0; x += 1.0)
    {
      time += 1.0;
      input1.append(time, x);
      input2.append(time, y);
      output.append(time, function(x, y));
    }
  }

  Leph::Plot plot;
  for (size_t i = 0; i < input1.size(); i++)
  {
    plot.add(Leph::VectorLabel("time", input1[i].time, "in1", input1[i].value, "in2", input2[i].value, "out",
                               output[i].value));
  }
  plot.plot("time", "all").render();
  plot.plot("in1", "in2", "out", Leph::Plot::Points).render();
  plot.clear();

  Leph::RegressionLWPR regression;
  regression.addInputDeltaTime(&input1);
  regression.addInputDeltaTime(&input2);
  regression.setOutput(&output);

  regression.optimizeParameters(0, time, 0, time, 50, false);
  regression.parameterPrint();
  Leph::LWPRPrint(regression.model());
  std::cout << "MSE = " << regression.rangeMSE(0, time) << std::endl;

  for (double t = 0; t <= time; t += 1.0)
  {
    try
    {
      double yp = regression.predict(t);
      plot.add(
          Leph::VectorLabel("time", t, "in1", input1.get(t), "in2", input2.get(t), "out", output.get(t), "fitted", yp));
    }
    catch (const std::runtime_error& e)
    {
      // Nothing
    }
  }
  plot.plot("time", "all").render();
  plot.plot("in1", "in2", "out", Leph::Plot::Points).plot("in1", "in2", "fitted", Leph::Plot::Points).render();

  // Test propagate and future mode
  std::cout << "Testing propagate and future mode" << std::endl;
  double oldTime = time;
  // Generate input data
  for (double y = -15.0; y <= 15.0; y += 2.1)
  {
    for (double x = -15.0; x <= 15.0; x += 2.1)
    {
      time += 1.0;
      input1.append(time, x);
      input2.append(time, y);
    }
  }
  // Compute future prediction
  output.enableFutureMode();
  regression.computePropagate();
  // Disable future mode and append real data
  output.disableFutureMode();
  for (double t = oldTime + 1.0; t < time; t += 1.0)
  {
    output.append(t, function(input1.get(t), input2.get(t)));
    regression.learn(t);
  }
  // Plot future and real data
  plot.clear();
  for (size_t i = 0; i < output.sizeFuture(); i++)
  {
    double t = output.atFuture(i).time;
    if (!output.isTimeValid(t))
    {
      continue;
    }
    plot.add(Leph::VectorLabel("time", t, "in1", input1.get(t), "in2", input2.get(t), "old", output.atFuture(i).value,
                               "fitted", regression.predict(t), "target", output.get(t)));
  }
  plot.plot("in1", "in2", "target", Leph::Plot::Points)
      .plot("in1", "in2", "fitted", Leph::Plot::Points)
      .plot("in1", "in2", "old", Leph::Plot::Points)
      .render();

  regression.save("/tmp/saveRegressionLWPR.bin");
  regression.load("/tmp/saveRegressionLWPR.bin");

  return 0;
}
