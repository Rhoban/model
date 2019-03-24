#include <iostream>
#include <sstream>
#include "Spline/CubicSpline.hpp"
#include "Spline/LinearSpline.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::CubicSpline spline;
  Leph::LinearSpline splineLin;
  spline.addPoint(0.0, 0.0, 0.0);
  spline.addPoint(1.0, 1.0, 0.0);
  spline.addPoint(2.0, 0.5, 0.0);
  spline.addPoint(2.0, 0.5, 4.0);
  spline.addPoint(3.0, -2.0, 0.0);
  splineLin.addPoint(0.0, 0.0);
  splineLin.addPoint(1.0, 1.0);
  splineLin.addPoint(2.0, 0.5);
  splineLin.addPoint(2.0, 0.5);
  splineLin.addPoint(3.0, -2.0);

  Leph::Plot plot;
  for (double t = -0.5; t <= 3.5; t += 0.01)
  {
    plot.add(Leph::VectorLabel("t", t, "pos", spline.pos(t), "pos linear", splineLin.pos(t), "vel", spline.vel(t),
                               "acc", spline.acc(t)));
  }
  plot.plot("t", "all").render();
  plot.clear();

  Leph::CubicSpline spline2;
  spline2.addPoint(0.0, 0.0);
  spline2.addPoint(1.0, 1.0);
  spline2.addPoint(2.0, -2.0, 1.0);
  spline2.addPoint(3.0, 0.0);
  std::ostringstream oss;
  spline2.exportData(oss);
  std::string rawStr = oss.str();
  for (int k = 0; k < 10; k++)
  {
    Leph::CubicSpline splineTmp;
    std::istringstream iss(rawStr);
    splineTmp.importData(iss);
    splineTmp.subdivide(2);
    splineTmp.randomNoise(0.2, 0.5, false);
    for (double t = spline2.min(); t < spline2.max(); t += 0.01)
    {
      if (k == 0)
      {
        plot.add(Leph::VectorLabel("t", t, "valOrigin", spline2.pos(t)));
      }
      plot.add(Leph::VectorLabel("t", t, "val_" + std::to_string(k), splineTmp.pos(t)));
    }
  }
  plot.plot("t", "all").render();

  return 0;
}
