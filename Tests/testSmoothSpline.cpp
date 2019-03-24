#include <iostream>
#include "Spline/SmoothSpline.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::SmoothSpline spline;
  spline.addPoint(0.0, 0.0, 0.0, 0.0);
  spline.addPoint(1.0, 1.0, -1.0, 2.0);
  spline.addPoint(2.0, 0.5, 0.0, 0.0);

  Leph::Plot plot;
  for (double t = -0.5; t <= 2.5; t += 0.01)
  {
    plot.add(Leph::VectorLabel("t", t, "pos", spline.pos(t), "vel", spline.vel(t), "acc", spline.acc(t)));
  }
  plot.plot("t", "all").render();
  for (size_t i = 0; i < spline.points().size(); i++)
  {
    std::cout << "Part " << i << ": " << spline.points()[i].time << " " << spline.points()[i].position << " "
              << spline.points()[i].velocity << " " << spline.points()[i].acceleration << " " << std::endl;
  }

  std::ostringstream oss;
  spline.exportData(oss);

  Leph::SmoothSpline spline2;
  std::string rawStr = oss.str();
  std::istringstream iss(rawStr);
  spline2.importData(iss);

  plot.clear();
  for (double t = -0.5; t <= 2.5; t += 0.01)
  {
    plot.add(Leph::VectorLabel("t", t, "pos", spline2.pos(t), "vel", spline2.vel(t), "acc", spline2.acc(t)));
  }
  plot.plot("t", "all").render();
  for (size_t i = 0; i < spline.points().size(); i++)
  {
    std::cout << "Part " << i << ": " << spline.points()[i].time << " " << spline.points()[i].position << " "
              << spline.points()[i].velocity << " " << spline.points()[i].acceleration << " " << std::endl;
  }

  spline.plot().plot("t", "all").render();

  return 0;
}
