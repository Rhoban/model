#include <iostream>
#include "Utils/Differentiation.hpp"
#include "Types/MatrixLabel.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::Plot plot;
  // Use 10 size rolling buffer and
  // degree 3 interpolation polynoms
  Leph::Differentiation diff(10, 3);

  // Load logged data
  Leph::MatrixLabel logs;
  logs.load("../../These/Data/logs-2015-05-16/model_2015-05-16-18-47-31.log");
  // Print data informations
  std::cout << "Loaded " << logs.size() << " points with " << logs.dimension() << " entries" << std::endl;

  for (size_t i = 0; i < logs.size(); i++)
  {
    // Add target points
    double t = logs[i]("time:timestamp");
    plot.add(Leph::VectorLabel("t", t, "target", logs[i]("motor:left_knee")));
    // Compute fitted position and derivative
    diff.add(t, logs[i]);
    if (diff.isFull())
    {
      plot.add(Leph::VectorLabel("t", t, "fitted", diff.position(t)("motor:left_knee"), "fitted acc",
                                 1000.0 * diff.acceleration(t)("motor:left_knee")));
    }
  }
  plot.plot("t", "all").render();

  return 0;
}
