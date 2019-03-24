#include <iostream>
#include "Types/VectorLabel.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::Plot plot;
  for (int i = 0; i < 100; i++)
  {
    Leph::VectorLabel vect("aa:toto", i, "aa:titi", -i, "bb:tata", 1.0, "bb:tete", -0.5 * i);
    plot.add(vect);
  }
  plot.add(Leph::VectorLabel("cc:tutu", 0.0));
  plot.add(Leph::VectorLabel("cc:tutu", 0.0));
  plot.rangeX(0.0, 120.0)
      .rangeY(-20.0, 20.0)
      .plot("index", "all")
      .plot("index", "aa:*", Leph::Plot::Lines)
      .plot("index", "aa:titi", Leph::Plot::Points)
      .plot("index", "bb:tata", Leph::Plot::Points, "aa:toto")
      .render();
  plot.rangeX(0.0, 120.0)
      .rangeY(-20.0, 20.0)
      .plot("index", "all")
      .plot("index", "aa:titi", Leph::Plot::Points)
      .plot("index", "bb:tata", Leph::Plot::Points, "aa:toto")
      .render("/tmp/testPlot.plot");
  plot.rangeUniform().plot("index", "all").render();

  plot.add({
      "t",
      1.0,
      "val",
      1.0,
  });
  plot.add({
      "t",
      2.0,
      "val",
      3.0,
  });
  plot.add({
      "t",
      3.0,
      "val",
      5.0,
  });
  plot.rangeX(0.0, 120.0)
      .rangeY(-20.0, 20.0)
      .multiplot(1, 2)
      .plot("index", "bb:tata", Leph::Plot::Points, "aa:toto")
      .nextPlot()
      .plot("index", "aa:titi", Leph::Plot::Points)
      .nextPlot()
      .plot("t", "val")
      .plot("index", "aa:toto", Leph::Plot::Points)
      .render();

  return 0;
}
