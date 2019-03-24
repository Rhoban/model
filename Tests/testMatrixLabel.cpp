#include <iostream>
#include <cassert>
#include "Types/MatrixLabel.hpp"

int main()
{
  Leph::MatrixLabel serie;
  assert(serie.size() == 0);
  assert(serie.dimension() == 0);

  serie.append(Leph::VectorLabel("x", 1.0, "y", 2.0));
  serie.append(Leph::VectorLabel("x", 1.0, "y", 3.0));
  serie.append(Leph::VectorLabel("x", 2.0, "y", -2.0));
  assert(serie.size() == 3);
  assert(serie.dimension() == 2);
  assert(serie[0]("y") == 2.0);

  assert(serie.range(1, 2).size() == 2);
  assert(serie.range(1, 2).dimension() == 2);
  assert(serie.range(1, 2)[0]("y") == 3.0);

  std::cout << serie.mean() << std::endl;
  std::cout << serie.range(0, 1).mean() << std::endl;
  std::cout << serie.variance() << std::endl;
  std::cout << serie.range(0, 1).variance() << std::endl;
  std::cout << serie.stdDev() << std::endl;
  std::cout << serie.range(0, 1).stdDev() << std::endl;

  Leph::MatrixLabel serie2 = serie.copy();
  serie2.normalize();
  serie.plot().plot("index", "all").render();
  serie2.plot().plot("index", "all").render();

  serie.save("/tmp/testMatrixLabel.csv");
  Leph::MatrixLabel serie3;
  serie3.load("/tmp/testMatrixLabel.csv");
  assert(serie3.size() == serie.size());
  serie3.save("/tmp/testMatrixLabel.csv");
  serie3.plot().plot("index", "all").render();

  serie.clear();
  assert(serie.size() == 0);
  assert(serie.dimension() == 0);

  return 0;
}
