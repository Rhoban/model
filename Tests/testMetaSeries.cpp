#include <iostream>
#include <cassert>
#include "TimeSeries/MetaSeries.hpp"

int main()
{
  Leph::MetaSeries series;

  assert(series.metaCount() == 0);

  series.metaUpdate(1.0, 1.5);
  assert(series.metaCount() == 1);
  assert(series.metaMin() == 1.5);
  assert(series.metaMax() == 1.5);

  series.metaUpdate(2.0, 2.5);
  series.metaUpdate(3.0, 3.5);
  assert(series.metaCount() == 3);
  assert(series.metaMin() == 1.5);
  assert(series.metaMax() == 3.5);

  std::cout << series << std::endl;

  return 0;
}
