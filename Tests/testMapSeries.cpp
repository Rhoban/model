#include <iostream>
#include <cassert>
#include "Types/MapSeries.hpp"

int main()
{
  Leph::MapSeries map;
  assert(map.dimension() == 0);
  assert(map.exist("test") == false);

  map.append("test", 0.0, 1.0);
  map.append("test", 1.0, 2.0);
  map.append("test", 2.0, 4.0);
  map.append("test", 3.0, 6.0);
  map.append("test2", 1.5, 1.0);
  map.append("test2", 2.5, 2.0);
  map.append("test2", 3.5, 4.0);
  map.append("test2", 4.5, 6.0);
  assert(map.dimension() == 2);
  assert(map.exist("test") == true);
  assert(map.size("test") == 4);
  assert(map.timeMin("test") == 0.0);
  assert(map.timeMax("test") == 3.0);
  assert(map.timeMin() == 1.5);
  assert(map.timeMax() == 3.0);
  assert(map.front("test").value == 1.0);
  assert(map.back("test").value == 6.0);
  assert(map.at("test", 2).value == 4.0);
  assert(map.get("test", 1.5) == 3.0);
  assert(map.get("test", 2.0) == 4.0);
  assert(map.get("test", -1.0) == 1.0);
  assert(map.get("test", 4.0) == 6.0);
  assert(map.get("test", 3.0) == 6.0);

  map.exportData("/tmp/testMapSeries.log");
  map.plot().plot("time", "all").render();

  Leph::MapSeries map2;
  map2.importData("/tmp/testMapSeries.log");
  assert(map2.dimension() == 2);
  assert(map2.exist("test") == true);
  assert(map2.size("test") == 4);
  map2.exportData("/tmp/testMapSeries.log");

  map2.clear();
  assert(map2.dimension() == 0);
  assert(map2.exist("test") == false);

  return 0;
}
