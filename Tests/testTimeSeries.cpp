#include <iostream>
#include <cassert>
#include <sstream>
#include "TimeSeries/TimeSeries.hpp"

void testFutureMode()
{
  Leph::TimeSeries series("test");

  assert(series.name() == "test");
  assert(series.size() == 0);
  assert(series.sizeFuture() == 0);
  assert(series.isFutureMode() == false);

  series.append(1.0, 1.5);
  series.append(2.0, 2.5);

  series.enableFutureMode();
  assert(series.name() == "test");
  assert(series.size() == 2);
  assert(series.timeMin() == 1.0);
  assert(series.timeMax() == 2.0);
  assert(series.lastValue() == 2.5);
  assert(series.get(1.5) == 2.0);
  assert(series.sizeFuture() == 0);
  assert(series.isFutureMode() == true);

  series.append(3.0, 3.5);
  series.append(4.0, 4.5);
  assert(series.name() == "test");
  assert(series.size() == 4);
  assert(series.timeMin() == 1.0);
  assert(series.timeMax() == 4.0);
  assert(series.lastValue() == 4.5);
  assert(series.get(1.5) == 2.0);
  assert(series.get(2.5) == 3.0);
  assert(series.get(3.5) == 4.0);
  assert(series.sizeFuture() == 2);
  assert(series.isFutureMode() == true);

  series.disableFutureMode();
  assert(series.name() == "test");
  assert(series.size() == 2);
  assert(series.timeMin() == 1.0);
  assert(series.timeMax() == 2.0);
  assert(series.lastValue() == 2.5);
  assert(series.get(1.5) == 2.0);
  assert(series.sizeFuture() == 2);
  assert(series.isFutureMode() == false);

  series.append(3.0, 3.5);
  assert(series.size() == 3);
  assert(series.timeMin() == 1.0);
  assert(series.timeMax() == 3.0);
  assert(series.lastValue() == 3.5);
  assert(series.get(1.5) == 2.0);
  assert(series.sizeFuture() == 2);
  assert(series.isFutureMode() == false);
  assert(series.atFuture(0).time == 4.0);
  assert(series.atFuture(1).time == 3.0);

  try
  {
    series.enableFutureMode();
    assert(false);
  }
  catch (...)
  {
  }
  assert(series.isFutureMode() == false);

  series.clearFuture();
  assert(series.size() == 3);
  assert(series.timeMin() == 1.0);
  assert(series.timeMax() == 3.0);
  assert(series.lastValue() == 3.5);
  assert(series.get(1.5) == 2.0);
  assert(series.sizeFuture() == 0);
  assert(series.isFutureMode() == false);
}

void testMeta()
{
  Leph::TimeSeries series("test");

  assert(series.metaCount() == 0);
  assert(series.isTimeValid(1.0) == false);
  assert(series.isTimeValid(1.5) == false);

  series.append(1.0, 1.5);
  assert(series.metaCount() == 1);
  assert(series.metaMin() == 1.5);
  assert(series.metaMax() == 1.5);
  assert(series.isTimeValid(1.0) == false);
  assert(series.isTimeValid(1.5) == false);

  series.append(2.0, 2.5);
  series.append(3.0, 3.5);
  assert(series.metaCount() == 3);
  assert(series.metaMin() == 1.5);
  assert(series.metaMax() == 3.5);
  assert(series.isTimeValid(1.0) == true);
  assert(series.isTimeValid(1.5) == true);
  assert(series.isTimeValid(0.5) == false);

  std::cout << series << std::endl;

  series.enableFutureMode();
  series.append(4.0, 4.5);
  series.append(5.0, 5.5);
  assert(series.metaCount() == 3);
  assert(series.metaMin() == 1.5);
  assert(series.metaMax() == 3.5);
}

int main()
{
  Leph::TimeSeries series1("test1");
  Leph::TimeSeries series2("test2", 4);

  assert(series1.name() == "test1");
  assert(series2.name() == "test2");
  assert(series1.size() == 0);
  assert(series2.size() == 0);
  assert(series1.sizeFuture() == 0);
  assert(series2.sizeFuture() == 0);
  assert(series1.isFutureMode() == false);
  assert(series2.isFutureMode() == false);

  series1.append(1.0, 1.5);
  series2.append(1.0, 1.5);

  assert(series1.name() == "test1");
  assert(series2.name() == "test2");
  assert(series1.size() == 1);
  assert(series2.size() == 1);
  assert(series1.lastTime() == 1.0);
  assert(series1.lastValue() == 1.5);
  assert(series2.lastTime() == 1.0);
  assert(series2.lastValue() == 1.5);
  assert(series1[0].time == 1.0);
  assert(series1[0].value == 1.5);
  assert(series2[0].time == 1.0);
  assert(series2[0].value == 1.5);
  assert(series1.timeMin() == 1.0);
  assert(series1.timeMax() == 1.0);
  assert(series2.timeMin() == 1.0);
  assert(series2.timeMax() == 1.0);
  assert(series1.sizeFuture() == 0);
  assert(series2.sizeFuture() == 0);
  assert(series1.isFutureMode() == false);
  assert(series2.isFutureMode() == false);

  series1.append(2.0, 2.5);
  series2.append(2.0, 2.5);
  series1.append(3.0, 3.5);
  series2.append(3.0, 3.5);

  assert(series1.size() == 3);
  assert(series2.size() == 3);
  assert(series1.lastTime() == 3.0);
  assert(series1.lastValue() == 3.5);
  assert(series2.lastTime() == 3.0);
  assert(series2.lastValue() == 3.5);
  assert(series1[0].time == 3.0);
  assert(series1[0].value == 3.5);
  assert(series2[0].time == 3.0);
  assert(series2[0].value == 3.5);
  assert(series1[2].time == 1.0);
  assert(series1[2].value == 1.5);
  assert(series2[2].time == 1.0);
  assert(series2[2].value == 1.5);
  assert(series1.timeMin() == 1.0);
  assert(series1.timeMax() == 3.0);
  assert(series2.timeMin() == 1.0);
  assert(series2.timeMax() == 3.0);
  assert(series1.get(2.5) == 3.0);
  assert(series2.get(2.5) == 3.0);
  assert(series1.get(3.0) == 3.5);
  assert(series2.get(3.0) == 3.5);
  assert(series1.getLowerIndex(2.5) == 0);
  assert(series1.getUpperIndex(2.5) == 1);
  assert(series1.getLowerIndex(2.0) == 0);
  assert(series1.getUpperIndex(2.0) == 1);
  assert(series1.getLowerIndex(3.0) == 0);
  assert(series1.getUpperIndex(3.0) == 1);
  assert(series1.getClosestIndex(3.0) == 0);
  assert(series1.getClosestIndex(2.0) == 1);
  assert(series1.getClosestIndex(1.0) == 2);

  series1.append(4.0, 4.5);
  series2.append(4.0, 4.5);
  series1.append(5.0, 5.5);
  series2.append(5.0, 5.5);

  assert(series1.size() == 5);
  assert(series2.size() == 4);
  assert(series1.lastTime() == 5.0);
  assert(series1.lastValue() == 5.5);
  assert(series2.lastTime() == 5.0);
  assert(series2.lastValue() == 5.5);
  assert(series1[0].time == 5.0);
  assert(series1[0].value == 5.5);
  assert(series2[0].time == 5.0);
  assert(series2[0].value == 5.5);
  assert(series1[4].time == 1.0);
  assert(series1[4].value == 1.5);
  assert(series2[3].time == 2.0);
  assert(series2[3].value == 2.5);
  assert(series1.timeMin() == 1.0);
  assert(series1.timeMax() == 5.0);
  assert(series2.timeMin() == 2.0);
  assert(series2.timeMax() == 5.0);
  assert(series1.get(4.5) == 5.0);
  assert(series2.get(4.5) == 5.0);
  assert(series1.get(4.0) == 4.5);
  assert(series2.get(4.0) == 4.5);
  assert(series1.getLowerIndex(2.5) == 2);
  assert(series1.getUpperIndex(2.5) == 3);
  assert(series1.getLowerIndex(2.0) == 2);
  assert(series1.getUpperIndex(2.0) == 3);
  assert(series1.getLowerIndex(3.0) == 1);
  assert(series1.getUpperIndex(3.0) == 2);
  std::cout << series2 << std::endl;

  std::ostringstream oss;
  series1.save(oss);
  series1.save(oss);
  std::cout << oss.str();
  std::cout << std::endl;

  Leph::TimeSeries series3;
  Leph::TimeSeries series4;
  Leph::TimeSeries series5;
  std::istringstream iss(oss.str());
  series3.load(iss);
  series4.load(iss);
  series5.load(iss);

  assert(series3.size() == 5);
  assert(series3.lastTime() == 5.0);
  assert(series3.lastValue() == 5.5);
  assert(series3[0].time == 5.0);
  assert(series3[0].value == 5.5);
  assert(series3[4].time == 1.0);
  assert(series3[4].value == 1.5);
  assert(series3.timeMin() == 1.0);
  assert(series3.timeMax() == 5.0);
  assert(series3.get(4.5) == 5.0);

  assert(series5.size() == 0);

  series3.save(std::cout);
  series4.save(std::cout);
  series5.save(std::cout);

  series4.clear();
  assert(series4.size() == 0);

  testFutureMode();

  testMeta();

  return 0;
}
