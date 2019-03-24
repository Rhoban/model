#include <iostream>

// CODE
#include "StaticWalk/StaticWalk.hpp"
#include "Utils/Scheduling.hpp"

// UTILS
#include "SDKConnection.hpp"
#include "SDKInterface.hpp"

int main()
{
  Leph::StaticWalk walk;
  Leph::VectorLabel params = walk.buildParams();
  params("param:step");
  0.0;
  params("param:lateral");
  0.0;
  Leph::VectorLabel outputs = walk.initPose(params);

  double freq = 50.0;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(freq);
  while (true)
  {
    // StaticWalk generator
    Leph::VectorLabel outputs = walk.exec(1.0 / freq, params);
    // Waiting
    scheduling.wait();
  }

  return 0;
}
