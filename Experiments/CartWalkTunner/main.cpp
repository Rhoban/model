#include <iostream>
#include <string>
#include <chrono>
#include <thread>

// CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Ncurses/InterfaceCLI.hpp"

// UTILS
#include "SDKConnection.hpp"

/**
 * Return current time in milliseconds
 * (Relative to system start)
 */
unsigned long now()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main()
{
  // Initialisation of CartWalk parameters
  Leph::CartWalkProxy walk;
  Leph::VectorLabel params = walk.buildParams();

  // Initialize the connection
  Leph::SDKConnection sdkConnection;

  // Initialize the interface
  std::string statusEnabled = "Walk is Disabled";
  Leph::InterfaceCLI interface("CartWalk Tunner");
  interface.addParameters("Dynamic Parameters", params, "dynamic");
  interface.addParameters("Static Parameter", params, "static");
  interface.addStatus(statusEnabled);
  interface.addBinding(' ', "Toggle walk enable", [&params, &statusEnabled]() {
    params("dynamic:enabled") = !params("dynamic:enabled");
    if (params("dynamic:enabled"))
    {
      statusEnabled = "Walk is Enabled";
    }
    else
    {
      statusEnabled = "Walk is Disabled";
    }
  });

  // Main loop
  unsigned long timerOld = now();
  unsigned long timerNew = now();
  const double freq = 50.0;
  while (interface.tick())
  {
    // Generate walk orders
    walk.exec(1.0 / freq, params);
    Leph::VectorLabel outputs = walk.lastOutputs();
    // Sending them to the robot
    sdkConnection.setMotorAngles(outputs);
    // Waiting
    timerNew = now();
    unsigned long waitDelay = 0;
    unsigned long timerDiff = timerNew - timerOld;
    if (timerDiff <= 1000 / freq)
    {
      waitDelay = 1000 / freq - timerDiff;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(waitDelay));
    timerOld = now();
  }
  interface.quit();

  // Printing last used parameters
  std::cout << params << std::endl;

  return 0;
}
