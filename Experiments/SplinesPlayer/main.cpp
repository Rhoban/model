#include <iostream>

// CODE
#include "Utils/time.h"
#include "Types/VectorLabel.hpp"
#include "Types/MatrixLabel.hpp"
#include "Spline/SplineContainer.hpp"
#include "Utils/Scheduling.hpp"
#include "Ncurses/InterfaceCLI.hpp"

// UTILS
#include "SDKConnection.hpp"
#include "MotorsLayout.h"

int main(int argc, char** argv)
{
  // Command line arguments
  if (argc != 2)
  {
    std::cout << "Usage: ./experiment splinesFile" << std::endl;
    return -1;
  }
  std::string splinesFile = argv[1];

  // Load splines
  Leph::SplineContainer<Leph::Spline> splines;
  splines.importData(splinesFile);
  // Initialize outputs vector
  Leph::VectorLabel outputs;
  for (const auto& sp : splines.get())
  {
    outputs.append("output:" + sp.first, 0.0);
  }

  // Retrieve splines bounds
  double timeMin = splines.min();
  double timeMax = splines.max();

  // Initialize the robot connection
  Leph::SDKConnection sdkConnection;

  // Initialize Log time serie
  Leph::MatrixLabel serie;
  Leph::VectorLabel logVector = outputs;

  // Initialize Ncurses Interface and state
  bool isRunning = false;
  bool isLogging = false;
  std::string statusRunning = "Movement STOPPED";
  std::string statusLogging = "Logging STOPPED";
  Leph::InterfaceCLI interface("Splines Player");
  interface.addStatus(statusRunning);
  interface.addStatus(statusLogging);
  // Movement enable control
  interface.addBinding(' ', "Movement toggle", [&isRunning, &statusRunning]() {
    isRunning = !isRunning;
    if (isRunning)
    {
      statusRunning = "Movement RUNNING";
    }
    else
    {
      statusRunning = "Movement STOPPED";
    }
  });
  // Logging control
  interface.addBinding('l', "Logging toggle", [&isLogging, &statusLogging, &serie, &interface]() {
    isLogging = !isLogging;
    if (isLogging)
    {
      statusLogging = "Logging RUNNING";
    }
    else
    {
      // Export logs
      statusLogging = "Logging STOPPED";
      std::string fileName = "/tmp/log-" + Leph::currentDate() + "_raw.csv";
      interface.terminalOut() << "Logging " << serie.size() << " points to " << fileName << std::endl;
      serie.save(fileName);
    }
  });

  // Main loop
  double freq = 50.0;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(freq);
  double t = timeMin;
  while (interface.tick(false))
  {
    // Compute outputs reference from splines
    for (const auto& sp : splines.get())
    {
      outputs("output:" + sp.first) = sp.second.pos(t);
    }
    // Conversion to real Mowgly motors signs
    MotorsLayoutConversion(outputs, "output:");
    // Send references to motors
    sdkConnection.setMotorAngles(outputs);
    // Logging
    if (isLogging)
    {
      logVector.setOrAppend("time:timestamp", scheduling.timestamp());
      logVector.setOrAppend("time:duration", scheduling.duration());
      logVector.setOrAppend("time:phase", t);
      logVector.mergeInter(outputs);
      sdkConnection.getMotorAngles(logVector);
      sdkConnection.getSensorValues(logVector);
      MotorsLayoutConversion(logVector, "output:");
      MotorsLayoutConversion(logVector, "motor:");
      serie.append(logVector);
    }
    // Phase cycling
    if (isRunning)
    {
      t += 1.0 / freq;
      if (t >= timeMax)
        t = timeMin;
    }
    // Waiting
    scheduling.wait();
    // Scheduling information
    if (scheduling.isError())
    {
      interface.terminalOut() << "Scheduling error: " << scheduling.timeError() << std::endl;
    }
  }

  return 0;
}
