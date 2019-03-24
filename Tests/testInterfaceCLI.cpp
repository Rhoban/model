#include <iostream>
#include <chrono>
#include <thread>
#include "Ncurses/InterfaceCLI.hpp"

int main()
{
  Leph::InterfaceCLI cli("Test interface");

  Leph::VectorLabel param("aa:label 1", 1.0, "aa:label 2", 2.0, "aa:label 3", 3.0, "bb:label2 1", 4.0, "bb:label2 2",
                          5.0);
  Leph::VectorLabel values("cc:value 1", 1.0, "cc:value 2", 2.0, "cc:value 3", 3.0, "dd:value 1", 4.0, "dd:value 2",
                           5.0);

  cli.addParameters("Params 1 (aa)", param, "aa");
  cli.addParameters("Params 2 (bb)", param, "bb");
  cli.addMonitors("Values 1 (cc)", values, "cc");
  cli.addMonitors("Values 2 (dd)", values, "dd");

  std::string status1 = "Pouet";
  std::string status2 = "Incr Disable";
  cli.addStatus(status1);
  cli.addStatus(status2);

  bool doIncr = false;
  cli.addBinding('i', "Toggle values incr", [&doIncr, &status2]() {
    doIncr = !doIncr;
    if (doIncr)
    {
      status2 = "Incr Enabled";
    }
    else
    {
      status2 = "Incr Disabled";
    }
  });

  cli.terminalOut() << "First line test\n";
  cli.terminalOut() << "Second line test\n";
  cli.terminalOut() << "Very very very long long long long line\n";

  while (cli.tick())
  {
    if (doIncr)
    {
      values(1) += 0.1;
      values(2) -= 0.2;
      values(3) *= 1.01;
      cli.terminalOut() << values(1) << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return 0;
}
