#include <iostream>
#include <fstream>
#include "DMP/DMPSpline.hpp"
#include "Spline/SplineContainer.hpp"
#include "Plot/Plot.hpp"

int main()
{
  unsigned int kernelNum = 5;
  double overlap = 0.1;
  Leph::DMPSpline spline(kernelNum, overlap, 0.005);
  spline.addPoint(1.0, 0.0);
  spline.addPoint(1.5, 1.0, -2.0, -1.0);
  spline.addPoint(5.0, -1.0);
  spline.setKernelWeights(Eigen::VectorXd::Ones(kernelNum) * 2.0);
  std::cout << spline.getKernelWidths().transpose() << std::endl;
  std::cout << spline.getKernelWeights().transpose() << std::endl;

  Leph::DMPSpline spline2(kernelNum, overlap, 0.005);
  spline2.addPoint(1.0, 0.0);
  spline2.addPoint(5.0, -1.0);
  spline2.setKernelWeights(Eigen::VectorXd::Ones(kernelNum) * 2.0);
  std::cout << spline2.getKernelWidths().transpose() << std::endl;
  std::cout << spline2.getKernelWeights().transpose() << std::endl;

  std::ofstream testFile("/tmp/testDMPSpline.dmpspline");
  spline.exportData(testFile);
  spline2.exportData(testFile);
  testFile.close();

  std::ifstream testFile2("/tmp/testDMPSpline.dmpspline");
  spline.importData(testFile2);
  spline2.importData(testFile2);
  testFile2.close();

  std::ofstream testFile3("/tmp/testDMPSpline2.dmpspline");
  spline.exportData(testFile3);
  spline2.exportData(testFile3);
  testFile3.close();

  // Display added points
  for (size_t i = 0; i < spline.points().size(); i++)
  {
    std::cout << "Point " << i << ": " << spline.points()[i].time << " " << spline.points()[i].position << " "
              << spline.points()[i].velocity << " " << spline.points()[i].acceleration << std::endl;
  }
  // Display computed DMP part
  for (size_t i = 0; i < spline.parts().size(); i++)
  {
    std::cout << "Part " << i << ": start=" << spline.parts()[i].timeBegin << " end=" << spline.parts()[i].timeEnd
              << " kernelFirst=" << spline.parts()[i].kernelFirstIndex
              << " kernelLast=" << spline.parts()[i].kernelLastIndex << " Centers: ";
    for (size_t j = 0; j < spline.parts()[i].dmp.kernelNum(); j++)
    {
      std::cout << spline.parts()[i].dmp.kernelCenter(j) << " ";
    }
    std::cout << std::endl;
  }

  Leph::Plot plot;
  for (double t = -1.0; t < 6.0; t += 0.01)
  {
    plot.add(Leph::VectorLabel("t", t, "pos", spline.pos(t), "vel", spline.vel(t), "acc", spline.acc(t), "phase",
                               spline.phase(t), "gating", spline.gating(t), "forcing", spline.rawForcingFunction(t)));
    plot.add(Leph::VectorLabel("t", t, "pos2", spline2.pos(t), "vel2", spline2.vel(t), "acc2", spline2.acc(t), "phase2",
                               spline2.phase(t), "gating2", spline2.gating(t), "forcing2",
                               spline2.rawForcingFunction(t)));
  }
  plot.plot("t", "pos")
      .plot("t", "vel")
      .plot("t", "acc")
      .plot("t", "pos2")
      .plot("t", "vel2")
      .plot("t", "acc2")
      .render();
  plot.plot("t", "phase")
      .plot("t", "gating")
      .plot("t", "forcing")
      .plot("t", "phase2")
      .plot("t", "gating2")
      .plot("t", "forcing2")
      .render();

  // Test plot api
  spline.setKernelWeights(Eigen::VectorXd::Ones(kernelNum) * 2.0);
  spline.plot().plot("t", "all").render();

  // Test compatibility with SplineContainer template
  Leph::SplineContainer<Leph::DMPSpline> container;
  container.add("dim1", 5, 0.1, 0.005);
  container.add("dim2", 5, 0.1, 0.005);
  container.get("dim1").addPoint(0.0, 0.0, 0.0, 0.0);
  container.get("dim1").addPoint(1.0, 1.0, 0.0, 0.0);
  container.get("dim1").addPoint(2.0, 2.0, 0.0, 0.0);
  container.get("dim2").addPoint(0.0, 0.0, 0.0, 0.0);
  container.get("dim2").addPoint(2.0, 2.0, 0.0, 0.0);
  container.exportData("/tmp/testDMPSpline.container");

  return 0;
}
