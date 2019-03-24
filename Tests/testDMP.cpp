#include <iostream>
#include <libcmaes/cmaes.h>
#include "DMP/DMP.hpp"
#include "Plot/Plot.hpp"

static void testLearning()
{
  Leph::SmoothSpline target;
  target.addPoint(0.0, 0.0, 0.0, 0.0);
  target.addPoint(1.0, 0.5, 0.5, -0.6);
  target.addPoint(2.0, -0.5, 0.4, 0.7);
  target.addPoint(3.0, 1.0, 0.0, 0.0);
  target.plot().plot("time", "all").render();

  // Fitness function
  libcmaes::FitFuncEigen fitness = [&target](const Eigen::VectorXd& params) {
    Leph::DMP dmp(1, 5);
    dmp.setParameters(params, true, false, true);
    Eigen::VectorXd startPos(1);
    Eigen::VectorXd startVel(1);
    Eigen::VectorXd startAcc(1);
    Eigen::VectorXd endPos(1);
    Eigen::VectorXd endVel(1);
    Eigen::VectorXd endAcc(1);
    startPos(0) = target.pos(0);
    startVel(0) = target.vel(0);
    startAcc(0) = target.acc(0);
    endPos(0) = target.pos(target.max());
    endVel(0) = target.vel(target.max());
    endAcc(0) = target.acc(target.max());
    dmp.init(target.max(), startPos, startVel, startAcc, endPos, endVel, endAcc);
    double cost = 0.0;
    while (dmp.currentTime() <= target.max())
    {
      cost += pow(target.pos(dmp.currentTime()) - dmp.statePos()(0), 2);
      dmp.step(0.01);
    }

    return cost;
  };

  // CMAES initialization
  Leph::DMP dmp(1, 5);
  Eigen::VectorXd params = dmp.getParameters(true, false, true);
  libcmaes::CMAParameters<> cmaparams(params, -1.0, 10);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(true);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_elitism(true);
  cmaparams.set_restarts(4);
  cmaparams.set_max_iter(5000);

  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);

  // Retrieve best Trajectories and score
  params = cmasols.get_best_seen_candidate().get_x_dvec();
  double score = cmasols.get_best_seen_candidate().get_fvalue();
  std::cout << "BestScore: " << score << std::endl;

  dmp.setParameters(params, true, false, true);
  Eigen::VectorXd startPos(1);
  Eigen::VectorXd startVel(1);
  Eigen::VectorXd startAcc(1);
  Eigen::VectorXd endPos(1);
  Eigen::VectorXd endVel(1);
  Eigen::VectorXd endAcc(1);
  startPos(0) = target.pos(0);
  startVel(0) = target.vel(0);
  startAcc(0) = target.acc(0);
  endPos(0) = target.pos(target.max());
  endVel(0) = target.vel(target.max());
  endAcc(0) = target.acc(target.max());
  Leph::Plot plot;
  dmp.init(target.max(), startPos, startVel, startAcc, endPos, endVel, endAcc);
  while (dmp.currentTime() <= target.max())
  {
    plot.add(Leph::VectorLabel("t", dmp.currentTime(), "fitted", dmp.statePos()(0), "target",
                               target.pos(dmp.currentTime())));
    dmp.step(0.01);
  }
  dmp.init(target.max(), startPos, startVel, startAcc, 2.0 * endPos, endVel, endAcc);
  while (dmp.currentTime() <= target.max())
  {
    plot.add(Leph::VectorLabel("t", dmp.currentTime(), "fitted", dmp.statePos()(0)));
    dmp.step(0.01);
  }
  dmp.init(target.max(), startPos, startVel, startAcc, -2.0 * endPos, endVel, endAcc);
  while (dmp.currentTime() <= target.max())
  {
    plot.add(Leph::VectorLabel("t", dmp.currentTime(), "fitted", dmp.statePos()(0)));
    dmp.step(0.01);
  }
  plot.plot("t", "all").render();
}

static void testSpline()
{
  Leph::Plot plot;
  unsigned int dim = 1;
  unsigned int num = 5;
  double timeLength = 2.0;

  Leph::DMP dmp1(dim, num);
  Leph::DMP dmp2(dim, num);
  Eigen::VectorXd startPos1 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd startVel1 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd startAcc1 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endPos1 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endVel1 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endAcc1 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd startPos2 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd startVel2 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd startAcc2 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endPos2 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endVel2 = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endAcc2 = Eigen::VectorXd::Zero(dim);
  startPos1(0) = 0.0;
  startVel1(0) = 0.0;
  startAcc1(0) = 0.0;
  endPos1(0) = 2.0;
  endVel1(0) = -1.0;
  endAcc1(0) = 1.0;
  startPos2(0) = endPos1(0);
  startVel2(0) = endVel1(0);
  startAcc2(0) = endAcc1(0);
  endPos2(0) = 0.0;
  endVel2(0) = 0.0;
  endAcc2(0) = 0.0;
  for (size_t i = 0; i < num; i++)
  {
    dmp1.kernelWeight(0, i) = 20.0;
    dmp2.kernelWeight(0, i) = -20.0;
  }

  dmp1.init(timeLength, startPos1, startVel1, startAcc1, endPos1, endVel1, endAcc1);
  dmp2.init(timeLength, startPos2, startVel2, startAcc2, endPos2, endVel2, endAcc2);

  while (true)
  {
    if (dmp1.currentTime() >= timeLength)
    {
      break;
    }
    dmp1.step(0.01);
    plot.add(Leph::VectorLabel("t", dmp1.currentTime(), "phase", dmp1.statePhase(), "gating", dmp1.stateGating(),
                               "forcing", dmp1.forcingFunction(dmp1.statePhase(), dmp1.stateGating())(0), "pos",
                               dmp1.statePos()(0), "vel", dmp1.stateVel()(0), "acc", dmp1.stateAcc()(0)));
  }
  while (true)
  {
    if (dmp2.currentTime() >= timeLength)
    {
      break;
    }
    dmp2.step(0.01);
    plot.add(Leph::VectorLabel("t", timeLength + dmp2.currentTime(), "phase", dmp2.statePhase(), "gating",
                               dmp2.stateGating(), "forcing",
                               dmp2.forcingFunction(dmp2.statePhase(), dmp2.stateGating())(0), "pos",
                               dmp2.statePos()(0), "vel", dmp2.stateVel()(0), "acc", dmp2.stateAcc()(0)));
  }
  plot.plot("t", "all").render();
}

int main()
{
  Leph::Plot plot;

  unsigned int dim = 1;
  unsigned int num = 5;
  Leph::DMP dmp(dim, num);
  double timeLength = 2.0;
  Eigen::VectorXd startPos = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd startVel = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd startAcc = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endPos = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endVel = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd endAcc = Eigen::VectorXd::Zero(dim);
  startPos(0) = 0.0;
  startVel(0) = 1.0;
  startAcc(0) = 3.0;
  endPos(0) = 2.0;
  endVel(0) = -1.0;
  endAcc(0) = 0.0;

  dmp.init(timeLength, startPos, startVel, startAcc, endPos, endVel, endAcc);
  for (size_t i = 0; i < num; i++)
  {
    std::cout << "Kernel " << i << ": center=" << dmp.kernelCenter(i) << " width=" << dmp.kernelWidth(i) << std::endl;
  }
  while (true)
  {
    if (dmp.currentTime() >= timeLength)
    {
      break;
    }
    dmp.step(0.01);
    plot.add(Leph::VectorLabel("t", dmp.currentTime(), "phase", dmp.statePhase(), "gating", dmp.stateGating(),
                               "forcing", dmp.forcingFunction(dmp.statePhase(), dmp.stateGating())(0), "pos",
                               dmp.statePos()(0), "vel", dmp.stateVel()(0), "acc", dmp.stateAcc()(0)));
  }
  plot.plot("t", "all").render();

  // Test two DMP merging acceleration
  // continuity
  testSpline();
  // Test DMP fitting
  testLearning();

  return 0;
}
