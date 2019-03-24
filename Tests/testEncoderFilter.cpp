#include <iostream>
#include <libcmaes/cmaes.h>
#include "Utils/EncoderFilter.hpp"
#include "Plot/Plot.hpp"
#include "Spline/CubicSpline.hpp"
#include "Spline/FittedSpline.hpp"
#include "Types/MatrixLabel.hpp"

std::mt19937 generator;
std::uniform_real_distribution<double> uniformDist(-1.0, 1.0);

void testFilterTuning()
{
  Leph::CubicSpline splineGoal;
  splineGoal.addPoint(0.0, 0.0);
  splineGoal.addPoint(1.0, 1.0);
  splineGoal.addPoint(2.0, -2.0, 5.0);
  splineGoal.addPoint(3.0, 0.0);

  Eigen::VectorXd params = Eigen::VectorXd::Ones(4);
  libcmaes::CMAParameters<> cmaparams(params, -1.0);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(false);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_restarts(10);
  cmaparams.set_max_iter(1000);
  cmaparams.set_elitism(true);
  libcmaes::FitFuncEigen fitness = [&splineGoal](const Eigen::VectorXd& params) {
    Leph::EncoderFilter filter(0.0, 0.0, 10.0, 100.0);
    for (size_t i = 0; i < (size_t)params.size(); i++)
    {
      if (params(i) <= 0.0)
      {
        return 100.0 - 100.0 * params(i);
      }
    }
    filter.setObservationVar(params(0));
    filter.setPredictionVar(Eigen::Vector3d(params(1), params(2), params(3)));
    double score = 0.0;
    for (double t = splineGoal.min(); t <= splineGoal.max(); t += 0.01)
    {
      filter.update(0.01, splineGoal.pos(t));
      score += 0.01 * (fabs(filter.pos() - splineGoal.pos(t)) + fabs(filter.vel() - splineGoal.vel(t)) +
                       fabs(filter.acc() - splineGoal.acc(t)));
    }
    return score;
  };
  // Do opimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
  params = cmasols.get_best_seen_candidate().get_x_dvec();
  std::cout << "Best Parameters: " << params.transpose() << std::endl;

  // Display best parameeters
  Leph::Plot plot;
  Leph::EncoderFilter filter(0.0, 0.0, 10.0, 100.0);
  filter.setObservationVar(params(0));
  filter.setPredictionVar(Eigen::Vector3d(params(1), params(2), params(3)));
  for (double t = splineGoal.min(); t <= splineGoal.max(); t += 0.01)
  {
    filter.update(0.01, splineGoal.pos(t));
    plot.add(Leph::VectorLabel("t", t, "target pos", splineGoal.pos(t), "filtered pos", filter.pos(), "target vel",
                               splineGoal.vel(t), "filtered vel", filter.vel(), "target acc", splineGoal.acc(t),
                               "filtered acc", filter.acc()));
  }
  plot.plot("t", "all").render();
}

void testFilterVsSpline()
{
  Leph::CubicSpline splineGoal;
  splineGoal.addPoint(0.0, 0.0);
  splineGoal.addPoint(1.0, 1.0);
  splineGoal.addPoint(2.0, -2.0, 5.0);
  splineGoal.addPoint(3.0, 0.0);

  Leph::EncoderFilter filter(0.0, 0.0, 10.0, 100.0);
  filter.setObservationVar(0.0000001);
  filter.setPredictionVar(Eigen::Vector3d(0.0000001, 0.0000001, 0.5));

  std::vector<double> dataVal;
  std::vector<double> dataTime;
  std::vector<double> dataVelTarget;
  std::vector<double> dataAccTarget;
  std::vector<double> dataVel2;
  std::vector<double> dataVel3;
  for (double t = splineGoal.min(); t <= splineGoal.max(); t += 0.01)
  {
    dataTime.push_back(t);
    dataVal.push_back(splineGoal.pos(t));
    dataVelTarget.push_back(splineGoal.vel(t));
    dataAccTarget.push_back(splineGoal.acc(t));
  }

  Leph::Plot plot;
  for (size_t i = 0; i < dataVal.size(); i++)
  {
    const size_t len = 4;
    if (i >= len)
    {
      Leph::FittedSpline fitted;
      for (size_t j = i - len; j <= i; j++)
      {
        fitted.addPoint(dataTime[j], dataVal[j]);
      }
      fitted.fittingGlobal(3, len);
      plot.add(Leph::VectorLabel("t", dataTime[i], "fitted", fitted.pos(dataTime[i]), "fitted vel",
                                 fitted.vel(dataTime[i]), "fitted acc", fitted.acc(dataTime[i])));
    }
    filter.update(0.01, dataVal[i]);
    plot.add(Leph::VectorLabel("t", dataTime[i], "val", dataVal[i], "filtered", filter.pos(), "target vel",
                               dataVelTarget[i], "filtered vel", filter.vel(), "target acc", dataAccTarget[i],
                               "filtered acc", filter.acc()));
  }
  plot.plot("t", "all").render();
}

int main()
{
  // testFilterTuning();
  // return 0;
  testFilterVsSpline();
  return 0;

  Leph::CubicSpline spline;
  spline.addPoint(0.0, 0.0);
  spline.addPoint(1.0, 1.0);
  spline.addPoint(2.0, -2.0, 5.0);
  spline.addPoint(3.0, 0.0);

  Leph::EncoderFilter filter1(0.0, 0.0, 0.0, 1.0);
  filter1.setObservationVar(1.0);
  filter1.setPredictionVar(1.0);
  Leph::EncoderFilter filter2(0.0, 0.0, 0.0, 0.1);
  filter2.setObservationVar(0.001);
  filter2.setPredictionVar(Eigen::Vector3d(0.001, 0.1, 1.0));

  /*
  Leph::MatrixLabel logs;
  logs.load("../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-45.log");
  logs.plot()
      .plot("t", "pos:left_knee")
      .plot("t", "goal:left_knee")
      .plot("t", "pos:left_hip_roll")
      .plot("t", "goal:left_hip_roll")
      .render();
  */

  std::vector<double> goalData;
  std::vector<double> readData;
  std::vector<double> timeData;
  Leph::FittedSpline fitted;
  double readVal = 0.0;
  for (double t = 0; t <= 3.0; t += 0.02)
  {
    double goalVal = spline.pos(t);
    readVal = goalVal;
    // readVal = 0.95*readVal + 0.05*goalVal + 0.1*uniformDist(generator);
    goalData.push_back(goalVal);
    readData.push_back(readVal);
    timeData.push_back(t);
    fitted.addPoint(t, readVal);
  }
  fitted.fittingGlobal(3, 6);

  Leph::Plot plot;
  for (size_t i = 0; i < timeData.size(); i++)
  {
    filter1.update(0.02, readData[i]);
    filter2.update(0.02, readData[i]);
    plot.add(Leph::VectorLabel("t", timeData[i],
                               //"goal", goalData[i],
                               //"goal vel", spline.vel(timeData[i]),
                               //"read", readData[i],
                               //"fitted", fitted.pos(timeData[i]),
                               "fitted vel", fitted.vel(timeData[i]), "filtered 1 vel", filter1.vel(), "filtered 2 vel",
                               filter2.vel(), "filtered 1", filter1.pos(), "filtered 2", filter2.pos()));
  }
  plot.plot("t", "all").render();

  return 0;
}
