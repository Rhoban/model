#include <iostream>
#include <Eigen/Dense>
#include "EigenLevenbergMarquardt/LevenbergMarquardt"
#include "Types/VectorLabel.hpp"
#include "Plot/Plot.hpp"

double targetFunction(double x, double y)
{
  return x * x + y * y + cos(4.0 * x) + 0.5;
}
double targetFunctionDiffX(double x, double y)
{
  (void)y;
  return 2.0 * x - 4.0 * sin(4.0 * x);
}
double targetFunctionDiffY(double x, double y)
{
  (void)x;
  return 2.0 * y;
}

struct TestFunctor : public Eigen::DenseFunctor<double>
{
  int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
  {
    fvec(0) = targetFunction(x(0), x(1));
    fvec(1) = 0.0;
    return 0;
  }
  int df(const Eigen::VectorXd& x, Eigen::MatrixXd& fjac) const
  {
    fjac(0, 0) = targetFunctionDiffX(x(0), x(1));
    fjac(0, 1) = targetFunctionDiffY(x(0), x(1));
    fjac(1, 0) = 0.0;
    fjac(1, 1) = 0.0;
    return 0;
  }

  int inputs() const
  {
    return 2;
  }
  int values() const
  {
    return 2;
  }

  void gradientProjection(const Eigen::VectorXd& state, Eigen::VectorXd& gradient)
  {
    if (state(1) + gradient(1) < 0.6)
    {
      gradient(1) = 0.6 - state(1);
    }
  }
};

int main()
{
  Leph::Plot plot;

  for (double x = -1.0; x <= 1.0; x += 0.05)
  {
    for (double y = -1.0; y <= 1.0; y += 0.05)
    {
      double val = targetFunction(x, y);
      plot.add(Leph::VectorLabel("x", x, "y", y, "target", val));
    }
  }

  TestFunctor functor;
  Eigen::LevenbergMarquardt<TestFunctor> lm(functor);
  Eigen::VectorXd init(2);
  init(0) = 0.01;
  init(1) = 0.8;
  plot.add(Leph::VectorLabel("x", init(0), "y", init(1), "step", targetFunction(init(0), init(1))));

  Eigen::LevenbergMarquardtSpace::Status st = lm.minimizeInit(init);
  std::cout << st << std::endl;
  do
  {
    st = lm.minimizeOneStep(init);
    if (st != Eigen::LevenbergMarquardtSpace::Running)
      break;
    std::cout << "State " << init(0) << " " << init(1) << " ==> " << targetFunction(init(0), init(1)) << std::endl;
    plot.add(Leph::VectorLabel("x", init(0), "y", init(1), "step", targetFunction(init(0), init(1))));
  } while (st == Eigen::LevenbergMarquardtSpace::Running);
  std::cout << st << std::endl;

  plot.plot("x", "y", "target", Leph::Plot::Points, "target").plot("x", "y", "step", Leph::Plot::LinesPoints).render();

  return 0;
}
