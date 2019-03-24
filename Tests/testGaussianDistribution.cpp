#include <iostream>
#include <cassert>
#include "Utils/GaussianDistribution.hpp"
#include "Plot/Plot.hpp"

/**
 * Test GaussianDistribution for
 * circular dimension.
 * !!!
 * !!! Note that the current implementation
 * !!! is not really the wrapped normal
 * !!! distribution.
 * !!! Probability and fittting methods are
 * !!! not correct the more the circular
 * !!! std deviantion is high.
 * !!!
 */
void testCircularDistribution()
{
  Leph::Plot plot;
  std::default_random_engine engine;

  Eigen::VectorXd mean1(1);
  Eigen::VectorXd point1(1);
  mean1 << 3.0 * M_PI / 4.0;
  Eigen::MatrixXd cov1(1, 1);
  cov1 << 1.0;
  Eigen::VectorXi isCircular1(1);
  isCircular1 << 1;
  Leph::GaussianDistribution dist1(mean1, cov1, isCircular1);
  assert(dist1.isCircular().size() == 1);
  assert(dist1.isCircular()(0) == 1);

  for (size_t k = 0; k < 100; k++)
  {
    point1 = dist1.sample(engine);
    plot.add({
        "angle",
        point1(0),
        "points",
        0.1,
    });
  }
  for (double a = -4.0; a <= 4.0; a += 0.02)
  {
    point1 << a;
    plot.add({
        "angle",
        a,
        "proba",
        dist1.probability(point1),
        "logproba",
        dist1.logProbability(point1),
    });
  }
  plot.plot("angle", "proba").plot("angle", "points", Leph::Plot::Points, "index").render();
  plot.plot("angle", "logproba").render();
  plot.clear();

  Eigen::VectorXd mean2(2);
  Eigen::VectorXd point2(2);
  mean2 << 0.0, 3.0 * M_PI / 4.0;
  Eigen::MatrixXd cov2(2, 2);
  cov2 << 0.5, 0.4, 0.4, 0.6;
  Eigen::VectorXi isCircular2(2);
  isCircular2 << 0, 1;
  Leph::GaussianDistribution dist2(mean2, cov2, isCircular2);
  assert(dist1.isCircular().size() == 2);
  assert(dist1.isCircular()(0) == 0);
  assert(dist1.isCircular()(1) == 1);

  for (size_t k = 0; k < 500; k++)
  {
    point2 = dist2.sample(engine);
    plot.add({
        "x",
        point2(0),
        "y",
        point2(1),
        "points",
        0.1,
    });
  }
  for (double x = -4.0; x <= 4.0; x += 0.1)
  {
    for (double y = -4.0; y <= 4.0; y += 0.1)
    {
      point2 << x, y;
      plot.add({
          "x",
          x,
          "y",
          y,
          "proba",
          dist2.probability(point2),
      });
    }
  }
  plot.plot("x", "y", "proba", Leph::Plot::Points, "proba").plot("x", "y", "points", Leph::Plot::Points).render();

  // Check that large number fitting converge
  // to good mean and covariance
  std::vector<Eigen::VectorXd> points;
  for (size_t j = 0; j < 500; j++)
  {
    for (size_t k = 0; k < 1000; k++)
    {
      point2 = dist2.sample(engine);
      points.push_back(point2);
    }
    Leph::GaussianDistribution dist3;
    dist3.fit(points, isCircular2);
    std::cout << "Size=" << points.size() << " MeanError=" << (dist3.mean() - dist2.mean()).norm()
              << " CovError=" << (dist3.covariance() - dist2.covariance()).norm() << std::endl;
  }
}

int main()
{
  Leph::Plot plot;
  std::default_random_engine engine;

  Eigen::VectorXd mean1(1);
  Eigen::VectorXd point1(1);
  mean1 << 1.0;
  Eigen::MatrixXd cov1(1, 1);
  cov1 << 0.5;
  Leph::GaussianDistribution dist1(mean1, cov1);
  assert(dist1.mean().size() == 1);
  assert(dist1.mean()(0) == 1.0);
  assert(dist1.covariance().rows() == 1);
  assert(dist1.covariance().cols() == 1);
  assert(dist1.covariance()(0, 0) == 0.5);
  assert(dist1.isCircular().size() == 1);
  assert(dist1.isCircular()(0) == 0);

  for (double x = -4.0; x <= 4.0; x += 0.02)
  {
    point1 << x;
    plot.add({
        "x",
        x,
        "probability",
        dist1.probability(point1),
        "log_probability",
        dist1.logProbability(point1),
    });
  }
  for (size_t k = 0; k < 100; k++)
  {
    point1 = dist1.sample(engine);
    plot.add({
        "x",
        point1(0),
        "points",
        0.1,
    });
  }
  plot.plot("x", "probability").plot("x", "points", Leph::Plot::Points, "index").render();
  plot.plot("x", "log_probability").render();
  plot.clear();

  Eigen::VectorXd mean2(2);
  Eigen::VectorXd point2(2);
  mean2 << 0.0, 0.0;
  Eigen::MatrixXd cov2(2, 2);
  cov2 << 0.5, 0.4, 0.4, 1.5;
  Eigen::VectorXi isCircular2(2);
  isCircular2 << 0, 0;
  Leph::GaussianDistribution dist2(mean2, cov2, isCircular2);
  assert(dist2.mean().size() == 2);
  assert(dist2.mean()(0) == 0.0);
  assert(dist2.mean()(1) == 0.0);
  assert(dist2.covariance().rows() == 2);
  assert(dist2.covariance().cols() == 2);
  assert(dist2.covariance()(0, 0) == 0.5);
  assert(dist2.covariance()(0, 1) == 0.1);
  assert(dist2.covariance()(1, 0) == 0.1);
  assert(dist2.covariance()(1, 1) == 0.8);
  assert(dist2.isCircular().size() == 2);
  assert(dist2.isCircular()(0) == 0);
  assert(dist2.isCircular()(1) == 0);

  for (double x = -4.0; x <= 4.0; x += 0.1)
  {
    for (double y = -4.0; y <= 4.0; y += 0.1)
    {
      point2 << x, y;
      plot.add({
          "x",
          x,
          "y",
          y,
          "probability",
          dist2.probability(point2),
      });
    }
  }
  std::vector<Eigen::VectorXd> points;
  for (size_t k = 0; k < 1000; k++)
  {
    point2 = dist2.sample(engine);
    points.push_back(point2);
    plot.add({
        "x",
        point2(0),
        "y",
        point2(1),
        "points",
        0.05,
    });
  }
  plot.plot("x", "y", "probability", Leph::Plot::Points, "probability")
      .plot("x", "y", "points", Leph::Plot::Points)
      .render();
  plot.clear();

  // Check that large number fitting converge
  // to good mean and covariance
  for (size_t j = 0; j < 500; j++)
  {
    for (size_t k = 0; k < 1000; k++)
    {
      point2 = dist2.sample(engine);
      points.push_back(point2);
    }
    Leph::GaussianDistribution dist3;
    dist3.fit(points);
    std::cout << "Size=" << points.size() << " MeanError=" << (dist3.mean() - dist2.mean()).norm()
              << " CovError=" << (dist3.covariance() - dist2.covariance()).norm() << std::endl;
  }

  testCircularDistribution();

  return 0;
}
