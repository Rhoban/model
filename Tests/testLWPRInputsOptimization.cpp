#include <iostream>
#include <random>
#include "Types/VectorLabel.hpp"
#include "Types/MatrixLabel.hpp"
#include "Plot/Plot.hpp"
#include "Utils/LWPRInputsOptimization.hpp"

std::mt19937 generator;

static Leph::VectorLabel randomPoint(bool reset)
{
  static double x = 0.0;
  static double y = 0.0;
  static double dx = 0.0;
  static double dy = 0.0;
  if (reset)
  {
    x = 0.0;
    y = 0.0;
    dx = 0.0;
    dy = 0.0;
  }

  std::normal_distribution<double> normalDist(0.0, 1.0);
  double w = normalDist(generator);
  double ddx = 0.1 * normalDist(generator);
  double ddy = 0.1 * normalDist(generator);

  dx = 0.8 * dx + 0.2 * ddx;
  dy = 0.8 * dy + 0.2 * ddy;
  x += dx;
  y += dy;
  double z = 0.1 * sin(x) * x * x;

  Leph::VectorLabel vect;
  vect.append("w", w);
  vect.append("x", x);
  vect.append("y", y);
  vect.append("z", z);
  return vect;
}

static void computeFunction(Leph::MatrixLabel& seq)
{
  std::normal_distribution<double> normalDist(0.0, 1.0);
  for (size_t i = 0; i < seq.size(); i++)
  {
    double f;
    /*
    if (i < 5) {
        f = sin(seq[i]("x"))*0;
    } else {
        f = sin(seq[i]("x"))*(seq[i-3]("y")-seq[i-5]("y"));
    }
    */
    if (i < 2)
    {
      f = 1.0;
    }
    else
    {
      // f = 2.0*(seq[i-2]("x") - seq[i-6]("x"));
      // f = 2.0*(seq[i-2]("x")-seq[i-6]("x")) - 2.0;
      f = 2.0 * sin(2.0 * seq[i - 2]("x")) + 3.0 * seq[i - 2]("y") + 1.0;
    }
    seq[i].append("f", f);
  }
}

int main()
{
  // Build sequences of data
  std::vector<Leph::MatrixLabel> seqs;
  for (size_t i = 0; i <= 50; i++)
  {
    seqs.push_back(Leph::MatrixLabel());
    randomPoint(true);
    for (size_t j = 0; j < 100; j++)
    {
      Leph::VectorLabel point;
      point = randomPoint(false);
      seqs.back().append(point);
    }
    computeFunction(seqs.back());
  }
  // Merge all sequences for ploting
  Leph::MatrixLabel merged;
  for (size_t i = 0; i < seqs.size(); i++)
  {
    for (size_t j = 0; j < seqs[i].size(); j++)
    {
      merged.append(seqs[i][j]);
    }
  }
  // merged.plot().plot("x", "y", "f", Leph::Plot::LinesPoints, "index").render();
  // merged.plot().plot("z", "y", "f", Leph::Plot::LinesPoints, "index").render();

  // Learning
  Leph::LWPRInputsOptimization optim(0.5, 1);
  optim.setOuput("f");
  // optim.setInputs({"w"});
  // optim.setInputs({"x", "y"});
  optim.setInputs({ "w", "x", "y", "z" });
  for (size_t i = 0; i < seqs.size(); i++)
  {
    optim.addSequence(seqs[i]);
  }
  optim.optimize();

  return 0;
}
