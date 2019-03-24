#include <algorithm>
#include <random>
#include <stdexcept>
#include <iostream>       //TODO
#include "Plot/Plot.hpp"  //TODO
#include "LinearRegression/MedianLinearRegression.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"
#include "Utils/Combination.hpp"
#include "Utils/GeometricMedian.hpp"

namespace Leph
{
void MedianLinearRegression::clear()
{
  _inputs.clear();
  _outputs.clear();
}

size_t MedianLinearRegression::size() const
{
  return _inputs.size();
}
size_t MedianLinearRegression::dimension() const
{
  if (size() > 0)
  {
    return _inputs.front().size();
  }
  else
  {
    return 0;
  }
}

void MedianLinearRegression::add(const Vector& input, double output)
{
  if (size() > 0 && dimension() != (size_t)input.size())
  {
    throw std::logic_error("MedianLinearRegression invalid dimension");
  }

  _inputs.push_back(input);
  _outputs.push_back(output);
}

Vector MedianLinearRegression::regression()
{
  if (size() == 0 || dimension() == 0)
  {
    throw std::logic_error("MedianLinearRegression empty data");
  }

  unsigned long maxUsedCombination = 2000;
  Combination combination;

  // Compute the number of all possible combinaisons
  unsigned long combinationMaxNumber = combination.binomialCoefficient(dimension(), size());

  std::cout << "dim=" << dimension() << " size=" << size() << " combNum=" << combinationMaxNumber << std::endl;

  // Create the container of combinations
  std::vector<std::vector<size_t>> allCombinations;
  if (combinationMaxNumber > 2 * maxUsedCombination)
  {
    // Random sample
    // TODO
    throw 0;
  }
  else
  {
    // Iterate throught all possible combinations
    combination.startCombination(dimension(), size());
    std::vector<size_t> comb;
    while ((comb = combination.nextCombination()).size() > 0)
    {
      allCombinations.push_back(comb);
    }
    // Shuffle them
    std::shuffle(allCombinations.begin(), allCombinations.end(), std::default_random_engine());
    // Reduce to expected size
    while (allCombinations.size() > maxUsedCombination)
    {
      allCombinations.pop_back();
    }
  }

  Plot plot;  // TODO
  std::cout << allCombinations.size() << std::endl;
  // Compute the parameters
  GeometricMedian median;
  for (size_t i = 0; i < allCombinations.size(); i++)
  {
    // Regresse parameters for all combinations
    SimpleLinearRegression reg;
    for (size_t j = 0; j < allCombinations[i].size(); j++)
    {
      reg.add(_inputs[allCombinations[i][j]], _outputs[allCombinations[i][j]]);
    }
    median.add(reg.regression());
    double dist = (_inputs[allCombinations[i][0]] - _inputs[allCombinations[i][1]]).norm();
    plot.add(VectorLabel("cst", reg.parameters()(0), "slop", reg.parameters()(1), "dist", dist));
    std::cout << reg.parameters().transpose() << " --- " << dist << std::endl;
  }

  // Compute de median
  Vector med = median.median(-1, 100);
  std::cout << med << std::endl;
  plot.add(VectorLabel("label 0", med(0), "Med", med(1)));
  plot.add(VectorLabel("label 0", -20.0, "Target", 10.0));
  plot.plot("cst", "slop", Plot::Points, "dist").render();

  return med;
}

}  // namespace Leph
