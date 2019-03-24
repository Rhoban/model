#include <limits>
#include <iostream>
#include <cassert>
#include "Utils/Combination.hpp"

int main()
{
  Leph::Combination combination;
  assert(combination.binomialCoefficient(3, 5) == 10);
  assert(combination.binomialCoefficient(3, 4) == 4);
  assert(combination.binomialCoefficient(10, 20) == 184756);

  int count = 0;
  combination.startCombination(3, 5);
  std::vector<size_t> comb;
  while ((comb = combination.nextCombination()).size() > 0)
  {
    for (size_t i = 0; i < comb.size(); i++)
    {
      std::cout << comb[i] << " ";
    }
    std::cout << std::endl;
    count++;
  }
  assert(count == 10);

  std::cout << combination.binomialCoefficient(15, 100) << std::endl;
  std::cout << std::numeric_limits<unsigned long>::max() << std::endl;
  try
  {
    std::cout << combination.binomialCoefficient(18, 100) << std::endl;
    assert(false);
  }
  catch (...)
  {
    std::cout << "Exception Overflow OK" << std::endl;
  }

  return 0;
}
