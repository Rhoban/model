#include <iostream>
#include <cassert>
#include "Types/VectorLabel.hpp"
#include "Utils/CircularBuffer.hpp"

int main()
{
  Leph::VectorLabel vect("x", 0.0, "y", 0.0);
  Leph::CircularBuffer buffer(5);
  assert(buffer.size() == 5);
  assert(buffer.count() == 0);
  assert(buffer.dimension() == 0);
  buffer.add(vect);
  assert(buffer.size() == 5);
  assert(buffer.count() == 1);
  assert(buffer.dimension() == 2);

  vect = Leph::VectorLabel("x", 1.0, "y", 1.0);
  buffer.add(vect);
  assert(buffer.size() == 5);
  assert(buffer.count() == 2);
  assert(buffer.dimension() == 2);

  std::cout << buffer[0] << std::endl;
  std::cout << buffer[1] << std::endl;

  vect = Leph::VectorLabel("x", 2.0, "y", 2.0);
  buffer.add(vect);
  vect = Leph::VectorLabel("x", 3.0, "y", 3.0);
  buffer.add(vect);
  vect = Leph::VectorLabel("x", 4.0, "y", 4.0);
  buffer.add(vect);
  vect = Leph::VectorLabel("x", 5.0, "y", 5.0);
  buffer.add(vect);

  std::cout << buffer[0] << std::endl;
  std::cout << buffer[1] << std::endl;
  std::cout << buffer[2] << std::endl;
  std::cout << buffer[3] << std::endl;
  std::cout << buffer[4] << std::endl;

  std::cout << buffer.mean() << std::endl;
  std::cout << buffer.variance() << std::endl;

  return 0;
}
