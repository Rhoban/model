#include <iostream>
#include <chrono>
#include <fstream>
#include "Types/VectorLabel.hpp"

/**
 * Return current time in milliseconds
 * (Relative to system start)
 */
unsigned long now()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main()
{
  double smooth = 0.8;
  double timer1 = 0.0;
  double timer2 = 0.0;
  double timer3 = 0.0;
  double timer4 = 0.0;
  double timer5 = 0.0;
  double timer6 = 0.0;
  // Looping for averaging
  for (size_t k = 0; k < 100; k++)
  {
    unsigned long begin;
    unsigned long end;
    // Append 1
    begin = now();
    Leph::VectorLabel vect1;
    for (size_t i = 0; i < 100; i++)
    {
      std::ostringstream oss;
      oss << "vect1:" << 2 * i;
      vect1.append(oss.str(), 0.0);
    }
    end = now();
    timer1 = smooth * timer1 + (1.0 - smooth) * (end - begin);
    // Append 2
    begin = now();
    Leph::VectorLabel vect2;
    for (size_t i = 0; i < 100; i++)
    {
      std::ostringstream oss;
      oss << "vect1:" << i;
      vect2.append(oss.str(), 0.0);
    }
    end = now();
    timer2 = smooth * timer2 + (1.0 - smooth) * (end - begin);
    // Copy
    begin = now();
    Leph::VectorLabel vect3 = vect1;
    end = now();
    timer3 = smooth * timer3 + (1.0 - smooth) * (end - begin);
    // Merging
    begin = now();
    vect2.mergeUnion(vect1);
    end = now();
    timer4 = smooth * timer4 + (1.0 - smooth) * (end - begin);
    // Inter
    begin = now();
    vect3.mergeInter(vect2);
    end = now();
    timer5 = smooth * timer5 + (1.0 - smooth) * (end - begin);
    // Add
    begin = now();
    vect1.addOp(vect2, "vect1");
    end = now();
    timer6 = smooth * timer6 + (1.0 - smooth) * (end - begin);
  }
  std::cout << "Append1 : " << timer1 << std::endl;
  std::cout << "Append2 : " << timer2 << std::endl;
  std::cout << "Copy    : " << timer3 << std::endl;
  std::cout << "Merging : " << timer4 << std::endl;
  std::cout << "Inter   : " << timer5 << std::endl;
  std::cout << "AddOp   : " << timer6 << std::endl;

  return 0;
}
