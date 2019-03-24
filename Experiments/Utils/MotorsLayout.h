#ifndef LEPH_MOTORSLAYOUT_H
#define LEPH_MOTORSLAYOUT_H

#include "Types/VectorLabel.hpp"

namespace Leph
{
/**
 * Motors sign conversion between specific robot
 * mecanical layout and SigmabanModel convention
 * (Mowgly)
 * prefix is string filter prefix
 */
inline void MotorsLayoutConversion(VectorLabel& vect, const std::string& prefix = "")
{
  vect(prefix + "right foot roll") *= -1.0;
  vect(prefix + "left foot pitch") *= -1.0;
  vect(prefix + "left knee") *= -1.0;
  vect(prefix + "left hip pitch") *= -1.0;
  vect(prefix + "left hip roll") *= -1.0;
  vect(prefix + "right hip roll") *= -1.0;
}

}  // namespace Leph

#endif
