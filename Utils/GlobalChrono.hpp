#ifndef LEPH_GLOBALCHRONO_HPP
#define LEPH_GLOBALCHRONO_HPP

#include "Utils/Chrono.hpp"

namespace Leph
{
/**
 * GlobalChrono
 *
 * Global singleton of
 * Chrono class
 */
class GlobalChrono
{
public:
  /**
   * Return global Chrono instance
   */
  inline static Chrono& get()
  {
    static Chrono chrono;
    return chrono;
  }
};

}  // namespace Leph

#endif
