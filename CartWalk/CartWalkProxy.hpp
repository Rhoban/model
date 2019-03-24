#ifndef LEPH_CARTWALKPROXY_HPP
#define LEPH_CARTWALKPROXY_HPP

#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalk.h"

namespace Leph
{
/**
 * CartWalkProxy
 *
 * Clean API for Rhoban
 * CartWalk motor primitive
 */
class CartWalkProxy
{
public:
  /**
   * Initialization
   */
  CartWalkProxy();

  /**
   * Return the cyclic movement
   * phase between 0 and 1
   */
  double getPhase() const;

  /**
   * Set movement phase
   * (between 0 and 1)
   */
  void setPhase(double phase);

  /**
   * Return initialized label vector
   * for motor outputs and walk information,
   * static parameters and dynamic parameters
   * with default values
   */
  VectorLabel buildOutputs() const;
  VectorLabel buildParams() const;

  /**
   * Return label vector with minimum
   * and maximum bound for static and
   * dynamic parameters
   */
  VectorLabel buildParamsMin() const;
  VectorLabel buildParamsMax() const;

  /**
   * Return VectorLabel with typical delta
   * length scale for all parameters
   * (1/50 of max-min range)
   */
  VectorLabel buildParamsDelta() const;

  /**
   * Return a labeled vector of motor outputs
   * given time step (updating the phase) and
   * walk parameters
   */
  VectorLabel exec(double deltaTime, const VectorLabel& params);

  /**
   * Return last computed VectorLabel
   * outputs and info (walk information)
   */
  VectorLabel lastOutputs() const;

private:
  /**
   * Wrapped CartWalk
   */
  Rhoban::CartWalk _walk;

  /**
   * Last computed Outputs
   * and Info
   */
  VectorLabel _lastOutputs;
};

}  // namespace Leph

#endif
