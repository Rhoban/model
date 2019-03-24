#ifndef LEPH_FALLDETECTORCONCEPT_HPP
#define LEPH_FALLDETECTORCONCEPT_HPP

#include "TimeSeries/Concept.hpp"

namespace Leph
{
/**
 * FallDetectorConcept
 *
 * Implement robot fall detection
 * from IMU data.
 * Inputs:
 * 0: sensor_pitch
 * 1: sensor_roll
 * Outputs:
 * 0: is_fallen
 */
class FallDetectorConcept : public Concept
{
public:
  /**
   * Inherit Concept
   */
  virtual std::string name() const override;
  virtual size_t inputSize() const override;
  virtual size_t outputSize() const override;

  /**
   * Inherit Optimize
   */
  virtual size_t parameterSize() const override;
  virtual Leph::MetaParameter defaultParameter(size_t index) const override;

protected:
  /**
   * Inherit Concept
   */
  virtual bool doCompute(double time) override;
};

}  // namespace Leph

#endif
