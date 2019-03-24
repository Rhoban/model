#ifndef LEPH_FOOTSTEPINTEGRATORCONCEPT_HPP
#define LEPH_FOOTSTEPINTEGRATORCONCEPT_HPP

#include "TimeSeries/Concept.hpp"

namespace Leph
{
/**
 * FootStepIntegratorConcept
 *
 * Implement an position intregration
 * from delta inputs with respect to footstep.
 * Inputs:
 * 0: is_support_foot_left
 * 1: delta_x
 * 2: delta_y
 * 3: delta_theta
 * Outputs:
 * 0: pos_x
 * 1: pos_y
 * 2: pos_theta
 */
class FootStepIntegratorConcept : public Concept
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
