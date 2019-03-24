#ifndef LEPH_HUMANOIDFIXEDPRESSUREMODEL_HPP
#define LEPH_HUMANOIDFIXEDPRESSUREMODEL_HPP

#include <string>
#include <Eigen/Dense>
#include "Types/VectorLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"

namespace Leph
{
/**
 * HumanoidFixedPressureModel
 *
 * Extends HumanoidFixedModel with
 * foot pressure information. Foot pressure
 * is used to set the supporting foot
 */
class HumanoidFixedPressureModel : public HumanoidFixedModel
{
public:
  /**
   * Initialization with given model type.
   * If inertia data and name are not empty,
   * given inertia override default model data.
   * If geometry data and name are not empty,
   * given geometry override default model data.
   */
  HumanoidFixedPressureModel(RobotType type, const Eigen::MatrixXd& inertiaData = Eigen::MatrixXd(),
                             const std::map<std::string, size_t>& inertiaName = {},
                             const Eigen::MatrixXd& geometryData = Eigen::MatrixXd(),
                             const std::map<std::string, size_t>& geometryName = {});

  /**
   * Set current pressure state
   */
  void setPressure(const Leph::VectorLabel& vect);
  void setPressure(double weight, double leftRatio, double rightRatio, double leftX, double leftY, double rightX,
                   double rightY);

  /**
   * @Inherit
   * Choose support foot according to
   * foot pressure
   */
  void updateBase() override;

  /**
   * Return total, left and right weight in kilograms,
   * left and right foot weight ratio between
   * 0 and 1.
   */
  double pressureWeight() const;
  double pressureLeftWeight() const;
  double pressureRightWeight() const;
  double pressureLeftRatio() const;
  double pressureRightRatio() const;

  /**
   * Compute and return the center of pressure
   * for left, right and both foot in given frame
   * (in meters)
   */
  Eigen::Vector3d centerOfPressureLeft(const std::string& frame);
  Eigen::Vector3d centerOfPressureRight(const std::string& frame);
  Eigen::Vector3d centerOfPressure(const std::string& frame);

private:
  /**
   * Pressure state.
   * _weight is total measure weight in kilograms.
   * _leftRatio and _rightRatio are percentage
   * of measure weight on meft and right foot.
   * _leftCOP and _rightCOP are left and right foot
   * center of pressure with respect to foot center
   * in meters. (Z component is zero)
   */
  double _weight;
  double _leftRatio;
  double _rightRatio;
  Eigen::Vector3d _leftCOP;
  Eigen::Vector3d _rightCOP;

  /**
   * Print warning message if pressure inputs
   * are not valid
   */
  void checkPressure();
};

}  // namespace Leph

#endif
