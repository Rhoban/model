#include <iostream>
#include <fstream>
#include "Plot/Plot.hpp"
#include "Spline/FittedSpline.hpp"
#include "Spline/SplineContainer.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Model/InverseKinematics.hpp"

/**
 * Generate simple circular center of mass
 * trajectory for Sigmaban and export them
 * as fitted polynomial splines
 */
int main()
{
  // Trajectory generation config
  const double configCOMZOffset = 0.01;
  const double configTimePeriod = 2.0;
  const double configCircleRadius = 0.02;

  // Initialize instance
  Leph::Plot plot;
  Leph::HumanoidFloatingModel model(Leph::SigmabanModel);
  Leph::InverseKinematics inverseModel(model);

  // Set foot model on zero z
  model.putOnGround();
  // Add allowed degrees of freedom
  inverseModel.addDOF("left_ankle_roll");
  inverseModel.addDOF("left_ankle_pitch");
  inverseModel.addDOF("left_knee");
  inverseModel.addDOF("left_hip_roll");
  inverseModel.addDOF("left_hip_pitch");
  inverseModel.addDOF("left_hip_yaw");
  inverseModel.addDOF("right_ankle_roll");
  inverseModel.addDOF("right_ankle_pitch");
  inverseModel.addDOF("right_knee");
  inverseModel.addDOF("right_hip_roll");
  inverseModel.addDOF("right_hip_pitch");
  inverseModel.addDOF("right_hip_yaw");
  inverseModel.addDOF("base_x");
  inverseModel.addDOF("base_z");
  inverseModel.addDOF("base_y");
  // Add target constraints
  inverseModel.addTargetPosition("left_foot", "left_foot_tip");
  inverseModel.addTargetPosition("right_foot", "right_foot_tip");
  inverseModel.addTargetOrientation("left_foot", "left_foot_tip");
  inverseModel.addTargetOrientation("right_foot", "right_foot_tip");
  inverseModel.addTargetCOM();
  // Add degrees of freedom bounds
  inverseModel.setLowerBound("left_knee", 0.0);
  inverseModel.setLowerBound("right_knee", 0.0);
  // Compute initial COM
  double initCOMZOffset = model.centerOfMass("origin").z();

  // Initialize outputs Vector and fitted splines
  Leph::VectorLabel outputs = model.getDOF();
  Leph::SplineContainer<Leph::FittedSpline> splines;
  for (size_t i = 0; i < outputs.size(); i++)
  {
    splines.add(outputs.getLabel(i));
  }

  for (double t = 0; t <= configTimePeriod; t += 0.01)
  {
    // Build COM trajectory
    inverseModel.targetCOM().z() = initCOMZOffset - configCOMZOffset;
    inverseModel.targetCOM().x() = configCircleRadius * cos(2.0 * M_PI * t / configTimePeriod);
    inverseModel.targetCOM().y() = configCircleRadius * sin(2.0 * M_PI * t / configTimePeriod);

    // Compute inverse kinematics
    for (unsigned int k = 0; k < 100; k++)
    {
      inverseModel.run(0.001, 100);
    }
    // Check target error
    if (inverseModel.errorPosition("left_foot") > 0.001 || inverseModel.errorPosition("right_foot") > 0.001 ||
        inverseModel.errorCOM() > 0.001)
    {
      std::cerr << "InverseKinematics failed to converge" << std::endl;
      return -1;
    }

    // Convertion from radian to degrees
    outputs.mergeInter(model.getDOF());
    outputs.mulOp(180.0 / M_PI);

    // Save computed points for spline fitting
    for (size_t i = 0; i < outputs.size(); i++)
    {
      splines.get(outputs.getLabel(i)).addPoint(t, outputs(i));
    }

    // Plot
    plot.add(outputs);
  }
  plot.plot("index", "all").render();

  // Splines fittings
  for (size_t i = 0; i < splines.size(); i++)
  {
    splines.get(outputs.getLabel(i)).fittingPieces(0.3, true);
  }

  // Export computed splines
  splines.exportData("/tmp/appSplineGenerationCircle.splines");

  // Plot fitted splines
  for (double t = 0; t < configTimePeriod; t += 0.01)
  {
    for (size_t i = 0; i < splines.size(); i++)
    {
      outputs(i) = splines.get(outputs.getLabel(i)).pos(t);
    }
    plot.add(outputs.rename("", "fitted"));
  }
  plot.plot("index", "all").render();

  return 0;
}
