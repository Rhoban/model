#include <iostream>
#include <vector>
#include "Spline/SplineContainer.hpp"
#include "Spline/FittedSpline.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Model/HumanoidFixedModel.hpp"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

int main(int argc, char** argv)
{
  // Check command line
  if (argc < 2)
  {
    std::cout << "Usage: ./app [Trajectories filename]" << std::endl;
    return 1;
  }
  std::string filename = std::string(argv[1]);

  // Load Trajectories
  Leph::Trajectories traj;
  traj.importData(filename);

  // Initialize DOF spline container
  Leph::SplineContainer<Leph::FittedSpline> trajDOF;
  Leph::SplineContainer<Leph::FittedSpline> trajTorque;
  for (const std::string& name : Leph::NamesDOFLeg)
  {
    trajDOF.add(name);
    trajTorque.add(name);
  }

  // Iterate over the given Trajectories
  Leph::Plot plot;
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  for (double t = traj.min(); t <= traj.max(); t += 0.01)
  {
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
    bool isIKSuccess = TrajectoriesComputeKinematics(t, traj, model, dq, ddq);
    if (!isIKSuccess)
    {
      std::cout << "IK ERROR t=" << t << std::endl;
    }
    // Compute DOF torques
    bool isDoubleSupport;
    Leph::HumanoidFixedModel::SupportFoot supportFoot;
    TrajectoriesSupportFootState(t, traj, isDoubleSupport, supportFoot);
    Eigen::VectorXd torques;
    if (isDoubleSupport)
    {
      if (supportFoot == Leph::HumanoidFixedModel::LeftSupportFoot)
      {
        torques = model.get().inverseDynamicsClosedLoop("right_foot_tip", nullptr, false, dq, ddq);
      }
      else
      {
        torques = model.get().inverseDynamicsClosedLoop("left_foot_tip", nullptr, false, dq, ddq);
      }
    }
    else
    {
      torques = model.get().inverseDynamics(dq, ddq);
    }
    // Assign DOF position and torques
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      trajDOF.get(name).addPoint(t, model.get().getDOF(name));
      trajTorque.get(name).addPoint(t, torques(model.get().getDOFIndex(name)));
      plot.add(Leph::VectorLabel("t", t, "pos:" + name, model.get().getDOF(name), "torque:" + name,
                                 torques(model.get().getDOFIndex(name))));
    }
  }

  // Compute fitting
  for (const std::string& name : Leph::NamesDOFLeg)
  {
    double maxError1 = trajDOF.get(name).fittingPolynomPieces(4, 0.25, 1.0);
    double maxError2 = trajTorque.get(name).fittingPolynomPieces(4, 0.25, 1.0);
    std::cout << "Position " << name << " max fitting error: " << maxError1 << std::endl;
    std::cout << "Torque " << name << " max fitting error: " << maxError2 << std::endl;
  }

  // Display retulting fitting
  for (double t = traj.min(); t <= traj.max(); t += 0.01)
  {
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      plot.add(Leph::VectorLabel("t", t, "fitted_pos:" + name, trajDOF.get(name).pos(t), "fitted_torque:" + name,
                                 trajTorque.get(name).pos(t)));
    }
  }
  plot.plot("t", "pos:*").plot("t", "fitted_pos:*").render();
  plot.plot("t", "torque:*").plot("t", "fitted_torque:*").render();

  return 0;
}
