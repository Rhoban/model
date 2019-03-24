#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
#include "Utils/FileEigen.h"
#include "Model/NamesModel.h"

namespace Leph
{
TrajectoryParameters DefaultTrajParameters()
{
  // Default trajectory parameter initialization
  //(No Optimized)
  TrajectoryParameters parameters;
  // CMA-ES parameters
  parameters.add("cmaes_max_iterations", 5000.0);
  parameters.add("cmaes_restarts", 5.0);
  parameters.add("cmaes_lambda", 100.0);
  parameters.add("cmaes_sigma", -1.0);
  parameters.add("cmaes_elitism", 0.0);
  // Fitness maximum torque yaw
  parameters.add("fitness_max_torque_yaw", 1.5);
  // Fitness maximum voltage ratio
  parameters.add("fitness_max_volt_ratio", 1.0);
  // Double support static position
  parameters.add("static_double_pos_trunk_pos_x", 0.009816280388);
  parameters.add("static_double_pos_trunk_pos_y", -0.07149996496);
  parameters.add("static_double_pos_trunk_pos_z", 0.27351975164429987);
  parameters.add("static_double_pos_trunk_axis_x", 0.0);
  parameters.add("static_double_pos_trunk_axis_y", 0.1919862181);
  parameters.add("static_double_pos_trunk_axis_z", 0.0);
  parameters.add("static_double_pos_foot_pos_x", 0.0);
  parameters.add("static_double_pos_foot_pos_y", -0.1429999995);
  parameters.add("static_double_pos_foot_pos_z", 0.0);
  parameters.add("static_double_pos_foot_axis_x", 0.0);
  parameters.add("static_double_pos_foot_axis_y", 0.0);
  parameters.add("static_double_pos_foot_axis_z", 0.0);
  // Single support static position
  parameters.add("static_single_pos_trunk_pos_x", -0.00557785331559037);
  parameters.add("static_single_pos_trunk_pos_y", -0.0115849568418458);
  parameters.add("static_single_pos_trunk_pos_z", 0.285);
  parameters.add("static_single_pos_trunk_axis_x", -0.672036398746933);
  parameters.add("static_single_pos_trunk_axis_y", 0.0743358280850477);
  parameters.add("static_single_pos_trunk_axis_z", 0.0028323027017884);
  parameters.add("static_single_pos_foot_pos_x", 0.0208647084129351);
  parameters.add("static_single_pos_foot_pos_y", -0.095);
  parameters.add("static_single_pos_foot_pos_z", 0.0591693358237435);
  parameters.add("static_single_pos_foot_axis_x", 0.0);
  parameters.add("static_single_pos_foot_axis_y", 0.0);
  parameters.add("static_single_pos_foot_axis_z", 0.0);

  return parameters;
}

TrajectoryGeneration::ScoreFunc DefaultFuncScore(const TrajectoryParameters& trajParams)
{
  return [&trajParams](double t, HumanoidFixedModel& model, const std::map<std::string, JointModel>& joints,
                       const Eigen::VectorXd& torques, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                       bool isDoubleSupport, HumanoidFixedModel::SupportFoot supportFoot,
                       std::vector<double>& data) -> double {
    (void)t;

    // Init data
    if (data.size() == 0)
    {
      //[0] Count summed
      data.push_back(0.0);
      //[1] Sum voltage
      data.push_back(0.0);
      //[2] Max ZMP
      data.push_back(0.0);
      //[3] Max voltage overload
      data.push_back(0.0);
      //[4] Max torque yaw
      data.push_back(0.0);
    }

    // Penalize foot cleats under the ground
    Eigen::Vector3d cleatLeft1 = model.get().position("left_cleat_1", "origin");
    Eigen::Vector3d cleatLeft2 = model.get().position("left_cleat_2", "origin");
    Eigen::Vector3d cleatLeft3 = model.get().position("left_cleat_3", "origin");
    Eigen::Vector3d cleatLeft4 = model.get().position("left_cleat_4", "origin");
    Eigen::Vector3d cleatRight1 = model.get().position("right_cleat_1", "origin");
    Eigen::Vector3d cleatRight2 = model.get().position("right_cleat_2", "origin");
    Eigen::Vector3d cleatRight3 = model.get().position("right_cleat_3", "origin");
    Eigen::Vector3d cleatRight4 = model.get().position("right_cleat_4", "origin");
    if (cleatLeft1.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatLeft1.z();
    }
    if (cleatLeft2.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatLeft2.z();
    }
    if (cleatLeft3.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatLeft3.z();
    }
    if (cleatLeft4.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatLeft4.z();
    }
    if (cleatRight1.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatRight1.z();
    }
    if (cleatRight2.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatRight2.z();
    }
    if (cleatRight3.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatRight3.z();
    }
    if (cleatRight4.z() < -5e-3)
    {
      return 1000.0 - 1000.0 * cleatRight4.z();
    }

    // Retrieve support and
    // flying foot names
    std::string supportName;
    std::string footName;
    if (supportFoot == HumanoidFixedModel::LeftSupportFoot)
    {
      supportName = "left_foot_tip";
      footName = "right_foot_tip";
    }
    else
    {
      supportName = "right_foot_tip";
      footName = "left_foot_tip";
    }

    // Compute the Zero Moment Point in
    // support foot frame
    Eigen::Vector3d zmp;
    if (!isDoubleSupport)
    {
      // In case of single support
      // compute the ZMP from torques
      zmp = model.zeroMomentPointSingleSupport(supportName, torques);
    }
    else
    {
      // In case of double support, compute
      // the ZMP as if it were in signe support
      //(torques needs to be recomputed)
      zmp = model.zeroMomentPoint(supportName, dq, ddq, false);
    }
    zmp.z() = 0.0;
    // Compute the ZMP error distance
    double zmpError = 0.0;
    if (isDoubleSupport)
    {
      // In case of double support, compute the distance
      // from the segment between the two foot centers
      Eigen::Vector3d footPos = model.get().position(footName, supportName);
      footPos.z() = 0.0;
      double affix = (footPos.dot(zmp)) / footPos.squaredNorm();
      if (affix <= 0.0)
      {
        affix = 0.0;
      }
      else if (affix >= 1.0)
      {
        affix = 1.0;
      }
      zmpError = (zmp - (affix * footPos)).lpNorm<Eigen::Infinity>();
    }
    else
    {
      // In single support, the error id simply the
      // distance for the support foot center
      zmpError = zmp.lpNorm<Eigen::Infinity>();
    }
    // Max ZMP error
    if (data[2] < zmpError)
    {
      data[2] = zmpError;
    }

    // Voltage
    for (const std::string& name : NamesDOF)
    {
      size_t index = model.get().getDOFIndex(name);
      double volt = fabs(joints.at(name).computeElectricTension(dq(index), ddq(index), torques(index)));
      double maxVolt = trajParams.get("fitness_max_volt_ratio") * fabs(joints.at(name).getMaxVoltage());
      double voltRatio = volt / maxVolt;
      // Maximum voltage ratio
      if (data[3] < voltRatio)
      {
        data[3] = voltRatio;
      }
      data[0] += 1.0;
      data[1] += volt;
    }

    // Support torque yaw if single support
    if (!isDoubleSupport)
    {
      double torqueSupportYaw = fabs(torques(model.get().getDOFIndex("base_yaw")));
      // Max support yaw
      if (data[4] < torqueSupportYaw)
      {
        data[4] = torqueSupportYaw;
      }
    }

    return 0.0;
  };
}

TrajectoryGeneration::EndScoreFunc DefaultFuncEndScore(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params, const Trajectories& traj, double score, std::vector<double>& data,
                       bool verbose) -> double {
    (void)params;
    (void)traj;
    // Skip if the first iteration
    // has not ended (data not initialize)
    if (data.size() != 5)
    {
      return 0.0;
    }
    // Verbose
    if (verbose)
    {
      std::cout << "MeanVolt=" << data[1] / data[0] << " MaxZMP=" << data[2] << " MaxVoltRatio=" << data[3]
                << " MaxTorqueYaw=" << data[4] << std::endl;
    }
    // Penalize high motor voltage ratio
    double costVoltRatio = data[3];
    // Penalize motor mean voltage
    double costVoltMean = (data[1] / data[0]) / 5.0;
    // Penalize max ZMP
    double costZMP = data[2] / 0.02;
    // Penalize high yaw support torque
    double costTorqueYaw = 0.0;
    if (data[4] > trajParams.get("fitness_max_torque_yaw"))
    {
      costTorqueYaw = data[4] / trajParams.get("fitness_max_torque_yaw");
    }
    if (verbose)
    {
      std::cout << "MeanVoltCost=" << costVoltMean << " MaxZMPCost=" << costZMP << " MaxVoltRatioCost=" << costVoltRatio
                << " MaxTorqueYawCost=" << costTorqueYaw << std::endl;
    }
    return costVoltMean + costZMP + costVoltRatio + costTorqueYaw;
  };
}

TrajectoryGeneration::SaveFunc DefaultFuncSave(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const std::string& filename, const Trajectories& traj, const Eigen::VectorXd& params) -> void {
    if (filename != "")
    {
      traj.exportData(filename + ".splines");
      std::cout << "****** Saving Trajectories to: " << filename + ".splines" << std::endl;
      trajParams.exportData(filename + ".params", params);
      std::cout << "****** Saving Parameters to: " << filename + ".params" << std::endl;
    }
  };
}

}  // namespace Leph
