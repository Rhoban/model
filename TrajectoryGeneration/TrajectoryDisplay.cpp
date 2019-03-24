#include <map>
#include "TrajectoryGeneration/TrajectoryDisplay.h"
#include "Plot/Plot.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"
#include "Utils/FileModelParameters.h"

namespace Leph
{
void TrajectoriesDisplay(const Trajectories& traj, RobotType type, const std::string& modelParamsPath)
{
  // Plot Cartesian trajectory
  Plot plot;
  for (double t = traj.min(); t <= traj.max(); t += 0.01)
  {
    plot.add(VectorLabel("time", t, "is_double_support", traj.get("is_double_support").pos(t), "is_left_support_foot",
                         traj.get("is_left_support_foot").pos(t)));
    if (traj.exist("base_pitch"))
    {
      plot.add(VectorLabel("time", t, "base_pitch", traj.get("base_pitch").pos(t), "base_pitch_vel",
                           traj.get("base_pitch").vel(t)));
    }
    if (traj.exist("base_roll"))
    {
      plot.add(VectorLabel("time", t, "base_roll", traj.get("base_roll").pos(t), "base_roll_vel",
                           traj.get("base_roll").vel(t)));
    }
  }
  plot.plot("time", "all").render();
  plot.clear();
  for (double t = traj.min(); t <= traj.max(); t += 0.01)
  {
    plot.add(VectorLabel("time", t, "trunk_pos_x", traj.get("trunk_pos_x").pos(t), "trunk_pos_y",
                         traj.get("trunk_pos_y").pos(t), "trunk_pos_z", traj.get("trunk_pos_z").pos(t),
                         "trunk_pos_x_vel", traj.get("trunk_pos_x").vel(t), "trunk_pos_y_vel",
                         traj.get("trunk_pos_y").vel(t), "trunk_pos_z_vel", traj.get("trunk_pos_z").vel(t)));
  }
  plot.plot("time", "all").render();
  plot.clear();
  for (double t = traj.min(); t <= traj.max(); t += 0.01)
  {
    plot.add(VectorLabel("time", t, "trunk_axis_x", traj.get("trunk_axis_x").pos(t), "trunk_axis_y",
                         traj.get("trunk_axis_y").pos(t), "trunk_axis_z", traj.get("trunk_axis_z").pos(t),
                         "trunk_axis_x_vel", traj.get("trunk_axis_x").vel(t), "trunk_axis_y_vel",
                         traj.get("trunk_axis_y").vel(t), "trunk_axis_z_vel", traj.get("trunk_axis_z").vel(t)));
  }
  plot.plot("time", "all").render();
  plot.clear();
  for (double t = traj.min(); t <= traj.max(); t += 0.01)
  {
    plot.add(VectorLabel("time", t, "foot_pos_x", traj.get("foot_pos_x").pos(t), "foot_pos_y",
                         traj.get("foot_pos_y").pos(t), "foot_pos_z", traj.get("foot_pos_z").pos(t), "foot_pos_x_vel",
                         traj.get("foot_pos_x").vel(t), "foot_pos_y_vel", traj.get("foot_pos_y").vel(t),
                         "foot_pos_z_vel", traj.get("foot_pos_z").vel(t)));
  }
  plot.plot("time", "all").render();
  plot.clear();
  for (double t = traj.min(); t <= traj.max(); t += 0.01)
  {
    plot.add(VectorLabel("time", t, "foot_axis_x", traj.get("foot_axis_x").pos(t), "foot_axis_y",
                         traj.get("foot_axis_y").pos(t), "foot_axis_z", traj.get("foot_axis_z").pos(t),
                         "foot_axis_x_vel", traj.get("foot_axis_x").vel(t), "foot_axis_y_vel",
                         traj.get("foot_axis_y").vel(t), "foot_axis_z_vel", traj.get("foot_axis_z").vel(t)));
  }
  plot.plot("time", "all").render();
  plot.clear();
  // Load model parameters
  Eigen::MatrixXd jointData;
  std::map<std::string, size_t> jointName;
  Eigen::MatrixXd inertiaData;
  std::map<std::string, size_t> inertiaName;
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  if (modelParamsPath != "")
  {
    ReadModelParameters(modelParamsPath, jointData, jointName, inertiaData, inertiaName, geometryData, geometryName);
  }
  // Joint Model for each DOF
  std::map<std::string, JointModel> jointModels;
  for (const std::string& name : NamesDOF)
  {
    jointModels[name] = JointModel();
    if (modelParamsPath != "" && jointName.count(name) > 0)
    {
      jointModels[name].setParameters(jointData.row(jointName.at(name)).transpose());
    }
  }
  // Sigmaban fixed model
  HumanoidFixedModel model(type, inertiaData, inertiaName, geometryData, geometryName);
  // Display Trajectory
  ModelViewer viewer(1200, 900);
  Scheduling scheduling;
  scheduling.setFrequency(50.0);
  double t = traj.min();
  bool isLoop = false;
  double sumTorques = 0.0;
  double sumZMP = 0.0;
  double maxZMP = -1.0;
  double maxTorqueSupportYaw = -1.0;
  double maxVolt = -1.0;
  double minBoundIKDistance = 1000.0;
  double waiting = 0.0;
  while (viewer.update())
  {
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
    double boundIKDistance = 0.0;
    bool isIKSuccess = TrajectoriesComputeKinematics(t, traj, model, dq, ddq, &boundIKDistance);
    if (minBoundIKDistance > boundIKDistance)
    {
      minBoundIKDistance = boundIKDistance;
    }
    if (!isIKSuccess)
    {
      std::cout << "IK ERROR t=" << t << " boundIKDistance=" << boundIKDistance << std::endl;
      t += 0.01;
      continue;
    }
    Eigen::VectorXd positions = model.get().getDOFVect();
    // Compute DOF torques
    bool isDoubleSupport;
    HumanoidFixedModel::SupportFoot supportFoot;
    TrajectoriesSupportFootState(t, traj, isDoubleSupport, supportFoot);
    Eigen::VectorXd torques;
    if (isDoubleSupport)
    {
      if (supportFoot == HumanoidFixedModel::LeftSupportFoot)
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
    // Compute ZMP
    Eigen::Vector3d zmp = Eigen::Vector3d::Zero();
    zmp = model.zeroMomentPoint("origin", dq, ddq, false);
    zmp.z() = 0.0;
    viewer.addTrackedPoint(zmp, ModelViewer::Yellow);
    // Compute CoM
    Eigen::Vector3d com = model.get().centerOfMass("origin");
    com.z() = 0.0;
    // Retrieve torque on yaw support foot
    double torqueSupportYaw = torques(model.get().getDOFIndex("base_yaw"));
    // Disable base DOF
    torques(model.get().getDOFIndex("base_x")) = 0.0;
    torques(model.get().getDOFIndex("base_y")) = 0.0;
    torques(model.get().getDOFIndex("base_z")) = 0.0;
    torques(model.get().getDOFIndex("base_yaw")) = 0.0;
    torques(model.get().getDOFIndex("base_pitch")) = 0.0;
    torques(model.get().getDOFIndex("base_roll")) = 0.0;
    // Display ZMP and trunk/foot trajectory
    viewer.addTrackedPoint(com, ModelViewer::Red);
    viewer.addTrackedPoint(model.get().position("trunk", "origin"), ModelViewer::Purple);
    viewer.addTrackedPoint(model.get().position("right_foot_tip", "origin"), ModelViewer::Cyan);
    // Model display
    ModelDraw(model.get(), viewer);
    // Scheduling
    scheduling.wait();
    if (waiting > 0.0)
    {
      waiting -= 0.01;
    }
    else
    {
      if (t >= traj.max())
      {
        t = traj.min();
        isLoop = true;
        // waiting = 0.5;
      }
      else
      {
        t += 0.01;
      }
    }
    if (!isLoop)
    {
      VectorLabel vect;
      vect.append("t", t);
      vect.append("zmp_x", zmp.x());
      vect.append("zmp_y", zmp.y());
      vect.append("torque_support_yaw", torqueSupportYaw);
      for (const std::string& name : NamesDOFLegLeft)
      {
        size_t index = model.get().getDOFIndex(name);
        // Compute voltage
        double volt = jointModels.at(name).computeElectricTension(dq(index), ddq(index), torques(index));
        if (maxVolt < 0.0 || maxVolt < volt)
        {
          maxVolt = volt;
        }
        // Plot
        vect.append("left_torque:" + name, torques(index));
        vect.append("left_volt:" + name, volt);
        vect.append("left_q:" + name, positions(index));
        vect.append("left_dq:" + name, dq(index));
        vect.append("left_ddq:" + name, ddq(index));
      }
      for (const std::string& name : NamesDOFLegRight)
      {
        size_t index = model.get().getDOFIndex(name);
        // Compute voltage
        double volt = jointModels.at(name).computeElectricTension(dq(index), ddq(index), torques(index));
        if (maxVolt < 0.0 || maxVolt < volt)
        {
          maxVolt = volt;
        }
        // Plot
        vect.append("right_torque:" + name, torques(index));
        vect.append("right_volt:" + name, volt);
        vect.append("right_q:" + name, positions(index));
        vect.append("right_dq:" + name, dq(index));
        vect.append("right_ddq:" + name, ddq(index));
      }
      plot.add(vect);
      sumTorques += 0.01 * torques.norm();
      sumZMP += 0.01 * zmp.norm();
      if (maxZMP < 0.0 || maxZMP < zmp.lpNorm<Eigen::Infinity>())
      {
        maxZMP = zmp.lpNorm<Eigen::Infinity>();
      }
      if (maxTorqueSupportYaw < 0.0 || maxTorqueSupportYaw < fabs(torqueSupportYaw))
      {
        maxTorqueSupportYaw = fabs(torqueSupportYaw);
      }
    }
  }
  // Display dynamics info
  std::cout << "Time length: " << traj.max() - traj.min() << std::endl;
  std::cout << "Minimum IK Bound Distance: " << minBoundIKDistance << std::endl;
  std::cout << "Mean Torques Norm: " << sumTorques << std::endl;
  std::cout << "Mean ZMP Norm: " << sumZMP << std::endl;
  std::cout << "Max ZMP: " << maxZMP << std::endl;
  std::cout << "Max TorqueSupportYaw: " << maxTorqueSupportYaw << std::endl;
  std::cout << "Max Volt: " << maxVolt << std::endl;
  // Plot DOF torques and ZMP
  plot.plot("t", "left_torque:*").render();
  plot.plot("t", "right_torque:*").render();
  plot.plot("t", "left_volt:*").render();
  plot.plot("t", "right_volt:*").render();
  plot.plot("t", "left_q:*").render();
  plot.plot("t", "right_q:*").render();
  plot.plot("t", "left_dq:*").render();
  plot.plot("t", "right_dq:*").render();
  plot.plot("t", "left_ddq:*").render();
  plot.plot("t", "right_ddq:*").render();
  plot.plot("t", "zmp_x").plot("t", "zmp_y").render();
  plot.plot("t", "torque_support_yaw").render();
}

}  // namespace Leph
