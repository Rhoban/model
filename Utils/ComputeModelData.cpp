#include "Utils/ComputeModelData.h"
#include "Utils/AxisAngle.h"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Spline/FittedSpline.hpp"

namespace Leph
{
void ComputeModelData(MatrixLabel& logs, RobotType type)
{
  // Humanoid model for read position and goal
  HumanoidFixedModel modelGoal(type);
  HumanoidFixedPressureModel modelPos(type);
  // Loop on all records
  for (size_t i = 0; i < logs.size(); i++)
  {
    // Assign DOF
    VectorLabel vectDOFGoal = logs[i].extract("goal").rename("goal", "");
    modelGoal.get().setDOF(vectDOFGoal, false);
    VectorLabel vectDOFPos = logs[i].extract("pos").rename("pos", "");
    modelPos.get().setDOF(vectDOFPos, false);
    // Assign pressure
    modelPos.setPressure(logs[i]("pressure:weight"), logs[i]("pressure:left_ratio"), logs[i]("pressure:right_ratio"),
                         logs[i]("pressure:left_x"), logs[i]("pressure:left_y"), logs[i]("pressure:right_x"),
                         logs[i]("pressure:right_y"));
    // Contraint the model on the ground
    modelGoal.updateBase();
    modelPos.updateBase();
    // Set IMU data for read model state
    modelPos.setOrientation(Eigen::AngleAxisd(logs[i]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
                                Eigen::AngleAxisd(logs[i]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
                            false);
    // Append base DOF
    logs[i].append("pos:base_pitch", modelPos.get().getDOF("base_pitch"));
    logs[i].append("pos:base_roll", modelPos.get().getDOF("base_roll"));
    // Compute joint errors
    VectorLabel vectDOFErrors = vectDOFPos;
    vectDOFErrors.subOp(vectDOFGoal);
    logs[i].mergeUnion(vectDOFErrors.rename("", "error"));
    // Get support foot
    std::string supportGoalName;
    std::string supportPosName;
    if (modelGoal.getSupportFoot() == HumanoidFixedModel::LeftSupportFoot)
    {
      supportGoalName = "left";
    }
    else
    {
      supportGoalName = "right";
    }
    if (modelPos.getSupportFoot() == HumanoidFixedModel::LeftSupportFoot)
    {
      supportPosName = "left";
    }
    else
    {
      supportPosName = "right";
    }
    logs[i].append("goal_model:is_left_support_foot", (supportGoalName == "left"));
    logs[i].append("pos_model:is_left_support_foot", (supportPosName == "left"));
    // Compute model goal
    logs[i].append("goal_model:trunk_pos_x", modelGoal.get().position("trunk", supportGoalName + "_foot_tip").x());
    logs[i].append("goal_model:trunk_pos_y", modelGoal.get().position("trunk", supportGoalName + "_foot_tip").y());
    logs[i].append("goal_model:trunk_pos_z", modelGoal.get().position("trunk", supportGoalName + "_foot_tip").z());
    logs[i].append("goal_model:foot_pos_x",
                   modelGoal.get().position("right_foot_tip", supportGoalName + "_foot_tip").x());
    logs[i].append("goal_model:foot_pos_y",
                   modelGoal.get().position("right_foot_tip", supportGoalName + "_foot_tip").y());
    logs[i].append("goal_model:foot_pos_z",
                   modelGoal.get().position("right_foot_tip", supportGoalName + "_foot_tip").z());
    logs[i].append("goal_model:trunk_axis_x",
                   MatrixToAxis(modelGoal.get().orientation("trunk", supportGoalName + "_foot_tip").transpose()).x());
    logs[i].append("goal_model:trunk_axis_y",
                   MatrixToAxis(modelGoal.get().orientation("trunk", supportGoalName + "_foot_tip").transpose()).y());
    logs[i].append("goal_model:trunk_axis_z",
                   MatrixToAxis(modelGoal.get().orientation("trunk", supportGoalName + "_foot_tip").transpose()).z());
    logs[i].append(
        "goal_model:foot_axis_x",
        MatrixToAxis(modelGoal.get().orientation("right_foot_tip", supportGoalName + "_foot_tip").transpose()).x());
    logs[i].append(
        "goal_model:foot_axis_y",
        MatrixToAxis(modelGoal.get().orientation("right_foot_tip", supportGoalName + "_foot_tip").transpose()).y());
    logs[i].append(
        "goal_model:foot_axis_z",
        MatrixToAxis(modelGoal.get().orientation("right_foot_tip", supportGoalName + "_foot_tip").transpose()).z());
    logs[i].append("goal_model:com_x", modelGoal.get().centerOfMass(supportGoalName + "_foot_tip").x());
    logs[i].append("goal_model:com_y", modelGoal.get().centerOfMass(supportGoalName + "_foot_tip").y());
    logs[i].append("goal_model:com_z", modelGoal.get().centerOfMass(supportGoalName + "_foot_tip").z());
    // Compute model pos
    logs[i].append("model:trunk_pos_x", modelPos.get().position("trunk", supportPosName + "_foot_tip").x());
    logs[i].append("model:trunk_pos_y", modelPos.get().position("trunk", supportPosName + "_foot_tip").y());
    logs[i].append("model:trunk_pos_z", modelPos.get().position("trunk", supportPosName + "_foot_tip").z());
    logs[i].append("model:foot_pos_x", modelPos.get().position("right_foot_tip", supportPosName + "_foot_tip").x());
    logs[i].append("model:foot_pos_y", modelPos.get().position("right_foot_tip", supportPosName + "_foot_tip").y());
    logs[i].append("model:foot_pos_z", modelPos.get().position("right_foot_tip", supportPosName + "_foot_tip").z());
    logs[i].append("model:trunk_axis_x",
                   MatrixToAxis(modelPos.get().orientation("trunk", supportPosName + "_foot_tip").transpose()).x());
    logs[i].append("model:trunk_axis_y",
                   MatrixToAxis(modelPos.get().orientation("trunk", supportPosName + "_foot_tip").transpose()).y());
    logs[i].append("model:trunk_axis_z",
                   MatrixToAxis(modelPos.get().orientation("trunk", supportPosName + "_foot_tip").transpose()).z());
    logs[i].append(
        "model:foot_axis_x",
        MatrixToAxis(modelPos.get().orientation("right_foot_tip", supportPosName + "_foot_tip").transpose()).x());
    logs[i].append(
        "model:foot_axis_y",
        MatrixToAxis(modelPos.get().orientation("right_foot_tip", supportPosName + "_foot_tip").transpose()).y());
    logs[i].append(
        "model:foot_axis_z",
        MatrixToAxis(modelPos.get().orientation("right_foot_tip", supportPosName + "_foot_tip").transpose()).z());
    logs[i].append("model:com_x", modelPos.get().centerOfMass(supportPosName + "_foot_tip").x());
    logs[i].append("model:com_y", modelPos.get().centerOfMass(supportPosName + "_foot_tip").y());
    logs[i].append("model:com_z", modelPos.get().centerOfMass(supportPosName + "_foot_tip").z());
  }
}

}  // namespace Leph
