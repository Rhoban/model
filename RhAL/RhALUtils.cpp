#include "RhAL/RhALUtils.h"
#include "Model/NamesModel.h"

namespace Leph
{
void RhALWriteStateGoal(RhAL::StandardManager& manager, const HumanoidModel& model, bool writeLegs, bool writeArms,
                        bool writeHead, bool smoothing)
{
  if (writeLegs && !smoothing)
  {
    manager.dev<RhAL::DXL>("left_hip_yaw").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_hip_yaw"));
    manager.dev<RhAL::DXL>("left_hip_pitch").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_hip_pitch"));
    manager.dev<RhAL::DXL>("left_hip_roll").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_hip_roll"));
    manager.dev<RhAL::DXL>("left_knee").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_knee"));
    manager.dev<RhAL::DXL>("left_ankle_pitch").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_ankle_pitch"));
    manager.dev<RhAL::DXL>("left_ankle_roll").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_ankle_roll"));
    manager.dev<RhAL::DXL>("right_hip_yaw").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_hip_yaw"));
    manager.dev<RhAL::DXL>("right_hip_pitch").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_hip_pitch"));
    manager.dev<RhAL::DXL>("right_hip_roll").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_hip_roll"));
    manager.dev<RhAL::DXL>("right_knee").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_knee"));
    manager.dev<RhAL::DXL>("right_ankle_pitch").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_ankle_pitch"));
    manager.dev<RhAL::DXL>("right_ankle_roll").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_ankle_roll"));
  }
  if (writeArms && !smoothing)
  {
    manager.dev<RhAL::DXL>("left_shoulder_pitch").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_shoulder_pitch"));
    manager.dev<RhAL::DXL>("left_shoulder_roll").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_shoulder_roll"));
    manager.dev<RhAL::DXL>("left_elbow").goalPosition() = RhAL::Rad2Deg(model.getDOF("left_elbow"));
    manager.dev<RhAL::DXL>("right_shoulder_pitch").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_shoulder_pitch"));
    manager.dev<RhAL::DXL>("right_shoulder_roll").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_shoulder_roll"));
    manager.dev<RhAL::DXL>("right_elbow").goalPosition() = RhAL::Rad2Deg(model.getDOF("right_elbow"));
  }
  if (writeHead && !smoothing)
  {
    manager.dev<RhAL::DXL>("head_pitch").goalPosition() = RhAL::Rad2Deg(model.getDOF("head_pitch"));
    manager.dev<RhAL::DXL>("head_yaw").goalPosition() = RhAL::Rad2Deg(model.getDOF("head_yaw"));
  }
  // Go with smoothing enable
  if (writeLegs && smoothing)
  {
    manager.dev<RhAL::DXL>("left_hip_yaw").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_hip_yaw")), 1.0);
    manager.dev<RhAL::DXL>("left_hip_pitch").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_hip_pitch")), 1.0);
    manager.dev<RhAL::DXL>("left_hip_roll").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_hip_roll")), 1.0);
    manager.dev<RhAL::DXL>("left_knee").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_knee")), 1.0);
    manager.dev<RhAL::DXL>("left_ankle_pitch")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_ankle_pitch")), 1.0);
    manager.dev<RhAL::DXL>("left_ankle_roll")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_ankle_roll")), 1.0);
    manager.dev<RhAL::DXL>("right_hip_yaw").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_hip_yaw")), 1.0);
    manager.dev<RhAL::DXL>("right_hip_pitch")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_hip_pitch")), 1.0);
    manager.dev<RhAL::DXL>("right_hip_roll").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_hip_roll")), 1.0);
    manager.dev<RhAL::DXL>("right_knee").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_knee")), 1.0);
    manager.dev<RhAL::DXL>("right_ankle_pitch")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_ankle_pitch")), 1.0);
    manager.dev<RhAL::DXL>("right_ankle_roll")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_ankle_roll")), 1.0);
  }
  if (writeArms && smoothing)
  {
    manager.dev<RhAL::DXL>("left_shoulder_pitch")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_shoulder_pitch")), 1.0);
    manager.dev<RhAL::DXL>("left_shoulder_roll")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_shoulder_roll")), 1.0);
    manager.dev<RhAL::DXL>("left_elbow").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("left_elbow")), 1.0);
    manager.dev<RhAL::DXL>("right_shoulder_pitch")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_shoulder_pitch")), 1.0);
    manager.dev<RhAL::DXL>("right_shoulder_roll")
        .setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_shoulder_roll")), 1.0);
    manager.dev<RhAL::DXL>("right_elbow").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("right_elbow")), 1.0);
  }
  if (writeHead && smoothing)
  {
    manager.dev<RhAL::DXL>("head_pitch").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("head_pitch")), 1.0);
    manager.dev<RhAL::DXL>("head_yaw").setGoalPositionSmooth(RhAL::Rad2Deg(model.getDOF("head_yaw")), 1.0);
  }
}

void RhALAppendLog(MapSeries& map, RhAL::StandardManager& manager)
{
  // Current time
  double time = RhAL::duration_float(RhAL::getTimePoint());
  // Log goal position
  for (const std::string& name : Leph::NamesDOF)
  {
    map.append("goal:" + name, time, RhAL::Deg2Rad(manager.dev<RhAL::DXL>(name).goalPosition().getWrittenValue()));
  }
  // Log position
  for (const std::string& name : Leph::NamesDOF)
  {
    RhAL::ReadValueFloat value = manager.dev<RhAL::DXL>(name).position().readValue();
    if (!value.isError)
    {
      map.append("pos:" + name, RhAL::duration_float(value.timestamp), RhAL::Deg2Rad(value.value));
    }
  }
}

}  // namespace Leph
