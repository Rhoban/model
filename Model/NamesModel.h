#ifndef LEPH_NAMESMODEL_H
#define LEPH_NAMESMODEL_H

#include <vector>
#include <string>

namespace Leph
{
/**
 * All DOF and Base names
 */
const std::vector<std::string> NamesDOFAll = {
  "base_x",
  "base_y",
  "base_z",
  "base_roll",
  "base_pitch",
  "base_yaw",
  "head_pitch",
  "head_yaw",
  "left_shoulder_roll",
  "left_shoulder_pitch",
  "left_elbow",
  "left_hip_roll",
  "left_hip_pitch",
  "left_hip_yaw",
  "left_ankle_roll",
  "left_ankle_pitch",
  "left_knee",
  "right_shoulder_roll",
  "right_shoulder_pitch",
  "right_elbow",
  "right_hip_roll",
  "right_hip_pitch",
  "right_hip_yaw",
  "right_ankle_roll",
  "right_ankle_pitch",
  "right_knee",
};

/**
 * All DOF names
 */
const std::vector<std::string> NamesDOF = {
  "head_pitch",           "head_yaw",         "left_shoulder_roll", "left_shoulder_pitch",
  "left_elbow",           "left_hip_roll",    "left_hip_pitch",     "left_hip_yaw",
  "left_ankle_roll",      "left_ankle_pitch", "left_knee",          "right_shoulder_roll",
  "right_shoulder_pitch", "right_elbow",      "right_hip_roll",     "right_hip_pitch",
  "right_hip_yaw",        "right_ankle_roll", "right_ankle_pitch",  "right_knee",
};

/**
 * All Base names
 */
const std::vector<std::string> NamesBase = {
  "base_x", "base_y", "base_z", "base_roll", "base_pitch", "base_yaw",
};

/**
 * Leg DOF names
 */
const std::vector<std::string> NamesDOFLeg = {
  "left_hip_roll",  "left_hip_pitch",  "left_hip_yaw",  "left_ankle_roll",  "left_ankle_pitch",  "left_knee",
  "right_hip_roll", "right_hip_pitch", "right_hip_yaw", "right_ankle_roll", "right_ankle_pitch", "right_knee",
};

/**
 * Left leg DOF names
 */
const std::vector<std::string> NamesDOFLegLeft = {
  "left_hip_roll", "left_hip_pitch", "left_hip_yaw", "left_ankle_roll", "left_ankle_pitch", "left_knee",
};

/**
 * Right leg DOF names
 */
const std::vector<std::string> NamesDOFLegRight = {
  "right_hip_roll", "right_hip_pitch", "right_hip_yaw", "right_ankle_roll", "right_ankle_pitch", "right_knee",
};

/**
 * Arm DOF names
 */
const std::vector<std::string> NamesDOFArm = {
  "left_shoulder_roll",  "left_shoulder_pitch",  "left_elbow",
  "right_shoulder_roll", "right_shoulder_pitch", "right_elbow",
};

/**
 * Left arm DOF names
 */
const std::vector<std::string> NamesDOFArmLeft = {
  "left_shoulder_roll",
  "left_shoulder_pitch",
  "left_elbow",
};

/**
 * Right arm DOF names
 */
const std::vector<std::string> NamesDOFArmRight = {
  "right_shoulder_roll",
  "right_shoulder_pitch",
  "right_elbow",
};

/**
 * Head DOF names
 */
const std::vector<std::string> NamesDOFHead = {
  "head_pitch",
  "head_yaw",
};

/**
 * Trunk-Foot IK cartesian names
 */
const std::vector<std::string> NamesCart = {
  "trunk_pos_x", "trunk_pos_y", "trunk_pos_z", "trunk_axis_x", "trunk_axis_y", "trunk_axis_z",
  "foot_pos_x",  "foot_pos_y",  "foot_pos_z",  "foot_axis_x",  "foot_axis_y",  "foot_axis_z",
};

/**
 * Trunk-Foot IK position cartesian names
 */
const std::vector<std::string> NamesCartPos = {
  "trunk_pos_x", "trunk_pos_y", "trunk_pos_z", "foot_pos_x", "foot_pos_y", "foot_pos_z",
};

/**
 * Trunk-Foot IK axis cartesian names
 */
const std::vector<std::string> NamesCartAxis = {
  "trunk_axis_x", "trunk_axis_y", "trunk_axis_z", "foot_axis_x", "foot_axis_y", "foot_axis_z",
};

/**
 * Trunk IK cartesian names
 */
const std::vector<std::string> NamesCartTrunk = {
  "trunk_pos_x", "trunk_pos_y", "trunk_pos_z", "trunk_axis_x", "trunk_axis_y", "trunk_axis_z",
};

/**
 * Foot IK cartesian names
 */
const std::vector<std::string> NamesCartFoot = {
  "foot_pos_x", "foot_pos_y", "foot_pos_z", "foot_axis_x", "foot_axis_y", "foot_axis_z",
};

}  // namespace Leph

#endif
