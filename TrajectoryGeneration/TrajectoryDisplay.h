#ifndef LEPH_TRAJECTORYDISPLAY_H
#define LEPH_TRAJECTORYDISPLAY_H

#include <string>
#include "Model/HumanoidFixedModel.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"

namespace Leph
{
/**
 * Plot and Display the given Trajectories
 * spline container.
 */
void TrajectoriesDisplay(const Trajectories& traj, RobotType type = SigmabanModel,
                         const std::string& modelParamsPath = "");

}  // namespace Leph

#endif
