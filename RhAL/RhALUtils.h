#ifndef LEPH_RHALUTILS_H
#define LEPH_RHALUTILS_H

#include <RhAL.hpp>
#include "Model/HumanoidModel.hpp"
#include "Types/MapSeries.hpp"

namespace Leph
{
/**
 * Write to given RhAL Manager the
 * degrees of freedom target from given
 * Humanoid Model.
 * writeLegs, writeArms, writeHead flags
 * enable writing the DOF of legs, arms
 * or the head.
 */
void RhALWriteStateGoal(RhAL::StandardManager& manager, const HumanoidModel& model, bool writeLegs, bool writeArms,
                        bool writeHead, bool smoothing = false);

/**
 * Append to given MapSeries
 * degrees of freedom goal targets,
 * read positions and available sensors.
 */
void RhALAppendLog(MapSeries& map, RhAL::StandardManager& manager);

}  // namespace Leph

#endif
