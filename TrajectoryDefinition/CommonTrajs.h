#ifndef LEPH_COMMONTRAJS_H
#define LEPH_COMMONTRAJS_H

#include "TrajectoryGeneration/TrajectoryParameters.hpp"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"

namespace Leph
{
/**
 * Return default TrajectoryParameters instance.
 * Definition of CMA-ES and static single
 * and double support parameters.
 */
TrajectoryParameters DefaultTrajParameters();

/**
 * Return a standard fitness function from
 * given trajectory parameters
 */
TrajectoryGeneration::ScoreFunc DefaultFuncScore(const TrajectoryParameters& trajParams);

/**
 * Return a standard end fitness function from
 * given trajectory parameters
 */
TrajectoryGeneration::EndScoreFunc DefaultFuncEndScore(const TrajectoryParameters& trajParams);

/**
 * Return a standard saving function for
 * trajectories and parameters
 */
TrajectoryGeneration::SaveFunc DefaultFuncSave(const TrajectoryParameters& trajParams);

}  // namespace Leph

#endif
