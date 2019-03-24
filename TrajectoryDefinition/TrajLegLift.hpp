#ifndef LEPH_TRAJLEGLIFT_HPP
#define LEPH_TRAJLEGLIFT_HPP

#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryParameters.hpp"

namespace Leph
{
/**
 * TrajLegLift
 *
 * Template for right leg lift
 * trajectory generation
 */
class TrajLegLift
{
public:
  static void initializeParameters(TrajectoryParameters& trajParams);

  static TrajectoryGeneration::GenerationFunc funcGeneration(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::CheckParamsFunc funcCheckParams(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::CheckStateFunc funcCheckState(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::CheckDOFFunc funcCheckDOF(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::ScoreFunc funcScore(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::EndScoreFunc funcEndScore(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::SaveFunc funcSave(const TrajectoryParameters& trajParams);
};

}  // namespace Leph

#endif
