#ifndef LEPH_TRAJKICKDOUBLE_HPP
#define LEPH_TRAJKICKDOUBLE_HPP

#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryParameters.hpp"

namespace Leph
{
/**
 * TrajKickDouble
 *
 * Template for right double
 * support kick trajectory generation
 */
class TrajKickDouble
{
public:
  /**
   * If isFwd is true, the motion parameters
   * are created for complete forward optimization.
   */
  static void initializeParameters(TrajectoryParameters& trajParams, bool isFwd = false);

  static TrajectoryGeneration::GenerationFunc funcGeneration(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::CheckParamsFunc funcCheckParams(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::CheckStateFunc funcCheckState(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::CheckDOFFunc funcCheckDOF(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::ScoreFunc funcScore(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::EndScoreFunc funcEndScore(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::ScoreSimFunc funcScoreSim(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::EndScoreSimFunc funcEndScoreSim(const TrajectoryParameters& trajParams);

  static TrajectoryGeneration::SaveFunc funcSave(const TrajectoryParameters& trajParams);
};

}  // namespace Leph

#endif
