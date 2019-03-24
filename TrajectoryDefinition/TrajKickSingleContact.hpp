#ifndef LEPH_TRAJKICKSINGLECONTACT_HPP
#define LEPH_TRAJKICKSINGLECONTACT_HPP

#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryParameters.hpp"

namespace Leph
{
/**
 * TrajKickSingleContact
 *
 * Template for right single
 * support kick trajectory generation
 * with acceleration plateau.
 * The max velocity range is no a peak but
 * a accelerating plateau.
 * Expect to transmit more energy to the ball.
 */
class TrajKickSingleContact
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

private:
  /**
   * Compute delta time and acceleration
   * for constant acceleration polynomial
   * spline beween given position and velocity
   * bounds. (Fit 3th polynomial).
   */
  static double splineComputeTime(double pos1, double pos2, double vel1, double vel2);
  static double splineComputeAcc(double pos1, double pos2, double vel1, double vel2);
};

}  // namespace Leph

#endif
