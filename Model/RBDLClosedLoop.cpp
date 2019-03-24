#include <stdexcept>
#include <libcmaes/cmaes.h>
#include <cassert>
#include "Model/RBDLClosedLoop.h"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

namespace Leph
{
/**
 * Compute and return Featherstone's K acceleration
 * constraint equation representing given body id fixed
 * in base coordinates (building closed loop system)
 *
 * K * QDDot = k
 *
 * k is not computed since it is not used
 * in the following Inverse Dynamics computations.
 */
static RBDLMath::MatrixNd RBDLClosedLoopConstraintEquation(RBDL::Model& model, unsigned int fixedBodyId,
                                                           const RBDLMath::VectorNd& Q)
{
  // Compute spatial jacobian for fixed body
  // See Featherstone's book "Rigid Body Dynamics Algorithms"
  // Chapter 8.2 page 145
  RBDLMath::MatrixNd G(6, model.qdot_size);
  G.setZero();
  RBDL::CalcBodySpatialJacobian(model, Q, fixedBodyId, G, true);

  return G;
}

/**
 * See See Featherstone's book "Rigid Body Dynamics Algorithms"
 * Chapter 8 for formulae and notations
 */
RBDLMath::VectorNd RBDLClosedLoopInverseDynamics(RBDL::Model& model, const RBDLMath::VectorNd& Q,
                                                 const RBDLMath::VectorNd& QDot, const RBDLMath::VectorNd& QDDot,
                                                 unsigned int fixedBodyId,
                                                 RigidBodyDynamics::Math::VectorNd* fixedBodyForce,
                                                 bool useInfinityNorm)
{
  // Retrieve the total number of DOF
  // and the number of floating base DOF
  unsigned int allDOFCount = model.dof_count;
  unsigned int virtualDOFCount = 0;
  for (unsigned int i = 1; i < model.mBodies.size(); i++)
  {
    if (model.mBodies[i].mIsVirtual || model.mBodies[i - 1].mIsVirtual)
    {
      virtualDOFCount++;
    }
    if (!model.mBodies[i].mIsVirtual)
    {
      break;
    }
  }
  unsigned int usedDOFCount = allDOFCount - virtualDOFCount;

  // Compute constraint equation
  RBDLMath::MatrixNd allK = RBDLClosedLoopConstraintEquation(model, fixedBodyId, Q);
  // Trim constraints to remove floating base DOF
  RBDLMath::MatrixNd K = allK.rightCols(usedDOFCount);

  // Compute K kernel basis G such as K*G = 0
  Eigen::FullPivLU<RBDLMath::MatrixNd> lu(K);
  RBDLMath::MatrixNd G = lu.kernel();

  // Compute Inverse Dynamics for kinematics tree model
  // without loop and constraint
  RBDLMath::VectorNd allTauID(allDOFCount);
  allTauID.setZero();
  RBDL::InverseDynamics(model, Q, QDot, QDDot, allTauID, NULL);

  // Trim resulting torques to real used DOF as InverseDynamics
  // torques does only depend on children bodies
  //(floating base in single support does not change
  // InverseDynamics result)
  RBDLMath::VectorNd tauID = allTauID.tail(usedDOFCount);

  // Compute right side of equation to solve
  // G' * tau = G' * tauID = tauIDGt
  RBDLMath::VectorNd tauIDGt = G.transpose() * tauID;

  // Build SVD decomposition of G' in order to retrieve it
  // pseudo inverse.
  Eigen::JacobiSVD<RBDLMath::MatrixNd> svd(G.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Solve the equation G' * tau = tauIDGt with least square
  // since the equation are underdetermined (infinite solution)
  // and we are looking for smallest possible torques configuration
  // Base DOFs are trimmed in order to not take them into
  // account in torque L2 minimization.
  RBDLMath::VectorNd resultTau = svd.solve(tauIDGt);

  // The given fixedBodyId is possible a fixed body
  //(id >= max_int/2). Retrieve parent movable body
  // id and transformation matrix.
  // Used movable body id
  unsigned int referenceBodyId = fixedBodyId;
  // Transformation from base to fixedBodyId
  RBDLMath::SpatialTransform baseToBody;
  if (model.IsFixedBodyId(fixedBodyId))
  {
    unsigned int fbodyId = fixedBodyId - model.fixed_body_discriminator;
    referenceBodyId = model.mFixedBodies[fbodyId].mMovableParent;
    baseToBody = model.mFixedBodies[fbodyId].mParentTransform * model.X_base[referenceBodyId];
  }
  else
  {
    baseToBody = model.X_base[referenceBodyId];
  }

  // Solve the system with CMA-ES and minimize infinity norm
  if (useInfinityNorm)
  {
    // Compute kernel basis to solve the homogenous system
    Eigen::FullPivLU<Eigen::MatrixXd> lu2(G.transpose());
    Eigen::MatrixXd GKernel = lu2.kernel();
    // Check full kernel kernel
    if (GKernel.cols() != 6)
    {
      throw std::logic_error("RBDLClosedLoop unexpected kernel size: " + GKernel.cols());
    }
    // CMA-ES fitness function
    libcmaes::FitFunc fitness = [&GKernel, &resultTau](const double* x, const int N) {
      // Conversion to Eigen vector
      Eigen::VectorXd y(N);
      for (int i = 0; i < N; i++)
      {
        y(i) = x[i];
      }
      // The solutions of system G' * tau = tauIDGt can be writen
      // as tau = GKernel * y + d with d a solution of the
      // equation and y any vector. y is optimized to search
      // for minimum infinity norm solution
      Eigen::VectorXd tau = GKernel * y + resultTau;
      // Minimize both infinity and norm 2 because minimizing only
      // infinity norm yield to many solutions. Thus we are imposing
      // an other constraint
      return tau.lpNorm<Eigen::Infinity>() + tau.norm();
    };
    // Initializing CMA-ES starting point
    std::vector<double> x0(GKernel.cols(), 0.0);
    double sigma = 0.01;
    libcmaes::CMAParameters<> cmaparams(x0, sigma);
    cmaparams.set_str_algo("acmaes");
    // Run optimization
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
    // Compute resulting new solution
    resultTau = GKernel * cmasols.best_candidate().get_x_dvec() + resultTau;
  }

  // Compute the cartesian constraints force 6D lambda
  //(moments, linear forces) in fixedBodyId frame.
  // resultTau = (H.ddq + C - tauA) - K'.lambda
  //=> Solve K'.lambda = tauID - resultTau
  // Lambda is expressed in fixedBodyId frame.
  Eigen::VectorXd lambda = K.transpose().colPivHouseholderQr().solve(tauID - resultTau);
  if (fixedBodyForce != nullptr)
  {
    *fixedBodyForce = lambda;
  }

  // Now, we need to recompute an InverseDynamics to compute
  // the base DOFs that we trimmed for pseudo inverse L2 minimization.
  // Single support InverseDynamics is called with contact forces added
  // on fixedBodyId in order to compute allResultTau for double support.
  std::vector<RBDLMath::SpatialVector> externalForcesInBase;
  for (size_t i = 0; i < model.mBodies.size(); i++)
  {
    externalForcesInBase.push_back(RBDLMath::SpatialVectorZero);
  }
  // External forces are expressed in base frame.
  // Lambda is in fixedBodyId frame. So conversion is needed.
  //(moments are dependent of the reference point).
  externalForcesInBase[referenceBodyId] = baseToBody.inverse().toMatrixAdjoint() * lambda;
  // Compute InverseDynamics
  RBDLMath::VectorNd allResultTau(allDOFCount);
  allResultTau.setZero();
  RBDL::InverseDynamics(model, Q, QDot, QDDot, allResultTau, &externalForcesInBase);

  // Sanity checks
  assert((G.transpose() * resultTau - tauIDGt).norm() < 1e-8);
  assert((G.transpose() * allResultTau.tail(usedDOFCount) - tauIDGt).norm() < 1e-8);
  assert((tauID - K.transpose() * lambda - resultTau).norm() < 1e-8);
  assert((resultTau - allResultTau.tail(usedDOFCount)).norm() < 1e-8);

  return allResultTau;
}

}  // namespace Leph
