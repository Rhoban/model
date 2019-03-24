#include <stdexcept>
#include <sstream>
#include "Model/RBDLContactLCP.h"
#include "LCPMobyDrake/LCPSolver.hpp"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

namespace Leph
{
void RBDLContactLCP(RBDL::Model& model, const RBDLMath::VectorNd& Q, const RBDLMath::VectorNd& QDot,
                    const RBDLMath::VectorNd& Tau, const RBDLMath::VectorNd& inertiaOffset, RBDL::ConstraintSet& CS,
                    const Eigen::VectorXi& isBilateralConstraint)
{
  /* XXX
  std::cout << "================================================================================" << std::endl;
  std::cout << "================================================================================" << std::endl;
  std::cout << "================================================================================" << std::endl;
  */
  // Size check
  size_t sizeDOF = model.dof_count;
  if ((size_t)Q.size() != sizeDOF || (size_t)QDot.size() != sizeDOF || (size_t)Tau.size() != sizeDOF ||
      (size_t)inertiaOffset.size() != sizeDOF || (size_t)isBilateralConstraint.size() != CS.size())
  {
    throw std::logic_error("RBDLContactLCP invalid inputs size");
  }

  // Compute H, C, and G matrixes.
  RBDL::CalcContactSystemVariables(model, Q, QDot, Tau, CS);
  // Add inertial diagonal offsets
  for (size_t i = 0; i < (size_t)inertiaOffset.size(); i++)
  {
    CS.H(i, i) += inertiaOffset(i);
  }

  // Count unilateral and bilateral constraints
  size_t sizeUnilateral = 0;
  size_t sizeBilateral = 0;
  for (size_t i = 0; i < (size_t)isBilateralConstraint.size(); i++)
  {
    if (isBilateralConstraint(i) == 0)
    {
      sizeUnilateral++;
    }
    else
    {
      sizeBilateral++;
    }
  }
  /* XXX
  std::cout << "SIZE TOTAL: " << isBilateralConstraint.size() << std::endl;
  std::cout << "SIZE UNILATERAL: " << sizeUnilateral << std::endl;
  std::cout << "SIZE BILATERAL : " << sizeBilateral << std::endl;
  */

  // Split constraint matrixes in
  // unilateral and bilateral
  RBDLMath::MatrixNd GUnilateral(sizeUnilateral, CS.G.cols());
  RBDLMath::MatrixNd GBilateral(sizeBilateral, CS.G.cols());
  RBDLMath::VectorNd gammaUnilateral(sizeUnilateral);
  RBDLMath::VectorNd gammaBilateral(sizeBilateral);
  size_t indexUnilateral = 0;
  size_t indexBilateral = 0;
  for (size_t i = 0; i < (size_t)isBilateralConstraint.size(); i++)
  {
    if (isBilateralConstraint(i) == 0)
    {
      GUnilateral.row(indexUnilateral) = CS.G.row(i);
      gammaUnilateral(indexUnilateral) = CS.gamma(i);
      indexUnilateral++;
    }
    else
    {
      GBilateral.row(indexBilateral) = CS.G.row(i);
      gammaBilateral(indexBilateral) = CS.gamma(i);
      indexBilateral++;
    }
  }

  // Featherstone's book chapter 11.3 and 11.5.
  // The LCP to solve is:
  // zetaDot = M*lambda + d
  // with: zetaDot(i) >=0, lambda(i) >= 0, zetaDot(i)*lambda(i) = 0.
  // lambda: contact force.
  // zetaDot: contact distance acceleration.
  // zeta: contact distance velocity = G*QDot.
  // M = G*Hinv*Gt.
  // d = G*Hinv*(Tau-C) + GDot*QDot.
  // GDot*QDot = -CS.gamma = -k (in Featherstone)
  // Note that in Featherstone's boot,
  // notation is G = T.transpose().

  // Check that given position and velocity
  // comply with given constraint.
  // If not, an impuse need to be apply.
  RBDLMath::VectorNd ZetaUnilateral = GUnilateral * QDot;
  RBDLMath::VectorNd ZetaBilateral = GBilateral * QDot;
  if (ZetaUnilateral.lpNorm<Eigen::Infinity>() > 1e-3 || ZetaBilateral.lpNorm<Eigen::Infinity>() > 1e-3)
  {
    std::ostringstream ss;
    ss << " ZetaUnilateral: " << ZetaUnilateral.transpose();
    ss << " sizeUnilateral: " << sizeUnilateral;
    ss << " ZetaBilateral: " << ZetaBilateral.transpose();
    ss << " sizeBilateral: " << sizeBilateral;
    /* XXX
    throw std::logic_error(
        "RBDLContactLCP velocity not constrainted. Impulse needed."
        + ss.str());
    */
  }

  // Compute H inverse
  auto HQR = CS.H.colPivHouseholderQr();
  if (!HQR.isInvertible())
  {
    /* XXX
    throw std::logic_error(
        "RBDLContactLCP non invertible H matrix");
    */
  }
  RBDLMath::MatrixNd Hinv = HQR.inverse();

  // Compute M matrix and d vector
  // for unilateral constraints
  RBDLMath::MatrixNd MUnilateral(sizeUnilateral, sizeUnilateral);
  RBDLMath::VectorNd dUnilateral(sizeUnilateral);
  MUnilateral = GUnilateral * Hinv * GUnilateral.transpose();
  dUnilateral = GUnilateral * Hinv * (Tau - CS.C) - gammaUnilateral;

  // Compute M matrix and d vector
  // for bilateral constraints
  RBDLMath::MatrixNd MBilateral(sizeBilateral, sizeBilateral);
  RBDLMath::VectorNd dBilateral(sizeBilateral);
  MBilateral = GBilateral * Hinv * GBilateral.transpose();
  dBilateral = GBilateral * Hinv * (Tau - CS.C) - gammaBilateral;

  // XXX TODO XXX TODO TEST2
  Eigen::MatrixXd HHH(sizeDOF + sizeBilateral, sizeDOF + sizeBilateral);
  Eigen::MatrixXd TTT(sizeUnilateral, sizeDOF + sizeBilateral);
  Eigen::VectorXd CCC(sizeDOF + sizeBilateral);
  Eigen::MatrixXd MMM(sizeUnilateral, sizeUnilateral);
  Eigen::VectorXd DDD(sizeUnilateral);
  Eigen::VectorXd LLL(sizeUnilateral);
  HHH.setZero();
  TTT.setZero();
  CCC.setZero();
  MMM.setZero();
  DDD.setZero();
  LLL.setZero();
  HHH.block(0, 0, sizeDOF, sizeDOF) = CS.H;
  HHH.block(sizeDOF, 0, sizeBilateral, sizeDOF) = GBilateral;
  HHH.block(0, sizeDOF, sizeDOF, sizeBilateral) = -GBilateral.transpose();
  TTT.block(0, 0, sizeUnilateral, sizeDOF) = GUnilateral;
  CCC.segment(0, sizeDOF) = Tau - CS.C;
  CCC.segment(sizeDOF, sizeBilateral) = gammaBilateral;
  auto HHHQR = HHH.colPivHouseholderQr();
  if (!HHHQR.isInvertible())
  {
    // XXX std::cout << "NOT HHH INVERTIBLE !!!" << std::endl;
    /* TODO XXX
    throw std::logic_error(
        "RBDLContactLCP non invertible HHH matrix");
    */
  }
  Eigen::MatrixXd HHHinv = HHHQR.inverse();
  MMM = TTT * HHHinv * TTT.transpose();
  DDD = TTT * HHHinv * CCC - gammaUnilateral;
  Drake::MobyLCPSolver solverTMP;
  bool isSuccessTMP = solverTMP.SolveLcpLemkeRegularized(MMM, DDD, &LLL);
  (void)isSuccessTMP;
  /* XXX
  std::cout << isSuccessTMP << " ?????? LAMBDA   = " << LLL.transpose() << std::endl;
  std::cout << isSuccessTMP << " ?????? ZETA_DOT = " << (MMM*LLL+DDD).transpose() << std::endl;
  */

  // XXX TODO XXX TODO TEST3
  Eigen::MatrixXd HHH_(sizeDOF + sizeBilateral, sizeDOF + sizeBilateral);
  Eigen::MatrixXd TTT_(sizeUnilateral, sizeDOF + sizeBilateral);
  Eigen::VectorXd CCC_(sizeDOF + sizeBilateral);
  Eigen::MatrixXd MMM_(sizeUnilateral, sizeUnilateral);
  Eigen::VectorXd DDD_(sizeUnilateral);
  Eigen::VectorXd LLL_(sizeUnilateral);
  HHH_.setZero();
  TTT_.setZero();
  CCC_.setZero();
  MMM_.setZero();
  DDD_.setZero();
  LLL_.setZero();
  HHH_.block(0, 0, sizeDOF, sizeDOF) = CS.H;
  HHH_.block(sizeDOF, 0, sizeBilateral, sizeDOF) = GBilateral;
  HHH_.block(0, sizeDOF, sizeDOF, sizeBilateral) = -0.0001 * GBilateral.transpose();
  TTT_.block(0, 0, sizeUnilateral, sizeDOF) = GUnilateral;
  CCC_.segment(0, sizeDOF) = 0.0001 * (Tau - CS.C) + CS.H * QDot;
  auto HHH_QR = HHH_.colPivHouseholderQr();
  if (!HHH_QR.isInvertible())
  {
    // XXX std::cout << "NOT HHH_ INVERTIBLE !!!" << std::endl;
    /* TODO XXX
    throw std::logic_error(
        "RBDLContactLCP non invertible HHH matrix");
    */
  }
  Eigen::MatrixXd HHH_inv = HHH_QR.inverse();
  MMM_ = TTT_ * HHH_inv * (0.0001 * TTT_.transpose());
  DDD_ = TTT_ * HHH_inv * CCC_;
  Drake::MobyLCPSolver solverTMP_;
  bool isSuccessTMP_ = solverTMP_.SolveLcpLemkeRegularized(MMM_, DDD_, &LLL_);
  (void)isSuccessTMP_;
  /* XXX
  std::cout << isSuccessTMP_ << " ###### LAMBDA   = " << LLL_.transpose() << std::endl;
  std::cout << isSuccessTMP_ << " ###### ZETA     = " << (MMM_*LLL_+DDD_).transpose() << std::endl;
  */

  // Compute LCP solution
  RBDLMath::VectorNd lambda(sizeUnilateral);
  Drake::MobyLCPSolver solver;
  bool isSuccess = solver.SolveLcpLemkeRegularized(MUnilateral, dUnilateral, &lambda);
  if (!isSuccess)
  {
    std::ostringstream ss;
    ss << std::endl;
    ss << "size:" << sizeUnilateral << std::endl;
    ss << "success:" << isSuccess << std::endl;
    ss << "lambda:" << lambda.transpose() << std::endl;
    ss << "zetaDot:" << (MUnilateral * lambda + dUnilateral).transpose() << std::endl;
    ss << "zeta:" << (GUnilateral * QDot).transpose() << std::endl;
    // XXX std::cout << "RBDLContactLCP LCP not success: " + ss.str() << std::endl;
    /*
    throw std::logic_error(
        "RBDLContactLCP LCP not success."
        + ss.str());
    */
  }
  /* XXX
  std::cout << "size:" << sizeUnilateral << std::endl;
  std::cout << "isBilateralConstraint:" << isBilateralConstraint.transpose() << std::endl;
  std::cout << "success:" << isSuccess << std::endl;
  std::cout << "lambda:" << lambda.transpose() << std::endl;
  std::cout << "zetaDot:" << (MUnilateral*lambda+dUnilateral).transpose() << std::endl;
  std::cout << "zeta:" << (GUnilateral * QDot).transpose() << std::endl;
  */

  // Assign computed lambda in
  // constraint set
  indexUnilateral = 0;
  indexBilateral = 0;
  for (size_t i = 0; i < (size_t)isBilateralConstraint.size(); i++)
  {
    if (isBilateralConstraint(i) == 0)
    {
      CS.force(i) = LLL(indexUnilateral);
      // CS.force(i) = lambda(indexUnilateral);
      indexUnilateral++;
    }
    else
    {
      CS.force(i) = 1.0;
      indexBilateral++;
    }
  }
}

}  // namespace Leph
