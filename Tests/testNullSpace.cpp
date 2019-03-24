#include <iostream>
#include <algorithm>
#include <libcmaes/cmaes.h>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/InverseKinematics.hpp"
#include "Model/NullSpace.hpp"

int main()
{
  // Sigmaban foot fixed model
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);

  // Viewer
  Leph::ModelViewer viewer(1200, 900);

  // Add degree of freedom subset
  Leph::InverseKinematics inv(model.get());
  inv.addDOF("right_hip_yaw");
  inv.addDOF("right_hip_pitch");
  inv.addDOF("right_hip_roll");
  inv.addDOF("right_knee");
  inv.addDOF("right_ankle_pitch");
  inv.addDOF("right_ankle_roll");
  inv.addDOF("left_hip_yaw");
  inv.addDOF("left_hip_pitch");
  inv.addDOF("left_hip_roll");
  inv.addDOF("left_knee");
  inv.addDOF("left_ankle_pitch");
  inv.addDOF("left_ankle_roll");

  // Add degree of freedom limit bounds
  inv.setLowerBound("right_hip_yaw", -M_PI / 4);
  inv.setUpperBound("right_hip_yaw", M_PI / 4);
  inv.setLowerBound("right_hip_pitch", -M_PI / 3);
  inv.setUpperBound("right_hip_pitch", M_PI / 3);
  inv.setLowerBound("right_hip_roll", -M_PI / 4);
  inv.setUpperBound("right_hip_roll", M_PI / 4);
  inv.setLowerBound("right_knee", 0.0);
  inv.setUpperBound("right_knee", M_PI / 2);
  inv.setLowerBound("right_ankle_pitch", -M_PI / 2);
  inv.setUpperBound("right_ankle_pitch", M_PI / 2);
  inv.setLowerBound("right_ankle_roll", -M_PI / 2);
  inv.setUpperBound("right_ankle_roll", M_PI / 2);
  inv.setLowerBound("left_hip_yaw", -M_PI / 4);
  inv.setUpperBound("left_hip_yaw", M_PI / 4);
  inv.setLowerBound("left_hip_pitch", -M_PI / 3);
  inv.setUpperBound("left_hip_pitch", M_PI / 3);
  inv.setLowerBound("left_hip_roll", -M_PI / 4);
  inv.setUpperBound("left_hip_roll", M_PI / 4);
  inv.setLowerBound("left_knee", 0.0);
  inv.setUpperBound("left_knee", M_PI / 2);
  inv.setLowerBound("left_ankle_pitch", -M_PI / 2);
  inv.setUpperBound("left_ankle_pitch", M_PI / 2);
  inv.setLowerBound("left_ankle_roll", -M_PI / 2);
  inv.setUpperBound("left_ankle_roll", M_PI / 2);

  // Set Center of Mass target position
  inv.addTargetCOM();
  inv.targetCOM().z() -= 0.04;
  // Set flying foot constraints
  inv.addTargetPosition("flying_foot", "right_foot_tip");
  inv.addTargetOrientation("flying_foot", "right_foot_tip");

  // Initial convergence
  inv.run(0.0001, 100);

  std::vector<Eigen::VectorXd> exploredContainer;
  // Set up Nullspace
  Leph::NullSpace nullspace(inv);
  // Explore and discretize the Nullspace
  nullspace.exploreKernelDiscretized(inv.getDOFSubset(), exploredContainer, 0.1, 20000, false);

  // Sort explored state by minimum torques
  std::sort(exploredContainer.begin(), exploredContainer.end(),
            [&inv, &model](const Eigen::VectorXd& state1, const Eigen::VectorXd& state2) {
              inv.setDOFSubset(state1);
              double tmp1 = model.get().inverseDynamicsClosedLoop("right_foot_tip").tail(20).lpNorm<Eigen::Infinity>();
              inv.setDOFSubset(state2);
              double tmp2 = model.get().inverseDynamicsClosedLoop("right_foot_tip").tail(20).lpNorm<Eigen::Infinity>();
              return tmp1 < tmp2;
            });

  // Display all found null space discretized states
  size_t index = 0;
  double t = 0;
  while (viewer.update())
  {
    t += 0.005;
    inv.setDOFSubset(exploredContainer[index]);

    Leph::ModelDraw(model.get(), viewer);

    std::cout << "Index=" << index << "/" << exploredContainer.size() - 1
              << " --- COM: " << model.get().centerOfMass("origin").transpose() << std::endl;
    Eigen::VectorXd tau = model.get().inverseDynamicsClosedLoop(model.get().getFrameIndex("right_foot_tip"));
    std::cout << "TorquesSumMax: " << tau.tail(20).lpNorm<Eigen::Infinity>() << std::endl;
    std::cout << "TorquesSumNorm: " << tau.tail(20).norm() << std::endl;

    if (t > 1.0)
    {
      index++;
      t = 0.0;
    }
    if (index >= exploredContainer.size())
    {
      index = 0;
    }
  }

  // Try to find minimum torques configuration with
  // CMA-ES optimization
  // Use infinity norm minimization
  libcmaes::FitFunc fitness = [&model, &inv, &nullspace](const double* x, const int N) {
    double value = 0.0;
    Eigen::VectorXd y(N);
    for (int i = 0; i < N; i++)
    {
      y(i) = x[i];
    }
    inv.setDOFSubset(y);
    if (!nullspace.checkConstraints(y))
    {
      value += 1000.0;
    }
    value += model.get().inverseDynamicsClosedLoop("right_foot_tip", nullptr, true).tail(20).lpNorm<Eigen::Infinity>();
    Eigen::VectorXd fvect(inv.values());
    inv.operator()(y, fvect);
    value += 100.0 * fvect.norm();
    return value;
  };
  // Starting point
  std::vector<double> x0(inv.getDOFSubset().size(), 0.0);
  for (int i = 0; i < inv.getDOFSubset().size(); i++)
  {
    x0[i] = inv.getDOFSubset()(i);
  }
  // Optimization init
  double sigma = 0.1;
  libcmaes::CMAParameters<> cmaparams(x0, sigma);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(false);
  cmaparams.set_str_algo("acmaes");
  cmaparams.set_max_iter(1000);
  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
  std::cout << "best solution: " << cmasols << std::endl;
  std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
  Eigen::VectorXd tau =
      model.get().inverseDynamicsClosedLoop(model.get().getFrameIndex("right_foot_tip"), nullptr, true);
  std::cout << "TorquesSumMax: " << tau.tail(20).lpNorm<Eigen::Infinity>() << std::endl;
  std::cout << "TorquesSumNorm: " << tau.tail(20).norm() << std::endl;

  // Display best found CMA-ES torques pose
  inv.setDOFSubset(cmasols.best_candidate().get_x_dvec());
  while (viewer.update())
  {
    Leph::ModelDraw(model.get(), viewer);
  }

  return 0;
}
