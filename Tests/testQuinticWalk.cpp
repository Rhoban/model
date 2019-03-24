#include <iostream>
#include "Model/HumanoidFixedModel.hpp"
#include "QuinticWalk/QuinticWalk.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

int main()
{
  Leph::QuinticWalk walk;
  Leph::VectorLabel params = walk.getParameters();
  std::cout << params << std::endl;
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);

  Leph::ModelViewer viewer(1200, 900);
  Leph::Scheduling scheduling(100.0);
  Leph::Plot plot;
  double t = 0.0;
  double dt = 0.001;
  while (viewer.update())
  {
    walk.update(dt);
    bool isSuccess = walk.assignModel(model);
    if (!isSuccess)
    {
      std::cout << "IK Error" << std::endl;
      break;
    }
    // Track moving points
    viewer.addTrackedPoint(model.get().position("left_foot_tip", "origin"), Leph::ModelViewer::Red);
    viewer.addTrackedPoint(model.get().position("trunk", "origin"), Leph::ModelViewer::Green);
    viewer.addTrackedPoint(model.get().centerOfMass("origin"), Leph::ModelViewer::Blue);
    viewer.addTrackedPoint(model.get().position("right_foot_tip", "origin"), Leph::ModelViewer::Purple);
    // Drawing
    Leph::ModelDraw(model.get(), viewer);
    // Plot
    plot.add({
        "t",
        t,
        "isEnabled",
        walk.isEnabled(),
        "phase",
        walk.getPhase(),
        "trunk_x",
        model.get().position("trunk", "origin").x(),
        "trunk_y",
        model.get().position("trunk", "origin").y(),
        "trunk_yaw",
        model.get().orientationYaw("trunk", "origin"),
        "traj_trunk_yaw_pos",
        walk.getTrajectories().get("trunk_axis_z").pos(walk.getTrajsTime()),
        "traj_trunk_yaw_vel",
        walk.getTrajectories().get("trunk_axis_z").vel(walk.getTrajsTime()),
        "traj_trunk_yaw_acc",
        walk.getTrajectories().get("trunk_axis_z").acc(walk.getTrajsTime()),
    });
    // Waiting
    scheduling.wait();
    t += dt;
    // Set orders
    if (t < 1.7)
    {
      walk.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), false);
    }
    else if (t < 3.0)
    {
      walk.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), true, true);
    }
    else if (t < 4.0)
    {
      walk.setOrders(Eigen::Vector3d(0.04, 0.0, 0.0), true);
    }
    else if (t < 6.0)
    {
      walk.setOrders(Eigen::Vector3d(0.0, -0.03, 0.0), true);
    }
    else if (t < 8.0)
    {
      walk.setOrders(Eigen::Vector3d(0.0, 0.0, 0.2), true);
    }
    else if (t < 10.0)
    {
      walk.setOrders(Eigen::Vector3d(-0.01, 0.03, -0.2), true);
    }
    else if (t < 15.0)
    {
      walk.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), true);
    }
    else if (t < 20.0)
    {
      walk.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), false);
    }
    walk.setParameters(params);
    /*
    if (t > 1.8 && t < 2.4) {
        walk.setOrders(Eigen::Vector3d(0.04, -0.05, 0.0), true, false);
    } else {
       walk.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), false, false);
    }
    */
  }
  plot.plot("t", "traj_trunk_yaw_pos").plot("t", "traj_trunk_yaw_vel").plot("t", "traj_trunk_yaw_acc").render();
  plot.plot("t", "all").render();

  return 0;
}
