#include <iostream>
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Types/MatrixLabel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"

int main()
{
  Leph::MatrixLabel logs;
  logs.load("../../These/Data/model_2015-06-29-19-48-17.log");

  // Initialize model instances
  Leph::HumanoidFixedPressureModel model(Leph::SigmabanModel);
  Leph::HumanoidFixedModel model2(Leph::SigmabanModel);
  Leph::ModelViewer viewer(1200, 900);
  Leph::Scheduling scheduling(50.0);

  size_t index = 0;
  while (viewer.update())
  {
    // Assign model
    Leph::VectorLabel pos = logs[index].extract("motor").rename("motor", "");
    Leph::VectorLabel pressure = logs[index].extract("pressure").rename("pressure", "");
    model.get().setDOF(pos, false);
    model2.get().setDOF(pos, false);
    model.setPressure(logs[index]("pressure:w"), logs[index]("pressure:w1") / 1000.0 / logs[index]("pressure:w"),
                      logs[index]("pressure:w2") / 1000.0 / logs[index]("pressure:w"),
                      logs[index]("pressure:x1") / 100.0, logs[index]("pressure:y1") / 100.0,
                      logs[index]("pressure:x2") / 100.0, logs[index]("pressure:y2") / 100.0);
    Eigen::Vector3d copLeft = model.centerOfPressureLeft("origin");
    Eigen::Vector3d copRight = model.centerOfPressureRight("origin");
    Eigen::Vector3d copMiddle = model.centerOfPressure("origin");
    // Update support foot and compute odometry
    model.updateBase();
    model2.updateBase();
    // Set IMU data for motors real model state
    model.setOrientation(
        Eigen::AngleAxisd(logs[index]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(logs[index]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
        false);
    model2.setOrientation(
        Eigen::AngleAxisd(logs[index]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(logs[index]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
        false);
    // Display foot pressure force
    viewer.drawBox(0.005, 0.005, 0.1 * model.pressureLeftRatio(),
                   copLeft + Eigen::Vector3d(0, 0, 0.1 * model.pressureLeftRatio()), Eigen::Matrix3d::Identity(), 1.0,
                   0.0, 0.0);
    viewer.drawBox(0.005, 0.005, 0.1 * model.pressureRightRatio(),
                   copRight + Eigen::Vector3d(0, 0, 0.1 * model.pressureRightRatio()), Eigen::Matrix3d::Identity(), 0.0,
                   1.0, 0.0);
    // Display centers of pressures trajectory
    viewer.addTrackedPoint(copLeft, Leph::ModelViewer::Red);
    viewer.addTrackedPoint(copRight, Leph::ModelViewer::Green);
    viewer.addTrackedPoint(copMiddle, Leph::ModelViewer::Yellow);
    // Display model
    Leph::ModelDraw(model.get(), viewer);
    Leph::ModelDraw(model2.get(), viewer);
    // Scheduling
    scheduling.wait();
    index++;
    if (index >= logs.size())
    {
      break;
    }
  }

  return 0;
}
