#include "TimeSeries/SeriesUtils.h"
#include "Concepts/HumanoidModelConcept.hpp"
#include "Concepts/HumanoidSensorsModelConcept.hpp"
#include "Concepts/FootStepDifferentiatorConcept.hpp"
#include "Concepts/FootStepIntegratorConcept.hpp"
#include "Concepts/FallDetectorConcept.hpp"

namespace Leph
{
void plotPhase(Leph::Plot& plot, const Leph::ModelSeries& model, const std::string& labelX, const std::string& nameX,
               const std::string& labelY, const std::string& nameY, double maxTime)
{
  if (model.series(nameX).size() == 0)
  {
    return;
  }
  if (model.series(nameY).size() == 0)
  {
    return;
  }

  double min = model.series(nameX).timeMin();
  double max = model.series(nameX).timeMax();
  if (model.series(nameY).timeMin() > min)
  {
    min = model.series(nameY).timeMin();
  }
  if (model.series(nameY).timeMax() < max)
  {
    max = model.series(nameY).timeMax();
  }
  if (maxTime > 0.0 && maxTime < max)
  {
    max = maxTime;
  }

  size_t indexMin = model.series(nameX).getClosestIndex(min);
  size_t indexMax = model.series(nameX).getClosestIndex(max);
  for (size_t i = indexMax; i <= indexMin; i++)
  {
    bool isPoint = true;
    double t = model.series(nameX).at(i).time;
    double val1 = model.series(nameX).at(i).value;
    Leph::VectorLabel vect;
    vect.append("time", t);
    vect.append(labelX, val1);
    if (model.series(nameY).isTimeValid(t))
    {
      double val2 = model.series(nameY).get(t);
      vect.append(labelY, val2);
    }
    else
    {
      isPoint = false;
    }
    if (isPoint)
    {
      plot.add(vect);
    }
  }
}

void plotFuture(Leph::Plot& plot, Leph::ModelSeries& model, const std::string& name)
{
  if (model.series(name).size() == 0 || model.series(name).sizeFuture() == 0)
  {
    return;
  }

  for (size_t i = 0; i < model.series(name).sizeFuture(); i++)
  {
    double time = model.series(name).atFuture(i).time;
    if (model.series(name).isTimeValid(time))
    {
      plot.add(Leph::VectorLabel("time", time, name + "_future", model.series(name).atFuture(i).value, name,
                                 model.series(name).get(time)));
    }
  }
}

double modelPredict(Leph::ModelSeries& model, const std::string& name, const std::vector<double>& inputs)
{
  Eigen::VectorXd vect(inputs.size());
  for (size_t i = 0; i < inputs.size(); i++)
  {
    vect(i) = inputs[i];
  }

  return model.regression(name).model().predict(vect, 0.0)(0);
}

void seriesCompare(const Leph::TimeSeries& series1, const Leph::TimeSeries& series2, double beginTime, double endTime,
                   double& sumError, double& sumSquaredError, int& count)
{
  if (series1.size() < 2 || series2.size() < 2)
  {
    sumError = -1.0;
    sumSquaredError = -1.0;
    count = 0;
  }

  double min = beginTime;
  if (min < series1.timeMin())
  {
    min = series1.timeMin();
  }
  if (min < series2.timeMin())
  {
    min = series2.timeMin();
  }
  double max = endTime;
  if (max > series1.timeMax())
  {
    max = series1.timeMax();
  }
  if (max > series2.timeMax())
  {
    max = series2.timeMax();
  }

  double time = min;
  sumError = 0.0;
  sumSquaredError = 0.0;
  count = 0;
  while (time < max)
  {
    size_t index1 = series1.getLowerIndex(time);
    size_t index2 = series2.getLowerIndex(time);
    double t1 = series1.at(index1).time;
    double t2 = series2.at(index2).time;
    double t = (t1 > t1) ? t1 : t2;
    double val1 = series1.get(t);
    double val2 = series2.get(t);
    sumError += fabs(val1 - val2);
    sumSquaredError += pow(fabs(val1 - val2), 2);
    count++;
    time = t;
    time += Leph::TIME_EPSILON;
  }
}

void initModelSeries(Leph::ModelSeries& model, bool withMocapConcept, bool withDeltaRegression, bool withWalkRegression,
                     bool noSensor)
{
  // Declare TimeSeries
  // Goals DOF
  model.addSeries("goal_left_ankle_roll");
  model.addSeries("goal_left_ankle_pitch");
  model.addSeries("goal_left_knee");
  model.addSeries("goal_left_hip_pitch");
  model.addSeries("goal_left_hip_roll");
  model.addSeries("goal_left_hip_yaw");
  model.addSeries("goal_right_ankle_roll");
  model.addSeries("goal_right_ankle_pitch");
  model.addSeries("goal_right_knee");
  model.addSeries("goal_right_hip_pitch");
  model.addSeries("goal_right_hip_roll");
  model.addSeries("goal_right_hip_yaw");
  model.addSeries("goal_left_shoulder_pitch");
  model.addSeries("goal_left_shoulder_roll");
  model.addSeries("goal_left_elbow");
  model.addSeries("goal_right_shoulder_pitch");
  model.addSeries("goal_right_shoulder_roll");
  model.addSeries("goal_right_elbow");
  model.addSeries("goal_head_yaw");
  model.addSeries("goal_head_pitch");
  // Inputs DOF
  model.addSeries("pos_left_ankle_roll");
  model.addSeries("pos_left_ankle_pitch");
  model.addSeries("pos_left_knee");
  model.addSeries("pos_left_hip_pitch");
  model.addSeries("pos_left_hip_roll");
  model.addSeries("pos_left_hip_yaw");
  model.addSeries("pos_right_ankle_roll");
  model.addSeries("pos_right_ankle_pitch");
  model.addSeries("pos_right_knee");
  model.addSeries("pos_right_hip_pitch");
  model.addSeries("pos_right_hip_roll");
  model.addSeries("pos_right_hip_yaw");
  model.addSeries("pos_left_shoulder_pitch");
  model.addSeries("pos_left_shoulder_roll");
  model.addSeries("pos_left_elbow");
  model.addSeries("pos_right_shoulder_pitch");
  model.addSeries("pos_right_shoulder_roll");
  model.addSeries("pos_right_elbow");
  model.addSeries("pos_head_yaw");
  model.addSeries("pos_head_pitch");
  // Inputs sensors
  model.addSeries("sensor_pitch");
  model.addSeries("sensor_roll");
  model.addSeries("sensor_gyro_yaw");
  model.addSeries("sensor_gyro_x");
  model.addSeries("sensor_gyro_y");
  model.addSeries("sensor_acc_x");
  model.addSeries("sensor_acc_y");
  // Inputs pressure
  model.addSeries("pressure_weight");
  model.addSeries("pressure_left_ratio");
  model.addSeries("pressure_right_ratio");
  model.addSeries("pressure_left_x");
  model.addSeries("pressure_left_y");
  model.addSeries("pressure_right_x");
  model.addSeries("pressure_right_y");
  // Inputs Mocap
  model.addSeries("mocap_is_data");
  model.addSeries("mocap_is_valid");
  model.addSeries("mocap_x");
  model.addSeries("mocap_y");
  model.addSeries("mocap_theta");
  // Humanoid with sensors model outputs
  model.addSeries("pos_is_support_foot_left");
  model.addSeries("pos_support_length");
  model.addSeries("pos_head_x");
  model.addSeries("pos_head_y");
  model.addSeries("pos_head_z");
  model.addSeries("pos_head_theta");
  // Humanoid with sensors model outputs
  model.addSeries("goal_is_support_foot_left");
  model.addSeries("goal_support_length");
  model.addSeries("goal_head_x");
  model.addSeries("goal_head_y");
  model.addSeries("goal_head_z");
  model.addSeries("goal_head_theta");
  // FootStep Differentiator outputs
  model.addSeries("goal_delta_head_x");
  model.addSeries("goal_delta_head_y");
  model.addSeries("goal_delta_head_theta");
  // FootStep Differentiator outputs (with sensors)
  model.addSeries("pos_delta_head_x");
  model.addSeries("pos_delta_head_y");
  model.addSeries("pos_delta_head_theta");
  // Mocap
  model.addSeries("delta_mocap_x");
  model.addSeries("delta_mocap_y");
  model.addSeries("delta_mocap_theta");
  // Walk orders reference
  model.addSeries("walk_step");
  model.addSeries("walk_lateral");
  model.addSeries("walk_turn");
  model.addSeries("walk_enabled");
  // FootStep Integrator output
  // head
  model.addSeries("integrated_head_x");
  model.addSeries("integrated_head_y");
  model.addSeries("integrated_head_theta");
  // Mocap
  model.addSeries("integrated_mocap_x");
  model.addSeries("integrated_mocap_y");
  model.addSeries("integrated_mocap_theta");
  // walk
  model.addSeries("integrated_walk_x");
  model.addSeries("integrated_walk_y");
  model.addSeries("integrated_walk_theta");

  // Initialize Model Concept
  model.addConcept(
      // HumanoidModelConcept allocation with sensors
      new Leph::HumanoidSensorsModelConcept(Leph::SigmabanModel),
      // Inputs
      { "pos_left_ankle_roll",
        "pos_left_ankle_pitch",
        "pos_left_knee",
        "pos_left_hip_pitch",
        "pos_left_hip_roll",
        "pos_left_hip_yaw",
        "pos_right_ankle_roll",
        "pos_right_ankle_pitch",
        "pos_right_knee",
        "pos_right_hip_pitch",
        "pos_right_hip_roll",
        "pos_right_hip_yaw",
        "pos_left_shoulder_pitch",
        "pos_left_shoulder_roll",
        "pos_left_elbow",
        "pos_right_shoulder_pitch",
        "pos_right_shoulder_roll",
        "pos_right_elbow",
        "pos_head_yaw",
        "pos_head_pitch",
        "sensor_pitch",
        "sensor_roll",
        "sensor_gyro_yaw",
        "pressure_weight",
        "pressure_left_ratio",
        "pressure_right_ratio",
        "pressure_left_x",
        "pressure_left_y",
        "pressure_right_x",
        "pressure_right_y" },
      // Outputs
      { "pos_is_support_foot_left", "pos_head_x", "pos_head_y", "pos_head_z", "pos_head_theta", "pos_support_length" });
  model.addConcept(
      // HumanoidModelConcept allocation
      new Leph::HumanoidModelConcept(Leph::SigmabanModel),
      // Inputs
      { "goal_left_ankle_roll",     "goal_left_ankle_pitch",   "goal_left_knee",        "goal_left_hip_pitch",
        "goal_left_hip_roll",       "goal_left_hip_yaw",       "goal_right_ankle_roll", "goal_right_ankle_pitch",
        "goal_right_knee",          "goal_right_hip_pitch",    "goal_right_hip_roll",   "goal_right_hip_yaw",
        "goal_left_shoulder_pitch", "goal_left_shoulder_roll", "goal_left_elbow",       "goal_right_shoulder_pitch",
        "goal_right_shoulder_roll", "goal_right_elbow",        "goal_head_yaw",         "goal_head_pitch" },
      // Outputs
      { "goal_is_support_foot_left", "goal_head_x", "goal_head_y", "goal_head_z", "goal_head_theta",
        "goal_support_length" });
  // Differentiation footstep concept
  model.addConcept(
      // FootStepDifferentiatorConcept allocation
      new Leph::FootStepDifferentiatorConcept(),
      // Inputs
      { "pos_is_support_foot_left", "pos_head_x", "pos_head_y", "pos_head_theta", "mocap_is_valid" },
      // Outputs
      { "pos_delta_head_x", "pos_delta_head_y", "pos_delta_head_theta" });
  model.addConcept(
      // FootStepDifferentiatorConcept allocation
      new Leph::FootStepDifferentiatorConcept(),
      // Inputs
      { "goal_is_support_foot_left", "goal_head_x", "goal_head_y", "goal_head_theta", "mocap_is_valid" },
      // Outputs
      { "goal_delta_head_x", "goal_delta_head_y", "goal_delta_head_theta" });
  if (withMocapConcept)
  {
    model.addConcept(
        // FootStepDifferentiatorConcept allocation
        new Leph::FootStepDifferentiatorConcept(),
        // Inputs
        { "pos_is_support_foot_left", "mocap_x", "mocap_y", "mocap_theta", "mocap_is_valid" },
        // Outputs
        { "delta_mocap_x", "delta_mocap_y", "delta_mocap_theta" });
  }
  model.addConcept(
      // FootStepIntegratorConcept allocation
      new Leph::FootStepIntegratorConcept(),
      // Inputs
      { "pos_is_support_foot_left", "pos_delta_head_x", "pos_delta_head_y", "pos_delta_head_theta" },
      // Outputs
      { "integrated_head_x", "integrated_head_y", "integrated_head_theta" });
  model.addConcept(
      // FootStepIntegratorConcept allocation
      new Leph::FootStepIntegratorConcept(),
      // Inputs
      { (noSensor ? "goal_is_support_foot_left" : "pos_is_support_foot_left"), "delta_mocap_x", "delta_mocap_y",
        "delta_mocap_theta" },
      // Outputs
      { "integrated_mocap_x", "integrated_mocap_y", "integrated_mocap_theta" });
  model.addConcept(
      // FootStepIntegratorConcept allocation
      new Leph::FootStepIntegratorConcept(),
      // Inputs
      { "goal_is_support_foot_left", "goal_delta_head_x", "goal_delta_head_y", "goal_delta_head_theta" },
      // Outputs
      { "integrated_walk_x", "integrated_walk_y", "integrated_walk_theta" });

  // Initialize Regression model for
  // mocap deltas
  auto funcAddRegressionDelta = [](Leph::ModelSeries& model, const std::string& regressionName,
                                   const std::string& seriesName) {
    model.addRegression(regressionName, seriesName);
    model.regressionAddInputDeltaIndex(regressionName, "pos_delta_head_x");
    model.regressionAddInputDeltaIndex(regressionName, "pos_delta_head_y");
    model.regressionAddInputDeltaIndex(regressionName, "pos_delta_head_theta");
    model.regressionAddInputDeltaIndex(regressionName, "pos_delta_head_x", 1);
    model.regressionAddInputDeltaIndex(regressionName, "pos_delta_head_y", 1);
    model.regressionAddInputDeltaIndex(regressionName, "pos_delta_head_theta", 1);
    model.regressionAddInputDeltaIndex(regressionName, "pos_support_length");
    model.regressionAddInputDeltaIndex(regressionName, "pos_support_length", 1);
    model.regressionAddInputDeltaIndex(regressionName, "pos_is_support_foot_left");
  };
  if (withDeltaRegression)
  {
    funcAddRegressionDelta(model, "model_delta_mocap_x", "delta_mocap_x");
    funcAddRegressionDelta(model, "model_delta_mocap_y", "delta_mocap_y");
    funcAddRegressionDelta(model, "model_delta_mocap_theta", "delta_mocap_theta");
  }

  // Initialize regression model for
  // delta step from walk orders
  auto funcAddRegressionWalk = [](Leph::ModelSeries& model, const std::string& regressionName,
                                  const std::string& seriesName) {
    model.addRegression(regressionName, seriesName);
    model.regressionAddInputDeltaIndex(regressionName, "goal_delta_head_x");
    model.regressionAddInputDeltaIndex(regressionName, "goal_delta_head_y");
    model.regressionAddInputDeltaIndex(regressionName, "goal_delta_head_theta");
    model.regressionAddInputDeltaIndex(regressionName, "goal_delta_head_x", 1);
    model.regressionAddInputDeltaIndex(regressionName, "goal_delta_head_y", 1);
    model.regressionAddInputDeltaIndex(regressionName, "goal_delta_head_theta", 1);
    model.regressionAddInputDeltaIndex(regressionName, "goal_is_support_foot_left");
    /*
    model.regressionAddInputDeltaTime(regressionName, "walk_step");
    model.regressionAddInputDeltaTime(regressionName, "walk_lateral");
    model.regressionAddInputDeltaTime(regressionName, "walk_turn");
    model.regressionAddInputDeltaTime(regressionName, "walk_step", 0.75);
    model.regressionAddInputDeltaTime(regressionName, "walk_lateral", 0.75);
    model.regressionAddInputDeltaTime(regressionName, "walk_turn", 0.75);
    model.regressionAddInputDeltaIndex(regressionName, "goal_is_support_foot_left");
    model.regressionAddInputDeltaTime(regressionName, "walk_enabled");
    */
  };
  if (withWalkRegression)
  {
    funcAddRegressionWalk(model, "model_walk_delta_x", "delta_mocap_x");
    funcAddRegressionWalk(model, "model_walk_delta_y", "delta_mocap_y");
    funcAddRegressionWalk(model, "model_walk_delta_theta", "delta_mocap_theta");
  }
}

void appendModelSeries(Leph::ModelSeries& model, double time, const Leph::VectorLabel& logs, bool invMocap)
{
  // Loading low level inputs
  // Degrees of freedom position
  model.series("pos_left_ankle_roll").append(time, logs("pos:left_ankle_roll"));
  model.series("pos_left_ankle_pitch").append(time, logs("pos:left_ankle_pitch"));
  model.series("pos_left_knee").append(time, logs("pos:left_knee"));
  model.series("pos_left_hip_pitch").append(time, logs("pos:left_hip_pitch"));
  model.series("pos_left_hip_roll").append(time, logs("pos:left_hip_roll"));
  model.series("pos_left_hip_yaw").append(time, logs("pos:left_hip_yaw"));
  model.series("pos_right_ankle_roll").append(time, logs("pos:right_ankle_roll"));
  model.series("pos_right_ankle_pitch").append(time, logs("pos:right_ankle_pitch"));
  model.series("pos_right_knee").append(time, logs("pos:right_knee"));
  model.series("pos_right_hip_pitch").append(time, logs("pos:right_hip_pitch"));
  model.series("pos_right_hip_roll").append(time, logs("pos:right_hip_roll"));
  model.series("pos_right_hip_yaw").append(time, logs("pos:right_hip_yaw"));
  model.series("pos_left_shoulder_roll").append(time, logs("pos:left_shoulder_roll"));
  model.series("pos_left_shoulder_pitch").append(time, logs("pos:left_shoulder_pitch"));
  model.series("pos_left_elbow").append(time, logs("pos:left_elbow"));
  model.series("pos_right_shoulder_roll").append(time, logs("pos:right_shoulder_roll"));
  model.series("pos_right_shoulder_pitch").append(time, logs("pos:right_shoulder_pitch"));
  model.series("pos_right_elbow").append(time, logs("pos:right_elbow"));
  model.series("pos_head_yaw").append(time, logs("pos:head_yaw"));
  model.series("pos_head_pitch").append(time, logs("pos:head_pitch"));
  // Degrees of freedom goal
  model.series("goal_left_ankle_roll").append(time, logs("goal:left_ankle_roll"));
  model.series("goal_left_ankle_pitch").append(time, logs("goal:left_ankle_pitch"));
  model.series("goal_left_knee").append(time, logs("goal:left_knee"));
  model.series("goal_left_hip_pitch").append(time, logs("goal:left_hip_pitch"));
  model.series("goal_left_hip_roll").append(time, logs("goal:left_hip_roll"));
  model.series("goal_left_hip_yaw").append(time, logs("goal:left_hip_yaw"));
  model.series("goal_right_ankle_roll").append(time, logs("goal:right_ankle_roll"));
  model.series("goal_right_ankle_pitch").append(time, logs("goal:right_ankle_pitch"));
  model.series("goal_right_knee").append(time, logs("goal:right_knee"));
  model.series("goal_right_hip_pitch").append(time, logs("goal:right_hip_pitch"));
  model.series("goal_right_hip_roll").append(time, logs("goal:right_hip_roll"));
  model.series("goal_right_hip_yaw").append(time, logs("goal:right_hip_yaw"));
  model.series("goal_left_shoulder_roll").append(time, logs("goal:left_shoulder_roll"));
  model.series("goal_left_shoulder_pitch").append(time, logs("goal:left_shoulder_pitch"));
  model.series("goal_left_elbow").append(time, logs("goal:left_elbow"));
  model.series("goal_right_shoulder_roll").append(time, logs("goal:right_shoulder_roll"));
  model.series("goal_right_shoulder_pitch").append(time, logs("goal:right_shoulder_pitch"));
  model.series("goal_right_elbow").append(time, logs("goal:right_elbow"));
  model.series("goal_head_yaw").append(time, logs("goal:head_yaw"));
  model.series("goal_head_pitch").append(time, logs("goal:head_pitch"));
  // Sensors
  // TODO SHIFT XXX
  double timeShift = 0.14;
  // double timeShift = 0.0;
  model.series("sensor_pitch").append(time - timeShift, logs("sensor:pitch"));
  model.series("sensor_roll").append(time - timeShift, logs("sensor:roll"));
  model.series("sensor_gyro_yaw").append(time, logs("sensor:gyro_yaw"));
  model.series("sensor_gyro_x").append(time, logs("sensor:gyro_x"));
  model.series("sensor_gyro_y").append(time, logs("sensor:gyro_y"));
  model.series("sensor_acc_x").append(time, logs("sensor:acc_x"));
  model.series("sensor_acc_y").append(time, logs("sensor:acc_y"));
  // Pressure
  model.series("pressure_weight").append(time, logs("pressure:weight"));
  model.series("pressure_left_ratio").append(time, logs("pressure:left_ratio"));
  model.series("pressure_right_ratio").append(time, logs("pressure:right_ratio"));
  model.series("pressure_left_x").append(time, logs("pressure:left_x"));
  model.series("pressure_left_y").append(time, logs("pressure:left_y"));
  model.series("pressure_right_x").append(time, logs("pressure:right_x"));
  model.series("pressure_right_y").append(time, logs("pressure:right_y"));
  // Walk orders
  model.series("walk_step").append(time, logs("walk:stepGain"));
  model.series("walk_lateral").append(time, logs("walk:lateralGain"));
  model.series("walk_turn").append(time, logs("walk:turnGain"));
  model.series("walk_enabled").append(time, logs("walk:enabledGain"));
  // Motion Capture
  bool isMocapValid = logs("mocap:is_valid");
  if (isMocapValid)
  {
    model.series("mocap_x").append(time, (invMocap ? -1.0 : 1.0) * logs("mocap:z"));
    model.series("mocap_y").append(time, (invMocap ? -1.0 : 1.0) * logs("mocap:x"));
    double mocapTheta = logs("mocap:azimuth") * M_PI / 180.0;
    model.series("mocap_theta").append(time, mocapTheta);
  }
  model.series("mocap_is_data").append(time, isMocapValid);
  model.series("mocap_is_valid").append(time, logs("mocap:check"));
}

}  // namespace Leph
