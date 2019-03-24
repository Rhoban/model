#include <unistd.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <Eigen/Dense>
#include "LegIK/LegIK.hpp"
#include "Model/Model.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/InverseKinematics.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Spline/FittedSpline.hpp"
#include "Plot/Plot.hpp"

/**
 * Load given filename in ReM1 format
 */
static std::vector<Leph::Spline> loadSplines(std::istream& file)
{
  std::vector<Leph::Spline> container;
  std::string line;
  double t = 0.0;
  while (file.good())
  {
    std::getline(file, line);
    if (line.size() == 0)
    {
      break;
    }
    if (line[0] >= 65 && line[0] <= 122)
    {
      container.push_back(Leph::Spline());
      t = 0.0;
    }
    else
    {
      double time = std::stod(line);
      time /= 10000.0;
      std::vector<double> coefs;
      std::string lineCoefs;
      while (true)
      {
        double val;
        file >> val;
        coefs.push_back(val);
        if (file.peek() == '\n' || file.peek() == -1 || file.peek() == '\r')
        {
          file.ignore();
          break;
        }
        file.ignore();
      }
      Leph::Polynom poly(coefs.size() - 1);
      for (size_t i = 0; i < coefs.size(); i++)
      {
        poly(i) = coefs[i];
      }
      container.back().addPart(poly, t, t + time);
      t += time;
    }
  }

  return container;
}

/**
 * Run analytical inverse kinematics on given
 * model with given position and orientation (Yaw-Pitch-Roll)
 * targets
 */
static bool inverseKinematics(Leph::Model& model, const Eigen::Vector3d& targetPos, const Eigen::Vector3d& targetAngle)
{
  double legHipToKnee = 0.1249;  // 0.09375;
  double legKneeToAnkle = 0105;
  double legAnkleToGround = 0.0319955;
  Eigen::Vector3d footToPen(0.089, 0.007, -0.0045);  //(0.075, 0.037, -0.0085);

  Eigen::Quaternion<double> quat;
  Eigen::AngleAxisd yawRot(targetAngle(0), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchRot(targetAngle(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rollRot(targetAngle(2), Eigen::Vector3d::UnitX());
  quat = rollRot * pitchRot * yawRot;
  Eigen::Matrix3d rotation = quat.matrix();

  Eigen::Vector3d footToPenWorld = rotation.transpose() * footToPen;
  Eigen::Vector3d footTargetPos = targetPos - footToPenWorld;

  // LegIK initialization
  LegIK::IK ik(legHipToKnee, legKneeToAnkle, legAnkleToGround);
  // Convert foot position from given
  // target to LegIK base
  LegIK::Vector3D legIKTarget;
  legIKTarget[0] = footTargetPos(0);
  legIKTarget[1] = footTargetPos(1);
  legIKTarget[2] = footTargetPos(2);
  // Convert orientation from given frame
  // to LegIK base
  LegIK::Frame3D legIKMatrix;
  legIKMatrix[0][0] = rotation(0, 0);
  legIKMatrix[0][1] = rotation(0, 1);
  legIKMatrix[0][2] = rotation(0, 2);
  legIKMatrix[1][0] = rotation(1, 0);
  legIKMatrix[1][1] = rotation(1, 1);
  legIKMatrix[1][2] = rotation(1, 2);
  legIKMatrix[2][0] = rotation(2, 0);
  legIKMatrix[2][1] = rotation(2, 1);
  legIKMatrix[2][2] = rotation(2, 2);

  // Run inverse kinematics
  LegIK::Position result;
  bool isSucess = ik.compute(legIKTarget, legIKMatrix, result, nullptr, true, false);

  // Update degrees of freedom on success
  if (isSucess)
  {
    model.setDOF("left_hip_yaw", result.theta[0]);
    model.setDOF("left_hip_roll", result.theta[1]);
    model.setDOF("left_hip_pitch", -result.theta[2]);
    model.setDOF("left_knee", result.theta[3]);
    model.setDOF("left_ankle_pitch", -result.theta[4]);
    model.setDOF("left_ankle_roll", result.theta[5]);
  }

  return isSucess;
}

/**
 * Return the absolute path to model file
 */
static std::string modelFilePath()
{
  char buff[2048];
  std::string pathToHere = "";
  ssize_t len = readlink("/proc/self/exe", buff, sizeof(buff) - 1);
  if (len != -1)
  {
    buff[len] = '\0';
    pathToHere = std::string(buff).substr(0, std::string(buff).find_last_of("/"));
  }
  else
  {
    throw std::logic_error("No absolute model path");
  }

  return std::string(pathToHere) + "/../Data/legWithPen.urdf";
}

/**
 * Parse position/torque file in ReM1 format
 */
static std::pair<Eigen::VectorXd, Eigen::VectorXd> loadPosTorques(std::istream& file)
{
  double pos;
  double torque;

  std::vector<std::pair<double, double>> container;
  Eigen::VectorXd positions(6);
  Eigen::VectorXd torques(6);
  for (size_t k = 0; k < 6; k++)
  {
    file >> pos;
    file >> torque;
    positions(k) = pos;
    torques(k) = torque;
  }

  return { positions, torques };
}

int main(int argc, char** argv)
{
  // Load raw model
  Leph::Model model(modelFilePath());

  // Parsing input arguments
  std::string mode;
  if (argc != 2)
  {
    std::cout << "Usage: ./app [quiet|gui|force|gravity]" << std::endl;
    return 1;
  }
  mode = std::string(argv[1]);

  if (mode == "force")
  {
    // Compute cartesian force and quit
    std::pair<Eigen::VectorXd, Eigen::VectorXd> posTorques = loadPosTorques(std::cin);
    Eigen::VectorXd positions = posTorques.first;
    Eigen::VectorXd torques = posTorques.second;
    // Set DOF positions
    model.setDOF("left_hip_yaw", positions(0));
    model.setDOF("left_hip_roll", positions(1));
    model.setDOF("left_hip_pitch", positions(2));
    model.setDOF("left_knee", positions(3));
    model.setDOF("left_ankle_pitch", positions(4));
    model.setDOF("left_ankle_roll", positions(5));
    // Compute foot jacobian matrix
    Eigen::MatrixXd jac = model.pointJacobian("left_foot_tip", "origin");
    jac.transposeInPlace();
    // Compute joint velocities
    Eigen::VectorXd f = jac.fullPivLu().solve(torques);
    std::cout << f << std::endl;
    return 0;
  }
  else if (mode == "gravity")
  {
    // Returns the torques to apply in order to mantain, at least localy, the position and velocity of each joint given
    // in input
    std::pair<Eigen::VectorXd, Eigen::VectorXd> posSpeeds = loadPosTorques(std::cin);
    Eigen::VectorXd positions = posSpeeds.first;
    Eigen::VectorXd speeds = posSpeeds.second;

    // Set DOF positions
    model.setDOF("left_hip_yaw", positions(0));
    model.setDOF("left_hip_roll", positions(1));
    model.setDOF("left_hip_pitch", positions(2));
    model.setDOF("left_knee", positions(3));
    model.setDOF("left_ankle_pitch", positions(4));
    model.setDOF("left_ankle_roll", positions(5));

    // Foot velocity and acceleration vector
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    Eigen::VectorXd pos = model.getDOFVect();
    Eigen::VectorXd vel(6, 1);
    Eigen::VectorXd acc(6, 1);
    // Roll-Pitch-Yaw convention
    // TODO velocity of orientation pitch and yaw are not considered
    vel(0) = 0.0;
    vel(1) = 0.0;
    vel(2) = 0.0;
    vel(3) = vx;
    vel(4) = vy;
    vel(5) = vz;
    acc(0) = 0.0;
    acc(1) = 0.0;
    acc(2) = 0.0;
    acc(3) = ax;
    acc(4) = ay;
    acc(5) = az;
    // Compute foot jacobian matrix
    Eigen::MatrixXd jac = model.pointJacobian("left_foot_tip", "origin");
    // Compute joint velocities
    Eigen::VectorXd dq = jac.fullPivLu().solve(vel);
    // Compute joint acceleration
    // acc = J(q)*ddq + dJ(q, dq)*dq
    //=> ddq = J(q)^-1*(acc - dJ*dq)
    // dJ*dq can be computed using pointAcceleration and setting
    // ddq to zero (thanks Martin Felis !).
    Eigen::VectorXd J_dot_q_dot =
        model.pointAcceleration("left_foot_tip", "origin", dq, Eigen::VectorXd::Zero(model.sizeDOF()));
    Eigen::VectorXd ddq = jac.fullPivLu().solve(acc - J_dot_q_dot);
    // Compute joint torques
    model.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    Eigen::VectorXd torques = model.inverseDynamics(dq, ddq);
    // Assign to splines
    std::cout << torques << std::endl;

    return 0;
  }

  // Loading cartesian trajectories
  std::vector<Leph::Spline> splinesCart = loadSplines(std::cin);
  double maxTime = splinesCart.front().max();

  // Spline container initialization
  std::vector<Leph::FittedSpline> splinesPos(6);
  std::vector<Leph::FittedSpline> splinesTorque(6);

  // Viewer
  Leph::ModelViewer* viewer = nullptr;
  if (mode == "gui")
  {
    viewer = new Leph::ModelViewer(1200, 900);
  }
  // Main Loop
  Leph::Plot plotTorque;
  Leph::Plot plotPos;
  double t = 0.0;
  while (t < maxTime)
  {
    if (mode == "gui" && !viewer->update())
    {
      break;
    }
    // Update target
    double x = splinesCart[0].pos(t);
    double y = splinesCart[1].pos(t);
    double z = splinesCart[2].pos(t);

    double minDist = 0.13;
    double maxDist = 0.25;
    double angleAtMin = M_PI / 2.0 + 0.7;
    double angleAtMax = M_PI / 2.0 - 0.7;

    double distTarget = sqrt(x * x + y * y);
    double pitch;
    if (distTarget < minDist)
    {
      pitch = angleAtMin;
    }
    else if (distTarget > maxDist)
    {
      pitch = angleAtMax;
    }
    else
    {
      double ratio = (distTarget - minDist) / (maxDist - minDist);
      pitch = (1.0 - ratio) * angleAtMin + ratio * angleAtMax;
    }

    double yaw = -atan2(y, x);
    // pitch = M_PI/2.0;
    // yaw = M_PI/2.0;

    bool isSucess = inverseKinematics(model, Eigen::Vector3d(x, y, z), Eigen::Vector3d(yaw, pitch, 0.0));
    if (!isSucess)
    {
      std::cout << "IK ERROR at t=" << t << std::endl;
    }

    // Foot velocity and acceleration vector
    double vx = splinesCart[0].vel(t);
    double vy = splinesCart[1].vel(t);
    double vz = splinesCart[2].vel(t);
    double ax = splinesCart[0].acc(t);
    double ay = splinesCart[1].acc(t);
    double az = splinesCart[2].acc(t);
    Eigen::VectorXd pos = model.getDOFVect();
    Eigen::VectorXd vel(6, 1);
    Eigen::VectorXd acc(6, 1);
    // Roll-Pitch-Yaw convention
    // TODO velocity of orientation pitch and yaw are not considered
    vel(0) = 0.0;
    vel(1) = 0.0;
    vel(2) = 0.0;
    vel(3) = vx;
    vel(4) = vy;
    vel(5) = vz;
    acc(0) = 0.0;
    acc(1) = 0.0;
    acc(2) = 0.0;
    acc(3) = ax;
    acc(4) = ay;
    acc(5) = az;
    // Compute foot jacobian matrix
    Eigen::MatrixXd jac = model.pointJacobian("left_foot_tip", "origin");
    // Compute joint velocities
    Eigen::VectorXd dq = jac.fullPivLu().solve(vel);
    // Compute joint acceleration
    // acc = J(q)*ddq + dJ(q, dq)*dq
    //=> ddq = J(q)^-1*(acc - dJ*dq)
    // dJ*dq can be computed using pointAcceleration and setting
    // ddq to zero (thanks Martin Felis !).
    Eigen::VectorXd J_dot_q_dot =
        model.pointAcceleration("left_foot_tip", "origin", dq, Eigen::VectorXd::Zero(model.sizeDOF()));
    Eigen::VectorXd ddq = jac.fullPivLu().solve(acc - J_dot_q_dot);
    // Compute joint torques
    model.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    Eigen::VectorXd torques = model.inverseDynamics(dq, ddq);
    // Assign to splines
    splinesTorque[0].addPoint(t, torques(0));
    splinesTorque[1].addPoint(t, torques(1));
    splinesTorque[2].addPoint(t, torques(2));
    splinesTorque[3].addPoint(t, torques(3));
    splinesTorque[4].addPoint(t, torques(4));
    splinesTorque[5].addPoint(t, torques(5));
    splinesPos[0].addPoint(t, pos(0));
    splinesPos[1].addPoint(t, pos(1));
    splinesPos[2].addPoint(t, pos(2));
    splinesPos[3].addPoint(t, pos(3));
    splinesPos[4].addPoint(t, pos(4));
    splinesPos[5].addPoint(t, pos(5));
    if (mode == "gui")
    {
      // Display model
      Eigen::Vector3d pt = model.position("left_foot_tip", "origin");
      viewer->addTrackedPoint(pt);
      viewer->maxTrajectory = 10000;
      Leph::ModelDraw(model, *viewer);
      // Plot
      plotTorque.add(Leph::VectorLabel("t", t, "t0", torques(0), "t1", torques(1), "t2", torques(2), "t3", torques(3),
                                       "t4", torques(4), "t5", torques(5)));
      plotPos.add(Leph::VectorLabel("t", t, "q0", pos(0), "q1", pos(1), "q2", pos(2), "q3", pos(3), "q4", pos(4), "q5",
                                    pos(5)));
    }
    t += 0.01;
  }

  double averageLength = 0.0;
  // Finding the average of the durations (needed for fittingGlobal)
  for (size_t i = 0; i < splinesCart[0].size(); i++)
  {
    double length = splinesCart[0].part(i).max - splinesCart[0].part(i).min;
    averageLength += length;
  }
  averageLength /= splinesCart[0].size();

  // Splines fitting
  for (size_t i = 0; i < splinesTorque.size(); i++)
  {
    splinesTorque[i].fittingCubic((unsigned int)100 * averageLength);
    splinesPos[i].fittingCubic((unsigned int)100 * averageLength);
  }

  if (mode == "gui")
  {
    // Ploting position and torque trajectories
    for (double t = 0; t < maxTime; t += maxTime / 1000.0)
    {
      plotTorque.add(Leph::VectorLabel("t", t, "t0 fitted", splinesTorque[0].pos(t), "t1 fitted",
                                       splinesTorque[1].pos(t), "t2 fitted", splinesTorque[2].pos(t), "t3 fitted",
                                       splinesTorque[3].pos(t), "t4 fitted", splinesTorque[4].pos(t), "t5 fitted",
                                       splinesTorque[5].pos(t)));
      plotPos.add(Leph::VectorLabel("t", t, "q0 fitted", splinesPos[0].pos(t), "q1 fitted", splinesPos[1].pos(t),
                                    "q2 fitted", splinesPos[2].pos(t), "q3 fitted", splinesPos[3].pos(t), "q4 fitted",
                                    splinesPos[4].pos(t), "q5 fitted", splinesPos[5].pos(t)));
    }
    plotPos.plot("t", "all").render();
    plotTorque.plot("t", "all").render();
  }

  // Dumping splines
  std::cout << "==== Torque Trajectories ====" << std::endl;
  for (size_t i = 0; i < splinesTorque.size(); i++)
  {
    std::cout << "motor_" << i + 1 << std::endl;
    for (size_t j = 0; j < splinesTorque[i].size(); j++)
    {
      std::cout << splinesTorque[i].part(j).max - splinesTorque[i].part(j).min << std::endl;
      for (size_t k = 0; k < splinesTorque[i].part(j).polynom.degree() + 1; k++)
      {
        std::cout << splinesTorque[i].part(j).polynom(k);
        std::cout << ", ";
      }
      std::cout << "0.0";
      std::cout << std::endl;
    }
  }
  std::cout << "==== Position Trajectories ====" << std::endl;
  for (size_t i = 0; i < splinesPos.size(); i++)
  {
    std::cout << "motor_" << i + 1 << std::endl;
    for (size_t j = 0; j < splinesPos[i].size(); j++)
    {
      std::cout << splinesPos[i].part(j).max - splinesPos[i].part(j).min << std::endl;
      for (size_t k = 0; k < splinesPos[i].part(j).polynom.degree() + 1; k++)
      {
        std::cout << splinesPos[i].part(j).polynom(k);
        std::cout << ", ";
      }
      std::cout << "0.0";
      std::cout << std::endl;
    }
  }

  // Free viewer if needed
  if (mode == "gui")
  {
    delete viewer;
  }
  return 0;
}
