#include <iostream>
#include <cmath>
#include "Plot/Plot.hpp"

/**
 * Problem configuration
 */
double massFoot = 0.2;
double massBall = 0.1;
double stiffSpring = 600.0;
double lengthSpring = 0.01;
double timeStep = 0.000001;
double accMotorFoot = 10.0;

/**
 * Mass dynamic state
 */
struct State
{
  double pos;
  double vel;
};

/**
 * Simulate and return the
 * ball velocity at contact end.
 */
double doSim(bool showSim)
{
  Leph::Plot plot;
  unsigned int iterations = 0;
  // Problem state initialization
  State foot;
  foot.pos = -lengthSpring;
  foot.vel = 1.0;
  State ball;
  ball.pos = 0.0;
  ball.vel = 0.0;

  double t = 0.0;
  // Loop until ball-foot separation
  while (fabs(ball.pos - foot.pos) <= lengthSpring)
  {
    // Force and acceleration computation (spring)
    double springLength = ball.pos - foot.pos - lengthSpring;
    double accFoot = stiffSpring / massFoot * springLength + accMotorFoot;
    double accBall = -stiffSpring / massBall * springLength;
    // Euler integration
    foot.vel += timeStep * accFoot;
    ball.vel += timeStep * accBall;
    foot.pos += timeStep * foot.vel;
    ball.pos += timeStep * ball.vel;
    t += timeStep;
    if (showSim && iterations % 10 == 0)
    {
      plot.add({
          "t",
          t,
          "foot_pos",
          foot.pos,
          "foot_vel",
          foot.vel,
          "ball_pos",
          ball.pos,
          "ball_vel",
          ball.vel,
      });
    }
    // Detect elastic collision
    if (ball.pos - foot.pos <= 0.0)
    {
      double massSum = massFoot + massBall;
      double nextVelFoot = (massFoot - massBall) / massSum * foot.vel + 2.0 * massBall / massSum * ball.vel;
      double nextVelBall = 2.0 * massFoot / massSum * foot.vel + (massBall - massFoot) / massSum * ball.vel;
      foot.vel = nextVelFoot;
      ball.vel = nextVelBall;
      ball.pos = foot.pos;
    }
    iterations++;
  }
  if (showSim)
  {
    plot.plot("t", "foot_pos").plot("t", "ball_pos").render();
    plot.plot("t", "foot_pos").plot("t", "ball_pos").plot("t", "foot_vel").plot("t", "ball_vel").render();
  }
  return ball.vel;
}

int main()
{
  // Show one simulation
  doSim(true);
  // Show final velocity with respect
  // to spring stiffness
  Leph::Plot plot;
  for (stiffSpring = 1.0; stiffSpring <= 2000.0; stiffSpring += 1.0)
  {
    double vel = doSim(false);
    plot.add({ "stiff", stiffSpring, "final_vel", vel });
  }
  plot.plot("stiff", "final_vel").render();

  return 0;
}
