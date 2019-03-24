#ifndef LEPH_RUNGEKUTTA4_HPP
#define LEPH_RUNGEKUTTA4_HPP

#include <Eigen/Dense>
#include <functional>

/**
 * Integrate a differential equation from given state
 * using the Runge-Kutta 4 fourth order method.
 * Current state and time step are given.
 * differential return the system differential for each
 * dimension at given state.
 */
inline Eigen::VectorXd RungeKutta4Integration(const Eigen::VectorXd& state, double dt,
                                              std::function<Eigen::VectorXd(const Eigen::VectorXd&)> differential)
{
  Eigen::VectorXd k1 = differential(state);
  Eigen::VectorXd k2 = differential(state + 0.5 * dt * k1);
  Eigen::VectorXd k3 = differential(state + 0.5 * dt * k2);
  Eigen::VectorXd k4 = differential(state + dt * k3);

  return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

#endif
