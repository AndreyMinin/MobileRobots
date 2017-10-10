/*
 * mpc.h
 *
 *  Created on: Oct 10, 2017
 *      Author: andreyminin
 */

#ifndef SRC_MPC_CONTROLLER_INCLUDE_MPC_H_
#define SRC_MPC_CONTROLLER_INCLUDE_MPC_H_
#include <acado/acado_optimal_control.hpp>

namespace mpc_controller
{

class MPC
{
  ACADO::DifferentialState x, y, fi, delta, vel;
  ACADO::Control delta_rate, acc;
  ACADO::DiscretizedDifferentialEquation f;
  double t_start = 0;
  double t_end;
  double steps;

  double max_vel;
  double max_acc;
  double max_delta;
  double max_delta_rate;
  double L;
public:
  void solve(double v0, double delta0, std::vector<double>& traj_coef, double& rate_u, double& acc_u);
  MPC(int steps, double dt, double max_vel, double max_acc, double max_delta, double max_delta_rate, double L);
  virtual ~MPC();
};

} /* namespace mpc_controller */

#endif /* SRC_MPC_CONTROLLER_INCLUDE_MPC_H_ */
