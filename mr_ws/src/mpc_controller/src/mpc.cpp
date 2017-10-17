/*
 * mpc.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: andreyminin
 */

#include "../include/mpc.h"

#include <ros/ros.h>

USING_NAMESPACE_ACADO
namespace mpc_controller
{

MPC::MPC(int steps, double dt, double max_vel, double max_acc, double max_delta, double max_delta_rate, double L,
         double kcte, double kepsi, double kv):
    f(dt),
    t_end(steps * dt),
    steps(steps),
    max_vel(max_vel),
    max_acc(max_acc),
    max_delta(max_delta),
    max_delta_rate(max_delta_rate),
    L(L),
    kcte(kcte),
    kepsi(kepsi),
    kev(kv)
{
  // discrete time system
  f << next(x) == x + vel*cos(fi)*dt;
  f << next(y) == y + vel*sin(fi)*dt;
  f << next(fi) == fi + vel*tan(delta)/L;
  f << next(delta) == delta + delta_rate*dt;
  f << next(vel) == vel + acc*dt;


}

void MPC::solve(double v0, double delta0, std::vector<double>& traj_coef, double& rate_u, double& acc_u)
{
  ROS_DEBUG_STREAM("solve mpc for v0 = "<<v0<<" angle0 = "<<delta0);
  assert(std::abs(delta0) < max_delta);
  ACADO::OCP ocp(t_start, t_end, steps);
  // constrains
  ocp.subjectTo(f);
  ocp.subjectTo( AT_START, x == 0 );
  ocp.subjectTo( AT_START, y == 0 );
  ocp.subjectTo( AT_START, fi == 0 );
  ocp.subjectTo( -max_acc <= acc <= max_acc );
  ocp.subjectTo( -max_delta_rate <=delta_rate <= max_delta_rate );
  ocp.subjectTo( -max_delta <= delta <= max_delta );
  ocp.subjectTo( AT_START, vel == v0 );
  ocp.subjectTo( AT_START, delta == delta0 );

  assert(traj_coef.size() == 4);
  double& a0 = traj_coef[0];
  double& a1 = traj_coef[1];
  double& a2 = traj_coef[2];
  double& a3 = traj_coef[3];

  // minimization objectives
  Expression cte = pow(y - a0 - a1*x - a2*x*x -a3*x*x*x, 2);
  Expression epsi = pow(fi - atan(a1 + 2*a2*x + 3*a3*x*x), 2);
  Expression verr = pow(vel - max_vel, 2);
//  double emax = std::max(0.25, a0*1.2);
//  ocp.subjectTo(cte <= emax);

  ocp.minimizeMayerTerm(kcte*cte + kepsi*epsi + kev*verr);

  OptimizationAlgorithm alg(ocp);
  alg.set(PRINTLEVEL, LOW);
  ROS_INFO_STREAM("start solving mpc");
  alg.solve();
  ROS_INFO_STREAM("finished solving mpc");
  VariablesGrid controls;
  alg.getControls(controls);
//  controls.print();
//  VariablesGrid states;
//  alg.getDifferentialStates(states);
//  states.print();
  //delta_u = controls
  rate_u = controls(0, 0);
  acc_u = controls(0, 1);
  ROS_INFO_STREAM("delta = "<<rate_u<<" acc = "<<acc_u);
}

MPC::~MPC()
{

}

} /* namespace mpc_controller */
