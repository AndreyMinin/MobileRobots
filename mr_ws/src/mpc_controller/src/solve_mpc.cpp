#include <acado/acado_optimal_control.hpp>
#include "mpccontroller.h"

USING_NAMESPACE_ACADO

namespace mpc_controller
{


void MPCController::solveMPC() {

  DifferentialState x, y, fi, delta, vel;
  Control delta_rate, acc;

  const double t_start = 0;
  const double t_end = mpc_steps * mpc_dt;
  double& a0 = control_coefs[0];
  double& a1 = control_coefs[1];
  double& a2 = control_coefs[2];
  double& a3 = control_coefs[3];

  DiscretizedDifferentialEquation f(mpc_dt);

  // discrete time system
  f << next(x) == x + vel*cos(fi)*mpc_dt;
  f << next(y) == y + vel*sin(fi)*mpc_dt;
  f << next(fi) == fi + vel*tan(delta)/wheel_base;
  f << next(delta) == delta + delta_rate*mpc_dt;
  f << next(vel) == vel + acc*mpc_dt;

  // optimal control problem
  OCP ocp(t_start, t_end, mpc_steps);
  ocp.subjectTo(f);
  ocp.subjectTo( AT_START, x == 0 );
  ocp.subjectTo( AT_START, y == 0 );
  ocp.subjectTo( AT_START, fi == 0);
  ocp.subjectTo(-max_acc <= acc <= max_acc);
  ocp.subjectTo(vel <= max_velocity);
  ocp.subjectTo(delta_rate <= max_steer_rate);
  ocp.subjectTo(delta <= max_steer_angle);

  Expression cte = pow(y - a0 - a1*x - a2*x*x -a3*x*x*x, 2);
  Expression epsi = pow(fi - atan(a1 + a2*x + a3*x*x), 2);
  ocp.minimizeMayerTerm(cte + epsi);

  OptimizationAlgorithm alg(ocp);
  ROS_INFO_STREAM("start solving mpc");
  alg.solve();
  ROS_INFO_STREAM("finished solving mpc");
  VariablesGrid controls;
  alg.getControls(controls);
  controls.print();
}


}
