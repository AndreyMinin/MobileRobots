/*
 * mpc_test.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: andreyminin
 */

#include "mpc.h"
using namespace mpc_controller;

int main(int argc, char* argv[]) {
  int steps = 20;
  double dt = 0.1;
  double max_vel = 2.0;
  double max_acc = 1;
  double max_delta = 0.5;
  double max_delta_rate = 0.1;
  double L = 1.5;
  MPC mpc(steps, dt, max_vel, max_acc, max_delta, max_delta_rate, L);

  double v = 0.0;
  double delta = 0;
  double a, r;
  std::vector<double> c(4, 0);
//  mpc.solve(v, delta, c, r, a);
//  std::vector<double> c1(4, 1);
//  mpc.solve(v, delta, c1, r, a);
  c = {-5.93742e-07, 2.15893e-05, 0.0199417, 4.12936e-05};
  v = 1.67203e-11;
  delta = 0;
  std::vector<double> x,y;
  mpc.solve(v, delta, c, r, a, x, y);
  for(int i = 0; i<x.size(); ++i) {
    std::cout<<"x = "<<x[i]<<" y = "<<y[i]<<std::endl;
  }
  return 0;
}


