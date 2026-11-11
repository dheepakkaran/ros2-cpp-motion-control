// Demo log: runs the controller core + MuJoCo Panda (no ROS, sim time, deterministic) for two of the
// fault-injection scenarios and writes every 1 ms tick to a CSV for plotting and rendering.
//   baseline:      the 2 s goal re-sent every 100 ms, then 1 s settle
//   commands_stop: the client stops sending at 0.8 s; the watchdog (0.5 s) trips and holds the arm
// Usage: trajectory_demo <model.xml> <out.csv>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include "arm_motion_control/controller.hpp"
#include "arm_motion_control/mujoco_sim.hpp"

using namespace arm_motion_control;

namespace {
JointLimits panda_limits() {
  return JointLimits{{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973},
                     {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973},
                     {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61}};
}
Gains panda_gains() {
  return Gains{{600, 600, 600, 600, 250, 150, 50}, {50, 50, 50, 50, 20, 15, 5}, {87, 87, 87, 87, 12, 12, 12}};
}
Vec goal_pose() { return {0.6, 0.4, 0, -1.9, 0, 1.9, -0.7853}; }  // same goal as test_fault_injection

void run(const char* model, const std::string& name, double t_stop, std::ofstream& out) {
  MujocoSim sim(model, 0.001);
  ControllerCore c(sim.limits(panda_limits().max_vel), panda_gains(), 0.5);
  c.hold(sim.q());
  double next = 0.0;
  const std::vector<Waypoint> wps = {{2.0, goal_pose(), {}, {}}};
  for (int k = 0; k < 3000; ++k) {
    const double t = sim.time();
    if (t + 1e-9 >= next && t <= t_stop) {
      next += 0.1;
      c.accept_goal(wps, 0.0, t);
    }
    const Vec q = sim.q();
    const Vec tau = c.update(t, q, sim.v(), sim.bias());
    const auto& des = c.last_desired();
    out << name << ',' << t << ',' << to_string(c.mode());
    for (double v : des.pos) out << ',' << v;
    for (double v : q) out << ',' << v;
    for (double v : tau) out << ',' << v;
    out << '\n';
    sim.step(tau);
  }
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) { std::cerr << "usage: trajectory_demo <model.xml> <out.csv>\n"; return 1; }
  std::ofstream out(argv[2]);
  out.precision(10);
  out << "scenario,t,mode";
  for (const char* p : {"des", "q", "tau"})
    for (int j = 1; j <= 7; ++j) out << ',' << p << j;
  out << '\n';
  run(argv[1], "baseline", 2.0, out);
  run(argv[1], "commands_stop", 0.8, out);
  std::cout << "wrote " << argv[2] << '\n';
  return 0;
}
