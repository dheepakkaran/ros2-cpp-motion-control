#pragma once
#include "arm_motion_control/controller.hpp"

namespace amc_test {
using namespace arm_motion_control;
// Panda limits (same as panda_nohand.xml jnt_range + datasheet velocity limits).
inline JointLimits panda_limits() {
  return JointLimits{{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973},
                     {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973},
                     {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61}};
}
inline Vec home() { return {0, 0, 0, -1.57079, 0, 1.57079, -0.7853}; }
inline Gains panda_gains() {
  return Gains{{600, 600, 600, 600, 250, 150, 50}, {50, 50, 50, 50, 20, 15, 5}, {87, 87, 87, 87, 12, 12, 12}};
}
}  // namespace amc_test
