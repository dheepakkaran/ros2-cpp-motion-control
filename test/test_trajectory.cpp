#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "arm_motion_control/trajectory.hpp"
#include "common.hpp"

using namespace amc_test;

TEST(Quintic, EndpointsMatchPosVelAcc) {
  auto q = Quintic::fit(0.1, 0.2, -0.3, 1.0, -0.5, 0.4, 1.7);
  EXPECT_NEAR(q.pos(0), 0.1, 1e-12);
  EXPECT_NEAR(q.vel(0), 0.2, 1e-12);
  EXPECT_NEAR(q.acc(0), -0.3, 1e-12);
  EXPECT_NEAR(q.pos(1.7), 1.0, 1e-9);
  EXPECT_NEAR(q.vel(1.7), -0.5, 1e-9);
  EXPECT_NEAR(q.acc(1.7), 0.4, 1e-9);
}

TEST(Trajectory, RestToRestEndpointsAndSmooth) {
  Trajectory t;
  JointState s{home(), {}, {}};
  Vec goal = home();
  goal[0] = 0.5;
  goal[3] = -1.2;
  ASSERT_FALSE(t.build(s, {{2.0, goal, {}, {}}}, panda_limits()).has_value());
  auto a = t.sample(0.0), b = t.sample(2.0), mid = t.sample(1.0);
  for (size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(a.pos[j], home()[j], 1e-12);
    EXPECT_NEAR(b.pos[j], goal[j], 1e-9);
    EXPECT_NEAR(a.vel[j], 0.0, 1e-12);
    EXPECT_NEAR(b.vel[j], 0.0, 1e-9);
    EXPECT_NEAR(b.acc[j], 0.0, 1e-9);
  }
  // Rest-to-rest quintic: midpoint is halfway, peak velocity 1.875 * dq / T at t = T/2.
  EXPECT_NEAR(mid.pos[0], 0.25, 1e-9);
  EXPECT_NEAR(mid.vel[0], 1.875 * 0.5 / 2.0, 1e-9);
  // Sampling past the end clamps to the final point.
  EXPECT_NEAR(t.sample(5.0).pos[0], 0.5, 1e-9);
}

TEST(Trajectory, MultiWaypointPassesThroughPoints) {
  Trajectory t;
  Vec p1 = home(), p2 = home();
  p1[1] = 0.3;
  p2[1] = -0.2;
  ASSERT_FALSE(t.build({home(), {}, {}}, {{1.0, p1, {}, {}}, {2.5, p2, {}, {}}}, panda_limits()).has_value());
  EXPECT_NEAR(t.sample(1.0).pos[1], 0.3, 1e-9);
  EXPECT_NEAR(t.sample(2.5).pos[1], -0.2, 1e-9);
  EXPECT_DOUBLE_EQ(t.duration(), 2.5);
}

TEST(Trajectory, RejectsPositionOverLimit) {
  Trajectory t;
  Vec g = home();
  g[1] = 1.9;  // limit 1.7628
  auto err = t.build({home(), {}, {}}, {{3.0, g, {}, {}}}, panda_limits());
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("outside"), std::string::npos) << *err;
}

TEST(Trajectory, RejectsVelocityOverLimit) {
  Trajectory t;
  Vec g = home();
  g[0] = 2.0;  // 2 rad in 0.5 s -> peak 7.5 rad/s > 2.175
  auto err = t.build({home(), {}, {}}, {{0.5, g, {}, {}}}, panda_limits());
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("peak velocity"), std::string::npos) << *err;
  // Same move slow enough is accepted: peak 1.875*2/2 = 1.875 rad/s.
  EXPECT_FALSE(t.build({home(), {}, {}}, {{2.0, g, {}, {}}}, panda_limits()).has_value());
}

TEST(Trajectory, RejectsNaNAndInf) {
  Trajectory t;
  Vec g = home();
  g[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(t.build({home(), {}, {}}, {{1.0, g, {}, {}}}, panda_limits()).has_value());
  g[4] = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(t.build({home(), {}, {}}, {{1.0, g, {}, {}}}, panda_limits()).has_value());
  EXPECT_TRUE(t.build({home(), {}, {}}, {{std::nan(""), home(), {}, {}}}, panda_limits()).has_value());
}

TEST(Trajectory, RejectsBadTimingAndShape) {
  Trajectory t;
  EXPECT_TRUE(t.build({home(), {}, {}}, {}, panda_limits()).has_value());                        // empty
  EXPECT_TRUE(t.build({home(), {}, {}}, {{0.0, home(), {}, {}}}, panda_limits()).has_value());   // t=0
  EXPECT_TRUE(t.build({home(), {}, {}}, {{1.0, home(), {}, {}}, {0.8, home(), {}, {}}},
                      panda_limits()).has_value());                                              // not increasing
  EXPECT_TRUE(t.build({home(), {}, {}}, {{1.0, Vec(6, 0.0), {}, {}}}, panda_limits()).has_value());  // size
}

TEST(Controller, RejectedGoalKeepsHolding) {
  ControllerCore c(panda_limits(), panda_gains(), 0.5);
  c.hold(home());
  Vec g = home();
  g[5] = 4.0;  // over limit
  auto r = c.accept_goal({{1.0, g, {}, {}}}, 0.0, 0.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(c.mode(), Mode::kHold);
  EXPECT_EQ(c.rejected_goals(), 1u);
  for (size_t j = 0; j < 7; ++j) EXPECT_DOUBLE_EQ(c.desired(0.1).pos[j], home()[j]);
}
