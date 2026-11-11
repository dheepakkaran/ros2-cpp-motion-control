#include <gtest/gtest.h>

#include "arm_motion_control/watchdog.hpp"
#include "common.hpp"

using namespace amc_test;

TEST(Watchdog, TripsOnlyAfterTimeout) {
  Watchdog w(0.5);
  EXPECT_FALSE(w.expired(10.0));  // not armed
  w.feed(1.0);
  EXPECT_FALSE(w.expired(1.5));
  EXPECT_TRUE(w.expired(1.5001));
  w.feed(1.4);
  EXPECT_FALSE(w.expired(1.8));
}

TEST(Watchdog, ZeroTimeoutDisables) {
  Watchdog w(0.0);
  w.feed(0.0);
  EXPECT_FALSE(w.expired(1e6));
}

TEST(Controller, WatchdogHoldsMeasuredPositionWhenCommandsStop) {
  ControllerCore c(panda_limits(), panda_gains(), 0.2);
  c.hold(home());
  Vec g = home();
  g[0] = 1.0;
  ASSERT_TRUE(c.accept_goal({{2.0, g, {}, {}}}, 0.0, 0.0).accepted);
  Vec zero(7, 0.0);
  c.update(0.1, home(), zero, zero);
  EXPECT_EQ(c.mode(), Mode::kTracking);
  Vec q_now = home();
  q_now[0] = 0.05;
  c.update(0.25, q_now, zero, zero);  // 0.25 s since last command > 0.2 s
  EXPECT_EQ(c.mode(), Mode::kWatchdogHold);
  EXPECT_EQ(c.watchdog_trips(), 1u);
  auto d = c.desired(1.0);
  EXPECT_DOUBLE_EQ(d.pos[0], 0.05);  // holds where the arm was, not the goal
  EXPECT_DOUBLE_EQ(d.vel[0], 0.0);
  // At the held position with zero bias, torque is zero.
  auto tau = c.update(1.0, q_now, zero, zero);
  for (double t : tau) EXPECT_NEAR(t, 0.0, 1e-12);
}

TEST(Controller, RefreshedCommandsKeepTracking) {
  ControllerCore c(panda_limits(), panda_gains(), 0.2);
  c.hold(home());
  Vec g = home();
  g[0] = 1.0;
  Vec zero(7, 0.0);
  for (int k = 0; k < 20; ++k) {
    double now = 0.1 * k;
    ASSERT_TRUE(c.accept_goal({{2.0, g, {}, {}}}, 0.0, now).accepted);  // same goal, same stamp
    c.update(now + 0.05, c.desired(now).pos, zero, zero);
    EXPECT_EQ(c.mode(), Mode::kTracking) << k;
  }
  EXPECT_EQ(c.watchdog_trips(), 0u);
}

TEST(Controller, FinishedTrajectoryHoldsGoalWithoutWatchdog) {
  ControllerCore c(panda_limits(), panda_gains(), 0.2);
  c.hold(home());
  Vec g = home();
  g[2] = 0.1;
  ASSERT_TRUE(c.accept_goal({{0.15, g, {}, {}}}, 0.0, 0.0).accepted);
  Vec zero(7, 0.0);
  c.update(0.16, g, zero, zero);
  EXPECT_EQ(c.mode(), Mode::kHold);
  c.update(5.0, g, zero, zero);
  EXPECT_EQ(c.mode(), Mode::kHold);
  EXPECT_EQ(c.watchdog_trips(), 0u);
  EXPECT_NEAR(c.desired(5.0).pos[2], 0.1, 1e-9);
}
