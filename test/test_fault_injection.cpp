// Fault injection on the controller core + MuJoCo Panda (sim time, deterministic).
// Faults: dropped commands, a command burst that stops, late commands, bad goals mid-motion.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <random>
#include <sstream>

#include "arm_motion_control/mujoco_sim.hpp"
#include "common.hpp"

using namespace amc_test;

namespace {

struct RunStats {
  double rms_err = 0, max_err = 0;     // tracking error while mode == tracking (rad)
  double max_ref_jump = 0;             // largest change of q_des between two 1 ms ticks (rad), trip tick excluded
  double trip_ref_jump = 0;            // q_des step on the tick the watchdog trips (desired -> measured q)
  double err_before_trip = 0;          // max |q_des - q| on the tick before the trip
  int saturated_ticks = 0;             // ticks where any joint torque hit its limit
  int watchdog_trips = 0, rejected = 0, accepted = 0, reanchored = 0;
  double final_err = 0;                // max |goal - q| at the end (rad)
  double hold_drift = 0;               // max |q - q_at_trip| after the watchdog trip (rad)
  double max_cmd_gap = 0;              // largest gap between delivered commands (s)
};

std::map<std::string, RunStats> g_results;

Vec goal_pose() {
  Vec g = home();
  g[0] = 0.6;
  g[1] = 0.4;
  g[3] = -1.9;
  g[5] = 1.9;
  return g;
}

// on_tick(t) returns true when a command (the goal) is delivered at sim time t, and sets its stamp.
RunStats run(double duration, double timeout,
             const std::function<bool(double t, double& stamp, std::vector<Waypoint>& wps)>& on_tick) {
  MujocoSim sim(MODEL_PATH, 0.001);
  ControllerCore c(sim.limits(panda_limits().max_vel), panda_gains(), timeout);
  c.hold(sim.q());
  RunStats s;
  double sum_sq = 0;
  size_t n_sq = 0;
  Vec prev_des;
  double last_cmd = -1, trip_t = -1;
  Vec q_trip;
  const auto gains = panda_gains();
  const int steps = static_cast<int>(std::lround(duration / sim.timestep()));
  for (int k = 0; k < steps; ++k) {
    const double t = sim.time();
    double stamp = t;
    std::vector<Waypoint> wps;
    if (on_tick(t, stamp, wps)) {
      if (last_cmd >= 0) s.max_cmd_gap = std::max(s.max_cmd_gap, t - last_cmd);
      last_cmd = t;
      auto r = c.accept_goal(wps, stamp, t);
      r.accepted ? ++s.accepted : ++s.rejected;
    }
    const Vec q = sim.q();
    const Mode before = c.mode();
    const Vec tau = c.update(t, q, sim.v(), sim.bias());
    const auto& des = c.last_desired();
    const bool trip_tick = before == Mode::kTracking && c.mode() == Mode::kWatchdogHold;
    if (!prev_des.empty())
      for (size_t j = 0; j < q.size(); ++j) {
        const double jump = std::abs(des.pos[j] - prev_des[j]);
        if (trip_tick) {
          s.trip_ref_jump = std::max(s.trip_ref_jump, jump);
          s.err_before_trip = std::max(s.err_before_trip, std::abs(prev_des[j] - q[j]));
        } else {
          s.max_ref_jump = std::max(s.max_ref_jump, jump);
        }
      }
    prev_des = des.pos;
    if (c.mode() == Mode::kTracking)
      for (size_t j = 0; j < q.size(); ++j) {
        const double e = des.pos[j] - q[j];
        sum_sq += e * e;
        ++n_sq;
        s.max_err = std::max(s.max_err, std::abs(e));
      }
    for (size_t j = 0; j < q.size(); ++j)
      if (std::abs(tau[j]) >= gains.max_torque[j] - 1e-9) { ++s.saturated_ticks; break; }
    if (c.mode() == Mode::kWatchdogHold) {
      if (trip_t < 0) { trip_t = t; q_trip = q; }
      for (size_t j = 0; j < q.size(); ++j) s.hold_drift = std::max(s.hold_drift, std::abs(q[j] - q_trip[j]));
    }
    sim.step(tau);
  }
  s.rms_err = n_sq ? std::sqrt(sum_sq / n_sq) : 0;
  s.watchdog_trips = static_cast<int>(c.watchdog_trips());
  s.reanchored = static_cast<int>(c.reanchored_goals());
  const Vec q = sim.q(), g = goal_pose();
  for (size_t j = 0; j < q.size(); ++j) s.final_err = std::max(s.final_err, std::abs(g[j] - q[j]));
  return s;
}

const double kMoveT = 2.0;
std::vector<Waypoint> goal_wps() { return {{kMoveT, goal_pose(), {}, {}}}; }

// Streaming client: re-sends the same goal (same stamp) every `period` seconds from t0 until t_stop.
auto streaming(double period, double t0, double t_stop, double drop_p, unsigned seed, double latency = 0.0) {
  auto rng = std::make_shared<std::mt19937>(seed);
  auto next = std::make_shared<double>(t0);
  return [=](double t, double& stamp, std::vector<Waypoint>& wps) {
    if (t + 1e-9 < *next || t > t_stop) return false;
    *next += period;
    std::uniform_real_distribution<double> u(0, 1);
    if (drop_p > 0 && u(*rng) < drop_p && t > t0 + 1e-9) return false;  // dropped (first send always arrives)
    stamp = t0 - latency;  // stamp = when the client planned the move
    wps = goal_wps();
    return true;
  };
}

constexpr double kSettle = 1.0;
const double kRunT = kMoveT + kSettle;

}  // namespace

TEST(FaultInjection, BaselineStreaming) {
  auto s = run(kRunT, 0.5, streaming(0.1, 0.0, kMoveT, 0.0, 1));
  g_results["baseline_streaming"] = s;
  EXPECT_EQ(s.watchdog_trips, 0);
  EXPECT_LT(s.final_err, 0.01);
  EXPECT_LT(s.max_ref_jump, 2.61 * 0.001 * 1.01);  // reference never jumps faster than the velocity limit
}

TEST(FaultInjection, DroppedMessages30Percent) {
  auto s = run(kRunT, 0.5, streaming(0.1, 0.0, kMoveT, 0.3, 42));
  g_results["dropped_30pct"] = s;
  EXPECT_LT(s.max_cmd_gap, 0.5);
  EXPECT_EQ(s.watchdog_trips, 0);
  EXPECT_LT(s.final_err, 0.01);
  EXPECT_LT(s.max_ref_jump, 2.61 * 0.001 * 1.01);
}

TEST(FaultInjection, CommandsStopMidMotion) {
  // Client dies at t = 0.8 s, mid-move. Watchdog (0.5 s) must trip and hold the arm still.
  auto s = run(kRunT, 0.5, streaming(0.1, 0.0, 0.8, 0.0, 1));
  g_results["commands_stop_mid_motion"] = s;
  EXPECT_EQ(s.watchdog_trips, 1);
  EXPECT_LE(s.trip_ref_jump, s.err_before_trip + 2.61 * 0.001);  // trip step = tracking error at that tick
  EXPECT_LT(s.hold_drift, 0.05);  // arm stops near where it was when the watchdog tripped
  EXPECT_LT(s.max_ref_jump, 2.61 * 0.001 * 1.01);
}

TEST(FaultInjection, LateMessages300ms) {
  // Every command arrives 300 ms after its stamp (network / executor delay).
  auto s = run(kRunT + 0.3, 0.5, streaming(0.1, 0.3, kMoveT + 0.3, 0.0, 1, 0.3));
  g_results["late_300ms"] = s;
  EXPECT_EQ(s.reanchored, 1);  // first late send re-anchored, re-sends reuse the same shift
  EXPECT_LT(s.final_err, 0.01);
  EXPECT_LT(s.max_ref_jump, 2.61 * 0.001 * 1.01);
}

TEST(FaultInjection, StaleMessageOlderThanTrajectory) {
  // One command whose stamp is 5 s old: the whole trajectory is already in the past.
  bool sent = false;
  auto s = run(kRunT, 0.5, [&](double t, double& stamp, std::vector<Waypoint>& wps) {
    if (sent || t < 0.2) return false;
    sent = true;
    stamp = t - 5.0;
    wps = goal_wps();
    return true;
  });
  g_results["stale_5s"] = s;
  EXPECT_EQ(s.reanchored, 1);        // executed from now instead of jumping to the end
  EXPECT_EQ(s.watchdog_trips, 1);    // single message, no refresh -> watchdog stops it at 0.5 s
  EXPECT_LT(s.max_ref_jump, 2.61 * 0.001 * 1.01);
  EXPECT_LE(s.trip_ref_jump, s.err_before_trip + 2.61 * 0.001);
  EXPECT_EQ(s.saturated_ticks, 0);
}

TEST(FaultInjection, OverLimitAndNaNGoalsMidMotion) {
  // Good goal streamed; at 0.5 s an over-limit goal and at 0.7 s a NaN goal arrive. Both rejected,
  // motion continues to the original goal.
  auto good = streaming(0.1, 0.0, kMoveT, 0.0, 1);
  auto s = run(kRunT, 0.5, [&](double t, double& stamp, std::vector<Waypoint>& wps) {
    if (std::abs(t - 0.5) < 1e-6 || std::abs(t - 0.7) < 1e-6) {
      Vec bad = goal_pose();
      bad[1] = (t < 0.6) ? 2.5 : std::numeric_limits<double>::quiet_NaN();
      stamp = t;
      wps = {{1.0, bad, {}, {}}};
      return true;
    }
    return good(t, stamp, wps);
  });
  g_results["bad_goals_mid_motion"] = s;
  EXPECT_EQ(s.rejected, 2);
  EXPECT_LT(s.final_err, 0.01);
  EXPECT_LT(s.max_ref_jump, 2.61 * 0.001 * 1.01);
}

class WriteResults : public ::testing::Environment {
 public:
  void TearDown() override {
    std::ofstream f(std::string(RESULTS_DIR) + "/fault_injection.json");
    f << "{\n";
    size_t i = 0;
    for (auto& [name, s] : g_results) {
      f << "  \"" << name << "\": {\"rms_tracking_err_rad\": " << s.rms_err << ", \"max_tracking_err_rad\": " << s.max_err
        << ", \"max_ref_jump_rad_per_tick\": " << s.max_ref_jump << ", \"trip_ref_jump_rad\": " << s.trip_ref_jump << ", \"err_before_trip_rad\": " << s.err_before_trip << ", \"saturated_ticks\": " << s.saturated_ticks
        << ", \"watchdog_trips\": " << s.watchdog_trips << ", \"goals_accepted\": " << s.accepted
        << ", \"goals_rejected\": " << s.rejected << ", \"goals_reanchored\": " << s.reanchored << ", \"final_err_rad\": " << s.final_err
        << ", \"hold_drift_rad\": " << s.hold_drift << ", \"max_cmd_gap_s\": " << s.max_cmd_gap << "}"
        << (++i < g_results.size() ? "," : "") << "\n";
    }
    f << "}\n";
  }
};

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new WriteResults);
  return RUN_ALL_TESTS();
}
