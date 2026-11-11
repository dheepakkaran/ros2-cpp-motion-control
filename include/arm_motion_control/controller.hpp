// Controller core (no ROS): goal handling, watchdog, PD + gravity compensation.
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "arm_motion_control/trajectory.hpp"
#include "arm_motion_control/watchdog.hpp"

namespace arm_motion_control {

enum class Mode { kHold, kTracking, kWatchdogHold };

inline const char* to_string(Mode m) {
  switch (m) {
    case Mode::kHold: return "hold";
    case Mode::kTracking: return "tracking";
    case Mode::kWatchdogHold: return "watchdog_hold";
  }
  return "?";
}

struct Gains {
  Vec kp, kd;
  Vec max_torque;
};

struct GoalResult {
  bool accepted = false;
  std::string reason;
};

class ControllerCore {
 public:
  ControllerCore(JointLimits lim, Gains g, double cmd_timeout_s)
      : lim_(std::move(lim)), g_(std::move(g)), wd_(cmd_timeout_s) {}

  const JointLimits& limits() const { return lim_; }
  Watchdog& watchdog() { return wd_; }
  Mode mode() const { return mode_; }
  std::size_t rejected_goals() const { return rejected_; }
  std::size_t watchdog_trips() const { return trips_; }
  std::size_t reanchored_goals() const { return reanchored_; }

  // Hold the given position (zero velocity).
  void hold(const Vec& q) {
    hold_ = q;
    mode_ = Mode::kHold;
  }

  // New goal. start_time: absolute time (s) the trajectory's t=0 maps to.
  // now: current time. q/v: measured state (used when idle).
  GoalResult accept_goal(const std::vector<Waypoint>& wps, double start_time, double now) {
    const double stamp = start_time;
    // A re-send of the active goal (same stamp) keeps the time shift chosen when it was re-anchored.
    if (mode_ == Mode::kTracking && stamp == active_stamp_) start_time += active_offset_;
    // The new trajectory starts from the current plan's state at its own t=0 (start_time).
    // See docs/rca/RCA-001: using desired(now) here made every re-send jump the reference.
    Trajectory t;
    if (auto err = t.build(desired(start_time), wps, lim_)) {
      ++rejected_;
      return {false, *err};
    }
    // Continuity check at `now`: a late/stale or future stamp (RCA-002) would make the reference
    // jump. If it would, re-anchor the goal to start now from the current desired state.
    const JointState cur = desired(now), nxt = t.sample(now - start_time);
    bool continuous = true;
    for (std::size_t j = 0; j < lim_.size(); ++j)
      if (std::abs(cur.pos[j] - nxt.pos[j]) > 1e-6 || std::abs(cur.vel[j] - nxt.vel[j]) > 1e-6) continuous = false;
    if (!continuous) {
      if (auto err = t.build(cur, wps, lim_)) {
        ++rejected_;
        return {false, *err};
      }
      start_time = now;
      ++reanchored_;
    }
    active_stamp_ = stamp;
    active_offset_ = start_time - stamp;
    traj_ = std::move(t);
    traj_start_ = start_time;
    mode_ = Mode::kTracking;
    wd_.feed(now);
    return {true, ""};
  }

  // Desired state at time `now`.
  JointState desired(double now) const {
    const std::size_t n = lim_.size();
    if (mode_ == Mode::kTracking) return traj_.sample(now - traj_start_);
    return JointState{hold_, Vec(n, 0.0), Vec(n, 0.0)};
  }

  // One control tick: returns joint torques. bias = MuJoCo qfrc_bias (gravity + Coriolis).
  Vec update(double now, const Vec& q, const Vec& v, const Vec& bias) {
    const std::size_t n = lim_.size();
    if (hold_.empty()) hold_ = q;
    if (mode_ == Mode::kTracking && wd_.expired(now)) {
      // Commands stopped: freeze where the arm is now.
      hold_ = q;
      mode_ = Mode::kWatchdogHold;
      wd_.disarm();
      ++trips_;
    }
    if (mode_ == Mode::kTracking && now - traj_start_ >= traj_.duration()) {
      // Finished: hold the final point.
      hold_ = traj_.sample(traj_.duration()).pos;
      mode_ = Mode::kHold;
      wd_.disarm();
    }
    last_des_ = desired(now);
    Vec tau(n);
    for (std::size_t j = 0; j < n; ++j) {
      double u = g_.kp[j] * (last_des_.pos[j] - q[j]) + g_.kd[j] * (last_des_.vel[j] - v[j]) + bias[j];
      tau[j] = std::clamp(u, -g_.max_torque[j], g_.max_torque[j]);
    }
    return tau;
  }

  const JointState& last_desired() const { return last_des_; }

 private:
  JointLimits lim_;
  Gains g_;
  Watchdog wd_;
  Mode mode_ = Mode::kHold;
  Vec hold_;
  Trajectory traj_;
  double traj_start_ = 0.0;
  double active_stamp_ = -1e300, active_offset_ = 0.0;
  JointState last_des_;
  std::size_t rejected_ = 0, trips_ = 0, reanchored_ = 0;
};

}  // namespace arm_motion_control
