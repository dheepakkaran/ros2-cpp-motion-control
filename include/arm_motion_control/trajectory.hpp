// Quintic joint-space trajectories with limit checks. Header-only, no ROS.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace arm_motion_control {

using Vec = std::vector<double>;

struct JointLimits {
  Vec lower;    // rad
  Vec upper;    // rad
  Vec max_vel;  // rad/s
  std::size_t size() const { return lower.size(); }
};

struct JointState {
  Vec pos, vel, acc;
};

// One waypoint of a goal, time measured from trajectory start.
struct Waypoint {
  double time_from_start = 0.0;
  Vec pos;
  Vec vel;  // empty = 0
  Vec acc;  // empty = 0
};

// Quintic polynomial for one joint: matches pos/vel/acc at both ends.
struct Quintic {
  double c[6] = {0, 0, 0, 0, 0, 0};
  double T = 0.0;

  static Quintic fit(double p0, double v0, double a0, double p1, double v1, double a1, double T) {
    Quintic q;
    q.T = T;
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    q.c[0] = p0;
    q.c[1] = v0;
    q.c[2] = a0 / 2.0;
    q.c[3] = (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3);
    q.c[4] = (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 * T4);
    q.c[5] = (12 * (p1 - p0) - 6 * (v1 + v0) * T - (a0 - a1) * T2) / (2 * T5);
    return q;
  }
  double pos(double t) const {
    return c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))));
  }
  double vel(double t) const {
    return c[1] + t * (2 * c[2] + t * (3 * c[3] + t * (4 * c[4] + t * 5 * c[5])));
  }
  double acc(double t) const {
    return 2 * c[2] + t * (6 * c[3] + t * (12 * c[4] + t * 20 * c[5]));
  }
};

inline bool all_finite(const Vec& v) {
  return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
}

// Piecewise quintic through a start state and a list of waypoints.
class Trajectory {
 public:
  // Builds and validates. Returns an error string on rejection (nullopt = ok).
  std::optional<std::string> build(const JointState& start, const std::vector<Waypoint>& wps,
                                   const JointLimits& lim) {
    segs_.clear();
    times_.clear();
    const std::size_t n = lim.size();
    if (start.pos.size() != n) return "start state has wrong size";
    if (wps.empty()) return "goal has no points";
    double prev_t = 0.0;
    JointState a = start;
    if (a.vel.empty()) a.vel.assign(n, 0.0);
    if (a.acc.empty()) a.acc.assign(n, 0.0);
    times_.push_back(0.0);
    for (std::size_t k = 0; k < wps.size(); ++k) {
      const Waypoint& w = wps[k];
      const std::string at = "point " + std::to_string(k) + ": ";
      if (!std::isfinite(w.time_from_start)) return at + "time_from_start is not finite";
      if (w.time_from_start <= prev_t) return at + "time_from_start must increase and be > 0";
      if (w.pos.size() != n) return at + "wrong number of positions";
      if (!all_finite(w.pos)) return at + "position is NaN/Inf";
      if (!w.vel.empty() && (w.vel.size() != n || !all_finite(w.vel))) return at + "bad velocity";
      if (!w.acc.empty() && (w.acc.size() != n || !all_finite(w.acc))) return at + "bad acceleration";
      for (std::size_t j = 0; j < n; ++j) {
        if (w.pos[j] < lim.lower[j] || w.pos[j] > lim.upper[j])
          return at + "joint " + std::to_string(j) + " position " + std::to_string(w.pos[j]) +
                 " outside [" + std::to_string(lim.lower[j]) + ", " + std::to_string(lim.upper[j]) + "]";
        if (!w.vel.empty() && std::abs(w.vel[j]) > lim.max_vel[j])
          return at + "joint " + std::to_string(j) + " waypoint velocity over limit";
      }
      JointState b{w.pos, w.vel.empty() ? Vec(n, 0.0) : w.vel, w.acc.empty() ? Vec(n, 0.0) : w.acc};
      const double T = w.time_from_start - prev_t;
      std::vector<Quintic> seg(n);
      for (std::size_t j = 0; j < n; ++j)
        seg[j] = Quintic::fit(a.pos[j], a.vel[j], a.acc[j], b.pos[j], b.vel[j], b.acc[j], T);
      // Peak velocity and position excursion inside the segment (dense sampling).
      constexpr int kSamples = 200;
      for (int s = 0; s <= kSamples; ++s) {
        const double t = T * s / kSamples;
        for (std::size_t j = 0; j < n; ++j) {
          const double v = seg[j].vel(t), p = seg[j].pos(t);
          if (std::abs(v) > lim.max_vel[j] * (1.0 + 1e-9))
            return at + "joint " + std::to_string(j) + " peak velocity " + std::to_string(std::abs(v)) +
                   " rad/s over limit " + std::to_string(lim.max_vel[j]);
          if (p < lim.lower[j] - 1e-9 || p > lim.upper[j] + 1e-9)
            return at + "joint " + std::to_string(j) + " overshoots position limit inside segment";
        }
      }
      segs_.push_back(std::move(seg));
      times_.push_back(w.time_from_start);
      prev_t = w.time_from_start;
      a = b;
    }
    n_ = n;
    return std::nullopt;
  }

  bool empty() const { return segs_.empty(); }
  double duration() const { return times_.empty() ? 0.0 : times_.back(); }

  // Sample at time t from trajectory start (clamped to [0, duration]).
  JointState sample(double t) const {
    JointState s{Vec(n_), Vec(n_), Vec(n_)};
    if (segs_.empty()) return s;
    t = std::clamp(t, 0.0, duration());
    std::size_t k = 0;
    while (k + 1 < segs_.size() && t > times_[k + 1]) ++k;
    const double tl = t - times_[k];
    for (std::size_t j = 0; j < n_; ++j) {
      s.pos[j] = segs_[k][j].pos(tl);
      s.vel[j] = segs_[k][j].vel(tl);
      s.acc[j] = segs_[k][j].acc(tl);
    }
    return s;
  }

 private:
  std::size_t n_ = 0;
  std::vector<std::vector<Quintic>> segs_;
  Vec times_;
};

}  // namespace arm_motion_control
