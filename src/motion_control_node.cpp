#include "arm_motion_control/motion_control_node.hpp"

#include <chrono>
#include <unordered_map>

namespace arm_motion_control {

MotionControlNode::MotionControlNode(const rclcpp::NodeOptions& opts) : Node("motion_control", opts) {
  const auto model = declare_parameter<std::string>("model_path", ARM_MOTION_CONTROL_DEFAULT_MODEL);
  const double dt = declare_parameter<double>("sim_timestep", 0.001);
  const double timeout = declare_parameter<double>("cmd_timeout", 0.5);
  publish_every_ = declare_parameter<int>("publish_every_n_steps", 4);
  max_catchup_ = declare_parameter<int>("max_catchup_steps", 20);
  // Franka Panda datasheet joint velocity limits (rad/s).
  const auto vmax = declare_parameter<std::vector<double>>(
      "max_velocity", {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61});
  const auto kp = declare_parameter<std::vector<double>>("kp", {600, 600, 600, 600, 250, 150, 50});
  const auto kd = declare_parameter<std::vector<double>>("kd", {50, 50, 50, 50, 20, 15, 5});
  const auto tmax = declare_parameter<std::vector<double>>("max_torque", {87, 87, 87, 87, 12, 12, 12});

  sim_ = std::make_unique<MujocoSim>(model, dt);
  ctrl_ = std::make_unique<ControllerCore>(sim_->limits(vmax), Gains{kp, kd, tmax}, timeout);
  ctrl_->hold(sim_->q());

  pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory", rclcpp::QoS(10),
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr m) { on_trajectory(m); });
  t0_ = now_s();
  timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] { on_timer(); });
  RCLCPP_INFO(get_logger(), "MuJoCo model %s, %zu joints, dt=%.4f s, cmd_timeout=%.3f s", model.c_str(),
              sim_->dof(), dt, timeout);
}

void MotionControlNode::on_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mu_);
  const double now = t0_ + sim_->time();
  auto reject = [&](const std::string& why) {
    ++met_.goals_rejected;
    last_reject_ = why;
    RCLCPP_WARN(get_logger(), "goal rejected: %s", why.c_str());
  };
  const auto& names = sim_->joint_names();
  if (msg->joint_names.size() != names.size()) return reject("joint_names size mismatch");
  std::unordered_map<std::string, std::size_t> idx;
  for (std::size_t i = 0; i < msg->joint_names.size(); ++i) idx[msg->joint_names[i]] = i;
  std::vector<std::size_t> order;
  for (const auto& n : names) {
    auto it = idx.find(n);
    if (it == idx.end()) return reject("missing joint " + n);
    order.push_back(it->second);
  }
  std::vector<Waypoint> wps;
  for (const auto& p : msg->points) {
    Waypoint w;
    w.time_from_start = rclcpp::Duration(p.time_from_start).seconds();
    auto pick = [&](const std::vector<double>& src, Vec& dst) -> bool {
      if (src.empty()) return true;
      if (src.size() != names.size()) return false;
      for (auto o : order) dst.push_back(src[o]);
      return true;
    };
    if (!pick(p.positions, w.pos) || !pick(p.velocities, w.vel) || !pick(p.accelerations, w.acc))
      return reject("point field size mismatch");
    wps.push_back(std::move(w));
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double start = (stamp.nanoseconds() == 0) ? now : stamp.seconds();
  auto res = ctrl_->accept_goal(wps, start, now);
  if (!res.accepted) return reject(res.reason);
  ++met_.goals_accepted;
}

void MotionControlNode::on_timer() {
  std::lock_guard<std::mutex> lk(mu_);
  ++met_.timer_ticks;
  const double target = now_s() - t0_;
  int steps = 0;
  while (sim_->time() + 0.5 * sim_->timestep() < target) {
    if (steps == max_catchup_) {
      // Too far behind wall time: drop the backlog instead of bursting.
      t0_ = now_s() - sim_->time();
      ++met_.lag_resets;
      break;
    }
    control_step(t0_ + sim_->time());
    ++steps;
  }
}

void MotionControlNode::control_step(double now) {
  const Vec q = sim_->q(), v = sim_->v();
  const Vec tau = ctrl_->update(now, q, v, sim_->bias());
  if (ctrl_->mode() == Mode::kTracking) {
    const auto& des = ctrl_->last_desired();
    for (std::size_t j = 0; j < q.size(); ++j) {
      const double e = des.pos[j] - q[j];
      met_.sum_sq_err += e * e;
      met_.max_abs_err = std::max(met_.max_abs_err, std::abs(e));
      ++met_.samples;
    }
  }
  met_.watchdog_trips = ctrl_->watchdog_trips();
  met_.goals_reanchored = ctrl_->reanchored_goals();
  sim_->step(tau);
  ++met_.sim_steps;
  if (met_.sim_steps % publish_every_ == 0) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = rclcpp::Time(static_cast<int64_t>((t0_ + sim_->time()) * 1e9), get_clock()->get_clock_type());
    js.name = sim_->joint_names();
    js.position = sim_->q();
    js.velocity = sim_->v();
    js.effort = tau;
    pub_->publish(js);
  }
}

TrackingMetrics MotionControlNode::metrics() const {
  std::lock_guard<std::mutex> lk(mu_);
  return met_;
}
void MotionControlNode::reset_metrics() {
  std::lock_guard<std::mutex> lk(mu_);
  met_ = TrackingMetrics{};
}
Mode MotionControlNode::mode() const {
  std::lock_guard<std::mutex> lk(mu_);
  return ctrl_->mode();
}
Vec MotionControlNode::joint_positions() const {
  std::lock_guard<std::mutex> lk(mu_);
  return sim_->q();
}
std::string MotionControlNode::last_reject_reason() const {
  std::lock_guard<std::mutex> lk(mu_);
  return last_reject_;
}
double MotionControlNode::sim_time() const {
  std::lock_guard<std::mutex> lk(mu_);
  return sim_->time();
}

}  // namespace arm_motion_control
