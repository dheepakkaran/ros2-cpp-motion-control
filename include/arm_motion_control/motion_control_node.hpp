// rclcpp node: MuJoCo Panda sim + joint trajectory controller.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_motion_control/controller.hpp"
#include "arm_motion_control/mujoco_sim.hpp"

namespace arm_motion_control {

struct TrackingMetrics {
  double sum_sq_err = 0.0;   // sum over ticks and joints of (q_des - q)^2 while tracking
  std::size_t samples = 0;   // ticks * joints
  double max_abs_err = 0.0;  // rad
  std::size_t goals_accepted = 0, goals_rejected = 0, watchdog_trips = 0, goals_reanchored = 0;
  std::size_t sim_steps = 0, timer_ticks = 0, lag_resets = 0;
  double rms() const { return samples ? std::sqrt(sum_sq_err / samples) : 0.0; }
};

class MotionControlNode : public rclcpp::Node {
 public:
  explicit MotionControlNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

  TrackingMetrics metrics() const;
  void reset_metrics();
  Mode mode() const;
  Vec joint_positions() const;
  std::string last_reject_reason() const;
  const std::vector<std::string>& joint_names() const { return sim_->joint_names(); }
  double sim_time() const;

 private:
  void on_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void on_timer();
  void control_step(double now);
  double now_s() const { return get_clock()->now().seconds(); }

  std::unique_ptr<MujocoSim> sim_;
  std::unique_ptr<ControllerCore> ctrl_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t0_ = 0.0;  // ROS time that maps to sim time 0
  int publish_every_ = 4;
  int max_catchup_ = 20;
  mutable std::mutex mu_;
  TrackingMetrics met_;
  std::string last_reject_;
};

}  // namespace arm_motion_control
