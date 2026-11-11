// Integration test: the real rclcpp node (MuJoCo Panda inside) + a client node in one process,
// spun by an executor. Talks only over ROS topics (/joint_trajectory, /joint_states).
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_motion_control/motion_control_node.hpp"

using namespace std::chrono_literals;
using arm_motion_control::Mode;
using arm_motion_control::MotionControlNode;
using arm_motion_control::Vec;

namespace {

struct Fixture {
  std::shared_ptr<MotionControlNode> ctrl;
  rclcpp::Node::SharedPtr client;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;
  std::mutex mu;
  sensor_msgs::msg::JointState last_js;
  std::atomic<int> js_count{0};
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 2};
  std::thread spin;

  explicit Fixture(double cmd_timeout) {
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"cmd_timeout", cmd_timeout}});
    ctrl = std::make_shared<MotionControlNode>(o);
    client = std::make_shared<rclcpp::Node>("test_client");
    pub = client->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
    sub = client->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mu);
          last_js = *m;
          ++js_count;
        });
    exec.add_node(ctrl);
    exec.add_node(client);
    spin = std::thread([this] { exec.spin(); });
    // Wait for discovery both ways.
    for (int i = 0; i < 200 && (js_count < 5 || pub->get_subscription_count() == 0); ++i) std::this_thread::sleep_for(10ms);
  }
  ~Fixture() {
    exec.cancel();
    spin.join();
  }
  trajectory_msgs::msg::JointTrajectory make_goal(const Vec& q, double T, rclcpp::Time stamp) {
    trajectory_msgs::msg::JointTrajectory m;
    m.header.stamp = stamp;
    m.joint_names = ctrl->joint_names();
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = q;
    p.time_from_start = rclcpp::Duration::from_seconds(T);
    m.points.push_back(p);
    return m;
  }
  Vec js_pos() {
    std::lock_guard<std::mutex> lk(mu);
    return last_js.position;
  }
};

Vec goal_pose() { return {0.6, 0.4, 0.0, -1.9, 0.0, 1.9, -0.7853}; }

std::string g_json;  // filled by the tests, written at the end

}  // namespace

TEST(Integration, GoalReachedWithLowTrackingError) {
  Fixture f(0.5);
  ASSERT_GT(f.js_count.load(), 0) << "no /joint_states received";
  const double T = 2.0;
  const Vec goal = goal_pose();
  f.ctrl->reset_metrics();
  const auto wall0 = std::chrono::steady_clock::now();
  const double sim0 = f.ctrl->sim_time();
  // Streaming client: same goal + same stamp re-sent every 100 ms while it runs (feeds the watchdog).
  const auto stamp = f.ctrl->get_clock()->now();
  const auto msg = f.make_goal(goal, T, stamp);
  for (int k = 0; k <= static_cast<int>(T / 0.1); ++k) {
    f.pub->publish(msg);
    std::this_thread::sleep_for(100ms);
  }
  std::this_thread::sleep_for(1000ms);  // settle
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
  const double sim = f.ctrl->sim_time() - sim0;
  const auto m = f.ctrl->metrics();
  const Vec q = f.js_pos();
  ASSERT_EQ(q.size(), goal.size());
  double final_err = 0;
  for (size_t j = 0; j < q.size(); ++j) final_err = std::max(final_err, std::abs(goal[j] - q[j]));
  EXPECT_EQ(f.ctrl->mode(), Mode::kHold);
  EXPECT_EQ(m.watchdog_trips, 0u);
  EXPECT_EQ(m.goals_rejected, 0u);
  EXPECT_LT(final_err, 0.01);
  EXPECT_LT(m.rms(), 0.02);
  std::ostringstream js;
  js << "  \"goal_tracking\": {\"move_duration_s\": " << T << ", \"final_joint_err_max_rad\": " << final_err
     << ", \"rms_tracking_err_rad\": " << m.rms() << ", \"max_tracking_err_rad\": " << m.max_abs_err
     << ", \"goals_accepted\": " << m.goals_accepted << ", \"goals_reanchored\": " << m.goals_reanchored << ", \"watchdog_trips\": " << m.watchdog_trips
     << ", \"wall_s\": " << wall << ", \"sim_s\": " << sim << ", \"sim_steps\": " << m.sim_steps
     << ", \"sim_steps_per_wall_s\": " << m.sim_steps / wall << ", \"timer_ticks_per_wall_s\": " << m.timer_ticks / wall
     << ", \"lag_resets\": " << m.lag_resets << ", \"joint_states_msgs\": " << f.js_count.load() << "}";
  g_json += js.str();
}

TEST(Integration, WatchdogHoldsWhenCommandsStop) {
  Fixture f(0.3);
  const auto msg = f.make_goal(goal_pose(), 3.0, f.ctrl->get_clock()->now());
  for (int k = 0; k < 5; ++k) {  // 0.5 s of commands, then silence
    f.pub->publish(msg);
    std::this_thread::sleep_for(100ms);
  }
  std::this_thread::sleep_for(600ms);
  EXPECT_EQ(f.ctrl->mode(), Mode::kWatchdogHold);
  const Vec q1 = f.js_pos();
  std::this_thread::sleep_for(1000ms);
  const Vec q2 = f.js_pos();
  double drift = 0;
  for (size_t j = 0; j < q1.size(); ++j) drift = std::max(drift, std::abs(q2[j] - q1[j]));
  EXPECT_LT(drift, 0.005);
  EXPECT_EQ(f.ctrl->metrics().watchdog_trips, 1u);
  g_json += ",\n  \"watchdog\": {\"cmd_timeout_s\": 0.3, \"trips\": " + std::to_string(f.ctrl->metrics().watchdog_trips) +
            ", \"drift_over_1s_after_trip_rad\": " + std::to_string(drift) + "}";
}

TEST(Integration, BadGoalRejectedOverRos) {
  Fixture f(0.5);
  const Vec q0 = f.js_pos();
  Vec bad = goal_pose();
  bad[2] = std::numeric_limits<double>::quiet_NaN();
  f.pub->publish(f.make_goal(bad, 1.0, rclcpp::Time(0, 0, RCL_ROS_TIME)));
  Vec over = goal_pose();
  over[1] = 2.5;  // joint2 limit 1.7628
  f.pub->publish(f.make_goal(over, 1.0, rclcpp::Time(0, 0, RCL_ROS_TIME)));
  std::this_thread::sleep_for(500ms);
  const auto m = f.ctrl->metrics();
  EXPECT_EQ(m.goals_rejected, 2u);
  EXPECT_EQ(m.goals_accepted, 0u);
  EXPECT_EQ(f.ctrl->mode(), Mode::kHold);
  const Vec q1 = f.js_pos();
  double drift = 0;
  for (size_t j = 0; j < q1.size(); ++j) drift = std::max(drift, std::abs(q1[j] - q0[j]));
  EXPECT_LT(drift, 0.005);
  g_json += ",\n  \"bad_goal_over_ros\": {\"rejected\": " + std::to_string(m.goals_rejected) +
            ", \"last_reason\": \"" + f.ctrl->last_reject_reason() + "\", \"drift_rad\": " + std::to_string(drift) + "}";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::ofstream(std::string(RESULTS_DIR) + "/metrics.json") << "{\n" << g_json << "\n}\n";
  return rc;
}
