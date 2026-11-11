// Command watchdog: trips when no command arrived for `timeout` seconds.
#pragma once

namespace arm_motion_control {

class Watchdog {
 public:
  explicit Watchdog(double timeout_s = 0.5) : timeout_(timeout_s) {}
  void set_timeout(double s) { timeout_ = s; }
  double timeout() const { return timeout_; }
  void feed(double now) { last_ = now; armed_ = true; }
  void disarm() { armed_ = false; }
  bool armed() const { return armed_; }
  // timeout <= 0 disables the watchdog.
  bool expired(double now) const { return armed_ && timeout_ > 0.0 && (now - last_) > timeout_; }

 private:
  double timeout_;
  double last_ = 0.0;
  bool armed_ = false;
};

}  // namespace arm_motion_control
