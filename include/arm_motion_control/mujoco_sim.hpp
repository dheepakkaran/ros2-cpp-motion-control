// Thin RAII wrapper over the MuJoCo C API for a torque-controlled arm.
#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "arm_motion_control/trajectory.hpp"

namespace arm_motion_control {

class MujocoSim {
 public:
  // Loads the MJCF, sets the physics step, disables the model's position actuators
  // (we apply torques ourselves through qfrc_applied).
  MujocoSim(const std::string& mjcf_path, double timestep_s = 0.001, const std::string& keyframe = "home");
  ~MujocoSim();
  MujocoSim(const MujocoSim&) = delete;
  MujocoSim& operator=(const MujocoSim&) = delete;

  std::size_t dof() const { return names_.size(); }
  const std::vector<std::string>& joint_names() const { return names_; }
  JointLimits limits(const Vec& max_vel) const;
  double timestep() const;
  double time() const;

  Vec q() const;
  Vec v() const;
  Vec bias() const;  // qfrc_bias after mj_forward (gravity + Coriolis + centrifugal)
  void step(const Vec& tau);

 private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  std::vector<std::string> names_;
  std::vector<int> qadr_, dadr_;
};

}  // namespace arm_motion_control
