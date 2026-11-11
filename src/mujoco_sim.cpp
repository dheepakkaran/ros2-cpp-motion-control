#include "arm_motion_control/mujoco_sim.hpp"

#include <stdexcept>

namespace arm_motion_control {

MujocoSim::MujocoSim(const std::string& mjcf_path, double timestep_s, const std::string& keyframe) {
  char err[1000] = "";
  m_ = mj_loadXML(mjcf_path.c_str(), nullptr, err, sizeof(err));
  if (!m_) throw std::runtime_error("mj_loadXML failed: " + std::string(err));
  m_->opt.timestep = timestep_s;
  m_->opt.disableflags |= mjDSBL_ACTUATION;  // torque control via qfrc_applied
  d_ = mj_makeData(m_);
  for (int j = 0; j < m_->njnt; ++j) {
    if (m_->jnt_type[j] != mjJNT_HINGE) continue;
    names_.emplace_back(mj_id2name(m_, mjOBJ_JOINT, j));
    qadr_.push_back(m_->jnt_qposadr[j]);
    dadr_.push_back(m_->jnt_dofadr[j]);
  }
  int key = mj_name2id(m_, mjOBJ_KEY, keyframe.c_str());
  if (key >= 0) mj_resetDataKeyframe(m_, d_, key);
  mj_forward(m_, d_);
}

MujocoSim::~MujocoSim() {
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

JointLimits MujocoSim::limits(const Vec& max_vel) const {
  JointLimits l;
  for (std::size_t i = 0; i < names_.size(); ++i) {
    int j = mj_name2id(m_, mjOBJ_JOINT, names_[i].c_str());
    l.lower.push_back(m_->jnt_range[2 * j]);
    l.upper.push_back(m_->jnt_range[2 * j + 1]);
  }
  l.max_vel = max_vel;
  return l;
}

double MujocoSim::timestep() const { return m_->opt.timestep; }
double MujocoSim::time() const { return d_->time; }

Vec MujocoSim::q() const {
  Vec out;
  for (int a : qadr_) out.push_back(d_->qpos[a]);
  return out;
}
Vec MujocoSim::v() const {
  Vec out;
  for (int a : dadr_) out.push_back(d_->qvel[a]);
  return out;
}
Vec MujocoSim::bias() const {
  Vec out;
  for (int a : dadr_) out.push_back(d_->qfrc_bias[a]);
  return out;
}

void MujocoSim::step(const Vec& tau) {
  mju_zero(d_->qfrc_applied, m_->nv);
  for (std::size_t i = 0; i < dadr_.size(); ++i) d_->qfrc_applied[dadr_[i]] = tau[i];
  mj_step(m_, d_);
  mj_forward(m_, d_);  // refresh qfrc_bias for the new state before the next control tick
}

}  // namespace arm_motion_control
