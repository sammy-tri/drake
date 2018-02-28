#include "drake/manipulation/pick_and_place_example/world_state.h"

#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"

namespace drake {
namespace manipulation {
namespace pick_and_place_example {

WorldState::WorldState(int num_arm_joints,
                       int num_tables,
                       const Vector3<double>& object_dimensions)
    : object_dimensions_(object_dimensions) {

  arm_time_ = -1;
  arm_base_ = Isometry3<double>::Identity();
  arm_q_ = VectorX<double>::Zero(num_arm_joints);
  arm_v_ = VectorX<double>::Zero(num_arm_joints);
  table_poses_.resize(num_tables, Isometry3<double>::Identity());

  gripper_time_ = -1;
  gripper_q_ = 0;
  gripper_v_ = 0;

  obj_time_ = -1;
  obj_pose_ = Isometry3<double>::Identity();
  obj_vel_.setZero();
}

WorldState::~WorldState() { }

void WorldState::SetArmStatus(
    double arm_time, VectorX<double> arm_q,
    VectorX<double> arm_v, const Isometry3<double>& arm_base) {

  arm_time_ = arm_time;
  arm_q_ = arm_q;
  arm_v_ = arm_v;
  arm_base_ = arm_base;
}

void WorldState::SetArmStatus(const lcmt_iiwa_status& iiwa_msg,
                                  const Isometry3<double>& iiwa_base) {
  DRAKE_ASSERT(static_cast<size_t>(iiwa_msg.num_joints) ==
               iiwa_msg.joint_velocity_estimated.size());
  DRAKE_ASSERT(static_cast<size_t>(iiwa_msg.num_joints) ==
               iiwa_msg.joint_position_measured.size());

  VectorX<double> new_q(iiwa_msg.num_joints);
  VectorX<double> new_v(iiwa_msg.num_joints);
  for (int i = 0; i < iiwa_msg.num_joints; ++i) {
    new_v[i] = iiwa_msg.joint_velocity_estimated[i];
    new_q[i] = iiwa_msg.joint_position_measured[i];
  }
  SetArmStatus(iiwa_msg.utime / 1e6, new_q, new_v, iiwa_base);
}

void WorldState::SetGripperStatus(
    double gripper_time, double gripper_q, double gripper_v) {
  gripper_time_ = gripper_time;
  gripper_q_ = gripper_q;
  gripper_v_ = gripper_v;
}

void WorldState::SetGripperStatus(const lcmt_schunk_wsg_status& wsg_msg) {
  bool is_first_msg = gripper_time_ == -1;
  double cur_time = wsg_msg.utime / 1e6;

  if (is_first_msg) {
    SetGripperStatus(cur_time, wsg_msg.actual_position_mm / 1000., 0);
    return;
  }

  double dt = cur_time - gripper_time_;

  if (!is_first_msg && dt == 0) return;

  // TODO(siyuanfeng): Need to filter
  double new_q = wsg_msg.actual_position_mm / 1000.;
  double new_v = (new_q - gripper_q_) / dt;
  SetGripperStatus(cur_time, new_q, new_v);
}

void WorldState::SetObjectStatus(double obj_time,
                                 const Isometry3<double>& obj_pose,
                                 const Vector6<double>& obj_velocity) {
  obj_time_ = obj_time;
  obj_pose_ = obj_pose;
  obj_vel_ = obj_velocity;
}


void WorldState::SetTableStatus(int index, const Isometry3<double>& pose) {
  DRAKE_THROW_UNLESS(index >= 0 &&
                     index < static_cast<int>(table_poses_.size()));
  table_poses_[index] = pose;
}

}  // namespace pick_and_place_example
}  // namespace manipulation
}  // namespace drake
