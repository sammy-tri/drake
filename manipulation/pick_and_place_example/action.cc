#include "drake/manipulation/pick_and_place_example/action.h"

#include <iostream>
#include <limits>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/manipulation/util/plan_utils.h"

namespace drake {
namespace manipulation {
namespace pick_and_place_example {

Action::~Action() {}

bool Action::ActionStarted() const {
  if (act_start_time_ < 0) return false;
  return true;
}

void Action::Reset() {
  act_start_time_ = -1;
}

void Action::StartAction(double start_time) {
  DRAKE_DEMAND(start_time >= 0);
  act_start_time_ = start_time;
}

IiwaMove::IiwaMove() {}

void IiwaMove::MoveJoints(const WorldState& est_state,
                          const std::vector<std::string>& joint_names,
                          const std::vector<double>& time_in,
                          const std::vector<VectorX<double>>& q,
                          robotlocomotion::robot_plan_t* plan) {
  std::vector<double> time = time_in;
  DRAKE_DEMAND(time.size() == q.size());
  DRAKE_DEMAND(plan != nullptr);

  std::vector<int> info(time.size(), 1);
  MatrixX<double> q_mat(q.front().size(), q.size());
  for (size_t i = 0; i < q.size(); ++i) {
    q_mat.col(i) = q[i];
    q_mat.col(i).tail(3) = est_state.get_arm_q().tail(3);
  }
  VectorX<double> iiwa_max_velocities =
      examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities();
  iiwa_max_velocities(0) = 0.628;
  iiwa_max_velocities(1) = 0.628;
  iiwa_max_velocities(2) = 0.628;
  iiwa_max_velocities(3) = 0.628;
  iiwa_max_velocities(4) = 0.837;
  iiwa_max_velocities(5) = 0.837;
  iiwa_max_velocities(6) = 0.837;
  VectorX<double> max_velocities = VectorX<double>::Ones(joint_names.size());
  max_velocities.head(iiwa_max_velocities.size()) = iiwa_max_velocities;

  ApplyJointVelocityLimits(max_velocities, q_mat, &time);
  *plan = EncodeKeyFrames(joint_names, time, info, q_mat);
  StartAction(est_state.get_arm_time());
  // Set the duration for this action to be longer than that of the plan to
  // ensure that we do not advance to the next action befor the robot finishes
  // executing the plan.
  const double additional_duaration{0.5};
  duration_ = time.back() + additional_duaration;
}

void IiwaMove::Reset() {
  Action::Reset();
  duration_ = std::numeric_limits<double>::infinity();
}

bool IiwaMove::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;

  const double max_finished_velocity = 1e-1;
  VectorX<double> arm_v = est_state.get_arm_v();
  arm_v.tail(3).fill(0);
  if (get_time_since_action_start(est_state.get_arm_time()) > duration_ &&
      arm_v.norm() < max_finished_velocity) {
    return true;
  } else {
    return false;
  }
}

WsgAction::WsgAction() {}

void WsgAction::OpenGripper(const WorldState& est_state,
                            const std::vector<std::string>& ignored,
                            double grip_force,
                            lcmt_schunk_wsg_command* msg) {
  StartAction(est_state.get_gripper_time());
  *msg = lcmt_schunk_wsg_command();
  msg->utime = est_state.get_gripper_time() * 1e6;
  msg->target_position_mm = 100;  // Maximum aperture for WSG
  msg->force = grip_force;
  last_command_ = kOpen;
}

void WsgAction::CloseGripper(const WorldState& est_state,
                             const std::vector<std::string>& ignored,
                             double grip_force,
                             lcmt_schunk_wsg_command* msg) {
  StartAction(est_state.get_gripper_time());
  *msg = lcmt_schunk_wsg_command();
  msg->utime = est_state.get_gripper_time() * 1e6;
  msg->target_position_mm = 8;  // 0 would smash the fingers together
                                // and keep applying force on a real
                                // WSG when no object is grasped.
  msg->force = grip_force;
  last_command_ = kClose;
}

bool WsgAction::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;
  if (std::abs(est_state.get_gripper_v()) < kFinalSpeedThreshold &&
      (get_time_since_action_start(est_state.get_gripper_time()) > 0.5)) {
    if (last_command_ == kOpen &&
        est_state.get_gripper_q() > kOpenPositionThreshold) {
      return true;
    } else if (last_command_ == kClose &&
               est_state.get_gripper_q() < kOpenPositionThreshold) {
      return true;
    }
  }
  return false;
}

JacoFingerAction::JacoFingerAction() {}

void JacoFingerAction::OpenGripper(const WorldState& est_state,
                                   const std::vector<std::string>& joint_names,
                                   double grip_force_ignored,
                                   robotlocomotion::robot_plan_t* plan) {
  StartAction(est_state.get_gripper_time());

  std::vector<double> time{0., 1.};
  std::vector<int> info(time.size(), 1);

  MatrixX<double> q_mat(joint_names.size(), time.size());
  q_mat.col(0) = est_state.get_arm_q();
  q_mat.col(1) = est_state.get_arm_q();
  q_mat.col(1).tail(3) = VectorX<double>::Zero(3);
  *plan = EncodeKeyFrames(joint_names, time, info, q_mat);
  last_command_ = kOpen;
}

void JacoFingerAction::CloseGripper(const WorldState& est_state,
                                    const std::vector<std::string>& joint_names,
                                    double grip_force,
                                    robotlocomotion::robot_plan_t* plan) {
  StartAction(est_state.get_gripper_time());

  std::vector<double> time{0., 1.};
  std::vector<int> info(time.size(), 1);

  MatrixX<double> q_mat(joint_names.size(), time.size());
  q_mat.col(0) = est_state.get_arm_q();
  q_mat.col(1) = est_state.get_arm_q();
  // The command will be translated into URDF coordinates.
  q_mat.col(1).tail(3) = VectorX<double>::Ones(3) * 1.68;
  *plan = EncodeKeyFrames(joint_names, time, info, q_mat);
  last_command_ = kClose;
}

bool JacoFingerAction::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;
  if (get_time_since_action_start(est_state.get_gripper_time()) > 0.5) {
    std::cout << "Gripper q: " << est_state.get_gripper_q()
              << " v: " << est_state.get_gripper_v()
              << "\n";
    if (last_command_ == kOpen &&
        est_state.get_gripper_q() < kOpenPositionThreshold) {
      return true;
    } else if (last_command_ == kClose &&
               est_state.get_gripper_q() > kClosedPositionThreshold &&
               est_state.get_gripper_v() < kFinalSpeedThreshold) {
      return true;
    }
  }
  return false;
}

}  // namespace pick_and_place_example
}  // namespace manipulation
}  // namespace drake
