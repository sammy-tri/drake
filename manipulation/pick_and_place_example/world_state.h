#pragma once

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace pick_and_place_example {

/**
 * A class that represents the iiwa pick and place world, which contains a
 * KUKA iiwa arm, a Schunk WSG gripper, and an object that is being
 * manipulated. These states are updated through LCM messages.
 */
class WorldState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WorldState)

  /**
   * Constructs an WorldState object that holds the states that represent a pick
   * and place scenario.
   *
   * No synchronization is attempted between the various states
   * (iiwa/wsg/obj), the accessors just return the most recently
   * received status.
   */
  WorldState(
      int num_arm_joints,
      int num_tables = 0,
      const Vector3<double>& object_dimensions = Vector3<double>::Zero());

  ~WorldState();

  /// Update the stored arm status.
  void SetArmStatus(double arm_time, VectorX<double> arm_q,
                    VectorX<double> arm_v, const Isometry3<double>& arm_base);

  /// Update the stored arm status from the measured joint positions in @p
  /// iiwa_msg and the base pose in @p iiwa_base.
  void SetArmStatus(const lcmt_iiwa_status& iiwa_msg,
                    const Isometry3<double>& iiwa_base);

  /// Update the gripper state.  @p gripper_q is the distance between
  /// the gripper fingers.
  void SetGripperStatus(
      double gripper_time, double gripper_q, double gripper_v);

  /// Update the gripper status from a schunk wsg status message.
  void SetGripperStatus(const lcmt_schunk_wsg_status& wsg_msg);

  /// Update the stored object status.
  void SetObjectStatus(double obj_time,
                       const Isometry3<double>& obj_pose,
                       const Vector6<double>& obj_velocity);

  /// Update the pose of table @p index from @p pose.
  void SetTableStatus(int index, const Isometry3<double>& pose);

  double get_arm_time() const { return arm_time_; }
  double get_gripper_time() const { return gripper_time_; }
  double get_obj_time() const { return obj_time_; }
  const std::vector<Isometry3<double>>& get_table_poses() const {
    return table_poses_;
  }

  const Isometry3<double>& get_object_pose() const { return obj_pose_; }
  const Vector6<double>& get_object_velocity() const { return obj_vel_; }
  const Vector3<double>& get_object_dimensions() const {
    return object_dimensions_;
  }
  const Isometry3<double>& get_arm_base() const { return arm_base_; }
  const VectorX<double>& get_arm_q() const { return arm_q_; }
  const VectorX<double>& get_arm_v() const { return arm_v_; }
  double get_gripper_q() const { return gripper_q_; }
  double get_gripper_v() const { return gripper_v_; }


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Iiwa status.
  double arm_time_{};
  Isometry3<double> arm_base_;
  VectorX<double> arm_q_;
  VectorX<double> arm_v_;

  // Gripper status.
  double gripper_time_{};
  double gripper_q_{};  // arbitrary units
  double gripper_v_{};  // arbitrary units

  // Object status.
  double obj_time_{};
  Isometry3<double> obj_pose_;
  Vector6<double> obj_vel_;
  Vector3<double> object_dimensions_;

  // Table status
  std::vector<Isometry3<double>> table_poses_;
};

}  // namespace pick_and_place_example
}  // namespace manipulation
}  // namespace drake
