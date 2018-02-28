#pragma once

#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {

/// Scales a plan so that no step exceeds the robot's maximum joint
/// velocities in @p max_velocities.  The number of columns in @p
/// keyframes must match the size of @p time.  Times must be in
/// strictly increasing order.  @see get_iiwa_max_joint_velocities
void ApplyJointVelocityLimits(const VectorX<double>& max_velocities,
                              const MatrixX<double>& keyframes,
                              std::vector<double>* time);

/// Makes a robotlocomotion::robot_plan_t message.  The number of
/// columns in @p keyframes must match the size of @p time.  Times
/// must be in strictly increasing order.
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const RigidBodyTree<double>& robot, const std::vector<double>& time,
    const std::vector<int>& info, const MatrixX<double>& keyframes);

/// Makes a robotlocomotion::robot_plan_t message.  The number of rows in @p
/// keyframes must match the size of @p joint_names.  The number of columns in
/// @p keyframes must match the size of @p time.  Times must be in strictly
/// increasing order.
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& time, const std::vector<int>& info,
    const MatrixX<double>& keyframes);

}  // namespace manipulation
}  // namespace drake
