#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/type_safe_index.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

using RobotBaseIndex = TypeSafeIndex<class RobotBaseTag>;
using TargetIndex = TypeSafeIndex<class TargetTag>;

/// Information required to track a model from an Optitrack frame.
struct OptitrackInfo {
  /// The identifier assigned to this object by the motion-capture system.
  int id{0};
  /// The transform from the Optitrack rigid-body frame (F) to the model base
  /// frame (M).
  Isometry3<double> X_MF{Isometry3<double>::Identity()};
};

struct RobotAttachment {
  std::string model;
  std::string attachment_frame;
};

/// Information describing an instance of a particular robot
/// (including gripper configuration).
struct RobotConfiguration {
  std::string model;
  Isometry3<double> pose;
  optional<RobotAttachment> fixture;
  optional<RobotAttachment> gripper;
};

/// Information required to set up a planner for a pick-and-place task.
struct PlannerConfiguration {
  /// Name of the end-effector link on the robot arm.
  std::string end_effector_name;
  /// Type-safe index indicating for which robot arm in the scenario this
  /// planner is intended.
  RobotBaseIndex robot_index{0};
  /// Type-safe index indicating which object in the scenario this planner
  /// should treat as the manipulation target.
  TargetIndex target_index{0};
  /// Axis-aligned bounding-box dimensions for the target object.
  Vector3<double> target_dimensions;
  /// Period at which the state machine should update.
  double period_sec{0.25};
  /// Number of tables for which the planner should expect to receive pose
  /// inputs.
  int num_tables{0};
  /// Gripping force to apply (in Newtons).
  double grip_force{40.};
  /// The distance from the end effector link to the grasp frame's
  /// origin (for a parallel jaw gripper, this would often be a
  /// position between the fingertips of the gripper.  For a gripper
  /// with an enveloping grasp, it might be in a different location
  /// along the fingers).
  double grasp_frame_translational_offset{0.19};
};

/// Information required to set up a simulation of a pick-and-place scenario
/// with multiple arms, tables, and manipulable objects.
struct SimulatedPlantConfiguration {
  /// Path (relative to DRAKE_RESOURCE_ROOT) to the model file describing each
  /// table in the scenario.
  std::vector<std::string> table_models;
  /// World poses of the model origin of each table in the scenario.
  std::vector<Isometry3<double>> table_poses;
  /// Path (relative to DRAKE_RESOURCE_ROOT) to the model file describing each
  /// object in the scenario.
  std::vector<std::string> object_models;
  /// World poses of the model origin of each object in the scenario.
  std::vector<Isometry3<double>> object_poses;
  /// @name Contact parameters
  /// Global contact parameters for the simulation. Configured to use the
  /// default values.
  ///@{
  systems::CompliantContactModelParameters contact_model_parameters;
  systems::CompliantMaterial default_contact_material;
  ///@}
};

/// Optitrack information required for a pick-and-place scenario with multiple
/// arms, tables, and manipulable objects.
struct OptitrackConfiguration {
  std::vector<OptitrackInfo> robot_base_optitrack_info;
  std::vector<OptitrackInfo> object_optitrack_info;
  std::vector<OptitrackInfo> table_optitrack_info;
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
