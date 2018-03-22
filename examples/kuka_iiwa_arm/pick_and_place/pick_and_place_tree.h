#pragma once

/// @file
///
/// This file containes helper functions for build a RigidBodyTree
/// from a pick and place configuration.

#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/// Adds @p robot with unique identifier @p robot_tag to the tree
/// being constructed by @p tree_builder.  @p arm_instance will be
/// populated with the base robot instance.  If a gripper is
/// specified, @p gripper_instance will be populated with the gripper
/// instance.  If @p gripper_instance is nullptr, the gripper will not be
/// added even if specified in @p robot.
void AddRobotToTree(
    const std::string& robot_tag,
    const RobotConfiguration& robot,
    manipulation::util::WorldSimTreeBuilder<double>* tree_builder,
    manipulation::util::ModelInstanceInfo<double>* arm_instance,
    manipulation::util::ModelInstanceInfo<double>* gripper_instance);



}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
