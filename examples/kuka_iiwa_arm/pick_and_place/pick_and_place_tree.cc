#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_tree.h"

#include <memory>
#include <string>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

void AddRobotToTree(
    const std::string& robot_tag,
    const RobotConfiguration& robot,
    manipulation::util::WorldSimTreeBuilder<double>* tree_builder,
    manipulation::util::ModelInstanceInfo<double>* arm_instance,
    manipulation::util::ModelInstanceInfo<double>* gripper_instance) {
  tree_builder->StoreDrakeModel(robot_tag, robot.model);

  // Add the arm.
  int robot_base_id = tree_builder->AddFixedModelInstance(
      robot_tag, robot.pose.translation(),
      drake::math::rotmat2rpy(robot.pose.linear()));
  *arm_instance = tree_builder->get_model_info_for_instance(robot_base_id);

  int fixture_id = -1;
  if (robot.fixture) {
    std::string fixture_tag = robot_tag + "_fixture";
    tree_builder->StoreDrakeModel(fixture_tag, robot.fixture->model);
    std::shared_ptr<RigidBodyFrame<double>> fixture_frame =
        tree_builder->tree().findFrame(
            robot.fixture->attachment_frame, arm_instance->instance_id);
    fixture_id = tree_builder->AddModelInstanceToFrame(
        fixture_tag, fixture_frame,
        drake::multibody::joints::kFixed);
  }

  if (robot.gripper && gripper_instance) {
    std::string gripper_tag = robot_tag + "_gripper";
    tree_builder->StoreDrakeModel(gripper_tag, robot.gripper->model);
    std::shared_ptr<RigidBodyFrame<double>> gripper_frame;
    // The gripper's attachment frame could be on the fixture or on
    // the arm.
    if (fixture_id > 0) {
      gripper_frame = tree_builder->tree().findFrame(
          robot.gripper->attachment_frame, fixture_id);
    } else {
      gripper_frame = tree_builder->tree().findFrame(
          robot.gripper->attachment_frame, arm_instance->instance_id);
    }

    int gripper_id = tree_builder->AddModelInstanceToFrame(
        gripper_tag, gripper_frame,
        drake::multibody::joints::kFixed);
    *gripper_instance = tree_builder->get_model_info_for_instance(gripper_id);
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
