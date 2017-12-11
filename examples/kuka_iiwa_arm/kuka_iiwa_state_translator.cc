/// @file
///
/// kuka_iiwa_state_translator is designed to wait for LCM messages
/// containing an lcmt_iiwa_status message and converts that message
/// to robot_state_t before re-sending.

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(robot_state_channel, "EST_ROBOT_STATE",
              "Channel on which to publish robot_state_t messages.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

class KukaIiwaStateTranslator {
 public:
  /// tree is aliased
  explicit KukaIiwaStateTranslator(const RigidBodyTree<double>& tree)
      : translator_(tree) {
    translator_.InitializeMessage(&robot_state_msg_);
    lcm_.subscribe(FLAGS_lcm_status_channel,
                   &KukaIiwaStateTranslator::HandleStatus, this);
  }

  void Run() {
    while (lcm_.handle() == 0) {}
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    VectorX<double> q(status->num_joints);
    VectorX<double> v(status->num_joints);
    for (int i = 0; i < status->num_joints; i++) {
      q(i) = status->joint_position_measured[i];
      v(i) = status->joint_velocity_estimated[i];
    }

    translator_.EncodeMessageKinematics(q, v, &robot_state_msg_);
    lcm_.publish(FLAGS_robot_state_channel, &robot_state_msg_);
  }

  lcm::LCM lcm_;
  manipulation::RobotStateLcmMessageTranslator translator_;
  bot_core::robot_state_t robot_state_msg_;
};

int DoMain() {
  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kFixed, &tree);
  KukaIiwaStateTranslator translator(tree);
  translator.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::DoMain();
}
