/// @file
///
/// Demo of moving the panda's end effector in cartesian space.  This
/// program creates a plan to move the end effector from the current
/// position to the location specified on the command line.  The
/// current calculated position of the end effector is printed before,
/// during, and after the commanded motion.

#include <map>

#include <bot_core/robot_state_t.hpp>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/manipulation/util/robot_plan_utils.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_robot_state_channel, "EST_ROBOT_STATE",
              "Channel on which to listen for robot_state_t messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to send robot_plan_t messages.");
DEFINE_double(x, 0.3, "x coordinate (meters) to move to");
DEFINE_double(y, -0.26, "y coordinate (meters) to move to");
DEFINE_double(z, 0.5, "z coordinate (meters) to move to");
DEFINE_double(roll, -1.7,
              "target roll (radians) about world x axis for end effector");
DEFINE_double(pitch, -1.3,
              "target pitch (radians) about world y axis for end effector");
DEFINE_double(yaw, -1.8,
              "target yaw (radians) about world z axis for end effector");
DEFINE_string(ee_name, "panda_link8", "Name of the end effector link");

namespace drake {
namespace examples {
namespace franka_panda_arm {
namespace {

using manipulation::planner::ConstraintRelaxingIk;

const char kUrdfPath[] =
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf";

class MoveDemoRunner {
 public:
  MoveDemoRunner()
      : plant_(0.0) {
    urdf_ =
        (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kUrdfPath));
    multibody::Parser(&plant_).AddModelFromFile(urdf_);
    plant_.WeldFrames(plant_.world_frame(),
                      plant_.GetFrameByName("panda_link0"));
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();

    lcm_.subscribe(FLAGS_lcm_robot_state_channel,
                   &MoveDemoRunner::HandleStatus, this);

    joint_names_ = manipulation::util::GetJointNames(plant_);
  }

  void Run() {
    while (lcm_.handle() >= 0) { }
  }

 private:
  // Handle the incoming status message from the iiwa.  This is a
  // fairly high rate operation (1000Hz is typical).  The plan is
  // calculated on the first status message received, after that
  // periodically display the current position of the end effector.
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const bot_core::robot_state_t* status) {
    status_count_++;
    Eigen::VectorXd panda_q(status->num_joints);
    for (int i = 0; i < status->num_joints; i++) {
      panda_q[i] = status->joint_position[i];
    }

    // Only print the position every 500 messages (this results in an
    // 0.5s period in a typical configuration).
    if (status_count_ % 500 == 1) {
      plant_.SetPositions(context_.get(), panda_q);

      const math::RigidTransform<double> ee_pose =
          plant_.EvalBodyPoseInWorld(
              *context_, plant_.GetBodyByName(FLAGS_ee_name));
      const math::RollPitchYaw<double> rpy(ee_pose.rotation());
      drake::log()->info("End effector at: {} {}",
                         ee_pose.translation().transpose(),
                         rpy.vector().transpose());
    }

    // If this is the first status we've received, calculate a plan
    // and send it (if it succeeds).
    if (status_count_ == 1) {
      ConstraintRelaxingIk ik(urdf_, FLAGS_ee_name);

      // Create a single waypoint for our plan (the destination).
      // This results in a trajectory with two knot points (the
      // current pose (read from the status message currently being
      // processes and passed directly to PlanSequentialTrajectory as
      // panda_q) and the calculated final pose).
      ConstraintRelaxingIk::IkCartesianWaypoint wp;
      wp.pose.set_translation(Eigen::Vector3d(FLAGS_x, FLAGS_y, FLAGS_z));
      const math::RollPitchYaw<double> rpy(FLAGS_roll, FLAGS_pitch, FLAGS_yaw);
      wp.pose.set_rotation(rpy);
      wp.constrain_orientation = true;
      std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
      waypoints.push_back(wp);
      std::vector<Eigen::VectorXd> q_sol;
      const bool result =
          ik.PlanSequentialTrajectory(waypoints, panda_q, &q_sol);
      drake::log()->info("IK result: {}", result);

      if (result) {
        drake::log()->info("IK sol size {}", q_sol.size());

        // Run the resulting plan over 2 seconds (which is a fairly
        // arbitrary choice).
        std::vector<double> times{0, 2};
        DRAKE_DEMAND(q_sol.size() == times.size());
        manipulation::util::ApplyJointVelocityLimits(
            q_sol, plant_.GetVelocityUpperLimits() * 0.9, &times);

        lcmt_robot_plan plan =
            manipulation::util::EncodeKeyFrames(
                joint_names_, times, q_sol);
        lcm_.publish(FLAGS_lcm_plan_channel, &plan);
      }
    }
  }

  ::lcm::LCM lcm_;
  std::string urdf_;
  multibody::MultibodyPlant<double> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  std::vector<std::string> joint_names_;
  int status_count_{0};
};

}  // namespace
}  // namespace franka_panda_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::franka_panda_arm::MoveDemoRunner runner;
  runner.Run();
}
