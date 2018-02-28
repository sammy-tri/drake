#include "drake/manipulation/util/plan_utils.h"

namespace drake {
namespace manipulation {

void ApplyJointVelocityLimits(const VectorX<double>& max_velocities,
                              const MatrixX<double>& keyframes,
                              std::vector<double>* time) {
  DRAKE_DEMAND(keyframes.cols() == static_cast<int>(time->size()));

  const int num_time_steps = keyframes.cols();

  // Calculate a matrix of velocities between each timestep.  We'll
  // use this later to determine by how much the plan exceeds the
  // joint velocity limits.
  Eigen::MatrixXd velocities(keyframes.rows(), num_time_steps - 1);
  for (int i = 0; i < velocities.rows(); i++) {
    for (int j = 0; j < velocities.cols(); j++) {
      DRAKE_ASSERT((*time)[j + 1] > (*time)[j]);
      velocities(i, j) =
          std::abs((keyframes(i, j + 1) - keyframes(i, j)) /
                   ((*time)[j + 1] - (*time)[j]));
    }
  }

  DRAKE_ASSERT(velocities.rows() == max_velocities.size());

  Eigen::VectorXd velocity_ratios(velocities.rows());
  for (int i = 0; i < velocities.rows(); i++) {
    const double max_plan_velocity = velocities.row(i).maxCoeff();
    // Maybe don't try max velocity at first...
    velocity_ratios(i) =
        max_plan_velocity / (max_velocities[i] * 0.9);
  }

  const double max_velocity_ratio = velocity_ratios.maxCoeff();
  if (max_velocity_ratio > 1) {
    // The code below slows the entire plan such that the fastest step
    // meets the limits.  If that step is much faster than the others,
    // the whole plan becomes very slow.
    drake::log()->debug("Slowing plan by {}", max_velocity_ratio);
    for (int j = 0; j < num_time_steps; j++) {
      (*time)[j] *= max_velocity_ratio;
    }
  }
}


robotlocomotion::robot_plan_t EncodeKeyFrames(
    const RigidBodyTree<double>& robot,
    const std::vector<double>& time,
    const std::vector<int>& info,
    const MatrixX<double>& keyframes) {
  const int num_positions = robot.get_num_positions();
  DRAKE_DEMAND(keyframes.rows() == num_positions);
  std::vector<std::string> joint_names(num_positions);
  for (int i = 0; i < num_positions; ++i) {
    joint_names[i] = robot.get_position_name(i);
  }

  return EncodeKeyFrames(joint_names, time, info, keyframes);
}

robotlocomotion::robot_plan_t EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& time,
    const std::vector<int>& info,
    const MatrixX<double>& keyframes) {

  DRAKE_DEMAND(info.size() == time.size());
  DRAKE_DEMAND(keyframes.cols() == static_cast<int>(time.size()));
  DRAKE_DEMAND(keyframes.rows() == static_cast<int>(joint_names.size()));

  const int num_time_steps = keyframes.cols();

  robotlocomotion::robot_plan_t plan{};
  plan.utime = 0;  // I (sam.creasey) don't think this is used?
  plan.robot_name = "iiwa";  // Arbitrary, probably ignored
  plan.num_states = num_time_steps;
  const bot_core::robot_state_t default_robot_state{};
  plan.plan.resize(num_time_steps, default_robot_state);
  plan.plan_info.resize(num_time_steps, 0);
  /// Encode the q_sol returned for each timestep into the vector of
  /// robot states.
  for (int i = 0; i < num_time_steps; i++) {
    bot_core::robot_state_t& step = plan.plan[i];
    step.utime = time[i] * 1e6;
    step.num_joints = keyframes.rows();
    for (int j = 0; j < step.num_joints; j++) {
      step.joint_name.push_back(joint_names[j]);
      step.joint_position.push_back(keyframes(j, i));
      step.joint_velocity.push_back(0);
      step.joint_effort.push_back(0);
    }
    plan.plan_info[i] = info[i];
  }
  plan.num_grasp_transitions = 0;
  plan.left_arm_control_type = plan.POSITION;
  plan.right_arm_control_type = plan.NONE;
  plan.left_leg_control_type = plan.NONE;
  plan.right_leg_control_type = plan.NONE;
  plan.num_bytes = 0;

  return plan;
}

}  // namespace manipulation
}  // namespace drake
