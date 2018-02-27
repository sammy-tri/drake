#pragma once

#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/manipulation/pick_and_place_example/action.h"
#include "drake/manipulation/pick_and_place_example/pick_and_place_configuration.h"
#include "drake/manipulation/pick_and_place_example/world_state.h"
namespace drake {
namespace manipulation {
namespace pick_and_place_example {

/// Different states for the pick and place task.
enum class PickAndPlaceState {
  kOpenGripper,
  kPlan,
  kApproachPickPregrasp,
  kApproachPick,
  kGrasp,
  kLiftFromPick,
  kApproachPlacePregrasp,
  kApproachPlace,
  kPlace,
  kLiftFromPlace,
  kReset,
  kDone,
};

/// A class which controls the pick and place actions for moving a
/// single target in the environment.
class PickAndPlaceStateMachine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PickAndPlaceStateMachine)

  typedef std::function<void(const robotlocomotion::robot_plan_t*)>
      IiwaPublishCallback;
  typedef std::function<void(const lcmt_schunk_wsg_command*)>
      WsgPublishCallback;

  /// Construct a pick and place state machine.  The state machine will move the
  /// item counter-clockwise around the tables specified in @p configuration.
  /// If @p single_move is true, the state machine will remain in the kDone
  /// state after moving the object once, otherwise it will loop through the
  /// pick and place.
  PickAndPlaceStateMachine(
      const PlannerConfiguration& configuration,
      bool single_move);

  ~PickAndPlaceStateMachine();

  /// Update the state machine based on the state of the world in @p
  /// env_state.  When a new robot plan is available, @p iiwa_callback
  /// will be invoked with the new plan.  If the desired gripper state
  /// changes, @p wsg_callback is invoked.
  void Update(const WorldState& env_state,
              const IiwaPublishCallback& iiwa_callback,
              const WsgPublishCallback& wsg_callback);

  PickAndPlaceState state() const { return state_; }

 private:
  optional<
      std::map<PickAndPlaceState, trajectories::PiecewisePolynomial<double>>>
  ComputeTrajectories(
      const WorldState& env_state,
      const trajectories::PiecewisePolynomial<double>& q_traj_seed,
      RigidBodyTree<double>* iiwa) const;

  bool single_move_;

  WsgAction wsg_act_;
  IiwaMove iiwa_move_;

  PickAndPlaceState state_;

  // Poses used for storing end-points of Iiwa trajectories_at various states
  // of the demo.
  Isometry3<double> X_Wend_effector_0_;
  Isometry3<double> X_Wend_effector_1_;

  // Desired object end pose relative to the base of the iiwa arm.
  Isometry3<double> X_IIWAobj_desired_;

  // Desired object end pose in the world frame.
  Isometry3<double> X_Wobj_desired_;

  Vector3<double> tight_pos_tol_;
  double tight_rot_tol_;

  Vector3<double> loose_pos_tol_;
  double loose_rot_tol_;

  PlannerConfiguration configuration_;

  // Desired interpolation results for various states
  optional<
      std::map<PickAndPlaceState, trajectories::PiecewisePolynomial<double>>>
      interpolation_result_map_{};

  // Measured location of object at planning time
  Isometry3<double> expected_object_pose_;

  // Joint position seed
  optional<trajectories::PiecewisePolynomial<double>> q_traj_seed_;

  // Counter for number of planning failures
  int planning_failure_count_{0};

  std::default_random_engine rand_generator_{1234};

  std::vector<std::string> joint_names_;
};

}  // namespace pick_and_place_example
}  // namespace manipulation
}  // namespace drake
