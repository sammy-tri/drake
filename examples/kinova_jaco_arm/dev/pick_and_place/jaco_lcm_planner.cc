#include "drake/examples/kinova_jaco_arm/dev/pick_and_place/jaco_lcm_planner.h"

#include <utility>
#include <vector>

#include "bot_core/robot_state_t.hpp"
#include "optitrack/optitrack_frame_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/manipulation/perception/optitrack_pose_extractor.h"

using bot_core::robot_state_t;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {
/* index of iiwastate */
const int kStateIndex = 0;

robotlocomotion::robot_plan_t MakeDefaultPlan() {
  robotlocomotion::robot_plan_t default_plan{};
  default_plan.utime = 0;
  default_plan.num_states = 0;
  return default_plan;
}

}  // namespace

using manipulation::pick_and_place_example::JacoFingerAction;
using manipulation::pick_and_place_example::PickAndPlaceState;
using manipulation::pick_and_place_example::OptitrackConfiguration;
using manipulation::pick_and_place_example::PlannerConfiguration;
using manipulation::pick_and_place_example::WorldState;

typedef manipulation::pick_and_place_example::PickAndPlaceStateMachine<
  JacoFingerAction> PickAndPlaceStateMachine;

namespace pick_and_place {

struct JacoLcmPlanner::InternalState {
  InternalState(int num_arm_joints,
                const PlannerConfiguration& configuration,
                bool single_move)
      : world_state(
            num_arm_joints,
            configuration.num_tables,
            configuration.target_dimensions),
        state_machine(configuration, single_move),
        last_plan(MakeDefaultPlan()) {}

  ~InternalState() {}

  WorldState world_state;
  PickAndPlaceStateMachine state_machine;
  robotlocomotion::robot_plan_t last_plan;
};

JacoLcmPlanner::JacoLcmPlanner(
    const PlannerConfiguration& configuration,
    const OptitrackConfiguration optitrack_configuration,
    int num_arm_joints,
    bool single_move)
    : configuration_(configuration),
      num_arm_joints_(num_arm_joints),
      single_move_(single_move) {

  input_port_jaco_status_ = this->DeclareAbstractInputPort().get_index();
  input_port_optitrack_message_ = this->DeclareAbstractInputPort().get_index();

  target_id_ = optitrack_configuration.object_optitrack_info[
      configuration.target_index].id;
  robot_base_id_ = optitrack_configuration.robot_base_optitrack_info[
      configuration.robot_index].id;
  for (int i = 0; i < static_cast<int>(
           optitrack_configuration.table_optitrack_info.size());
       ++i) {
    table_ids_.push_back(optitrack_configuration.table_optitrack_info[i].id);
  }

  output_port_jaco_plan_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultPlan(),
              &JacoLcmPlanner::CalcPlan)
          .get_index();

  this->DeclarePeriodicUnrestrictedUpdate(configuration.period_sec, 0);
}

std::unique_ptr<systems::AbstractValues>
JacoLcmPlanner::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(
      std::unique_ptr<systems::AbstractValue>(new systems::Value<InternalState>(
          InternalState(num_arm_joints_, configuration_, single_move_))));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void JacoLcmPlanner::SetDefaultState(
    const systems::Context<double>&, systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);
  internal_state = InternalState(num_arm_joints_, configuration_, single_move_);
}

void JacoLcmPlanner::CalcPlan(
    const systems::Context<double>& context,
    robotlocomotion::robot_plan_t* plan) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *plan = internal_state.last_plan;
}

void JacoLcmPlanner::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  /* Update world state from inputs. */
  const optitrack::optitrack_frame_t& optitrack_frame =
      this->EvalAbstractInput(context, input_port_optitrack_message_)
      ->GetValue<optitrack::optitrack_frame_t>();

  const std::map<int, Isometry3<double>> optitrack_map =
      manipulation::perception::ExtractOptitrackPoses(optitrack_frame);

  internal_state.world_state.SetObjectStatus(
      optitrack_frame.utime / 1e6,
      optitrack_map.at(target_id_),
      Vector6<double>::Zero());
  for (int i = 0; i < static_cast<int>(table_ids_.size()); i++) {
    internal_state.world_state.SetTableStatus(
        i, optitrack_map.at(table_ids_[i]));
  }

  const lcmt_jaco_status& jaco_status =
      this->EvalAbstractInput(context, input_port_jaco_status_)
          ->GetValue<lcmt_jaco_status>();
  DRAKE_THROW_UNLESS(
      (jaco_status.num_joints + jaco_status.num_fingers) == num_arm_joints_);

  Eigen::VectorXd arm_q(jaco_status.num_joints + jaco_status.num_fingers);
  Eigen::VectorXd arm_v(jaco_status.num_joints + jaco_status.num_fingers);

  for (int i = 0; i <  jaco_status.num_joints; i++) {
    arm_q(i) = jaco_status.joint_position[i];
    arm_v(i) = jaco_status.joint_velocity[i];
  }

  for (int i = 0; i <  jaco_status.num_fingers; i++) {
    arm_q(i + jaco_status.num_joints) = jaco_status.finger_position[i];
    arm_v(i + jaco_status.num_joints) = jaco_status.finger_velocity[i];
  }

  internal_state.world_state.SetArmStatus(
      jaco_status.utime / 1e6, arm_q, arm_v,
      optitrack_map.at(robot_base_id_));

  // TODO(sam.creasey) make this something sensible
  internal_state.world_state.SetGripperStatus(
      jaco_status.utime / 1e6, jaco_status.finger_position[0],
      jaco_status.finger_velocity[0]);

  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        internal_state.last_plan = *plan;
      });

  PickAndPlaceStateMachine::GripperPublishCallback gripper_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        internal_state.last_plan = *plan;
      });

  internal_state.state_machine.Update(internal_state.world_state, iiwa_callback,
                                      gripper_callback);
}

PickAndPlaceState JacoLcmPlanner::state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.state_machine.state();
}

const WorldState& JacoLcmPlanner::world_state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.world_state;
}

}  // namespace pick_and_place
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
