#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/state_machine_system.h"

#include <utility>
#include <vector>

#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/util/lcmUtil.h"

using bot_core::robot_state_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
/* index of iiwastate */
const int kStateIndex = 0;

robotlocomotion::robot_plan_t MakeDefaultIiwaPlan() {
  robotlocomotion::robot_plan_t default_plan{};
  default_plan.utime = 0;
  default_plan.num_states = 0;
  return default_plan;
}

lcmt_schunk_wsg_command MakeDefaultWsgCommand() {
  lcmt_schunk_wsg_command default_command{};
  default_command.utime = 0;
  default_command.target_position_mm = 110;  // maximum aperture
  default_command.force = 0;
  return default_command;
}
}  // namespace

using manipulation::pick_and_place_example::PickAndPlaceState;
using manipulation::pick_and_place_example::PlannerConfiguration;
using manipulation::pick_and_place_example::WorldState;
using manipulation::pick_and_place_example::WsgAction;

typedef manipulation::pick_and_place_example::PickAndPlaceStateMachine<
  WsgAction> PickAndPlaceStateMachine;

namespace pick_and_place {

struct PickAndPlaceStateMachineSystem::InternalState {
  InternalState(const PlannerConfiguration& configuration,
                bool single_move)
      : world_state(
            kIiwaArmNumJoints,
            configuration.num_tables,
            configuration.target_dimensions),
        state_machine(configuration, single_move),
        last_iiwa_plan(MakeDefaultIiwaPlan()),
        last_wsg_command(MakeDefaultWsgCommand()) {}

  WorldState world_state;
  PickAndPlaceStateMachine state_machine;
  robotlocomotion::robot_plan_t last_iiwa_plan;
  lcmt_schunk_wsg_command last_wsg_command;
};

PickAndPlaceStateMachineSystem::PickAndPlaceStateMachineSystem(
    const PlannerConfiguration& configuration, bool single_move)
    : configuration_(configuration), single_move_(single_move) {
  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_iiwa_base_pose_ = this->DeclareAbstractInputPort().get_index();
  input_port_box_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_wsg_status_ = this->DeclareAbstractInputPort().get_index();
  input_port_table_state_.resize(this->num_tables());
  for (int i = 0; i < this->num_tables(); ++i) {
    input_port_table_state_[i] = this->DeclareAbstractInputPort().get_index();
  }

  output_port_iiwa_plan_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultIiwaPlan(),
              &PickAndPlaceStateMachineSystem::CalcIiwaPlan)
          .get_index();

  output_port_wsg_command_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultWsgCommand(),
              &PickAndPlaceStateMachineSystem::CalcWsgCommand)
          .get_index();

  this->DeclarePeriodicUnrestrictedUpdate(configuration.period_sec, 0);

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      configuration_.absolute_model_path(),
      multibody::joints::kFixed, &iiwa_tree_);
}

std::unique_ptr<systems::AbstractValues>
PickAndPlaceStateMachineSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(
      std::unique_ptr<systems::AbstractValue>(new systems::Value<InternalState>(
          InternalState(configuration_, single_move_))));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void PickAndPlaceStateMachineSystem::SetDefaultState(
    const systems::Context<double>&, systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);
  internal_state = InternalState(configuration_, single_move_);
}

void PickAndPlaceStateMachineSystem::CalcIiwaPlan(
    const systems::Context<double>& context,
    robotlocomotion::robot_plan_t* iiwa_plan) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *iiwa_plan = internal_state.last_iiwa_plan;
}

void PickAndPlaceStateMachineSystem::CalcWsgCommand(
    const systems::Context<double>& context,
    lcmt_schunk_wsg_command* wsg_command) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *wsg_command = internal_state.last_wsg_command;
}

void PickAndPlaceStateMachineSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  /* Update world state from inputs. */
  const lcmt_iiwa_status& iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)
          ->GetValue<lcmt_iiwa_status>();
  const Isometry3<double>& iiwa_base_pose =
      this->EvalAbstractInput(context, input_port_iiwa_base_pose_)
          ->GetValue<Isometry3<double>>();
  const robot_state_t& box_state =
      this->EvalAbstractInput(context, input_port_box_state_)
          ->GetValue<robot_state_t>();
  const lcmt_schunk_wsg_status& wsg_status =
      this->EvalAbstractInput(context, input_port_wsg_status_)
          ->GetValue<lcmt_schunk_wsg_status>();

  internal_state.world_state.SetArmStatus(iiwa_state, iiwa_base_pose);
  internal_state.world_state.SetGripperStatus(wsg_status);
  internal_state.world_state.SetObjectStatus(
      box_state.utime / 1e6, DecodePose(box_state.pose),
      DecodeTwist(box_state.twist));

  const int kNumTables{num_tables()};
  for (int i = 0; i < kNumTables; ++i) {
    const Isometry3<double>& table_state =
        this->EvalAbstractInput(context, input_port_table_state_[i])
            ->GetValue<Isometry3<double>>();
    internal_state.world_state.SetTableStatus(i, table_state);
  }

  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        internal_state.last_iiwa_plan = *plan;
      });

  PickAndPlaceStateMachine::GripperPublishCallback wsg_callback =
      ([&](const lcmt_schunk_wsg_command* msg) {
        internal_state.last_wsg_command = *msg;
      });
  internal_state.state_machine.Update(internal_state.world_state, iiwa_callback,
                                      wsg_callback);
}

PickAndPlaceState PickAndPlaceStateMachineSystem::state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.state_machine.state();
}

const WorldState& PickAndPlaceStateMachineSystem::world_state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.world_state;
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
