#pragma once

#include <memory>
#include <string>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/lcmt_jaco_status.hpp"
#include "drake/manipulation/pick_and_place_example/pick_and_place_configuration.h"
#include "drake/manipulation/pick_and_place_example/pick_and_place_state_machine.h"
#include "drake/manipulation/pick_and_place_example/world_state.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_symbolic_inspector.h"

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace pick_and_place {

/**
 * A class that implements the Finite-State-Machine logic for the
 * Pick-And-Place demo. This system should be used by coupling the outputs with
 * the `JacoMove` and `GripperAction` systems and the inputs are to be
 * connected to the appropriate output ports of the `JacoStatusSender`,
 * `SchunkWsgStatusSender` and `OracularStateEstimator` systems.
 */
class JacoLcmPlanner : public systems::LeafSystem<double> {
 public:
  JacoLcmPlanner(
      // NOLINTNEXTLINE(whitespace/line_length)
      const manipulation::pick_and_place_example::PlannerConfiguration& configuration,
      // NOLINTNEXTLINE(whitespace/line_length)
      const manipulation::pick_and_place_example::OptitrackConfiguration optitrack_configuration,
      int num_arm_joints,
      bool single_move);

  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;

  // This kind of a system is not a direct feedthrough.
  optional<bool> DoHasDirectFeedthrough(int, int) const final { return false; }

  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const override;

  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const override;

  /**
   * Getter for the input port corresponding to the abstract input with jaco
   * state message (LCM `lcmt_jaco_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_jaco_status()
      const {
    return this->get_input_port(input_port_jaco_status_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the
   * optitrack message (LCM `optitrack::optitrack_frame_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_optitrack_message()
      const {
    return get_input_port(input_port_optitrack_message_);
  }

  const systems::OutputPort<double>& get_output_port_jaco_plan() const {
    return this->get_output_port(output_port_jaco_plan_);
  }

  /// Return the state of the pick and place state machine.
  manipulation::pick_and_place_example::PickAndPlaceState state(
      const systems::Context<double>&) const;

  /// Return the state of the pick and place world.  Note that this
  /// reference is into data contained inside the passed in context.
  const manipulation::pick_and_place_example::WorldState& world_state(
      const systems::Context<double>&) const;

 private:
  void CalcPlan(const systems::Context<double>& context,
                robotlocomotion::robot_plan_t* plan) const;

  void CalcWsgCommand(const systems::Context<double>& context,
                      lcmt_schunk_wsg_command* wsg_command) const;

  int num_tables() const { return configuration_.num_tables; }

  const std::string& end_effector_name() const {
    return configuration_.end_effector_name;
  }

  const Vector3<double>& target_dimensions() const {
    return configuration_.target_dimensions;
  }

  struct InternalState;

  // Input ports.
  int input_port_jaco_status_{-1};
  int input_port_optitrack_message_{-1};

  // Optitrack ids
  int target_id_{-1};
  int robot_base_id_{-1};
  std::vector<int> table_ids_;

  int output_port_jaco_plan_;

  // NOLINTNEXTLINE(whitespace/line_length)
  const manipulation::pick_and_place_example::PlannerConfiguration configuration_;

  int num_arm_joints_{-1};
  bool single_move_{false};
};

}  // namespace pick_and_place
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
