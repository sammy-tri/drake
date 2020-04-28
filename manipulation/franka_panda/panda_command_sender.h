#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/lcmt_panda_command.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

/// Creates and outputs lcmt_panda_command messages.
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// systems::lcm::LcmPublisherSystem::Make<lcmt_panda_command>().
///
/// This system has two vector-valued input ports, one for the commanded state
/// (position and velocity) (which must be connected) and one for commanded
/// torque (which is optional).  If the torque input port is not connected,
/// then zero values will be emitted in the resulting message.
///
/// This system has one abstract-valued output port of type lcmt_panda_command.
///
/// @system {
///   @input_port{state}
///   @input_port{torque (optional)}
///   @output_port{lcmt_panda_command}
/// }
///
/// @see `lcmt_panda_command.lcm` for additional documentation.
class PandaCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PandaCommandSender)

  explicit PandaCommandSender(int num_joints = kPandaArmNumJoints);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_state_input_port() const;
  const systems::InputPort<double>& get_torque_input_port() const;
  const systems::OutputPort<double>& get_output_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_panda_command*) const;

  const int num_joints_;
};

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
