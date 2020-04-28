#include "drake/manipulation/franka_panda/panda_command_sender.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

PandaCommandSender::PandaCommandSender(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareInputPort(
      "state", systems::kVectorValued, num_joints_ * 2);
  this->DeclareInputPort(
      "torque", systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort(
      "lcmt_panda_command", &PandaCommandSender::CalcOutput);
}

using InPort = systems::InputPort<double>;
const InPort& PandaCommandSender::get_state_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& PandaCommandSender::get_torque_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
const systems::OutputPort<double>& PandaCommandSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void PandaCommandSender::CalcOutput(
    const systems::Context<double>& context, lcmt_panda_command* output) const {
  const auto& state = get_state_input_port().Eval(context);
  const bool has_torque = get_torque_input_port().HasValue(context);

  lcmt_panda_command& command = *output;
  command.utime = context.get_time() * 1e6;
  command.num_joints = num_joints_;
  command.joint_position.resize(num_joints_);
  command.joint_velocity.resize(num_joints_);
  command.joint_torque.resize(num_joints_, 0);
  for (int i = 0; i < num_joints_; ++i) {
    command.joint_position[i] = state[i];
    command.joint_velocity[i] = state[i + num_joints_];
  }

  if (has_torque) {
    const auto& torque = get_torque_input_port().Eval(context);
    for (int i = 0; i < num_joints_; ++i) {
      command.joint_torque[i] = torque[i];
    }
  }
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
