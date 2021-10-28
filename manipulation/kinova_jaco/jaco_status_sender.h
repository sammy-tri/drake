#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/manipulation/kinova_jaco/jaco_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

/// Creates and outputs lcmt_jaco_status messages.
///
/// Note that this system does not actually send the message to an LCM
/// channel. To send the message, the output of this system should be
/// connected to a systems::lcm::LcmPublisherSystem::Make<lcmt_jaco_status>().
///
/// This system has many vector-valued input ports.  All input ports are of
/// size num_joints, except for finger_position and finger_velocity which are
/// of size num_fingers.  If the torque, torque_external, or current input
/// ports are not connected, the output message will use zeros.  Finger
/// velocities will be translated to the values used by the Kinova SDK from
/// values appropriate for the finger joints in the Jaco description (see
/// jaco_constants.h).  The finger input ports do not need to be connected if
/// `num_fingers` is zero.
///
/// This system has one abstract-valued output port of type lcmt_jaco_status.
///
/// This system is presently only used in simulation. The robot hardware drivers
/// publish directly to LCM and do not make use of this system.
///
/// @system
/// name: JacoStatusSender
/// input_ports:
/// - position
/// - velocity
/// - finger_position
/// - finger_velocity
/// - torque (optional)
/// - torque_external (optional)
/// - current (optional)
/// output_ports:
/// - lcmt_jaco_status
/// @endsystem
///
/// @see `lcmt_jaco_status.lcm` for additional documentation.
class JacoStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacoStatusSender)

  JacoStatusSender(int num_joints = kJacoDefaultArmNumJoints,
                   int num_fingers = kJacoDefaultArmNumFingers);

  DRAKE_DEPRECATED("2022-02-01", "Use position/velocity input ports instead.")
  const systems::InputPort<double>& get_state_input_port() const;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_position_input_port() const {
    return *position_input_;
  }
  const systems::InputPort<double>& get_velocity_input_port() const {
    return *velocity_input_;
  }
  const systems::InputPort<double>& get_finger_position_input_port() const {
    return *finger_position_input_;
  }
  const systems::InputPort<double>& get_finger_velocity_input_port() const {
    return *finger_velocity_input_;
  }
  const systems::InputPort<double>& get_torque_input_port() const {
    return *torque_input_;
  }
  const systems::InputPort<double>& get_torque_external_input_port() const {
    return *torque_external_input_;
  }
  const systems::InputPort<double>& get_current_input_port() const {
    return *current_input_;
  }
  //@}

  // TODO(sammy-tri) Add input ports for the finger torques and currents if
  // those values ever become meaningful to simulate.  IIRC the values
  // reported by the hardware/SDK aren't particularly useful.

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_jaco_status*) const;

  const int num_joints_;
  const int num_fingers_;
  const systems::InputPort<double>* state_input_{};
  const systems::InputPort<double>* position_input_{};
  const systems::InputPort<double>* velocity_input_{};
  const systems::InputPort<double>* finger_position_input_{};
  const systems::InputPort<double>* finger_velocity_input_{};
  const systems::InputPort<double>* torque_input_{};
  const systems::InputPort<double>* torque_external_input_{};
  const systems::InputPort<double>* current_input_{};
};

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
