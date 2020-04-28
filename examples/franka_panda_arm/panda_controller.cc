/// @file
///
/// Implements a controller for a Franka Panda arm.

#include <memory>

#include <bot_core/robot_state_t.hpp>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include <robotlocomotion/robot_plan_t.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/franka_panda/panda_command_sender.h"
#include "drake/manipulation/planner/robot_plan_interpolator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"

using robotlocomotion::robot_plan_t;

DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace franka_panda_arm {
namespace {

/// Handles lcmt_jaco_status messages from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// systems::lcm::LcmSubscriberSystem::Make<robot_state_t>().
///
/// This system has one abstract-valued input port of type robot_state_t.
///
/// This system has one vector valued output port which reports
/// measured position and velocity for each joint and finger.
///
/// All ports will continue to output their initial state (typically
/// zero) until a message is received.
class RobotStateReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateReceiver)

  explicit RobotStateReceiver(int num_joints);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_input_port() const;
  const systems::OutputPort<double>& get_output_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&,
                  systems::BasicVector<double>*) const;

  const int num_joints_;
};

using systems::BasicVector;
using systems::Context;

RobotStateReceiver::RobotStateReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort(
      "robot_state_t", Value<bot_core::robot_state_t>{});
  this->DeclareVectorOutputPort(
      "state", BasicVector<double>(num_joints_ * 2),
      &RobotStateReceiver::CalcOutput);
}

const systems::InputPort<double>& RobotStateReceiver::get_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const systems::OutputPort<double>& RobotStateReceiver::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void RobotStateReceiver::CalcOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status =
      get_input_port().Eval<bot_core::robot_state_t>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    DRAKE_THROW_UNLESS(status.num_joints == num_joints_);
    DRAKE_THROW_UNLESS(
        static_cast<int>(status.joint_position.size()) == num_joints_);
    DRAKE_THROW_UNLESS(
        static_cast<int>(status.joint_velocity.size()) == num_joints_);
    auto out = output->get_mutable_value();

    for (int i = 0; i < num_joints_; i++) {
      out(i) = status.joint_position[i];
      out(i + num_joints_) = status.joint_velocity[i];
    }
  }
}

using manipulation::franka_panda::PandaCommandSender;
using manipulation::planner::RobotPlanInterpolator;

const char* const kUrdfPath =
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf";
const char* const kLcmStatusChannel = "EST_ROBOT_STATE";
const char* const kLcmCommandChannel = "PANDA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

int DoMain() {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto plan_sub =
      builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
          kLcmPlanChannel, &lcm));

  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kUrdfPath));
  auto plan_source = builder.AddSystem<RobotPlanInterpolator>(urdf);

  builder.Connect(plan_sub->get_output_port(),
                  plan_source->get_plan_input_port());
  const int num_joints = 7;
  DRAKE_DEMAND(plan_source->plant().num_positions() == num_joints);
  DRAKE_DEMAND(plan_source->plant().num_velocities() == num_joints);

  // The driver is operating in joint velocity mode, so that's the
  // meaningful part of the command message we'll eventually
  // construct.  We create a pid controller which calculates
  //
  // y = kp * (q_desired - q) + kd * (v_desired - v)
  //
  // (feedback term) which we'll add to v_desired from the plan source
  // (feed forward term).
  Eigen::VectorXd panda_kp = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd panda_ki = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd panda_kd = Eigen::VectorXd::Zero(num_joints);

  // I (sam.creasey) have no idea what would be good values here.
  // This seems to work OK at the low speeds of the jaco.
  panda_kp.head(num_joints).fill(10);
  panda_kd.head(num_joints).fill(1);

  panda_kp.head(num_joints).fill(1);
  panda_kd.head(num_joints).fill(0);

  auto pid_controller = builder.AddSystem<systems::controllers::PidController>(
      panda_kp, panda_ki, panda_kd);

  // We'll directly fix the input to the status receiver later from our lcm
  // subscriber.
  auto status_receiver = builder.AddSystem<RobotStateReceiver>(num_joints);

  builder.Connect(status_receiver->get_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(plan_source->get_output_port(0),
                  pid_controller->get_input_port_desired_state());

  // Split the plan source into q_d (sent in the command message for
  // informational purposes) and v_d (feed forward term for control).
  auto target_demux =
      builder.AddSystem<systems::Demultiplexer>(num_joints * 2, num_joints);
  builder.Connect(plan_source->get_output_port(0),
                  target_demux->get_input_port(0));

  // Sum the outputs of the pid controller and v_d.
  auto adder = builder.AddSystem<systems::Adder>(2, num_joints);
  builder.Connect(pid_controller->get_output_port_control(),
                  adder->get_input_port(0));
  builder.Connect(target_demux->get_output_port(1),
                  adder->get_input_port(1));

  // Multiplex the q_d and velocity command streams back into a single
  // port.
  std::vector<int> mux_sizes(2, num_joints);
  auto command_mux = builder.AddSystem<systems::Multiplexer>(mux_sizes);
  builder.Connect(target_demux->get_output_port(0),
                  command_mux->get_input_port(0));
  builder.Connect(adder->get_output_port(),
                  command_mux->get_input_port(1));

  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_panda_command>(
          kLcmCommandChannel, &lcm));
  auto command_sender = builder.AddSystem<PandaCommandSender>(num_joints);
  builder.Connect(command_mux->get_output_port(0),
                  command_sender->get_state_input_port());
  builder.Connect(command_sender->get_output_port(),
                  command_pub->get_input_port());

  auto owned_diagram = builder.Build();
  const systems::Diagram<double>* diagram = owned_diagram.get();
  systems::Simulator<double> simulator(std::move(owned_diagram));

  // Wait for the first message.
  drake::log()->info("Waiting for first robot status");
  lcm::Subscriber<bot_core::robot_state_t> status_sub(&lcm, kLcmStatusChannel);
  LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });

  const bot_core::robot_state_t& first_status = status_sub.message();
  DRAKE_DEMAND(first_status.num_joints == 0 ||
               first_status.num_joints == num_joints);

  VectorX<double> q0 = VectorX<double>::Zero(num_joints);
  for (int i = 0; i < first_status.num_joints; ++i) {
    q0(i) = first_status.joint_position[i];
  }

  systems::Context<double>& diagram_context = simulator.get_mutable_context();
  const double t0 = first_status.utime * 1e-6;
  diagram_context.SetTime(t0);

  auto& plan_source_context =
      diagram->GetMutableSubsystemContext(*plan_source, &diagram_context);
  plan_source->Initialize(t0, q0,
                          &plan_source_context.get_mutable_state());

  systems::Context<double>& status_context =
      diagram->GetMutableSubsystemContext(*status_receiver, &diagram_context);
  auto& status_value = status_receiver->get_input_port().FixValue(
      &status_context, first_status);

  // Run forever, using the bot_core::robot_state_t message to dictate when
  // simulation time advances.  The robot_plan_t message is handled whenever
  // the next bot_core::robot_state_t occurs.
  drake::log()->info("Controller started");
  while (true) {
    // Wait for an bot_core::robot_state_t message.
    status_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });
    // Write the bot_core::robot_state_t message into the context and advance.
    status_value.GetMutableData()->set_value(status_sub.message());
    const double time = status_sub.message().utime * 1e-6;
    simulator.AdvanceTo(time);
    // Force-publish the lcmt_jaco_command (via the command_pub system within
    // the diagram).
    diagram->Publish(diagram_context);
  }

  // We should never reach here.
  return EXIT_FAILURE;
}

}  // namespace
}  // namespace franka_panda_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::franka_panda_arm::DoMain();
}
