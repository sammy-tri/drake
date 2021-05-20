/// @file
///
/// This demo sets up a gravity compensated Panda arm within a MultibodyPlant
/// simulation controlled via LCM.

#include <limits>
#include <memory>

#include <bot_core/joint_state_t.hpp>
#include <bot_core/robot_state_t.hpp>
#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

const double kPandaUpdatePeriod = 0.001;

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(realtime_rate, 1.0, "");
DEFINE_double(time_step, kPandaUpdatePeriod,
              "The time step to use for MultibodyPlant model "
              "discretization.  0 uses the continuous version of the plant.");

using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::controllers::InverseDynamicsController;

namespace drake {
namespace examples {
namespace franka_panda_arm {
namespace {

// TODO(sammy-tri) Add feed-through effort
// TODO(sammy-tri) Add configurable initial positions
/// Handles bot_core.joint_state_t messages from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// systems::lcm::LcmSubscriberSystem::Make<joint_state_t>().
///
/// This system has one abstract-valued input port of type joint_state_t.
///
/// This system has one vector valued output port which reports
/// desired position and velocity for each joint.
///
/// All ports will continue to output their initial state (typically
/// zero) until a message is received.
class JointStateReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointStateReceiver)

  explicit JointStateReceiver(int num_joints);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_input_port() const;
  const systems::OutputPort<double>& get_output_port() const;
  //@}

  /// Sets the initial commanded position of the controlled robot prior to any
  /// command messages being received.  If this function is not called, the
  /// starting position will be the zero configuration.  The initial commanded
  /// torque is always zero and cannot be set.
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const Eigen::VectorXd>& q) const;

 private:
  void CalcInput(const systems::Context<double>&,
                 bot_core::joint_state_t*) const;
  void CalcOutput(const systems::Context<double>&,
                  systems::BasicVector<double>*) const;

  const int num_joints_;
  const systems::CacheEntry* groomed_input_{};
};

using systems::BasicVector;
using systems::Context;

JointStateReceiver::JointStateReceiver(int num_joints)
    : num_joints_(num_joints) {
  // Our parameter stores the position when no message has been received.
  const BasicVector<double> default_position(VectorXd::Zero(num_joints));
  const systems::NumericParameterIndex param{
      DeclareNumericParameter(default_position)};
  DRAKE_DEMAND(param == 0);  // We're depending on that elsewhere.

  this->DeclareAbstractInputPort(
      "joint_state_t", Value<bot_core::joint_state_t>{});

  // Our input ports are mutually exclusive; exactly one connected input port
  // feeds our cache entry. The computation may be dependent on the above
  // parameter as well.
  groomed_input_ = &DeclareCacheEntry(
      "groomed_input", &JointStateReceiver::CalcInput,
      {all_input_ports_ticket(), numeric_parameter_ticket(param)});

  this->DeclareVectorOutputPort(
      "state", BasicVector<double>(num_joints_ * 2),
      &JointStateReceiver::CalcOutput);
}

const systems::InputPort<double>& JointStateReceiver::get_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const systems::OutputPort<double>& JointStateReceiver::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void JointStateReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorXd>& q) const {
  DRAKE_THROW_UNLESS(q.size() == num_joints_);
  context->get_mutable_numeric_parameter(0).SetFromVector(q);
}

void JointStateReceiver::CalcInput(
    const Context<double>& context, bot_core::joint_state_t* result) const {
  *result = get_input_port().Eval<bot_core::joint_state_t>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (result->num_joints == 0) {
    const VectorXd param = context.get_numeric_parameter(0).get_value();
    result->num_joints = param.size();
    DRAKE_THROW_UNLESS(result->num_joints == num_joints_);
    result->joint_position = {param.data(), param.data() + param.size()};
    result->joint_velocity.resize(num_joints_, 0);
    result->joint_effort.resize(num_joints_, 0);
  }
}

void JointStateReceiver::CalcOutput(
   const Context<double>& context, BasicVector<double>* output) const {
  const auto& status =
      groomed_input_->Eval<bot_core::joint_state_t>(context);

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

/// Creates and outputs bot_core.robot_state_t messages
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// systems::lcm::LcmPublisherSystem::Make<robot_state_t>().
///
/// This system has one vector-valued input port containing the current
/// position and velocity and another with the current joint torques.
///
/// This system has one abstract-valued output port of type
/// bot_core::robt_state_t;
///
class RobotStateSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateSender)

  explicit RobotStateSender(int num_joints);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_state_input_port() const;
  const systems::InputPort<double>& get_effort_input_port() const;
  const systems::OutputPort<double>& get_output_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&,
                  bot_core::robot_state_t*) const;

  const int num_joints_;
};

RobotStateSender::RobotStateSender(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareInputPort(
      "state", systems::kVectorValued, num_joints_ * 2);
  this->DeclareInputPort(
      "effort", systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort(
      "robot_state_t", &RobotStateSender::CalcOutput);
}

const systems::InputPort<double>& RobotStateSender::get_state_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}

const systems::InputPort<double>& RobotStateSender::get_effort_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}

const systems::OutputPort<double>& RobotStateSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void RobotStateSender::CalcOutput(
    const systems::Context<double>& context,
    bot_core::robot_state_t* output) const {
  const auto& state = get_state_input_port().Eval(context);
  const auto& effort = get_effort_input_port().Eval(context);

  output->utime = context.get_time() * 1e6;
  output->pose = bot_core::position_3d_t{};
  // XXX initialize twist and force torque
  output->num_joints = num_joints_;
  output->joint_name.resize(num_joints_);
  output->joint_position.resize(num_joints_);
  output->joint_velocity.resize(num_joints_);
  output->joint_effort.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    output->joint_position[i] = state[i];
    output->joint_velocity[i] = state[i + num_joints_];
    output->joint_effort[i] = effort[i];
  }
}

const char* const kUrdfPath =
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf";

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>* scene_graph = builder.AddSystem<SceneGraph>();
  MultibodyPlant<double>* panda_plant = builder.AddSystem<MultibodyPlant>(
      FLAGS_time_step);

  panda_plant->RegisterAsSourceForSceneGraph(scene_graph);
  builder.Connect(
      panda_plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(
          panda_plant->get_source_id().value()));
  builder.Connect(
      scene_graph->get_query_output_port(),
      panda_plant->get_geometry_query_input_port());

  const multibody::ModelInstanceIndex panda_id =
      Parser(panda_plant, scene_graph).AddModelFromFile(
          FindResourceOrThrow(kUrdfPath));
  panda_plant->WeldFrames(panda_plant->world_frame(),
                          panda_plant->GetFrameByName("panda_link0"));
  panda_plant->Finalize();

  // These gains are really just a guess.  Velocity limits are not enforced,
  // allowing much faster simulated movement than the actual robot.
  const int num_positions = panda_plant->num_positions();
  VectorXd kp = VectorXd::Constant(num_positions, 100);
  VectorXd kd = 2.0 * kp.array().sqrt();
  VectorXd ki = VectorXd::Zero(num_positions);

  auto panda_controller = builder.AddSystem<InverseDynamicsController>(
      *panda_plant, kp, ki, kd, false);

  builder.Connect(panda_plant->get_state_output_port(panda_id),
                  panda_controller->get_input_port_estimated_state());

  systems::lcm::LcmInterfaceSystem* lcm =
      builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);

  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<bot_core::joint_state_t>(
          "FRANKA_COMMAND", lcm));
  auto command_receiver = builder.AddSystem<JointStateReceiver>(num_positions);
  builder.Connect(command_sub->get_output_port(),
                  command_receiver->get_input_port());
  builder.Connect(command_receiver->get_output_port(),
                  panda_controller->get_input_port_desired_state());
  builder.Connect(panda_controller->get_output_port_control(),
                  panda_plant->get_actuation_input_port(panda_id));

  auto status_pub =
      builder.AddSystem(
          systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
              "EST_ROBOT_STATE", lcm, kPandaUpdatePeriod));
  auto status_sender = builder.AddSystem<RobotStateSender>(num_positions);
  builder.Connect(panda_plant->get_state_output_port(panda_id),
                  status_sender->get_state_input_port());
  builder.Connect(panda_controller->get_output_port_control(),
                  status_sender->get_effort_input_port());
  builder.Connect(status_sender->get_output_port(),
                  status_pub->get_input_port());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& root_context = simulator.get_mutable_context();

  VectorXd initial_position = VectorXd::Zero(num_positions);
  initial_position(3) = -1;
  initial_position(5) = 1;

  command_receiver->set_initial_position(
      &diagram->GetMutableSubsystemContext(*command_receiver, &root_context),
      initial_position);
  panda_plant->SetPositions(
      &diagram->GetMutableSubsystemContext(*panda_plant, &root_context),
      initial_position);

  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace franka_panda_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::franka_panda_arm::DoMain();
}
