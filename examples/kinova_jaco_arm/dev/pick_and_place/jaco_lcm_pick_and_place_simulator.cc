#include <gflags/gflags.h>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kinova_jaco_arm/jaco_common.h"
#include "drake/examples/kinova_jaco_arm/jaco_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"

#include "drake/manipulation/pick_and_place_example/pick_and_place_configuration.h"
#include "drake/manipulation/pick_and_place_example/pick_and_place_configuration_parsing.h"
#include "drake/manipulation/pick_and_place_example/pick_and_place_simulation_helpers.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(dt, 5e-4, "Integration step size");
DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, "
              "relative to realtime");
DEFINE_bool(quick, false,
            "Run only a brief simulation and return success "
            "without executing the entire task");
DEFINE_string(configuration_file,
              "drake/manipulation/pick_and_place_example/configuration/"
              "yellow_posts.pick_and_place_configuration",
              "Path to the configuration file.");

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace pick_and_place {
namespace {

using manipulation::pick_and_place_example::AddOptitrackComponents;
using manipulation::pick_and_place_example::BuildPickAndPlacePlant;
using manipulation::pick_and_place_example::OptitrackConfiguration;
using manipulation::pick_and_place_example::SimulatedPlantConfiguration;
using manipulation::util::ModelInstanceInfo;
using systems::ConstantVectorSource;

typedef std::vector<std::reference_wrapper<
  const systems::InputPortDescriptor<double>>> InputPortVector;
typedef std::vector<std::reference_wrapper<
  const systems::OutputPort<double>>> OutputPortVector;

void AddJacoInstancesToDiagram(
    const std::vector<manipulation::util::ModelInstanceInfo<double>>& jaco_instances,
    systems::RigidBodyPlant<double>* plant,
    InputPortVector* desired_state_input_ports,
    InputPortVector* desired_acceleration_input_ports,
    OutputPortVector* state_output_ports,
    manipulation::util::SimDiagramBuilder<double>* builder) {

  for (int i = 0; i < static_cast<int>(jaco_instances.size()); ++i) {

    auto single_arm = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        jaco_instances[i].absolute_model_path, multibody::joints::kFixed,
        jaco_instances[i].world_offset, single_arm.get());

    VectorX<double> jaco_kp, jaco_kd, jaco_ki;
    SetPositionControlledJacoGains(&jaco_kp, &jaco_ki, &jaco_kd,
                                   single_arm->get_num_positions());
    jaco_ki.head(7) = VectorX<double>::Ones(7);

    auto jaco_controller = builder->template AddController<
        systems::controllers::InverseDynamicsController<double>>(
        jaco_instances[i].instance_id, std::move(single_arm), jaco_kp, jaco_ki,
        jaco_kd, true /* with feedforward acceleration */);

    const std::string suffix{"_" + std::to_string(i)};
    jaco_controller->set_name("JacoInverseDynamicsController" + suffix);

    desired_state_input_ports->push_back(
        jaco_controller->get_input_port_desired_state());
    desired_acceleration_input_ports->push_back(
        jaco_controller->get_input_port_desired_acceleration());
    state_output_ports->push_back(
        plant->model_instance_state_output_port(jaco_instances[i].instance_id));
  }
}

JacoCommandReceiver* AddJacoCommandReceiver(
    const systems::InputPortDescriptor<double>& desired_state_input,
    const systems::InputPortDescriptor<double>& desired_acceleration_input,
    systems::DiagramBuilder<double>* builder) {
  int num_joints = 7;
  int num_fingers = 3;
  DRAKE_THROW_UNLESS((desired_state_input.size() == 18) ||
                     (desired_state_input.size() == 20));
  if (desired_state_input.size() == 18) {
    num_joints = 6;
  }

  JacoCommandReceiver* receiver =
      builder->AddSystem<JacoCommandReceiver>(num_joints, num_fingers);
  builder->Connect(receiver->get_output_port(0), desired_state_input);

  // lcmt_jaco_command does not include a reference acceleration, so use a
  // zero constant source for the controller's acceleration input.
  auto zero_feedforward_acceleration =
      builder->AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(
          desired_acceleration_input.size()));
  builder->Connect(
      zero_feedforward_acceleration->get_output_port(),
      desired_acceleration_input);
  return receiver;
}

JacoStatusSender* AddJacoStatusSender(
    const systems::OutputPort<double>& state_output,
    systems::DiagramBuilder<double>* builder) {
  int num_joints = 7;
  int num_fingers = 3;
  DRAKE_THROW_UNLESS((state_output.size() == 18) ||
                     (state_output.size() == 20));
  if (state_output.size() == 18) {
    num_joints = 6;
  }

  JacoStatusSender* sender =
      builder->AddSystem<JacoStatusSender>(num_joints, num_fingers);
  builder->Connect(state_output, sender->get_input_port(0));
  return sender;
}

int DoMain(void) {
  // Parse the configuration file.
  const SimulatedPlantConfiguration plant_configuration =
      // NOLINTNEXTLINE(whitespace/line_length)
      manipulation::pick_and_place_example::ParseSimulatedPlantConfigurationOrThrow(
          FLAGS_configuration_file);
  const OptitrackConfiguration optitrack_configuration =
      // NOLINTNEXTLINE(whitespace/line_length)
      manipulation::pick_and_place_example::ParseOptitrackConfigurationOrThrow(
          FLAGS_configuration_file);

  DRAKE_THROW_UNLESS(plant_configuration.robot_models.size() ==
                     optitrack_configuration.robot_base_optitrack_info.size());
  DRAKE_THROW_UNLESS(plant_configuration.object_models.size() ==
                     optitrack_configuration.object_optitrack_info.size());
  DRAKE_THROW_UNLESS(plant_configuration.table_models.size() ==
                     optitrack_configuration.table_optitrack_info.size());

  std::vector<ModelInstanceInfo<double>> jaco_instances,
      object_instances, table_instances;
  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildPickAndPlacePlant(
          plant_configuration, &jaco_instances, nullptr,
          &object_instances, &table_instances);
  model_ptr->set_default_compliant_material(
      plant_configuration.default_contact_material);
  model_ptr->set_contact_model_parameters(
      plant_configuration.contact_model_parameters);

  // Begin setting up the diagram.
  manipulation::util::SimDiagramBuilder<double> sim_diagram_builder;
  systems::DiagramBuilder<double>* builder =
      sim_diagram_builder.get_mutable_builder();
  systems::RigidBodyPlant<double>* plant =
      sim_diagram_builder.AddPlant(std::move(model_ptr));

  InputPortVector desired_state_input_ports;
  InputPortVector desired_acceleration_input_ports;
  OutputPortVector state_output_ports;
  AddJacoInstancesToDiagram(
      jaco_instances, plant, &desired_state_input_ports,
      &desired_acceleration_input_ports, &state_output_ports,
      &sim_diagram_builder);

  // Instantiate an LCM instance for use with publishers and subscribers.
  lcm::DrakeLcm lcm;
  int num_jaco = jaco_instances.size();
  std::vector<JacoCommandReceiver*> command_receivers;
  for (int i = 0; i < num_jaco; i++) {
    // All LCM traffic for this arm will occur on channels that end with the
    // suffix specified below.
    const std::string suffix = (num_jaco > 1) ? "_" + std::to_string(i) : "";
    auto jaco_command_sub = builder->AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<lcmt_jaco_command>(
            "disabled_KINOVA_JACO_COMMAND" + suffix, &lcm));
    jaco_command_sub->set_name("jaco_command_subscriber" + suffix);

    JacoCommandReceiver* receiver = AddJacoCommandReceiver(
        desired_state_input_ports[i],
        desired_acceleration_input_ports[i],
        builder);
    command_receivers.push_back(receiver);

    builder->Connect(jaco_command_sub->get_output_port(),
                     receiver->get_input_port(0));

    auto jaco_status_pub = builder->AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_jaco_status>(
            "disabled_KINOVA_JACO_STATUS" + suffix, &lcm));
    jaco_status_pub->set_name("jaco_status_publisher" + suffix);
    jaco_status_pub->set_publish_period(kJacoLcmStatusPeriod);

    JacoStatusSender* sender = AddJacoStatusSender(
        state_output_ports[i], builder);

    builder->Connect(sender->get_output_port(0),
                     jaco_status_pub->get_input_port());
  }


  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  const systems::OutputPort<double>& optitrack_lcm_output_port =
      AddOptitrackComponents(
          optitrack_configuration, tree,
          jaco_instances, object_instances,
          table_instances,
          plant->kinematics_results_output_port(),
          builder);

  // Add the visualizer.
  auto drake_visualizer =
      builder->AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm);
  drake_visualizer->set_publish_period(kJacoLcmStatusPeriod);
  builder->Connect(plant->get_output_port(0),
                   drake_visualizer->get_input_port(0));

  // Publish the contact results over LCM.
  auto contact_viz =
      builder->AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant->get_rigid_body_tree());
  auto contact_results_publisher = builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  builder->Connect(plant->contact_results_output_port(),
                   contact_viz->get_input_port(0));
  builder->Connect(contact_viz->get_output_port(0),
                   contact_results_publisher->get_input_port());
  contact_results_publisher->set_publish_period(kJacoLcmStatusPeriod);

  // Publish the mock Optitrack data over LCM.
  auto optitrack_pub = builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<optitrack::optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));
  optitrack_pub->set_publish_period(kJacoLcmStatusPeriod);
  builder->Connect(optitrack_lcm_output_port,
                   optitrack_pub->get_input_port());

  // Build the diagram.
  auto sys = sim_diagram_builder.Build();

  // Create and configure the simulator.
  systems::Simulator<double> simulator(*sys);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
      *sys, FLAGS_dt, &simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);
  simulator.set_publish_every_time_step(false);

  systems::Context<double>& context = simulator.get_mutable_context();

  // Open the gripper.
  for (int i = 0; i < num_jaco; i++) {
    int num_joints = desired_acceleration_input_ports[i].get().size();
    VectorX<double> initial_position = VectorX<double>::Zero(num_joints);
    initial_position(1) = 1.57;  // shoulder fore/aft angle, [rad]
    initial_position(3) = 2.0;   // elbow fore/aft angle, [rad]
    initial_position(5) = 2.0;   // elbow fore/aft angle, [rad]

    plant->SetModelInstancePositions(
        &sys->GetMutableSubsystemContext(*plant, &context),
        jaco_instances[i].instance_id, initial_position);

    command_receivers[i]->set_initial_position(
        &sys->GetMutableSubsystemContext(*(command_receivers[i]), &context),
        initial_position);
  }

  simulator.Initialize();

  // Simulate for the specified duration.
  lcm.StartReceiveThread();
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kinova_jaco_arm::pick_and_place::DoMain();
}
