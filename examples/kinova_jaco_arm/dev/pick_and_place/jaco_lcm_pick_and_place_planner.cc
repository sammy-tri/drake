#include <string>

#include <gflags/gflags.h>
#include "optitrack/optitrack_frame_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kinova_jaco_arm/dev/pick_and_place/jaco_lcm_planner.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/manipulation/pick_and_place_example/pick_and_place_configuration.h"
#include "drake/manipulation/pick_and_place_example/pick_and_place_configuration_parsing.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_int32(task_index, 0,
             "Index of the task to use from the configuration file.");
DEFINE_bool(use_channel_suffix, false,
            "If true, append a suffix to channel names");
DEFINE_string(configuration_file,
              "drake/manipulation/pick_and_place_example/configuration/"
              "yellow_posts.pick_and_place_configuration",
              "Path to the configuration file.");


namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace pick_and_place {
namespace {
using manipulation::pick_and_place_example::PlannerConfiguration;
using manipulation::pick_and_place_example::OptitrackConfiguration;

int DoMain(void) {
  // Parse the configuration file.
  const PlannerConfiguration planner_configuration =
      manipulation::pick_and_place_example::ParsePlannerConfigurationOrThrow(
          FLAGS_configuration_file,
          manipulation::pick_and_place_example::TaskIndex(FLAGS_task_index));

  const OptitrackConfiguration optitrack_configuration =
      manipulation::pick_and_place_example::ParseOptitrackConfigurationOrThrow(
          FLAGS_configuration_file);

  // Begin setting up the diagram.
  systems::DiagramBuilder<double> builder;

  // TODO(sam.creasey) detect the number of joints.

  // Add the LcmPlanner block, which contains all of the demo logic.
  auto planner = builder.AddSystem<JacoLcmPlanner>(
      planner_configuration, optitrack_configuration, 7, false);

  // All LCM traffic for this planner will occur on channels that end with the
  // suffix specified below.
  std::string suffix =
      (FLAGS_use_channel_suffix)
          ? "_" + std::to_string(planner_configuration.robot_index)
          : "";

  // Instantiate an LCM instance for use with publishers and subscribers.
  lcm::DrakeLcm lcm;

  // Add LCM subscribers.
  auto jaco_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_jaco_status>(
          "KINOVA_JACO_STATUS" + suffix, &lcm));
  auto optitrack_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<optitrack::optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));

  // Add LCM publishers.
  auto plan_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::robot_plan_t>(
          "COMMITTED_ROBOT_PLAN" + suffix, &lcm));
  plan_pub->set_publish_period(planner_configuration.period_sec);

  // Connect subscribers to planner input ports.
  builder.Connect(jaco_status_sub->get_output_port(),
                  planner->get_input_port_jaco_status());
  builder.Connect(optitrack_sub->get_output_port(),
                  planner->get_input_port_optitrack_message());

  // Connect publishers to planner output ports.
  builder.Connect(planner->get_output_port_jaco_plan(),
                  plan_pub->get_input_port());

  // Build the diagram.
  auto sys = builder.Build();

  systems::lcm::LcmDrivenLoop loop(
      *sys, *jaco_status_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<lcmt_jaco_status>>());
  loop.set_publish_on_every_received_message(false);

  // Wait for messages from each subscriber before proceeding.
  drake::log()->info("Waiting for optitrack message ...");
  optitrack_sub->WaitForMessage(0);
  drake::log()->info("Optitrack message received.");
  drake::log()->info("Waiting for Jaco status message ...");
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  drake::log()->info("Jaco status message received.");
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);

  // Initialize and run the lcm-driven loop.
  loop.get_mutable_context().set_time(msg_time);
  loop.RunToSecondsAssumingInitialized();

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
