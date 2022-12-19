#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace examples {
namespace turntable {
namespace {

using geometry::SceneGraph;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialVelocity;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_double(table_velocity, 1, "");
DEFINE_double(ball_initial_x_velocity, 0.05, "");

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/turntable/table_and_ball.sdf");
  Parser(&plant, &scene_graph).AddModels(full_name);
  plant.Finalize();

  visualization::AddDefaultVisualization(&builder);

  Vector1d kp(0);
  Vector1d ki(0);
  Vector1d kd(4);

  auto pid_controller = builder.AddSystem<systems::controllers::PidController>(
      kp, ki, kd);

  std::vector<JointIndex> joints;
  joints.push_back(plant.GetJointByName("table_to_world").index());
  MatrixX<double> selector_matrix = plant.MakeStateSelectorMatrix(joints);
  auto selector = builder.AddSystem<systems::MatrixGain<double>>(
      selector_matrix);
  builder.Connect(plant.get_state_output_port(),
                  selector->get_input_port());
  builder.Connect(selector->get_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(pid_controller->get_output_port_control(),
                  plant.get_actuation_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& pid_context =
      diagram->GetMutableSubsystemContext(*pid_controller, diagram_context.get());
  pid_controller->get_input_port_desired_state().FixValue(
      &pid_context, Eigen::Vector2d(0, FLAGS_table_velocity));

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  SpatialVelocity<double> ball_initial(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, FLAGS_ball_initial_x_velocity, 0));
  plant.SetFreeBodySpatialVelocity(
      &plant_context, plant.GetBodyByName("ball"), ball_initial);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  //simulator.AdvanceTo(FLAGS_simulation_time);
  while(true) {
    const double time = simulator.get_context().get_time();
    drake::log()->info("{}", plant.GetVelocities(plant_context).transpose());
    simulator.AdvanceTo(time + 1.);
  }

  return 0;
}

}  // namespace
}  // namespace turntable
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::turntable::do_main();
}
