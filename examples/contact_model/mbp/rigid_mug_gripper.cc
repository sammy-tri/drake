#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;
using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::AddModelFromSdfFile;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(finger_width, 0.1, "The initial distance between the gripper "
    "fingers, when gripper_force > 0.");

// Parameters for posing the mug.
DEFINE_double(px, 0, "The x-position of the center, bottom of the mug");
DEFINE_double(py, 0, "The y-position of the center, bottom of the mug");
DEFINE_double(pz, 0, "The z-position of the center, bottom of the mug");
DEFINE_double(rx, 0, "The x-rotation of the mug around its origin - the center "
    "of its bottom (in degrees). Rotation order: X, Y, Z");
DEFINE_double(ry, 0, "The y-rotation of the mug around its origin - the center "
    "of its bottom (in degrees). Rotation order: X, Y, Z");
DEFINE_double(rz, 0, "The z-rotation of the mug around its origin - the center "
    "of its bottom (in degrees). Rotation order: X, Y, Z");

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>();
  std::string full_name =
      FindResourceOrThrow("drake/examples/contact_model/mbp/schunk_wsg_50.sdf");
  AddModelFromSdfFile(full_name, &plant, &scene_graph);

  full_name =
      FindResourceOrThrow("drake/examples/contact_model/mbp/cylinder_mug.sdf");
  AddModelFromSdfFile(full_name, &plant, &scene_graph);

  // Add gravity to the model.
  plant.AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // Now the model is complete.
  plant.Finalize();

  DRAKE_DEMAND(plant.num_actuators() == 1);
  DRAKE_DEMAND(plant.num_actuated_dofs() == 1);

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // There is no input actuation in this example for the passive dynamics.
  plant_context.FixInputPort(
      plant.get_actuation_input_port().get_index(), Vector1d(0));

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& finger_slider =
      plant.GetJointByName<PrismaticJoint>("finger_sliding_joint");

  // Get mug body so we can set its initial pose.
  const Body<double>& mug = plant.GetBodyByName("Mug");

  Isometry3d X_WM;
  Vector3d rpy(FLAGS_rx * M_PI / 180,
               FLAGS_ry * M_PI / 180,
               (FLAGS_rz * M_PI / 180) + M_PI);
  X_WM.linear() = RotationMatrix<double>(RollPitchYaw<double>(rpy)).matrix();
  X_WM.translation() = Vector3d(FLAGS_px, FLAGS_py, FLAGS_pz);
  plant.model().SetFreeBodyPoseOrThrow(mug, X_WM, &plant_context);

  // Set initial state.
  finger_slider.set_translation(&plant_context, -FLAGS_finger_width);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple cart pole demo using Drake's MultibodyPlant,"
      "with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::cart_pole::do_main();
}
