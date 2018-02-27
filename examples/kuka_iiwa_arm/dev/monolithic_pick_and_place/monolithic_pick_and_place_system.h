#pragma once

#include <vector>

#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_planner.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_plant.h"
#include "drake/examples/kuka_iiwa_arm/lcm_plan_interpolator.h"
#include "drake/manipulation/pick_and_place_example/pick_and_place_configuration.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/** A custom systems::Diagram representing the entire pick-and-place demo. It
consists of a manipulation::pick_and_place_example::LcmPlant a manipulation::pick_and_place_example::LcmPlanner, an
LcmPlanInterpolator, and a systems::DrakeVisualizer. A systems::ZeroOrderHold is
used to prevent an algebraic loop between the plant and the plan interpolator.

                 ┌─────────┐ wsg command
     ┌──────────▶│         │─────────┐      ┌────────┐
     │           │ Lcm     │         │      │        │
     │ ┌────────▶│ Planner │         │      │        │
     │ │         │         │plan     │      │        │            ┌────────────┐
     │ │ ┌──────▶│         ├─────┐   │      │        │ vis info   │ Drake      │
     │ │ │       └─────────┘     │   └─────▶│        ├───────────▶│ Visualizer │
     │ │ │                       │          │        │            └────────────┘
     │ │ │    ┌──────────────────┘          │ Lcm    │ optitrack
     │ │ │    │                             │ Plant  ├────────────────┐
     │ │ │    │  ┌──────────────┐    iiwa   │        │                │
     │ │ │    └─▶│ Lcm          │ command   │        │ wsg_status     │
     │ │ │       │ Plan         ├──────────▶│        ├──────────────┐ │
     │ │ │    ┌─▶│ Interpolator │           │        │              │ │
     │ │ │    │  └──────────────┘           │        │ iiwa_status  │ │
     │ │ │ ┌──┴──┐                          │        ├────────────┐ │ │
     │ │ │ │ ZOH │◀──────────┐              └────────┘            │ │ │
     │ │ │ └─────┘           │                                    │ │ │
     │ │ │                   │                                    │ │ │
     │ │ └───────────────────┴────────────────────────────────────┘ │ │
     │ └────────────────────────────────────────────────────────────┘ │
     └────────────────────────────────────────────────────────────────┘
**/

class MonolithicPickAndPlaceSystem : public systems::Diagram<double> {
 public:
  MonolithicPickAndPlaceSystem(
      // NOLINTNEXTLINE(whitespace/line_length)
      const manipulation::pick_and_place_example::SimulatedPlantConfiguration& plant_configuration,
      // NOLINTNEXTLINE(whitespace/line_length)
      const manipulation::pick_and_place_example::OptitrackConfiguration& optitrack_configuration,
      // NOLINTNEXTLINE(whitespace/line_length)
      const std::vector<manipulation::pick_and_place_example::PlannerConfiguration>&
          planner_configurations,
      bool single_move);

  const systems::OutputPort<double>& get_output_port_contact_results() const {
    return this->get_output_port(output_port_contact_results_);
  }

  const systems::OutputPort<double>& get_output_port_plant_state() const {
    return this->get_output_port(output_port_plant_state_);
  }

  const RigidBodyTree<double>& get_tree() const {
    return this->plant_->get_tree();
  }

  bool is_done(const systems::Context<double>& context) const;

  void Initialize(systems::Context<double>* context);

  const manipulation::pick_and_place_example::WorldState& world_state(
      const systems::Context<double>& context, int index) const;

 private:
  // Output ports.
  int output_port_contact_results_{-1};
  int output_port_plant_state_{-1};

  // Subsystems
  pick_and_place::LcmPlant* plant_{};
  std::vector<pick_and_place::LcmPlanner*> planners_{};
  std::vector<LcmPlanInterpolator*> plan_interpolators_{};
};
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
