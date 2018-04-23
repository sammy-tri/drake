#include "drake/examples/multibody/bouncing_ball/make_bouncing_ball_plant.h"

#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

using geometry::GeometrySystem;
using geometry::Sphere;
using geometry::HalfSpace;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeBouncingBallPlant(double radius, double mass,
                      const CoulombFriction<double>& surface_friction,
                      const Vector3<double>& gravity_W,
                      double dt,
                      geometry::GeometrySystem<double>* geometry_system) {
  auto plant = std::make_unique<MultibodyPlant<double>>(dt);

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  if (geometry_system != nullptr) {
    plant->RegisterAsSourceForGeometrySystem(geometry_system);

    Vector3<double> normal_W(0, 0, 1);
    Vector3<double> point_W(0, 0, 0);

    // A half-space for the ground geometry.
    plant->RegisterCollisionGeometry(
        plant->world_body(),
        HalfSpace::MakePose(normal_W, point_W), HalfSpace(), surface_friction,
        geometry_system);
    plant->RegisterVisualGeometry(
        plant->world_body(),
        HalfSpace::MakePose(normal_W, point_W), HalfSpace(),
        geometry_system);

    // Add sphere geometry for the ball.
    //const double length = 4 * radius;
    plant->RegisterCollisionGeometry(
        ball,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        Isometry3<double>::Identity(),
      //  geometry::Cylinder(radius, length), surface_friction, geometry_system);
        Sphere(radius), surface_friction, geometry_system);
    plant->RegisterVisualGeometry(
        ball,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        Isometry3<double>::Identity(),
        //  geometry::Cylinder(radius, length), surface_friction, geometry_system);
        Sphere(radius), geometry_system);

#if 0
    // Add a bunch of little spheres to simulate "multi-contact".
    const int nspheres = 13;
    const double contact_spheres_radius = radius / 50.0;
    for (int i = 0; i < nspheres; ++i) {
      const double theta = 2.0 * i / nspheres * M_PI;
      const double x = cos(theta) * (radius + 1.01 * contact_spheres_radius);
      const double y = sin(theta) * (radius + 1.01 * contact_spheres_radius);
      Isometry3<double> X_BG = Isometry3<double>::Identity();
      // Top spheres:
      X_BG.translation() << x, y, length / 2;
      plant->RegisterCollisionGeometry(
          ball,
          /* Pose X_BG of the geometry frame G in the ball frame B. */
          X_BG,
          Sphere(contact_spheres_radius), surface_friction, geometry_system);

      // Bottom spheres:
      X_BG.translation() << x, y, -length / 2;
      plant->RegisterCollisionGeometry(
          ball,
          /* Pose X_BG of the geometry frame G in the ball frame B. */
          X_BG,
          Sphere(contact_spheres_radius), surface_friction, geometry_system);
    }
#endif
  }


  // A second geometry
  const RigidBody<double>& ball2 = plant->AddRigidBody("Ball2", M_Bcm);

  if (geometry_system != nullptr) {
    // Add sphere geometry for the ball.
    //const double length = 4 * radius;
    plant->RegisterCollisionGeometry(
        ball2,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        Isometry3<double>::Identity(),
        Sphere(radius), surface_friction, geometry_system);
    plant->RegisterVisualGeometry(
        ball2,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        Isometry3<double>::Identity(),
        Sphere(radius), geometry_system);

#if 0
    // Add a bunch of little spheres to simulate "multi-contact".
    const int nspheres = 13;
    const double contact_spheres_radius = radius / 50.0;
    for (int i = 0; i < nspheres; ++i) {
      const double theta = 2.0 * i / nspheres * M_PI;
      const double x = cos(theta) * (radius + 1.01 * contact_spheres_radius);
      const double y = sin(theta) * (radius + 1.01 * contact_spheres_radius);
      Isometry3<double> X_BG = Isometry3<double>::Identity();
      // Top spheres:
      X_BG.translation() << x, y, length / 2;
      plant->RegisterCollisionGeometry(
          ball,
          /* Pose X_BG of the geometry frame G in the ball frame B. */
          X_BG,
          Sphere(contact_spheres_radius), surface_friction, geometry_system);

      // Bottom spheres:
      X_BG.translation() << x, y, -length / 2;
      plant->RegisterCollisionGeometry(
          ball,
          /* Pose X_BG of the geometry frame G in the ball frame B. */
          X_BG,
          Sphere(contact_spheres_radius), surface_friction, geometry_system);
    }
#endif
  }

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(gravity_W);

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
