#include "drake/examples/multibody/objects_falling/make_objects_falling.h"

#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace objects_falling {

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::Cylinder;
using geometry::FrameId;
using geometry::GeometrySystem;
using geometry::Sphere;
using geometry::HalfSpace;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

void AddCylinderWithMultiContact(
    MultibodyPlant<double>* plant, GeometrySystem<double>* geometry_system,
    const RigidBody<double>& body,
    double radius, double length, const CoulombFriction<double>& friction,
    double contact_radius, int num_contacts) {
  // Add sphere geometry for the ball.
  plant->RegisterCollisionGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3d::Identity(), Cylinder(radius - 1.05 * contact_radius, length),
      friction, geometry_system);

  // Add a bunch of little spheres to simulate "multi-contact".
  const int nspheres = num_contacts;
  const double contact_spheres_radius = contact_radius;
  for (int i = 0; i < nspheres; ++i) {
    const double theta = 2.0 * i / nspheres * M_PI;
    const double x = cos(theta) * radius;
    const double y = sin(theta) * radius;
    Isometry3<double> X_BG = Isometry3<double>::Identity();
    // Top spheres:
    X_BG.translation() << x, y, length / 2;
    plant->RegisterCollisionGeometry(
        body,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        X_BG,
        Sphere(contact_spheres_radius), friction, geometry_system);

    // Bottom spheres:
    X_BG.translation() << x, y, -length / 2;
    plant->RegisterCollisionGeometry(
        body,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        X_BG,
        Sphere(contact_spheres_radius), friction, geometry_system);
  }
}

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeObjectsFallingPlant(
    double radius, double mass, const Vector3<double>& gravity,
    int nballs, int ncylinders,
    double time_step,
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_THROW_UNLESS(geometry_system != nullptr);

  double theta = M_PI / 6;  // each plane forms this angle with the x-y plane.
  int nplanes = 6;

  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);

  CoulombFriction<double> friction(1.0, 0.3);

  plant->RegisterAsSourceForGeometrySystem(geometry_system);

  // The world's geometry.
  // Projection aligned with xhat, at theta from x-y plane.
  Matrix3d R = AngleAxisd(2.0 * M_PI / nplanes, Vector3d::UnitZ()).matrix();
  Vector3d n1 = Vector3d(cos(theta), 0, sin(theta));
  Vector3d ni = n1;
  for (int i = 0; i < nplanes; ++i) {
    Vector3<double> point_W(0, 0, -0.2);
    plant->RegisterCollisionGeometry(
        plant->world_body(),
        HalfSpace::MakePose(ni, point_W), HalfSpace(), friction,
        geometry_system);
    ni = R * ni;
  }
  plant->RegisterCollisionGeometry(
      plant->world_body(),
      HalfSpace::MakePose(Vector3<double>::UnitZ(), Vector3<double>::Zero()),
      HalfSpace(), friction, geometry_system);
#if 0
  Vector3d n2 = R * n1;
  Vector3d n3 = R * n2;
  Vector3<double> point_W(0, 0, 0);
  plant->RegisterAnchoredGeometry(
      HalfSpace::MakePose(n1, point_W), HalfSpace(), geometry_system);
  plant->RegisterAnchoredGeometry(
      HalfSpace::MakePose(n2, point_W), HalfSpace(), geometry_system);
  plant->RegisterAnchoredGeometry(
      HalfSpace::MakePose(n3, point_W), HalfSpace(), geometry_system);
#endif

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  for (int i = 0; i < nballs; ++i) {
    std:: string name = "Ball" + std::to_string(i);
    const RigidBody<double> &ball = plant->AddRigidBody(name, M_Bcm);
    // Add sphere geometry for the ball.
    plant->RegisterCollisionGeometry(
        ball,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        Isometry3d::Identity(),
        Sphere(radius), friction, geometry_system);
  }

  for (int i = 0; i < ncylinders; ++i) {
    std:: string name = "Cylinder" + std::to_string(i);
    const RigidBody<double> &ball = plant->AddRigidBody(name, M_Bcm);

    AddCylinderWithMultiContact(
        plant.get(), geometry_system, ball,
        radius/2, 2 * radius, friction,
        radius / 50, 8);

    // Add sphere geometry for the ball.
    plant->RegisterCollisionGeometry(
        ball,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        Isometry3d::Identity(),
        Cylinder(radius/2, 2 * radius), friction, geometry_system);
  }
  
  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(gravity);

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

}  // namespace objects_falling
}  // namespace multibody
}  // namespace examples
}  // namespace drake
