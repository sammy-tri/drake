#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::aligned_allocator;
using std::string;
using std::vector;
using std::unique_ptr;
using std::make_unique;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

VectorX<double> get_iiwa_max_joint_velocities() {
  // These are the maximum joint velocities given in Section 4.3.2 "Axis data,
  // LBR iiwa 14 R820" of the "LBR iiwa 7 R800, LBR iiwa 14 R820 Specification".
  // That document is available here:
  // https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_lbr_iiwa_en.pdf
  return (VectorX<double>(7) << 1.483529,  //  85°/s in rad/s
          1.483529,                        //  85°/s in rad/s
          1.745329,                        // 100°/s in rad/s
          1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194)                        // 135°/s in rad/s
      .finished();
}

template <typename T>
Matrix6<T> ComputeLumpedGripperInertiaInEndEffectorFrame(
    const RigidBodyTree<T>& world_tree,
    int iiwa_instance, const std::string& end_effector_link_name,
    int wsg_instance) {
  KinematicsCache<T> world_cache = world_tree.CreateKinematicsCache();
  world_cache.initialize(world_tree.getZeroConfiguration());
  world_tree.doKinematics(world_cache);

  const RigidBody<T>* end_effector = world_tree.FindBody(
      end_effector_link_name, "iiwa14", iiwa_instance);
  Isometry3<T> X_WEE =
    world_tree.CalcBodyPoseInWorldFrame(world_cache, *end_effector);

  // The inertia of the added gripper is lumped into the last link of the
  // controller's iiwa arm model. This is motivated by the fact that the
  // gripper inertia is relatively large compared to the last couple links
  // in the iiwa arm model. And to completely rely on using feedback to cope
  // with added inertia, we need to either rely on larger gains (which will
  // cause simulation to explode without the gripper), or wait longer for
  // the integrator to kick in.

  // Computes the lumped inertia for the gripper.
  std::set<int> gripper_instance_set = {wsg_instance};
  Matrix6<T> lumped_gripper_inertia_W =
    world_tree.LumpedSpatialInertiaInWorldFrame(
        world_cache, gripper_instance_set);
  // Transfer it to the last iiwa link's body frame.
  Matrix6<T> lumped_gripper_inertia_EE =
      transformSpatialInertia(X_WEE.inverse(), lumped_gripper_inertia_W);
  lumped_gripper_inertia_EE += end_effector->get_spatial_inertia();

  return lumped_gripper_inertia_EE;
}

template Matrix6<double>
ComputeLumpedGripperInertiaInEndEffectorFrame(
    const RigidBodyTree<double>&, int, const std::string&, int);

void VerifyIiwaTree(const RigidBodyTree<double>& tree) {
  std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_1"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_1"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_2"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_2"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_3"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_3"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_4"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_4"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_5"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_5"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_6"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_6"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_7"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_7"] == joint_idx++);
}

void CreateTreedFromFixedModelAtPose(const std::string& model_file_name,
                                     RigidBodyTreed* tree,
                                     const Vector3d& position,
                                     const Vector3d& orientation) {
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr, position,
      orientation);

  // TODO(naveenoid) : consider implementing SDF version of this method.
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      model_file_name, drake::multibody::joints::kFixed,
      weld_to_frame, tree);
}

void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(7);
  *Kp << 100, 100, 100, 100, 100, 100, 100;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
  }
  *Ki = Eigen::VectorXd::Zero(7);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
