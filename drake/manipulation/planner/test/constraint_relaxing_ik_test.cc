#include "drake/manipulation/planner/constraint_relaxing_ik.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/find_resource.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

inline double get_orientation_difference(const Matrix3<double>& rot0,
                                         const Matrix3<double>& rot1) {
  AngleAxis<double> err(rot0.transpose() * rot1);
  return err.angle();
}
}

// N random samples are taken from the configuration space (q), and
// the corresponding end effector poses are computed with forward
// kinematics.  We use inverse kinematics to try to recover a set of
// joint angles that would achieve these poses. This test checks that
// an IK solution can be computed, and that the resulting pose lies
// within the given tolerance from the forward kinematics poses.
GTEST_TEST(ConstraintRelaxingIkTest, SolveIkFromFk) {
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  std::unique_ptr<RigidBodyTree<double>> iiwa =
      std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());

  KinematicsCache<double> cache = iiwa->CreateKinematicsCache();

  const std::string kEndEffectorLinkName = "iiwa_link_ee";
  const RigidBody<double>* end_effector = iiwa->FindBody(kEndEffectorLinkName);

  IKResults ik_res;
  ConstraintRelaxingIk ik_planner(kModelPath, kEndEffectorLinkName,
                                  Isometry3<double>::Identity());
  ConstraintRelaxingIk::IkCartesianWaypoint wp;
  wp.pos_tol = Vector3<double>(0.001, 0.001, 0.001);
  wp.rot_tol = 0.005;
  wp.constrain_orientation = true;
  std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints(1, wp);

  const VectorX<double> kQcurrent = iiwa->getZeroConfiguration();
  VectorX<double> q_fk;

  const double kEpsilon = 1e-8;
  const Vector3<double> kUpperBound =
      wp.pos_tol + kEpsilon * Vector3<double>::Ones();
  const Vector3<double> kLowerBound =
      -wp.pos_tol - kEpsilon * Vector3<double>::Ones();
  std::default_random_engine rand_generator(1234);

  for (int i = 0; i < 100; ++i) {
    q_fk = iiwa->getRandomConfiguration(rand_generator);
    cache.initialize(q_fk);
    iiwa->doKinematics(cache);

    Isometry3<double> fk_pose =
        iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);
    waypoints[0].pose = fk_pose;

    bool ret =
        ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &ik_res);
    EXPECT_TRUE(ret);

    cache.initialize(ik_res.q_sol[1]);
    iiwa->doKinematics(cache);
    Isometry3<double> ik_pose =
        iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);
    Vector3<double> pos_diff = ik_pose.translation() - fk_pose.translation();
    double rot_diff =
        get_orientation_difference(ik_pose.linear(), fk_pose.linear());

    EXPECT_TRUE((pos_diff.array() <= kUpperBound.array()).all());
    EXPECT_TRUE((pos_diff.array() >= kLowerBound.array()).all());

    // cos(ang_diff) >= cos(tol) is the actual constraint in the IK.
    EXPECT_TRUE(std::cos(rot_diff) + kEpsilon >= std::cos(wp.rot_tol));
  }
}

GTEST_TEST(ConstraintRelaxingIkTest, IiwaPickAndPlacePlan) {
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  std::unique_ptr<RigidBodyTree<double>> iiwa =
      std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());

  KinematicsCache<double> cache = iiwa->CreateKinematicsCache();

  const std::string kEndEffectorLinkName = "iiwa_link_ee";
  ConstraintRelaxingIk ik_planner(kModelPath, kEndEffectorLinkName,
                                  Isometry3<double>::Identity());
  ConstraintRelaxingIk::IkCartesianWaypoint wp;
  wp.pose.translation() = Vector3<double>(0.74, -0.36, 0.26);
  wp.pos_tol = Vector3<double>(0.005, 0.005, 0.005);
  wp.rot_tol = 0.05;
  wp.constrain_orientation = true;
  std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints(1, wp);

  VectorX<double> q_current(7);
  VectorX<double> q_expected(7);
  q_current << 1.48775, 1.57605, -0.293782, 0.622705, 0.130398, 0.597814,
      0.187618;
  q_expected << -0.698467, 1.25724, -2.77564, 0.974666, -0.976788, 0.673077,
      -2.69116;
  IKResults ik_res;
  bool ret = ik_planner.PlanSequentialTrajectory(waypoints, q_current, &ik_res);
  EXPECT_TRUE(ret);
  EXPECT_TRUE(CompareMatrices(q_expected, ik_res.q_sol.back(), 1e-1,
                              MatrixCompareType::absolute));

  ik_res.q_sol.clear();
  // Perturb the initial conditions very slightly and demonstrate that
  // the result changes significantly.  This isn't exactly a desirable
  // behavior, but it should be captured.
  q_current << 1.48739, 1.57592, -0.29502, 0.622659, 0.131036, 0.597764,
      0.188401;
  q_expected << -1.006, 1.79887, -1.44922, 0.992799, 0.563517, -0.0341407,
      0.750103;
  ret = ik_planner.PlanSequentialTrajectory(waypoints, q_current, &ik_res);
  EXPECT_TRUE(ret);
  EXPECT_TRUE(CompareMatrices(q_expected, ik_res.q_sol.back(), 1e-1,
                              MatrixCompareType::absolute));
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
