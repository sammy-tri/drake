#include "drake/manipulation/planner/constraint_relaxing_ik.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {
constexpr int kDefaultRandomSeed = 1234;
}  // namespace

ConstraintRelaxingIk::ConstraintRelaxingIk(
    const std::string& model_path,
    const std::string& end_effector_link_name,
    const Isometry3<double>& base_to_world)
    : rand_generator_(kDefaultRandomSeed) {
  auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      base_to_world);

  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, base_frame, robot_.get());

  SetEndEffector(end_effector_link_name);
}

bool ConstraintRelaxingIk::PlanSequentialTrajectory(
    const std::vector<IkCartesianWaypoint>& waypoints,
    const VectorX<double>& q_current,
    const VectorX<double>& q_nom_in,
   IKResults* ik_res) {
  DRAKE_DEMAND(ik_res);
  int num_steps = static_cast<int>(waypoints.size());

#if 1
  Eigen::VectorXd t = Eigen::VectorXd::Zero(num_steps + 1);
  Eigen::MatrixXd q_seed = Eigen::MatrixXd::Zero(q_current.size(), num_steps + 1);
  Eigen::MatrixXd q_nom = Eigen::MatrixXd::Zero(q_current.size(), num_steps + 1);
  std::vector<RigidBodyConstraint*> constraint_array;

  q_seed.col(0) = q_current;
  q_nom.col(0) = q_current;

  IKoptions ikoptions(robot_.get());;
  ikoptions.setDebug(true);
  for (size_t i = 0; i < waypoints.size(); i++) {
    q_seed.col(i + 1) = q_nom_in;
    q_nom.col(i + 1) = q_nom_in;
    const IkCartesianWaypoint& waypoint = waypoints[i];
    double time = i + 1;
    t(i + 1) = time;
    Eigen::Vector2d tspan(time - 0.1, time + 0.1);

    // Adds a position constraint.
    Vector3<double> pos_lb = waypoint.pose.translation() - waypoint.pos_tol;
    Vector3<double> pos_ub = waypoint.pose.translation() + waypoint.pos_tol;

    WorldPositionConstraint* pos_con =
        new WorldPositionConstraint(robot_.get(), end_effector_body_idx_,
                                    Vector3<double>::Zero(), pos_lb, pos_ub,
                                    tspan);

    constraint_array.push_back(pos_con);

    // Adds a rotation constraint.
    WorldQuatConstraint* quat_con =
        new WorldQuatConstraint(robot_.get(), end_effector_body_idx_,
                                math::rotmat2quat(waypoint.pose.linear()),
                                waypoint.rot_tol, tspan);
    if (waypoint.constrain_orientation) {
      constraint_array.push_back(quat_con);
    }
  }

  *ik_res = inverseKinTrajSimple(robot_.get(), t, q_seed, q_nom, constraint_array,
                                 ikoptions);

  if (ik_res->info[0] != 1) {
    return false;
  }

  for (size_t i = 0; i < ik_res->q_sol.size(); i++) {
    drake::log()->info("step {} sol {}", i, ik_res->q_sol[i].transpose());
  }
  return true;

#else

  VectorX<double> q_prev = q_current;
  VectorX<double> q0 = q_current;
  VectorX<double> q_sol = q_current;

  ik_res->infeasible_constraints.clear();
  ik_res->info.resize(num_steps + 1);
  ik_res->q_sol.resize(num_steps + 1);
  ik_res->info[0] = 1;
  ik_res->q_sol[0] = q_current;

  int step_ctr = 0;
  int relaxed_ctr = 0;
  int random_ctr = 0;

  enum class RelaxMode { kRelaxPosTol = 0, kRelaxRotTol = 1 };

  // These numbers are arbitrarily picked by siyuan.
  const int kMaxNumInitialGuess = 50;
  const int kMaxNumConstraintRelax = 10;
  const Vector3<double> kInitialPosTolerance(0.01, 0.01, 0.01);
  const double kInitialRotTolerance = 0.01;
  const double kConstraintShrinkFactor = 0.5;
  const double kConstraintGrowFactor = 1.5;

  for (const auto& waypoint : waypoints) {
    // Sets the initial constraints guess bigger than the desired tolerance.
    Vector3<double> pos_tol = kInitialPosTolerance;
    double rot_tol = kInitialRotTolerance;

    if (!waypoint.constrain_orientation) rot_tol = 0;

    // Sets mode to reduce position tolerance.
    RelaxMode mode = RelaxMode::kRelaxPosTol;

    // Solves point IK with constraint fiddling and random start.
    while (true) {
      if (!waypoint.constrain_orientation)
        DRAKE_DEMAND(mode == RelaxMode::kRelaxPosTol);

      std::vector<int> info;
      std::vector<std::string> infeasible_constraints;
      if (waypoints.size() == 1) {
        drake::log()->debug("Zeroing q_seed and q_nom");
        q0.fill(0);
        q_prev.fill(0);
      }
      bool res = SolveIk(waypoint, q0, q_prev, pos_tol, rot_tol, &q_sol, &info,
                         &infeasible_constraints);

      if (res) {
        // Breaks if the current tolerance is below given threshold.
        if ((rot_tol <= waypoint.rot_tol) &&
            (pos_tol.array() <= waypoint.pos_tol.array()).all()) {
          break;
        }

        drake::log()->info("Solution exceeded tol, rot {} {}, pos {} {}",
                           rot_tol, waypoint.rot_tol,
                           pos_tol.transpose(), waypoint.pos_tol.transpose());

        // Alternates between kRelaxPosTol and kRelaxRotTol
        if (mode == RelaxMode::kRelaxPosTol && waypoint.constrain_orientation) {
          rot_tol *= kConstraintShrinkFactor;
          mode = RelaxMode::kRelaxRotTol;
        } else {
          pos_tol *= kConstraintShrinkFactor;
          mode = RelaxMode::kRelaxPosTol;
        }
        // Sets the initial guess to the current solution.
        q0 = q_sol;
      } else {
        // Relaxes the constraints no solution is found.
        if (mode == RelaxMode::kRelaxRotTol && waypoint.constrain_orientation) {
          rot_tol *= kConstraintGrowFactor;
        } else {
          pos_tol *= kConstraintGrowFactor;
        }
        relaxed_ctr++;
      }

      // Switches to a different initial guess and start over if we have relaxed
      // the constraints for max times.
      if (relaxed_ctr > kMaxNumConstraintRelax) {
        // Make a random initial guess.
        q0 = robot_->getRandomConfiguration(rand_generator_);
        // Resets constraints tolerance.
        pos_tol = kInitialPosTolerance;
        rot_tol = kInitialRotTolerance;
        if (!waypoint.constrain_orientation) rot_tol = 0;
        mode = RelaxMode::kRelaxPosTol;
        drake::log()->warn(
            "IK FAILED at step {} max constraint relaxing iter: {}",
            step_ctr, relaxed_ctr);
        relaxed_ctr = 0;
        random_ctr++;
      }

      // Admits failure and returns false.
      if (random_ctr > kMaxNumInitialGuess) {
        drake::log()->error("IK FAILED at step {} max random starts: {}",
                            step_ctr, random_ctr);
        // Returns information about failure.
        ik_res->info[step_ctr + 1] = info[0];
        ik_res->q_sol[step_ctr + 1] = q_sol;
        ik_res->infeasible_constraints = infeasible_constraints;
        return false;
      }
    }

    // Sets next IK's initial and bias to current solution.
    q_prev = q_sol;
    q0 = q_sol;

    ik_res->info[step_ctr + 1] = 1;
    ik_res->q_sol[step_ctr + 1] = q_sol;
    step_ctr++;
  }

  for (size_t i = 0; i < ik_res->q_sol.size(); i++) {
    drake::log()->info("IK step {} info {} q {}",
                       i, ik_res->info[i], ik_res->q_sol[i].transpose());
  }

  return true;
#endif
}

bool ConstraintRelaxingIk::SolveIk(
    const IkCartesianWaypoint& waypoint,
    const VectorX<double>& q0,
    const VectorX<double>& q_nom,
    const Vector3<double>& pos_tol, double rot_tol,
    VectorX<double>* q_res, std::vector<int>* info,
    std::vector<std::string>* infeasible_constraints) {
  DRAKE_DEMAND(q_res);
  DRAKE_DEMAND(info);
  DRAKE_DEMAND(infeasible_constraints);

  info->resize(1);
  std::vector<RigidBodyConstraint*> constraint_array;
  IKoptions ikoptions(robot_.get());
  ikoptions.setDebug(true);
  ikoptions.setMajorOptimalityTolerance(2e-4);

  // Adds a position constraint.
  Vector3<double> pos_lb = waypoint.pose.translation() - pos_tol;
  Vector3<double> pos_ub = waypoint.pose.translation() + pos_tol;

  WorldPositionConstraint pos_con(robot_.get(), end_effector_body_idx_,
                                  Vector3<double>::Zero(), pos_lb, pos_ub,
                                  Vector2<double>::Zero());

  constraint_array.push_back(&pos_con);

  // Adds a rotation constraint.
  WorldQuatConstraint quat_con(robot_.get(), end_effector_body_idx_,
                               math::rotmat2quat(waypoint.pose.linear()),
                               rot_tol);//, Vector2<double>::Zero());
  if (waypoint.constrain_orientation) {
    constraint_array.push_back(&quat_con);
  }

  inverseKin(robot_.get(), q0, q_nom, constraint_array.size(),
             constraint_array.data(), ikoptions, q_res, info->data(),
             infeasible_constraints);
  drake::log()->info("info: {}", (*info)[0]);

  return (*info)[0] == 1;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
