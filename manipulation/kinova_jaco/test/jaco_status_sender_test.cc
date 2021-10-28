#include "drake/manipulation/kinova_jaco/jaco_status_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {
namespace {

using Eigen::VectorXd;
constexpr int N = kJacoDefaultArmNumJoints;
constexpr int N_F = kJacoDefaultArmNumFingers;

class JacoStatusSenderTestBase : public testing::Test {
 public:
  JacoStatusSenderTestBase(int num_joints, int num_fingers)
      : dut_(num_joints, num_fingers),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_jaco_status& output() const {
    return dut_.get_output_port().Eval<lcmt_jaco_status>(context_);
  }

  void Fix(const systems::InputPort<double>& port, const Eigen::VectorXd& v) {
    port.FixValue(&context_, v);
  }

 protected:
  JacoStatusSender dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
};

class JacoStatusSenderTest : public JacoStatusSenderTestBase {
 public:
  JacoStatusSenderTest()
      : JacoStatusSenderTestBase(
            kJacoDefaultArmNumJoints, kJacoDefaultArmNumFingers) {}
};

class JacoStatusSenderNoFingersTest : public JacoStatusSenderTestBase {
 public:
  JacoStatusSenderNoFingersTest()
      : JacoStatusSenderTestBase(
            kJacoDefaultArmNumJoints, 0) {}
};

const std::vector<double> ToStdVec(const Eigen::VectorXd& in) {
  return std::vector<double>{in.data(), in.data() + in.size()};
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

TEST_F(JacoStatusSenderTest, DeprecatedAcceptanceTest) {
  constexpr int total_dof = N + N_F;

  const VectorXd state = VectorXd::LinSpaced(total_dof * 2, 0.0, 1.0);
  const VectorXd torque = VectorXd::LinSpaced(N, 2.0, 3.0);
  const VectorXd torque_external = VectorXd::LinSpaced(N, 4.0, 5.0);
  const VectorXd current = VectorXd::LinSpaced(N, 6.0, 7.0);

  // Fix only the required inputs ...
  Fix(dut_.get_state_input_port(), state);

  // ... so that some outputs have passthrough values ...
  EXPECT_EQ(output().joint_position,
            ToStdVec(state.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().joint_velocity,
            ToStdVec(state.segment(total_dof, kJacoDefaultArmNumJoints) / 2));
  EXPECT_EQ(output().finger_position,
            ToStdVec(
                state.segment(kJacoDefaultArmNumJoints,
                              kJacoDefaultArmNumFingers) * kFingerUrdfToSdk));
  EXPECT_EQ(output().finger_velocity,
            ToStdVec(
                state.tail(kJacoDefaultArmNumFingers) * kFingerUrdfToSdk));
  // ... and some outputs have default values.
  EXPECT_EQ(output().joint_torque,
            std::vector<double>(kJacoDefaultArmNumJoints, 0.0));
  EXPECT_EQ(output().joint_torque_external,
            std::vector<double>(kJacoDefaultArmNumJoints, 0.0));
  EXPECT_EQ(output().joint_current,
            std::vector<double>(kJacoDefaultArmNumJoints, 0.0));
  EXPECT_EQ(output().finger_torque,
            std::vector<double>(kJacoDefaultArmNumFingers, 0.0));
  EXPECT_EQ(output().finger_torque_external,
            std::vector<double>(kJacoDefaultArmNumFingers, 0.0));
  EXPECT_EQ(output().finger_current,
            std::vector<double>(kJacoDefaultArmNumFingers, 0.0));

  // Fix all of the inputs ...
  Fix(dut_.get_torque_input_port(), torque);
  Fix(dut_.get_torque_external_input_port(), torque_external);
  Fix(dut_.get_current_input_port(), current);

  // ... so all ouputs have values.
  EXPECT_EQ(output().joint_torque,
            ToStdVec(torque.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().joint_torque_external,
            ToStdVec(torque_external.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().joint_current,
            ToStdVec(current.head(kJacoDefaultArmNumJoints)));
}

#pragma GCC diagnostic pop

TEST_F(JacoStatusSenderTest, AcceptanceTestWithFingers) {
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const VectorXd f_q0 = VectorXd::LinSpaced(N_F, 1.2, 0.3);
  const VectorXd f_v0 = VectorXd::LinSpaced(N_F, 1.3, 0.4);
  const VectorXd zero = VectorXd::Zero(N);
  const VectorXd finger_zero = VectorXd::Zero(N_F);

  dut_.get_position_input_port().FixValue(&context_, q0);
  dut_.get_velocity_input_port().FixValue(&context_, v0);
  dut_.get_finger_position_input_port().FixValue(&context_, f_q0);
  dut_.get_finger_velocity_input_port().FixValue(&context_, f_v0);

  EXPECT_EQ(output().num_joints, kJacoDefaultArmNumJoints);
  EXPECT_EQ(output().joint_position, ToStdVec(q0));
  EXPECT_EQ(output().joint_velocity, ToStdVec(v0 / 2));
  EXPECT_EQ(output().num_fingers, kJacoDefaultArmNumFingers);
  EXPECT_EQ(output().finger_position, ToStdVec(f_q0 * kFingerUrdfToSdk));
  EXPECT_EQ(output().finger_velocity, ToStdVec(f_v0 * kFingerUrdfToSdk));
  EXPECT_EQ(output().joint_torque, ToStdVec(zero));
  EXPECT_EQ(output().joint_torque_external, ToStdVec(zero));
  EXPECT_EQ(output().joint_current, ToStdVec(zero));
  EXPECT_EQ(output().finger_torque, ToStdVec(finger_zero));
  EXPECT_EQ(output().finger_torque_external, ToStdVec(finger_zero));
  EXPECT_EQ(output().finger_current, ToStdVec(finger_zero));

  // Fix the optional inputs and see that
  const VectorXd t1 = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd t_ext1 = VectorXd::LinSpaced(N, 0.5, 0.6);
  const VectorXd current1 = VectorXd::LinSpaced(N, 0.6, 0.7);

  dut_.get_torque_input_port().FixValue(&context_, t1);
  dut_.get_torque_external_input_port().FixValue(&context_, t_ext1);
  dut_.get_current_input_port().FixValue(&context_, current1);

  EXPECT_EQ(output().joint_torque, ToStdVec(t1));
  EXPECT_EQ(output().joint_torque_external, ToStdVec(t_ext1));
  EXPECT_EQ(output().joint_current, ToStdVec(current1));
}

TEST_F(JacoStatusSenderNoFingersTest, AcceptanceNoFingers) {
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N, 0.3, 0.4);

  dut_.get_position_input_port().FixValue(&context_, q0);
  dut_.get_velocity_input_port().FixValue(&context_, v0);

  EXPECT_EQ(output().num_joints, kJacoDefaultArmNumJoints);
  EXPECT_EQ(output().joint_position, ToStdVec(q0));
  EXPECT_EQ(output().joint_velocity, ToStdVec(v0 / 2));
  EXPECT_EQ(output().num_fingers, 0);
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
