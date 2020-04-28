#include "drake/manipulation/franka_panda/panda_command_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace franka_panda {
namespace {

using Eigen::VectorXd;
constexpr int N = kPandaArmNumJoints;

class PandaCommandSenderTest : public testing::Test {
 public:
  PandaCommandSenderTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_panda_command& output() const {
    return dut_.get_output_port().Eval<lcmt_panda_command>(context_);
  }

 protected:
  PandaCommandSender dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
};

TEST_F(PandaCommandSenderTest, AcceptanceTest) {
  const VectorXd qv0 = VectorXd::LinSpaced(N * 2, 0.1, 0.2);
  const VectorXd q0 = qv0.head(N);
  const VectorXd v0 = qv0.tail(N);
  const std::vector<double> std_q0 = {q0.data(), q0.data() + q0.size()};
  const std::vector<double> std_v0 = {v0.data(), v0.data() + v0.size()};
  dut_.get_state_input_port().FixValue(&context_, qv0);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0);
  EXPECT_EQ(output().joint_velocity, std_v0);
  EXPECT_EQ(output().joint_torque, std::vector<double>(N, 0));

  const VectorXd t0 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const std::vector<double> std_t0 = {t0.data(), t0.data() + t0.size()};
  dut_.get_torque_input_port().FixValue(&context_, t0);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0);
  EXPECT_EQ(output().joint_velocity, std_v0);
  EXPECT_EQ(output().joint_torque, std_t0);
}

}  // namespace
}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
