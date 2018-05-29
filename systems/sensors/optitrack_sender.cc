#include "drake/systems/sensors/optitrack_sender.h"

#include <vector>

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/systems/sensors/optitrack_encoder.h"

namespace drake {
namespace systems {
namespace sensors {

using systems::sensors::TrackedBody;

namespace {
void PopulateRigidBody(const Eigen::Isometry3d& T_WF,
                       optitrack::optitrack_rigid_body_t* msg) {
    const Eigen::Vector3d trans = T_WF.translation();
    const Eigen::Quaterniond rot = Eigen::Quaterniond(T_WF.linear());

   msg->xyz[0] = static_cast<float>(trans[0]);
   msg->xyz[1] = static_cast<float>(trans[1]);
   msg->xyz[2] = static_cast<float>(trans[2]);

   msg->quat[0] = static_cast<float>(rot.x());
   msg->quat[1] = static_cast<float>(rot.y());
   msg->quat[2] = static_cast<float>(rot.z());
   msg->quat[3] = static_cast<float>(rot.w());
}
}  // namespace

OptitrackLcmFrameSender::OptitrackLcmFrameSender(int num_rigid_bodies)
    : num_rigid_bodies_(num_rigid_bodies) {
  DRAKE_DEMAND(num_rigid_bodies >= 0);
  this->DeclareAbstractInputPort();
  this->DeclareAbstractOutputPort(
      &OptitrackLcmFrameSender::CreateNewMessage,
      &OptitrackLcmFrameSender::PopulateTrackedBodyMessage);
}

OptitrackLcmFrameSender::OptitrackLcmFrameSender(
    const std::map<geometry::FrameId,
    std::pair<std::string, int>>& frame_map)
    : num_rigid_bodies_(frame_map.size()),
      frame_map_(frame_map) {
  this->DeclareAbstractInputPort();
  this->DeclareAbstractOutputPort(
      &OptitrackLcmFrameSender::CreateNewMessage,
      &OptitrackLcmFrameSender::PopulatePoseMessage);
}

optitrack::optitrack_frame_t OptitrackLcmFrameSender::CreateNewMessage() const {
  optitrack::optitrack_frame_t msg{};

  msg.num_rigid_bodies = num_rigid_bodies_;
  msg.rigid_bodies.resize(static_cast<size_t>(num_rigid_bodies_));

  return msg;
}

void OptitrackLcmFrameSender::PopulateTrackedBodyMessage(
    const Context<double>& context,
    optitrack::optitrack_frame_t* output) const {
  output->utime = static_cast<int64_t >(context.get_time() * 1e6);

  const std::vector<TrackedBody>* mocap_objects =
      this->EvalInputValue<std::vector<TrackedBody>>(context, 0);

  for (size_t i = 0; i < mocap_objects->size(); ++i) {
    output->rigid_bodies[i].id = (*mocap_objects)[i].id;
    PopulateRigidBody((*mocap_objects)[i].T_WF,
                      &(output->rigid_bodies[i]));
  }
}

void OptitrackLcmFrameSender::PopulatePoseMessage(
    const Context<double>& context,
    optitrack::optitrack_frame_t* output) const {
  output->utime = static_cast<int64_t >(context.get_time() * 1e6);

  const geometry::FramePoseVector<double>* poses =
      this->EvalInputValue<geometry::FramePoseVector<double>>(context, 0);

  int output_idx = 0;
  for (const auto& frame : frame_map_) {
    const Eigen::Isometry3d& pose = poses->value(frame.first);
    output->rigid_bodies[output_idx].id = frame.second.second;
    PopulateRigidBody(pose, &(output->rigid_bodies[output_idx++]));
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
