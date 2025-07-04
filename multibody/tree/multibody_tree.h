#pragma once

#include <algorithm>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/random.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/topology/graph.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/articulated_body_force_cache.h"
#include "drake/multibody/tree/articulated_body_inertia_cache.h"
#include "drake/multibody/tree/element_collection.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T>
class RigidBodyFrame;
template <typename T>
class Frame;
template <typename T>
class RigidBody;
template <typename T>
class Joint;
template <typename T>
class JointActuator;
template <typename T>
class ForceElement;
template <typename T>
class UniformGravityFieldElement;

/// Enumeration that indicates whether the Jacobian is partial differentiation
/// with respect to q̇ (time-derivatives of generalized positions) or
/// with respect to v (generalized velocities).
enum class JacobianWrtVariable {
  kQDot,  ///< J = ∂V/∂q̇
  kV      ///< J = ∂V/∂v
};

/// @cond
// Helper macro to throw an exception within methods that should not be called
// post-finalize.
// This macro is constant-time and, per Drake's style guide, we allow to call
// it from within snake_case functions.
#define DRAKE_MBT_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
// This macro is constant-time and, per Drake's style guide, we allow to call
// it from within snake_case functions.
#define DRAKE_MBT_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

namespace internal {

template <typename T>
class BodyNode;
template <typename T>
class ModelInstance;
template <typename T>
class Mobilizer;
template <typename T>
class QuaternionFloatingMobilizer;

// %MultibodyTree provides a representation for a physical system consisting of
// a collection of interconnected rigid and deformable bodies. As such, it owns
// and manages each of the elements that belong to this physical system.
// Multibody dynamics elements include bodies, joints, force elements and
// constraints.
//
// @tparam_default_scalar
template <typename T>
class MultibodyTree {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTree);

  // Creates a MultibodyTree containing only a **world** body and a
  // UniformGravityFieldElement.
  MultibodyTree();

  ~MultibodyTree();

  // @name Methods to add new MultibodyTree elements.
  //
  // To create a %MultibodyTree users will add multibody elements like bodies,
  // joints, force elements, constraints, etc, using one of these methods.
  // Once a user is done adding multibody elements, the Finalize() method
  // **must** be called before invoking any %MultibodyTree method.
  // See Finalize() for details.
  // @{
  // TODO(amcastro-tri): add at least one example of a method that requires a
  // valid topology in this documentation.
  // See this Reviewable comment:
  // https://reviewable.io/reviews/robotlocomotion/drake/5583#-KgGqGisnX9uMuYDkHpx

  // Creates a rigid body with the provided name, model instance, and spatial
  // inertia.  This method returns a constant reference to the body just added,
  // which will remain valid for the lifetime of `this` %MultibodyTree.
  //
  // Example of usage:
  // @code
  //   MultibodyTree<T> model;
  //   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  //   ModelInstanceIndex model_instance = model.AddModelInstance("instance");
  //   const RigidBody<T>& body =
  //     model.AddRigidBody("BodyName", model_instance, spatial_inertia);
  // @endcode
  //
  // @param[in] name
  //   A string that identifies the new body to be added to `this` model. A
  //   std::runtime_error is thrown if a body named `name` already is part of
  //   @p model_instance. See HasBodyNamed(), RigidBody::name().
  // @param[in] model_instance
  //   A model instance index which this body is part of.
  // @param[in] M_BBo_B
  //   The SpatialInertia of the new rigid body to be added to `this` model,
  //   computed about the body frame origin `Bo` and expressed in the body
  //   frame B.
  // @returns A constant reference to the new RigidBody just added, which will
  //          remain valid for the lifetime of `this` %MultibodyTree.
  // @throws std::exception if a body named `name` already exists in this
  //         model instance.
  // @throws std::exception if the model instance does not exist.
  const RigidBody<T>& AddRigidBody(const std::string& name,
                                   ModelInstanceIndex model_instance,
                                   const SpatialInertia<double>& M_BBo_B);

  // Creates a rigid body with the provided name, model instance, and spatial
  // inertia.  The newly created body will be placed in the default model
  // instance.  This method returns a constant reference to the body just
  // added, which will remain valid for the lifetime of `this` %MultibodyTree.
  //
  // Example of usage:
  // @code
  //   MultibodyTree<T> model;
  //   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  //   const RigidBody<T>& body =
  //     model.AddRigidBody("BodyName", spatial_inertia);
  // @endcode
  //
  // @param[in] name
  //   A string that identifies the new body to be added to `this` model. A
  //   std::runtime_error is thrown if a body named `name` already is part of
  //   the model in the default model instance. See HasBodyNamed(),
  //   RigidBody::name().
  // @param[in] M_BBo_B
  //   The SpatialInertia of the new rigid body to be added to `this` model,
  //   computed about the body frame origin `Bo` and expressed in the body
  //   frame B.
  // @returns A constant reference to the new RigidBody just added, which will
  //          remain valid for the lifetime of `this` %MultibodyTree.
  // @throws std::exception if a body named `name` already exists.
  // @throws std::exception if additional model instances have been created
  //                        beyond the world and default instances.
  const RigidBody<T>& AddRigidBody(const std::string& name,
                                   const SpatialInertia<double>& M_BBo_B);

  // Takes ownership of `frame` and adds it to `this` %MultibodyTree. Returns
  // a constant reference to the frame just added, which will remain valid for
  // the lifetime of `this` %MultibodyTree.
  //
  // Example of usage:
  // @code
  //   MultibodyTree<T> model;
  //   // ... Define body and X_BF ...
  //   const FixedOffsetFrame<T>& frame =
  //       model.AddFrame(std::make_unique<FixedOffsetFrame<T>>(body, X_BF));
  // @endcode
  //
  // @throws std::exception if `frame` is a nullptr.
  // @throws std::exception if Finalize() was already called on `this` tree.
  //
  // @param[in] frame A unique pointer to a frame to be added to `this`
  //                  %MultibodyTree. The frame class must be specialized on
  //                  the same scalar type T as this %MultibodyTree.
  // @returns A constant reference of type `FrameType` to the created frame.
  //          This reference which will remain valid for the lifetime of `this`
  //          %MultibodyTree.
  //
  // @tparam FrameType The type of the specific sub-class of Frame to add. The
  //                   template needs to be specialized on the same scalar type
  //                   T of this %MultibodyTree.
  template <template <typename Scalar> class FrameType>
  const FrameType<T>& AddFrame(std::unique_ptr<FrameType<T>> frame);

  // Same as above but sets the `is_ephemeral` bit.
  template <template <typename Scalar> class FrameType>
  const FrameType<T>& AddEphemeralFrame(std::unique_ptr<FrameType<T>> frame);

  // Constructs a new frame with type `FrameType` with the given `args`, and
  // adds it to `this` %MultibodyTree, which retains ownership. The `FrameType`
  // will be specialized on the scalar type T of this %MultibodyTree.
  //
  // Example of usage:
  // @code
  //   MultibodyTree<T> model;
  //   // ... Define body and X_BF ...
  //   // Notice FixedOffsetFrame is a template an a scalar type.
  //   const FixedOffsetFrame<T>& frame =
  //       model.AddFrame<FixedOffsetFrame>(body, X_BF);
  // @endcode
  //
  // Note that for dependent names you must use the template keyword (say for
  // instance you have a MultibodyTree<T> member within your custom class):
  //
  // @code
  //   MultibodyTree<T> model;
  //   // ... Define body and X_BF ...
  //   const auto& frame =
  //       model.template AddFrame<FixedOffsetFrame>(body, X_BF);
  // @endcode
  //
  // @throws std::exception if Finalize() was already called on `this` tree.
  //
  // @param[in] args The arguments needed to construct a valid Frame of type
  //                 `FrameType`. `FrameType` must provide a public constructor
  //                 that takes these arguments.
  // @returns A constant reference of type `FrameType` to the created frame.
  //          This reference which will remain valid for the lifetime of `this`
  //          %MultibodyTree.
  //
  // @tparam FrameType A template for the type of Frame to construct. The
  //                   template will be specialized on the scalar type T of
  //                   this %MultibodyTree.
  template <template <typename Scalar> class FrameType, typename... Args>
  const FrameType<T>& AddFrame(Args&&... args);

  // Takes ownership of `mobilizer` and adds it to `this` %MultibodyTree.
  // Returns a constant reference to the mobilizer just added, which will
  // remain valid for the lifetime of `this` %MultibodyTree.
  //
  // Example of usage:
  // @code
  //   MultibodyTree<T> model;
  //   // ... Code to define inboard and outboard frames by calling
  //   // MultibodyTree::AddFrame() ...
  //   const RevoluteMobilizer<T>& pin =
  //     model.AddMobilizer(std::make_unique<RevoluteMobilizer<T>>(
  //       inboard_frame, outboard_frame,
  //       Vector3d::UnitZ() /*revolute axis*/));
  // @endcode
  //
  // A %Mobilizer effectively connects the two bodies to which the inboard and
  // outboard frames belong.
  //
  // @throws std::exception if `mobilizer` is a nullptr.
  // @throws std::exception if Finalize() was already called on `this` tree.
  // @throws std::exception if the new mobilizer attempts to connect a
  // frame with itself.
  // @throws std::exception if attempting to connect two bodies with more
  // than one mobilizer between them.
  //
  // @param[in] mobilizer A unique pointer to a mobilizer to add to `this`
  //                      %MultibodyTree. The mobilizer class must be
  //                      specialized on the same scalar type T as this
  //                      %MultibodyTree. Notice this is a requirement of this
  //                      method's signature and therefore an input mobilzer
  //                      specialized on a different scalar type than that of
  //                      this %MultibodyTree's T will fail to compile.
  // @returns A constant reference of type `MobilizerType` to the created
  //          mobilizer. This reference which will remain valid for the
  //          lifetime of `this` %MultibodyTree.
  //
  // @tparam MobilizerType The type of the specific sub-class of Mobilizer to
  //                       add. The template needs to be specialized on the
  //                       same scalar type T of this %MultibodyTree.
  template <template <typename Scalar> class MobilizerType>
  const MobilizerType<T>& AddMobilizer(
      std::unique_ptr<MobilizerType<T>> mobilizer);

  // Constructs a new mobilizer with type `MobilizerType` with the given
  // `args`, and adds it to `this` %MultibodyTree, which retains ownership.
  // The `MobilizerType` will be specialized on the scalar type T of this
  // %MultibodyTree.
  //
  // Example of usage:
  // @code
  //   MultibodyTree<T> model;
  //   // ... Code to define inboard and outboard frames by calling
  //   // MultibodyTree::AddFrame() ...
  //   // Notice RevoluteMobilizer is a template an a scalar type.
  //   const RevoluteMobilizer<T>& pin =
  //     model.template AddMobilizer<RevoluteMobilizer>(
  //       inboard_frame, outboard_frame,
  //       Vector3d::UnitZ() /*revolute axis*/);
  // @endcode
  //
  // Note that for dependent names _only_ you must use the template keyword
  // (say for instance you have a MultibodyTree<T> member within your custom
  // class).
  //
  // @throws std::exception if Finalize() was already called on `this` tree.
  // @throws std::exception if the new mobilizer attempts to connect a
  // frame with itself.
  // @throws std::exception if attempting to connect two bodies with more
  // than one mobilizer between them.
  //
  // @param[in] args The arguments needed to construct a valid Mobilizer of
  //                 type `MobilizerType`. `MobilizerType` must provide a
  //                 public constructor that takes these arguments.
  // @returns A constant reference of type `MobilizerType` to the created
  //          mobilizer. This reference which will remain valid for the
  //          lifetime of `this` %MultibodyTree.
  //
  // @tparam MobilizerType A template for the type of Mobilizer to construct.
  //                       The template will be specialized on the scalar type
  //                       T of `this` %MultibodyTree.
  template <template <typename Scalar> class MobilizerType, typename... Args>
  const MobilizerType<T>& AddMobilizer(Args&&... args);

  // Creates and adds to `this` %MultibodyTree (which retains ownership) a new
  // `ForceElement` member with the specific type `ForceElementType`. The
  // arguments to this method `args` are forwarded to `ForceElementType`'s
  // constructor.
  //
  // This method can only be called once for elements of type
  // UniformGravityFieldElement. That is, gravity can only be specified once
  // and std::runtime_error is thrown if the model already contains a gravity
  // field element.
  //
  // The newly created `ForceElementType` object will be specialized on the
  // scalar type T of this %MultibodyTree.
  template <template <typename Scalar> class ForceElementType>
  const ForceElementType<T>& AddForceElement(
      std::unique_ptr<ForceElementType<T>> force_element);

  // Helper function for AddForceElement that handles setting the gravity field.
  // If the `force_element` is a UniformGravityFieldElement it is processed as
  // such; if not, then this function is a no-op.
  void MaybeSetUniformGravityFieldElement(ForceElement<T>* force_element);

  // Adds a new force element model of type `ForceElementType` to `this`
  // %MultibodyTree.  The arguments to this method `args` are forwarded to
  // `ForceElementType`'s constructor.
  // @param[in] args
  //   Zero or more parameters provided to the constructor of the new force
  //   element. It must be the case that
  //   `JointType<T>(args)` is a valid constructor.
  // @tparam ForceElementType
  //   The type of the ForceElement to add.
  //   This method can only be called once for elements of type
  //   UniformGravityFieldElement. That is, gravity can only be specified once
  //   and std::runtime_error is thrown if the model already contains a gravity
  //   field element.
  // @returns A constant reference to the new ForceElement just added, of type
  //   `ForceElementType<T>` specialized on the scalar type T of `this`
  //   %MultibodyTree. It will remain valid for the lifetime of `this`
  //   %MultibodyTree.
  // @see The ForceElement class's documentation for further details on how a
  // force element is defined.
  // @throws std::exception if gravity was already added to the model.
  template <template <typename Scalar> class ForceElementType, typename... Args>
  const ForceElementType<T>& AddForceElement(Args&&... args);

  // See MultibodyPlant documentation. In addition internally we distinguish
  // user Joints from Joints added during modeling (called "ephemeral joints").
  template <template <typename Scalar> class JointType>
  const JointType<T>& AddJoint(std::unique_ptr<JointType<T>> joint,
                               bool is_ephemeral_joint = false);

  // This method adds a Joint of type `JointType` between two bodies.
  // The two bodies connected by this Joint object are referred to as _parent_
  // and _child_ bodies. The parent/child ordering defines the sign conventions
  // for the generalized coordinates and the coordinate ordering for multi-DOF
  // joints.  Our use of the terms _parent_ and _child_ does 𝐧𝐨𝐭 describe the
  // inboard/outboard relationship between bodies as our usage of inboard/
  // outboard is more general and is also meaningful for multibody systems
  // with loops, such as four-bar linkages.  However, when possible the
  // _parent_ body is made inboard and the _child_ outboard in the tree.
  //
  // As explained in the Joint class's documentation, in Drake we define a
  // frame Jp attached to the parent body P with pose `X_PJp` and a frame Jc
  // attached to the child body C with pose `X_CJc`. This method helps create
  // a joint between two bodies with fixed poses `X_PJp` and `X_CJc`.
  // Refer to the Joint class's documentation for more details. (We have
  // sometimes used F for Jp and M for Jc in documentation; don't confuse
  // those with the implementing Mobilizer's inboard F frame and outboard M
  // frame which in general are not the same.)
  //
  // The arguments to this method `args` are forwarded to `JointType`'s
  // constructor. The newly created `JointType` object will be specialized on
  // the scalar type T of this %MultibodyTree.
  //
  // @param[in] name
  //   The name of the joint.
  // @param[in] parent
  //   The parent body connected by the new joint.
  // @param[in] X_PJp
  //   The fixed pose of frame Jp attached to the parent body, measured in
  //   the frame P of that body. X_PJp is an optional parameter; empty curly
  //   braces {} imply that frame Jp **is** the same body frame P. If instead
  //   your intention is to make a separate frame Jp that is coincident
  //   (by default at least) with P, provide
  //   X_PJp = RigidTransform<double>::Identity() as your input.
  // @param[in] child
  //   The child body connected by the new joint.
  // @param[in] X_CJc
  //   The fixed pose of frame Jc attached to the child body, measured in
  //   the frame C of that body. X_CJc is an optional parameter; empty curly
  //   braces {} imply that frame Jc **is** the same body frame C. If instead
  //   your intention is to make a separate frame Jc that is coincident
  //   (by default at least) with C, provide
  //   X_CJc = RigidTransform<double>::Identity() as your input.
  // @tparam JointType
  //   The type of the new joint to add, which must be a subclass of Joint<T>.
  // @returns A const reference to the new joint just added, of type
  //   JointType<T> specialized on the scalar type T of `this`
  //   %MultibodyTree. It will remain valid for the lifetime of `this`
  //   %MultibodyTree.
  //
  // Example of usage:
  // @code
  //   MultibodyTree<T> model;
  //   // ... Code to define a parent body P and a child body C.
  //   const RigidBody<double>& parent_body =
  //     model.AddRigidBody(parent_name, SpatialInertia<double>(...));
  //   const RigidBody<double>& child_body =
  //     model.AddRigidBody(child_name, SpatialInertia<double>(...));
  //   // Define the pose X_CJc of a frame Jc rigidly attached to child body C.
  //   const RevoluteJoint<double>& elbow =
  //     model.AddJoint<RevoluteJoint>(
  //       "Elbow",                /* joint name */
  //       model.world_body(),     /* parent body */
  //       {},                     /* frame Jp IS the parent body frame P */
  //       pendulum,               /* child body, the pendulum */
  //       X_CJc,                  /* pose of frame Jc in child body frame C */
  //       Vector3d::UnitZ());     /* revolute axis in this case */
  // @endcode
  //
  // @throws std::exception if `this` model already contains a joint with the
  // given `name`.
  // See HasJointNamed(), Joint::name().
  //
  // @see The Joint class's documentation for further details on how a Joint
  // is defined.
  template <template <typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name, const RigidBody<T>& parent,
      const std::optional<math::RigidTransform<double>>& X_PJp,
      const RigidBody<T>& child,
      const std::optional<math::RigidTransform<double>>& X_CJc, Args&&... args);

  // See MultibodyPlant documentation.
  void RemoveJoint(const Joint<T>& joint);

  // Creates and adds a JointActuator model for an actuator acting on a given
  // `joint`. This method returns a const reference to the actuator just added,
  // which will remain valid for the lifetime of `this` %MultibodyTree.
  //
  // @param[in] name
  //   A string that identifies the new actuator to be added to `this`
  //   model. An exception is thrown if an actuator with the same name
  //   already exists in the same model instance as @p joint. See
  //   HasJointActuatorNamed().
  // @param[in] joint
  //   The Joint to be actuated by the new JointActuator.
  // @param[in] effort_limit
  //   The maximum effort for the actuator. It must be greater than 0. If
  //   the user does not set this value, the default value is +∞.
  // @returns A constant reference to the new JointActuator just added, which
  // will remain valid for the lifetime of `this` %MultibodyTree.
  // @throws std::exception if `this` model already contains a joint actuator
  // with the given `name`. See HasJointActuatorNamed(),
  // JointActuator::get_name().
  // TODO(amcastro-tri): consider adding sugar method to declare an actuated
  // joint with a single call. Maybe MBT::AddActuatedJoint() or the like.
  const JointActuator<T>& AddJointActuator(
      const std::string& name, const Joint<T>& joint,
      double effort_limit = std::numeric_limits<double>::infinity());

  // See MultibodyPlant documentation.
  void RemoveJointActuator(const JointActuator<T>& actuator);

  // Creates a new model instance.  Returns the index for a new model
  // instance (as there is no concrete object beyond the index).
  //
  // @param[in] name
  //   A string that uniquely identifies the new instance to be added to `this`
  //   model. An exception is thrown if an instance with the same name
  //   already exists in the model. See HasModelInstanceNamed().
  // @throws std::exception if Finalize() was already called on `this` tree.
  ModelInstanceIndex AddModelInstance(const std::string& name);

  // Registers a joint in the graph and also registers its joint type if we
  // haven't seen it before.
  void RegisterJointAndMaybeJointTypeInGraph(const Joint<T>& joint);

  // Renames an existing model instance.
  //
  // @param[in] model_instance
  //   The instance to rename.
  // @param[in] name
  //   A string that uniquely identifies the instance within `this`
  //   model.
  // @throws std::exception if Finalize() was already called on `this` tree.
  // @throws std::exception if `model_instance` is not a valid index.
  // @throws std::exception if HasModelInstanceNamed(`name`) is true.
  void RenameModelInstance(ModelInstanceIndex model_instance,
                           const std::string& name);

  // @}
  // Closes Doxygen section "Methods to add new MultibodyTree elements."

  // See MultibodyPlant method.
  int num_frames() const { return frames_.num_elements(); }

  // Returns the number of RigidBodies in the %MultibodyPlant including World.
  // Therefore the minimum number of bodies is one.
  int num_bodies() const { return rigid_bodies_.num_elements(); }

  // Returns the number of joints added with AddJoint() to the %MultibodyTree.
  int num_joints() const { return joints_.num_elements(); }

  // Returns the number of actuators in the model.
  // @see AddJointActuator().
  int num_actuators() const { return actuators_.num_elements(); }

  // After finalize, the number and ordering of mobilizers and body nodes
  // are identical (but they differ briefly _during_ finalize).
  int num_mobilizers() const { return ssize(mobilizers_); }
  int num_mobods() const { return ssize(body_nodes_); }

  // See MultibodyPlant method.
  int num_force_elements() const { return ssize(force_elements_); }

  // Returns the number of model instances in the MultibodyTree.
  int num_model_instances() const { return model_instances_.num_elements(); }

  // Returns the number of generalized positions of the model.
  int num_positions() const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return topology_.num_positions();
  }

  // Returns the number of generalized positions in a specific model instance.
  int num_positions(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.get_element(model_instance).num_positions();
  }

  // Returns the number of generalized velocities of the model.
  int num_velocities() const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return topology_.num_velocities();
  }

  // Returns the number of generalized velocities in a specific model instance.
  int num_velocities(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.get_element(model_instance).num_velocities();
  }

  // Returns the total size of the state vector in the model.
  int num_states() const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return topology_.num_states();
  }

  // Returns the total size of the state vector in a specific model instance.
  int num_states(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    const auto& model = model_instances_.get_element(model_instance);
    return model.num_positions() + model.num_velocities();
  }

  // See MultibodyPlant method.
  int num_actuated_dofs() const { return topology_.num_actuated_dofs(); }

  // See MultibodyPlant method.
  int num_actuators(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.get_element(model_instance).num_actuators();
  }

  // See MultibodyPlant method.
  int num_actuated_dofs(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.get_element(model_instance).num_actuated_dofs();
  }

  // Returns the height of the Forest data structure of `this` %MultibodyTree.
  // That is, the number of bodies in the longest kinematic path between the
  // world and any leaf body. For a model that only contains World, the height
  // of the forest is one.
  // Kinematic paths are created by Mobilizer objects connecting a chain of
  // frames. Therefore, this method does not count kinematic cycles, which
  // could only be considered in the model using constraints.
  int forest_height() const { return topology_.forest_height(); }

  // Returns a constant reference to the *world* body.
  const RigidBody<T>& world_body() const {
    // world_rigid_body_ is set in the constructor. So this assert is here only
    // to verify future constructors do not mess that up.
    DRAKE_ASSERT(world_rigid_body_ != nullptr);
    return *world_rigid_body_;
  }

  // Returns a constant reference to the *world* frame.
  const RigidBodyFrame<T>& world_frame() const {
    return rigid_bodies_.get_element_unchecked(world_index()).body_frame();
  }

  // See MultibodyPlant method.
  bool has_body(BodyIndex body_index) const {
    return rigid_bodies_.has_element(body_index);
  }

  // See MultibodyPlant method.
  const RigidBody<T>& get_body(BodyIndex body_index) const {
    return rigid_bodies_.get_element(body_index);
  }

  RigidBody<T>& get_mutable_body(BodyIndex body_index) {
    return rigid_bodies_.get_mutable_element(body_index);
  }

  // See MultibodyPlant method.
  bool has_joint(JointIndex joint_index) const {
    return joints_.has_element(joint_index);
  }

  // See MultibodyPlant method.
  const Joint<T>& get_joint(JointIndex joint_index) const {
    return joints_.get_element(joint_index);
  }

  // See MultibodyPlant method.
  Joint<T>& get_mutable_joint(JointIndex joint_index) {
    return joints_.get_mutable_element(joint_index);
  }

  // See MultibodyPlant method.
  bool has_joint_actuator(JointActuatorIndex actuator_index) const {
    return actuators_.has_element(actuator_index);
  }

  // See MultibodyPlant method.
  const JointActuator<T>& get_joint_actuator(
      JointActuatorIndex actuator_index) const {
    return actuators_.get_element(actuator_index);
  }

  // See MultibodyPlant method.
  JointActuator<T>& get_mutable_joint_actuator(
      JointActuatorIndex actuator_index) {
    return actuators_.get_mutable_element(actuator_index);
  }

  // See MultibodyPlant method.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    return frames_.get_element(frame_index);
  }

  Frame<T>& get_mutable_frame(FrameIndex frame_index) {
    return frames_.get_mutable_element(frame_index);
  }

  // See MultibodyPlant method.
  const Mobilizer<T>& get_mobilizer(MobodIndex mobilizer_index) const {
    DRAKE_THROW_UNLESS(mobilizer_index < num_mobilizers());
    return *mobilizers_[mobilizer_index];
  }

  Mobilizer<T>& get_mutable_mobilizer(MobodIndex mobilizer_index) {
    DRAKE_THROW_UNLESS(mobilizer_index < num_mobilizers());
    return *mobilizers_[mobilizer_index];
  }

  // See MultibodyPlant method.
  template <template <typename> class ForceElementType = ForceElement>
  const ForceElementType<T>& GetForceElement(
      ForceElementIndex force_element_index) const {
    static_assert(
        std::is_base_of_v<ForceElement<T>, ForceElementType<T>>,
        "ForceElementType<T> must be a sub-class of ForceElement<T>.");
    const ForceElement<T>* force_element =
        &get_force_element(force_element_index);

    const ForceElementType<T>* typed_force_element =
        dynamic_cast<const ForceElementType<T>*>(force_element);
    if (typed_force_element == nullptr) {
      throw std::logic_error(
          fmt::format("ForceElement is not of type '{}' but of type '{}'.",
                      NiceTypeName::Get<ForceElementType<T>>(),
                      NiceTypeName::Get(*force_element)));
    }

    return *typed_force_element;
  }

  const ForceElement<T>& get_force_element(
      ForceElementIndex force_element_index) const {
    DRAKE_THROW_UNLESS(force_element_index < num_force_elements());
    return *force_elements_[force_element_index];
  }

  ForceElement<T>& get_mutable_force_element(
      ForceElementIndex force_element_index) {
    DRAKE_THROW_UNLESS(force_element_index < num_force_elements());
    return *force_elements_[force_element_index];
  }

  // An accessor to the current gravity field.
  const UniformGravityFieldElement<T>& gravity_field() const {
    DRAKE_ASSERT(gravity_field_ != nullptr);
    return *gravity_field_;
  }

  // A mutable accessor to the current gravity field.
  UniformGravityFieldElement<T>& mutable_gravity_field() {
    DRAKE_ASSERT(gravity_field_ != nullptr);
    return *gravity_field_;
  }

  // See MultibodyPlant method.
  const std::string& GetModelInstanceName(
      ModelInstanceIndex model_instance) const;

  // Implements MultibodyPlant::HasUniqueFreeBaseBody.
  bool HasUniqueFreeBaseBodyImpl(ModelInstanceIndex model_instance) const;

  // Implements MultibodyPlant::GetUniqueFreeBaseBodyOrThrow.
  const RigidBody<T>& GetUniqueFreeBaseBodyOrThrowImpl(
      ModelInstanceIndex model_instance) const;

  // @name Querying for multibody elements by name
  // These methods allow a user to query whether a given multibody element is
  // part of `this` model. These queries can be performed at any time during
  // the lifetime of a %MultibodyTree model, i.e. there is no restriction on
  // whether they must be called before or after Finalize(). That is, these
  // queries can be performed while new multibody elements are being added to
  // the model.
  // @{

  // See MultibodyPlant method.
  int NumBodiesWithName(std::string_view name) const;

  // @returns `true` if a body named `name` was added to the model.
  // @see AddRigidBody().
  //
  // @throws std::exception if the body name occurs in multiple model
  // instances.
  bool HasBodyNamed(std::string_view name) const;

  // @returns `true` if a body named `name` was added to @p model_instance.
  // @see AddRigidBody().
  //
  // @throws std::exception if @p model_instance is not valid for this model.
  bool HasBodyNamed(std::string_view name,
                    ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  bool HasFrameNamed(std::string_view name) const;

  // See MultibodyPlant method.
  bool HasFrameNamed(std::string_view name,
                     ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  bool HasJointNamed(std::string_view name) const;

  // See MultibodyPlant method.
  bool HasJointNamed(std::string_view name,
                     ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  bool HasJointActuatorNamed(std::string_view name) const;

  // See MultibodyPlant method.
  bool HasJointActuatorNamed(std::string_view name,
                             ModelInstanceIndex model_instance) const;

  // See MultibodyMethod.
  bool HasModelInstanceNamed(std::string_view name) const;
  // @}

  // Returns a list of body indices associated with `model_instance`.
  std::vector<BodyIndex> GetBodyIndices(
      ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  const std::vector<JointIndex>& GetJointIndices() const {
    return joints_.indices();
  }

  // Returns a list of joint indices associated with `model_instance`.
  std::vector<JointIndex> GetJointIndices(
      ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  const std::vector<JointActuatorIndex>& GetJointActuatorIndices() const {
    return actuators_.indices();
  }

  // See MultibodyPlant method.
  std::vector<JointActuatorIndex> GetJointActuatorIndices(
      ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  std::vector<JointIndex> GetActuatedJointIndices(
      ModelInstanceIndex model_instance) const;

  // Returns a list of frame indices associated with `model_instance`
  std::vector<FrameIndex> GetFrameIndices(
      ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  const Frame<T>& GetFrameByName(std::string_view name) const;

  // See MultibodyPlant method.
  const Frame<T>& GetFrameByName(std::string_view name,
                                 ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  const RigidBody<T>& GetRigidBodyByName(std::string_view name) const;

  // See MultibodyPlant method.
  const RigidBody<T>& GetRigidBodyByName(
      std::string_view name, ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  template <template <typename> class JointType = Joint>
  const JointType<T>& GetJointByName(
      std::string_view name,
      std::optional<ModelInstanceIndex> model_instance = std::nullopt) const {
    static_assert(std::is_base_of_v<Joint<T>, JointType<T>>,
                  "JointType<T> must be a sub-class of Joint<T>.");
    const Joint<T>& joint = GetJointByNameImpl(name, model_instance);
    const JointType<T>* const typed_joint =
        dynamic_cast<const JointType<T>*>(&joint);
    if (typed_joint == nullptr) {
      ThrowJointSubtypeMismatch(joint, NiceTypeName::Get<JointType<T>>());
    }
    return *typed_joint;
  }

  // See MultibodyPlant method.
  template <template <typename> class JointType = Joint>
  JointType<T>& GetMutableJointByName(
      std::string_view name,
      std::optional<ModelInstanceIndex> model_instance = std::nullopt) {
    const JointType<T>& const_joint =
        GetJointByName<JointType>(name, model_instance);

    // Note: Using the const method to implement this non-const one
    // relies on the fact (true today) that no lower-level MultibodyTree code
    // needs to know we're obtaining mutable access here. For example,
    // this wouldn't work if a stored computation needed to be invalidated.
    return const_cast<JointType<T>&>(const_joint);
  }

  // See MultibodyPlant method.
  const JointActuator<T>& GetJointActuatorByName(std::string_view name) const;

  // See MultibodyPlant method.
  const JointActuator<T>& GetJointActuatorByName(
      std::string_view name, ModelInstanceIndex model_instance) const;

  // See MultibodyPlant method.
  ModelInstanceIndex GetModelInstanceByName(std::string_view name) const;
  // @}

  // Returns `true` if this %MultibodyTree was finalized with Finalize() after
  // all multibody elements were added, and `false` otherwise.
  // When a %MultibodyTree is instantiated, its topology remains invalid until
  // Finalize() is called, which validates the topology.
  // @see Finalize().
  bool topology_is_valid() const { return topology_.is_valid(); }

  // Returns the topology information for this multibody tree. Users should not
  // need to call this method since MultibodyTreeTopology is an internal
  // bookkeeping detail. Used at Finalize() stage by multibody elements to
  // retrieve a local copy of their topology.
  const MultibodyTreeTopology& get_topology() const { return topology_; }

  // Returns the set of RigidBodies that are affected kinematically by the given
  // Joints' degrees of freedom. Weld joints are ignored. Otherwise this is just
  // the set of Links in the subtrees rooted by these Joints' implementing
  // Mobods.
  std::set<BodyIndex> GetBodiesKinematicallyAffectedBy(
      const std::vector<JointIndex>& joint_indexes) const;

  // Returns the set of RigidBodies that are on the same Mobod or outboard of
  // the given bodies. This is just the set of rigid bodies in the subtrees
  // rooted by these bodies' implementing Mobods. The given bodies are always
  // included.
  std::set<BodyIndex> GetBodiesOutboardOfBodies(
      const std::vector<BodyIndex>& body_indexes) const;

  // Returns the mobilizer model for joint with index `joint_index`. The index
  // is invalid if the joint is not modeled with a mobilizer.
  MobodIndex get_joint_mobilizer(JointIndex joint_index) const {
    DRAKE_DEMAND(has_joint(joint_index));
    return joint_to_mobilizer_.at(joint_index);
  }

  // @name Model instance accessors
  // Many functions on %MultibodyTree expect vectors of tree state or
  // joint actuator inputs which encompass the entire tree.  Methods
  // in this section are convenience accessors for the portion of
  // those vectors which apply to a single model instance only.
  // @{

  // See MultibodyPlant method.
  VectorX<T> GetActuationFromArray(ModelInstanceIndex model_instance,
                                   const Eigen::Ref<const VectorX<T>>& u) const;

  // See MultibodyPlant method.
  void SetActuationInArray(ModelInstanceIndex model_instance,
                           const Eigen::Ref<const VectorX<T>>& u_instance,
                           EigenPtr<VectorX<T>> u) const;

  // See MultibodyPlant method.
  VectorX<T> GetPositionsFromArray(ModelInstanceIndex model_instance,
                                   const Eigen::Ref<const VectorX<T>>& q) const;

  // See MultibodyPlant method.
  void GetPositionsFromArray(ModelInstanceIndex model_instance,
                             const Eigen::Ref<const VectorX<T>>& q,
                             EigenPtr<VectorX<T>> q_out) const;

  // See MultibodyPlant method.
  void SetPositionsInArray(ModelInstanceIndex model_instance,
                           const Eigen::Ref<const VectorX<T>>& q_instance,
                           EigenPtr<VectorX<T>> q) const;

  // See MultibodyPlant method.
  VectorX<T> GetVelocitiesFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v) const;

  // See MultibodyPlant method.
  void GetVelocitiesFromArray(ModelInstanceIndex model_instance,
                              const Eigen::Ref<const VectorX<T>>& v,
                              EigenPtr<VectorX<T>> v_out) const;

  // Sets the vector of generalized velocities for `model_instance` in
  // `v` using `v_instance`, leaving all other elements in the array
  // untouched. This method throws an exception if `v` is not of size
  // MultibodyTree::num_velocities() or `v_instance` is not of size
  // `MultibodyTree::num_positions(model_instance)`.
  void SetVelocitiesInArray(ModelInstanceIndex model_instance,
                            const Eigen::Ref<const VectorX<T>>& v_instance,
                            EigenPtr<VectorX<T>> v) const;

  // @}
  // End of "Model instance accessors" section.

  // MultibodyPlant invokes this to construct a spanning forest/loop constraint
  // model we want to use to simulate the user's Link and Joint structure.
  const SpanningForest& BuildSpanningForest() {
    link_joint_graph_.BuildForest();
    return link_joint_graph_.forest();
  }

  [[nodiscard]] const LinkJointGraph& graph() const {
    return link_joint_graph_;
  }

  [[nodiscard]] LinkJointGraph& mutable_graph() { return link_joint_graph_; }

  [[nodiscard]] const SpanningForest& forest() const {
    DRAKE_ASSERT(graph().forest_is_valid());
    return graph().forest();
  }

  [[nodiscard]] const SpanningForest::Mobod& get_mobod(MobodIndex index) const {
    return forest().mobods(index);
  }

  // Finalize() must be called after all user-defined elements in the plant
  // (joints, bodies, force elements, constraints, etc.) have been added and
  // before any computations are performed. It compiles all the necessary
  // topological information, i.e. how bodies, mobilizers, and any other
  // elements connect with each other, and performs all the required
  // pre-processing to permit efficient computations at a later stage. During
  // this process, we may add additional elements (e.g. joints, frames,
  // and constraints) to facilitate computation; we call those "ephemeral"
  // elements and mark them as such in the base MultibodyElement class to
  // distinguish them from user-defined elements.
  //
  // If the Finalize operation is successful, the topology of this MultibodyTree
  // is validated, meaning that the topology is up-to-date after this call.
  // No more multibody plant elements can be added after a call to Finalize().
  //
  // @throws std::exception if called post-finalize.
  // TODO(amcastro-tri): Consider making this method private and calling it
  //  automatically when CreateDefaultContext() is called.
  void Finalize();

  // (Advanced) Allocates a new context for this %MultibodyTree uniquely
  // identifying the state of the multibody system.
  //
  // @throws std::exception if this is not owned by a MultibodyPlant /
  // MultibodyTreeSystem.
  std::unique_ptr<systems::LeafContext<T>> CreateDefaultContext() const;

  // See MultibodyPlant method.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const;

  // See MultibodyPlant method.
  void SetRandomState(const systems::Context<T>& context,
                      systems::State<T>* state,
                      RandomGenerator* generator) const;

  // Returns a const Eigen vector reference containing the vector
  // `[q; v]` of the model with `q` the vector of generalized positions and
  // `v` the vector of generalized velocities.
  // @note This method returns a reference to existing data, exhibits constant
  //       i.e., O(1) time complexity, and runs very quickly.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<const VectorX<T>> get_positions_and_velocities(
      const systems::Context<T>& context) const;

  // Returns a Eigen vector containing the multibody state `x = [q; v]`
  // of the model with `q` the vector of generalized positions and `v` the
  // vector of generalized velocities for model instance `model_instance`.
  // @throws std::exception if the `context` does not correspond to the context
  // for a multibody model or `model_instance` is invalid.
  // @note returns a dense vector of dimension `q.size() + v.size()` associated
  //          with `model_instance` in O(`q.size()`) time.
  // @pre `context` is a valid multibody system Context.
  VectorX<T> GetPositionsAndVelocities(const systems::Context<T>& context,
                                       ModelInstanceIndex model_instance) const;

  // Takes output vector qv_out and populates it with the multibody
  // state `x = [q; v]` of the model with `q` the vector of generalized
  // positions and `v` the vector of generalized velocities for model instance
  // `model_instance`.
  // @throws std::exception if the `context` does not correspond to the context
  // for a multibody model or `model_instance` is invalid.
  // @throws std::exception if the size of `qv_out` is not equal to
  //         'num_postions(model_instance)' + 'num_velocities(model_instance)'
  // @pre `context` is a valid multibody system Context.
  void GetPositionsAndVelocities(const systems::Context<T>& context,
                                 ModelInstanceIndex model_instance,
                                 EigenPtr<VectorX<T>> qv_out) const;

  // From a mutable State, returns a mutable Eigen vector containing the vector
  // `[q; v]` of the model with `q` the vector of generalized positions and `v`
  // the vector of generalized velocities.
  // @note This method returns a reference to existing data, exhibits constant
  //       i.e., O(1) time complexity, and runs very quickly. It does not cause
  //       cache invalidation so be careful!
  // @pre `state` is a valid multibody system State.
  Eigen::VectorBlock<VectorX<T>> get_mutable_positions_and_velocities(
      systems::State<T>* state) const;

  // This is a mutable-Context version of
  // `get_mutable_positions_and_velocities()`; see above.
  // @note Invalidates all q- or v-dependent cache entries.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositionsAndVelocities(
      systems::Context<T>* context) const;

  // Sets `context` to store the vector `[q; v]`
  // with `q` the vector of generalized positions and `v` the vector
  // of generalized velocities for model instance `model_instance`.
  // @throws std::exception if the `context` does not correspond to the context
  // for a multibody model, `context` is nullptr, `model_instance` is invalid,
  // or `instance_state.size()` does not equal `num_positions(model_instance)`
  // + `num_velocities(model_instance)`.
  // @pre `context` is a valid multibody system Context.
  void SetPositionsAndVelocities(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& instance_state,
      systems::Context<T>* context) const;

  // See MultibodyPlant::GetFreeBodyPose.
  math::RigidTransform<T> GetFreeBodyPoseOrThrow(
      const systems::Context<T>& context, const RigidBody<T>& body) const;

  // See MultibodyPlant::SetDefaultFreeBodyPose.
  void SetDefaultFreeBodyPose(const RigidBody<T>& body,
                              const math::RigidTransform<double>& X_WB);

  // See MultibodyPlant::GetDefaultFreeBodyPose.
  math::RigidTransform<double> GetDefaultFreeBodyPose(
      const RigidBody<T>& body) const;

  // See MultibodyPlant::SetFreeBodyPose.
  void SetFreeBodyPoseOrThrow(const RigidBody<T>& body,
                              const math::RigidTransform<T>& X_WB,
                              systems::Context<T>* context) const;

  // See MultibodyPlant::SetFreeBodySpatialVelocity.
  void SetFreeBodySpatialVelocityOrThrow(const RigidBody<T>& body,
                                         const SpatialVelocity<T>& V_WB,
                                         systems::Context<T>* context) const;

  // See MultibodyPlant::SetFreeBodyPose.
  void SetFreeBodyPoseOrThrow(const RigidBody<T>& body,
                              const math::RigidTransform<T>& X_WB,
                              const systems::Context<T>& context,
                              systems::State<T>* state) const;

  // See MultibodyPlant::SetFreeBodySpatialVelocity.
  void SetFreeBodySpatialVelocityOrThrow(const RigidBody<T>& body,
                                         const SpatialVelocity<T>& V_WB,
                                         const systems::Context<T>& context,
                                         systems::State<T>* state) const;

  // See MultibodyPlant::SetFreeBodyRandomTranslationDistribution.
  void SetFreeBodyRandomTranslationDistributionOrThrow(
      const RigidBody<T>& body,
      const Vector3<symbolic::Expression>& translation);

  // See MultibodyPlant::SetFreeBodyRandomRotationDistribution.
  void SetFreeBodyRandomRotationDistributionOrThrow(
      const RigidBody<T>& body,
      const Eigen::Quaternion<symbolic::Expression>& rotation);

  // See MultibodyPlant::SetFreeBodyRandomRotationDistribution.
  void SetFreeBodyRandomAnglesDistributionOrThrow(
      const RigidBody<T>& body,
      const math::RollPitchYaw<symbolic::Expression>& angles);

  // @name Kinematic computations
  // Kinematics computations are concerned with the motion of bodies in the
  // model without regard to their mass or the forces and moments that cause
  // the motion. Methods in this category include the computation of poses and
  // spatial velocities.
  // @{

  // See MultibodyPlant method.
  void CalcAllBodyPosesInWorld(
      const systems::Context<T>& context,
      std::vector<math::RigidTransform<T>>* X_WB) const;

  // See MultibodyPlant method.
  void CalcAllBodySpatialVelocitiesInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialVelocity<T>>* V_WB) const;

  // See MultibodyPlant method.
  math::RigidTransform<T> CalcRelativeTransform(
      const systems::Context<T>& context, const Frame<T>& frame_F,
      const Frame<T>& frame_G) const;

  // See MultibodyPlant method.
  math::RotationMatrix<T> CalcRelativeRotationMatrix(
      const systems::Context<T>& context, const Frame<T>& frame_F,
      const Frame<T>& frame_G) const;

  // See MultibodyPlant method.
  void CalcPointsPositions(const systems::Context<T>& context,
                           const Frame<T>& frame_B,
                           const Eigen::Ref<const MatrixX<T>>& p_BQi,
                           const Frame<T>& frame_A,
                           EigenPtr<MatrixX<T>> p_AQi) const;

  // See MultibodyPlant method.
  T CalcTotalMass(const systems::Context<T>& context) const;

  // See MultibodyPlant method.
  T CalcTotalMass(const systems::Context<T>& context,
                  const std::vector<ModelInstanceIndex>& model_instances) const;

  // See MultibodyPlant method.
  Vector3<T> CalcCenterOfMassPositionInWorld(
      const systems::Context<T>& context) const;

  // See MultibodyPlant method.
  Vector3<T> CalcCenterOfMassPositionInWorld(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const;

  // See MultibodyPlant method.
  SpatialInertia<T> CalcSpatialInertia(
      const systems::Context<T>& context, const Frame<T>& frame_F,
      const std::vector<BodyIndex>& body_indexes) const;

  // See MultibodyPlant method.
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context) const;

  // See MultibodyPlant method.
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const;

  // See MultibodyPlant method.
  Vector3<T> CalcCenterOfMassTranslationalAccelerationInWorld(
      const systems::Context<T>& context) const;

  // See MultibodyPlant method.
  Vector3<T> CalcCenterOfMassTranslationalAccelerationInWorld(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const;

  // See MultibodyPlant method.
  SpatialMomentum<T> CalcSpatialMomentumInWorldAboutPoint(
      const systems::Context<T>& context, const Vector3<T>& p_WoP_W) const;

  // See MultibodyPlant method.
  SpatialMomentum<T> CalcSpatialMomentumInWorldAboutPoint(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances,
      const Vector3<T>& p_WoP_W) const;

  // See MultibodyPlant method.
  const math::RigidTransform<T>& EvalBodyPoseInWorld(
      const systems::Context<T>& context, const RigidBody<T>& body_B) const;

  // See MultibodyPlantMethod.
  const SpatialVelocity<T>& EvalBodySpatialVelocityInWorld(
      const systems::Context<T>& context, const RigidBody<T>& body_B) const;

  // See MultibodyPlantMethod.
  const SpatialAcceleration<T>& EvalBodySpatialAccelerationInWorld(
      const systems::Context<T>& context, const RigidBody<T>& body_B) const;

  // @}
  // End of "Kinematic computations" section.

  // @name Methods to compute multibody Jacobians.
  // @{

  // See MultibodyPlant method.
  void CalcJacobianSpatialVelocity(const systems::Context<T>& context,
                                   JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Eigen::Ref<const Vector3<T>>& p_BP,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<MatrixX<T>> Js_V_ABp_E) const;

  // See MultibodyPlant method.
  void CalcJacobianAngularVelocity(const systems::Context<T>& context,
                                   JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<Matrix3X<T>> Js_w_AB_E) const;

  // Return a point's translational velocity Jacobian in a frame A with respect
  // to "speeds" 𝑠, where 𝑠 is either q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of
  // j generalized positions) or v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).
  // For each point Bi of (fixed to) a frame B whose translational velocity
  // `v_ABi` in a frame A is characterized by speeds 𝑠, Bi's translational
  // velocity Jacobian in A with respect to 𝑠 is defined as
  // <pre>
  //      Js_v_ABi = [ ∂(v_ABi)/∂𝑠₁,  ...  ∂(v_ABi)/∂𝑠ₙ ]    (n is j or k)
  // </pre>
  // Point Bi's velocity in A is linear in 𝑠₁, ... 𝑠ₙ and can be written
  // `v_ABi = Js_v_ABi ⋅ 𝑠`  where 𝑠 is [𝑠₁ ... 𝑠ₙ]ᵀ.
  //
  // @param[in] context The state of the multibody system.
  // @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  // JacobianWrtVariable::kV, indicating whether the Jacobian `Js_v_ABi` is
  // partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  // positions) or with respect to 𝑠 = v (generalized velocities).
  // @param[in] frame_B The frame on which point Bi is fixed (e.g., welded).
  // @param[in] frame_F The frame associated with `p_FoBi_F` (next argument),
  // which is usually (but is not necessarily) frame_B or the world frame W.
  // @param[in] p_FoBi_F A position vector or list of position vectors from
  // Fo (frame_F's origin) to points Bi (regarded as fixed to B), where each
  // position vector is expressed in frame F.
  // @param[in] frame_A The frame that measures `v_ABi` (Bi's velocity in A).
  // @param[in] frame_E The frame in which `v_ABi` is expressed on input and
  // the frame in which the Jacobian `Js_v_ABi` is expressed on output.
  // @param[out] Js_v_ABi_E Point Bi's velocity Jacobian in frame A with
  // respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  // `Js_v_ABi_E` is a `3 x n` matrix, where n is the number of elements in 𝑠.
  // The Jacobian is a function of only generalized positions q (which are
  // pulled from the context).
  // @throws std::exception if `Js_v_ABi_E` is nullptr or not of size `3 x n`.
  //
  // Note: This method is more general than the corresponding MultibodyPlant
  // method as it also contains the argument `frame_F`.
  void CalcJacobianTranslationalVelocity(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Frame<T>& frame_F,
      const Eigen::Ref<const Matrix3X<T>>& p_FoBi_F, const Frame<T>& frame_A,
      const Frame<T>& frame_E, EigenPtr<MatrixX<T>> Js_v_ABi_E) const;

  // See MultibodyPlant method.
  void CalcJacobianCenterOfMassTranslationalVelocity(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<Matrix3X<T>> Js_v_AScm_E) const;

  // See MultibodyPlant method.
  void CalcJacobianCenterOfMassTranslationalVelocity(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances,
      JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
      const Frame<T>& frame_E, EigenPtr<Matrix3X<T>> Js_v_AScm_E) const;

  // See MultibodyPlant method.
  Vector3<T> CalcBiasCenterOfMassTranslationalAcceleration(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_A, const Frame<T>& frame_E) const;

  // See MultibodyPlant method.
  Vector3<T> CalcBiasCenterOfMassTranslationalAcceleration(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances,
      JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
      const Frame<T>& frame_E) const;

  // See MultibodyPlant method.
  Matrix3X<T> CalcBiasTranslationalAcceleration(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
      const Frame<T>& frame_A, const Frame<T>& frame_E) const;

  // See MultibodyPlant method.
  SpatialAcceleration<T> CalcBiasSpatialAcceleration(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
      const Frame<T>& frame_A, const Frame<T>& frame_E) const;
  // @}
  // End of multibody Jacobian methods section.

  // @name Computational methods
  // These methods expose the computational capabilities of MultibodyTree to
  // compute kinematics, forward and inverse dynamics, and Jacobian matrices,
  // among others.
  // These methods follow Drake's naming scheme for methods performing a
  // computation and therefore are named `CalcFoo()`, where `Foo` corresponds
  // to the quantity or object of interest to be computed. They all take a
  // `systems::Context` as an input argument storing the state of the multibody
  // system.
  // @{

  // Computes into the position kinematics `pc` all the kinematic quantities
  // that depend on the generalized positions only. These include:
  //
  // - For each body B, the pose `X_BF` of each of the frames F attached to
  //   body B.
  // - Pose `X_WB` of each body B in the model as measured and expressed in
  //   the world frame W.
  // - Across-mobilizer and across-node hinge matrices `H_FM` and `H_PB_W`.
  // - Body specific quantities such as `com_W` and `M_Bo_W`.
  //
  // Aborts if `pc` is nullptr.
  void CalcPositionKinematicsCache(const systems::Context<T>& context,
                                   PositionKinematicsCache<T>* pc) const;

  // Computes the per-Tree block structured, World-frame System Jacobian
  // Jv_V_WB.
  void CalcBlockSystemJacobianCache(const systems::Context<T>& context,
                                    BlockSystemJacobianCache<T>* sjc) const;

  // Computes all the kinematic quantities that depend on the generalized
  // velocities and stores them in the velocity kinematics cache `vc`.
  // These include:
  // - Spatial velocity `V_WB` for each body B in the model as measured and
  //   expressed in the world frame W.
  // - Spatial velocity `V_PB` for each body B in the model as measured and
  //   expressed in the inboard (or parent) body frame P.
  //
  // @pre The position kinematics `pc` must have been previously updated with a
  // call to CalcPositionKinematicsCache().
  //
  // Aborts if `vc` is nullptr.
  void CalcVelocityKinematicsCache(const systems::Context<T>& context,
                                   const PositionKinematicsCache<T>& pc,
                                   VelocityKinematicsCache<T>* vc) const;

  // Computes the spatial inertia M_B_W(q) for each body B in the model about
  // its frame origin Bo and expressed in the world frame W.
  // @param[in] context
  //   The context storing the state of the model.
  // @param[out] M_B_W_all
  //   For each body in the model, entry RigidBody::mobod_index() in M_B_W_all
  //   contains the updated spatial inertia `M_B_W(q)` for that body. On input
  //   it must be a valid pointer to a vector of size num_bodies().
  // @throws std::exception if M_B_W_all is nullptr or if its size is not
  // num_bodies().
  void CalcSpatialInertiasInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialInertia<T>>* M_B_W_all) const;

  // Computes the reflected inertia Irefl for each velocity index.
  // @param[in] context
  //   The context storing the state of the model.
  // @param[out] reflected_inertia
  //   For each degree of freedom, reflected_inertia[i] contains the reflected
  //   inertia value for the i-th degree of freedom.
  // @throws std::exception if reflected_inertia is nullptr or if its size is
  // not num_velocities().
  void CalcReflectedInertia(const systems::Context<T>& context,
                            VectorX<T>* reflected_inertia) const;

  // Computes the joint damping for each velocity index.
  // @param[in] context
  //  The context storing the state of the model.
  // @param[out] joint_damping
  //  For each degree of freedom, joint_damping[i] contains the viscous damping
  //  coefficient for the i-th degree of freedom.
  // @throws std::exception if joint_damping is nullptr or if its size is not
  // num_velocities().
  void CalcJointDamping(const systems::Context<T>& context,
                        VectorX<T>* joint_damping) const;

  void CalcFrameBodyPoses(const systems::Context<T>& context,
                          FrameBodyPoseCache<T>* frame_body_poses) const;

  // Computes the composite body inertia K_BBo_W(q) for each body B in the
  // model about its frame origin Bo and expressed in the world frame W.
  // The composite body inertia is the effective mass properties B would have
  // if every joint outboard of B was welded in its current configuration.
  // @param[in] context
  //   The context storing the state of the model.
  // @param[out] K_BBo_W_all
  //   For each body B in the model, entry RigidBody::mobod_index() in
  //   M_BBo_W_all contains the updated composite body inertia K_BBo_W(q) for
  //   that body. On input it must be a valid pointer to a vector of size
  //   num_mobods().
  // @throws std::exception if K_BBo_W_all is nullptr or if its size is not
  //   num_mobods().
  void CalcCompositeBodyInertiasInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialInertia<T>>* K_BBo_W_all) const;

  // Computes the bias force `Fb_Bo_W(q, v)` for each body in the model.
  // For a body B, this is the bias term `Fb_Bo_W` in the equation
  // `F_BBo_W = M_Bo_W * A_WB + Fb_Bo_W`, where `M_Bo_W` is the spatial inertia
  // about B's origin Bo, `A_WB` is the spatial acceleration of B in W and
  // `F_BBo_W` is the spatial force applied on B about Bo, expressed in W.
  // @param[in] context
  //   The context storing the state of the model.
  // @param[out] Fb_Bo_W_all
  //   For each body in the model, entry RigidBody::mobod_index() in Fb_Bo_W_all
  //   contains the updated bias term `Fb_Bo_W(q, v)` for that body. On input
  //   it must be a valid pointer to a vector of size num_bodies().
  // @throws std::exception if Fb_Bo_W_cache is nullptr or if its size is not
  // num_bodies().
  void CalcDynamicBiasForces(const systems::Context<T>& context,
                             std::vector<SpatialForce<T>>* Fb_Bo_W_all) const;

  // Computes all the kinematic quantities that depend on the generalized
  // accelerations that is, the generalized velocities' time derivatives, and
  // stores them in the acceleration kinematics cache `ac`.
  // These include:
  // - Spatial acceleration `A_WB` for each body B in the model as measured and
  //   expressed in the world frame W.
  //
  // @param[in] context
  //   The context containing the state of the %MultibodyTree model.
  // @param[in] pc
  //   A position kinematics cache object already updated to be in sync with
  //   `context`.
  // @param[in] vc
  //   A velocity kinematics cache object already updated to be in sync with
  //   `context`.
  // @param[in] known_vdot
  //   A vector with the generalized accelerations for the full %MultibodyTree
  //   model.
  // @param[out] ac
  //   A pointer to a valid, non nullptr, acceleration kinematics cache. This
  //   method aborts if `ac` is nullptr.
  //
  // @pre The position kinematics `pc` must have been previously updated with a
  // call to CalcPositionKinematicsCache().
  // @pre The velocity kinematics `vc` must have been previously updated with a
  // call to CalcVelocityKinematicsCache().
  void CalcAccelerationKinematicsCache(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc, const VectorX<T>& known_vdot,
      AccelerationKinematicsCache<T>* ac) const;

  // See MultibodyPlant method.
  // @warning The output parameter `A_WB_array` is indexed by MobodIndex,
  // while MultibodyPlant's method returns accelerations indexed by BodyIndex.
  void CalcSpatialAccelerationsFromVdot(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc, const VectorX<T>& known_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  // See MultibodyPlant method.
  VectorX<T> CalcInverseDynamics(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      const MultibodyForces<T>& external_forces) const;

  // (Advanced) Given the state of this MultibodyTree in context and a
  // known vector of generalized accelerations vdot, this method computes the
  // set of generalized forces tau that would need to be applied at each
  // Mobilizer in order to attain the specified generalized accelerations.
  // Mathematically, this method computes: <pre>
  //   tau = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
  // </pre>
  // where M(q) is the %MultibodyTree mass matrix, C(q, v)v is the bias
  // term containing Coriolis and gyroscopic effects and tau_app consists
  // of a vector applied generalized forces. The last term is a summation over
  // all bodies in the model where Fapp_Bo_W is an applied spatial force on
  // body B at Bo which gets projected into the space of generalized forces
  // with the transpose of Jv_V_WB(q) (where Jv_V_WB is B's spatial
  // velocity Jacobian in W with respect to generalized velocities v).
  // Note: B's spatial velocity in W can be written as V_WB = Jv_V_WB * v.
  // This method does not compute explicit expressions for the mass matrix nor
  // for the bias term, which would be of at least O(n²) complexity, but it
  // implements an O(n) Newton-Euler recursive algorithm, where n is the
  // number of bodies in the MultibodyTree. The explicit formation of the
  // mass matrix M(q) would require the calculation of O(n²) entries while
  // explicitly forming the product C(q, v) * v could require up to O(n³)
  // operations (see [Featherstone 1987, §4]), depending on the implementation.
  // The recursive Newton-Euler algorithm is the most efficient currently known
  // general method for solving inverse dynamics [Featherstone 2008].
  //
  // @param[in] context
  //   The context containing the state of the MultibodyTree model.
  // @param[in] known_vdot
  //   A vector with the known generalized accelerations vdot for the full
  //   MultibodyTree model. Use Mobilizer::get_accelerations_from_array() to
  //   access entries into this array for a particular Mobilizer. You can use
  //   the mutable version of this method to write into this array.
  // @param[in] Fapplied_Bo_W_array
  //   An optional vector containing the spatial force Fapplied_Bo_W applied on
  //   each body at the body's frame origin Bo and expressed in the world frame
  //   W. Fapplied_Bo_W_array can have zero size which means there are no
  //   applied spatial forces. To apply non-zero forces, Fapplied_Bo_W_array
  //   must be of size equal to the number of mobilized bodies in this
  //   MultibodyTree model. This array must be ordered by MobodIndex, which for
  //   a given body can be retrieved with RigidBody::mobod_index().
  // @param[in] tau_applied_array
  //   An optional array of applied generalized forces for the entire model. For
  //   a given mobilizer, entries in this array can be accessed using the method
  //   Mobilizer::get_generalized_forces_from_array() while its mutable
  //   counterpart, Mobilizer::get_mutable_generalized_forces_from_array(),
  //   allows writing into this array. tau_applied_array can have zero size,
  //   which means there are no applied generalized forces. To apply non-zero
  //   forces, tau_applied_array must be of size equal to the number to the
  //   number of generalized velocities in the model, see
  //   MultibodyTree::num_velocities().
  // @param[out] A_WB_array
  //   A pointer to a valid, non nullptr, vector of SpatialAcceleration that on
  //   return will contain the spatial acceleration A_WB for each body. It must
  //   be of size equal to the number of mobilized bodies and is ordered by
  //   MobodIndex.
  // @param[out] F_BMo_W_array
  //   A pointer to a valid, non nullptr, vector of SpatialForce that on return
  //   will contain the spatial force F_BMo_W corresponding to each mobilized
  //   body B's inboard mobilizer reaction force applied at the origin Mo of the
  //   mobilizer's M frame, expressed in the world frame W. It must be of size
  //   equal to the number of mobilized bodies and is ordered by MobodIndex.
  // @param[out] tau_array
  //   A pointer to a valid, non nullptr, vector that on return will contain
  //   the generalized forces that are required to achieve the desired
  //   generalized accelerations given in known_vdot. The size must be
  //   MultibodyTree::num_velocities().
  //
  // @warning There is no mechanism to assert that either A_WB_array nor
  //   F_BMo_W_array are ordered by MobodIndex. You can use
  //   RigidBody::mobod_index() to obtain the node index for a given body.
  //
  // @note This method uses F_BMo_W_array and tau_array as the only local
  // temporaries and therefore no additional dynamic memory allocation is
  // performed.
  //
  // @pre F_BMo_W_array and Fapplied_Bo_W_array are distinct objects as are
  //   tau_array and tau_applied_array.
  void CalcInverseDynamics(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
      const Eigen::Ref<const VectorX<T>>& tau_applied_array,
      std::vector<SpatialAcceleration<T>>* A_WB_array,
      std::vector<SpatialForce<T>>* F_BMo_W_array,
      EigenPtr<VectorX<T>> tau_array) const;

  // Given the state stored in `context` and a
  // known vector of generalized accelerations `vdot`, this method computes the
  // set of generalized forces `tau_id` that would need to be applied at each
  // Mobilizer in order to attain the specified generalized accelerations.
  // Mathematically, this method computes: <pre>
  //   tau_id = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
  // </pre>
  // where `M(q)` is the mass matrix, `C(q, v)v` is the bias
  // term containing Coriolis and gyroscopic effects and `tau_app` consists
  // of a vector applied generalized forces.
  //
  // iff `ignore_velocities = true` velocity values stored in `context` are
  // ignored and are assumed to be zero. Therefore, C(q, v)v = 0 and it is not
  // computed to avoid unnecessary work.
  void CalcInverseDynamics(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
      const Eigen::Ref<const VectorX<T>>& tau_applied_array,
      bool ignore_velocities, std::vector<SpatialAcceleration<T>>* A_WB_array,
      std::vector<SpatialForce<T>>* F_BMo_W_array,
      EigenPtr<VectorX<T>> tau_array) const;

  // See MultibodyPlant method.
  void CalcForceElementsContribution(const systems::Context<T>& context,
                                     const PositionKinematicsCache<T>& pc,
                                     const VelocityKinematicsCache<T>& vc,
                                     MultibodyForces<T>* forces) const;

  // TODO(sherm1) Revise the comments below as #12942 is addressed.

  // See System method. Currently includes only gravity and explicit
  // ForceElement sources; potential energy of contact is ignored.
  // See issue #12942.
  T CalcPotentialEnergy(const systems::Context<T>& context) const;

  // See System method.
  T CalcKineticEnergy(const systems::Context<T>& context) const;

  // See System method. Currently includes only gravity and explicit
  // ForceElement sources; potential energy of contact is ignored.
  // See issue #12942.
  T CalcConservativePower(const systems::Context<T>& context) const;

  // See System method. Currently includes only explicit ForceElement sources.
  // Power from joint dampers, actuators, input ports, and contact are
  // not included.
  // See issue #12942.
  T CalcNonConservativePower(const systems::Context<T>& context) const;

  // See MultibodyPlant method.
  void CalcMassMatrixViaInverseDynamics(const systems::Context<T>& context,
                                        EigenPtr<MatrixX<T>> M) const;

  // See MultibodyPlant method.
  void CalcMassMatrix(const systems::Context<T>& context,
                      EigenPtr<MatrixX<T>> M) const;

  // See MultibodyPlant method.
  void CalcBiasTerm(const systems::Context<T>& context,
                    EigenPtr<VectorX<T>> Cv) const;

  // See MultibodyPlant method.
  VectorX<T> CalcGravityGeneralizedForces(
      const systems::Context<T>& context) const;

  // See MultibodyPlant method.
  bool IsVelocityEqualToQDot() const;

  // See MultibodyPlant method.
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const;

  // See MultibodyPlant method.
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const;

  // See MultibodyPlant method.
  Eigen::SparseMatrix<T> MakeVelocityToQDotMap(
      const systems::Context<T>& context) const;

  // See MultibodyPlant method.
  Eigen::SparseMatrix<T> MakeQDotToVelocityMap(
      const systems::Context<T>& context) const;

  /*
  @anchor internal_forward_dynamics
  @name Articulated Body Algorithm Forward Dynamics.
  The Articulated Body Algorithm (ABA) implements a forward dynamics
  computation with O(n) complexity. The algorithm is implemented in terms of
  three main passes:
  1. CalcArticulatedBodyInertiaCache(): which performs a tip-to-base pass to
     compute the ArticulatedBodyInertia for each body along with other ABA
     quantities that are configuration dependent only.
  2. CalcArticulatedBodyForceCache(): a second tip-to-base pass which
     essentially computes the bias terms in the ABA equations. These are a
     function of the full state x = [q; v] and externally applied actuation and
     forces.
  3. CalcArticulatedBodyAccelerations(): which performs a final base-to-tip
     recursion to compute the acceleration of each body in the model. These
     accelerations are a function of the ArticulatedBodyForceCache
     previously computed by CalcArticulatedBodyForces(). That is, accelerations
     are a function of state x and applied forces.

  The Newton-Euler equations governing the motion of a rigid body are: <pre>
    Fapp_B = M_B * A_WB + Fb_B
  </pre>
  which describe the effect of the total applied spatial forces Fapp_B on the
  spatial acceleration A_WB of a **rigid body** B with spatial inertia M_B.
  Fb_B is the spatial force bias containing gyroscopic terms.

  Similarly, it is possible to show that there exists a linear relationship
  between the spatial acceleration and external forces for a body that belongs
  to a system of rigid bodies or **articulated body**. In particular, if this
  body is the root (or handle) of an articulated body system, the reaction force
  needed to enforce its the motion with acceleration A_WB is given by: <pre>
    F_B = P_B * A_WB + Z_B                                                   (1)
  </pre>
  where F_B is now the spatial force needed to induce the spatial acceleration
  A_WB of this root body B being part of a larger articulated system. Z_B is
  the articulated body spatial forces bias term and P_B the articulated body
  inertia, see documentation in the ArticulatedBodyInertia class. The existence
  of P_B and Z_B can be proved by induction, see [Jain 2010, §6.2.1] and
  [Featherstone 2008, §7.2].

  In Drake we closely follow the notation and spatial algebra described in [Jain
  2010]. However, we follow the algebraic steps of ABA as described in
  [Featherstone 2008]. The main difference between the two presentations becomes
  apparent when contrasting [Jain 2010, Eq. 7.34] with [Featherstone, 2008, Eq.
  7.25] which show how to compute joint reaction forces for an articulated body.
  A more in depth analysis reveals that this difference stems from a different
  definition of the articulated force bias, see @ref abi_and_bias_force
  "Articulated Body Inertia and Bias Force". That is, Featherstone's and Jain's
  force biases have different numerical values.

  Both algorithms are equivalent but in Drake we like the neat parallelism
  between the Newton-Euler equations and the ABA equations as presented by
  [Featherstone, 2008]. That is, with Featherstone's definition of the force
  bias we can write F_B_W = P_B_W * A_WB + Z_B_W [Featherstone, 2008, Eq. 7.25]
  while Jain needs to first subtract the acceleration bias as in F_B_W = P_B_W *
  (A_WB - Ab_WB) + Z_B_W [Jain 2010, Eq. 7.34].

  The section below on @ref forward_dynamics_notation "Notation" summarizes the
  main differences in notation between [Featherstone, 2008], [Jain, 2010] and
  Drake, as well as it provides equations numbers for each reference.

  @anchor abi_and_bias_force
  <h3> Articulated Body Inertia and Force Bias </h3>

  We can prove the existence of P_B and Z_B for all bodies in a multibody
  system using an induction argument on Eq. (1). [Featherstone, 2008] does this
  in a very clear and compact form in Section 7.2.2 while at the same time
  deriving the recursive relations to compute these quantities. Here we limit
  ourselves to 1) introduce these equations with Drake's notation, making
  reference to the respective equations in [Featherstone, 2008], and
  2) to highlight the differences with the formulation in [Jain, 2010] for
     reference.

  Articulated body inertias and force biases can be computed by a recursive tip
  to base assembly process (Eqs. 7.21-7.24 in [Featherstone, 2008]): <pre>
    P_B_W = M_B_W + Σᵢ Pplus_BCib_W                                          (2)
    Z_B_W = Fb_B_W - Fapp_B_W + Σᵢ Zplus_Cib_W                               (3)
  </pre>
  where M_B_W is the SpatialInertia of body B, P_B_W its
  ArticulatedBodyInertia, Fapp_B_W are the externally applied forces, and
  Pplus_BCib_W and Zplus_Cib_W are the effective ABI and force bias of an
  articulated body with a massless handle at B and including all bodies outboard
  of Ci. Both Pplus_BCib_W and Zplus_Cib_W are shifted to B and expressed in W.
  The role of Pplus_BCib_W and Zplus_Cib_W is clearer when considering the
  equation to compute the reaction force at the mobilizer constraining the
  motion of body B (Eq. 7.25 in [Featherstone, 2008]): <pre>
    F_B_W = Pplus_PB_W * Aplus_WB + Zplus_B_W                                (4)
  </pre>
  This equation mirrors Eq. (1) but it is written in terms of the rigidly
  shifted spatial acceleration `Aplus_WB = Φᵀ(p_PB) * A_WP`, or
  Aplus_WB.Shift(p_PB_W) in code.

  The articulated body inertia Pplus can be computed once P_B_W is obtained from
  Eq. (2): <pre>
     Pplus_PB_W = P_B_W - P_B_W * H_PB_W * D_B⁻¹ * H_PB_Wᵀ * P_B_W
                = P_B_W - g_B_W * U_B_W                                      (5)
  </pre>
  where: <pre>
    D_B = H_PB_Wᵀ * P_B_W * H_PB_W ∈ ℝᵐˣᵐ                                    (6)
    U_B_W = H_PB_Wᵀ * P_B_W ∈ ℝᵐˣ⁶                                           (7)
    g_B_W = U_B_Wᵀ * D_B⁻¹ ∈ ℝ⁶ˣᵐ                                            (8)
  </pre>
  with m the number of mobilities of body B. U_B_W and g_B_W are useful
  configuration dependent quantities that appear several times in the ABA. The
  force bias Zplus across the mobilizer is computed once the Z_B_W is obtained
  from Eq. (3): <pre>
    Zplus_B_W = Z_B_W + Pplus_PB_W * Ab_WB + g_B_W * e_B                     (9)
    e_B = tau_B - H_PB_Wᵀ * Z_B_W                                           (10)
  </pre>
  where tau_B are the applied generalized forces on body B's mobilizer. Notice
  that, given their definition in Eqs. (3) and (9), the ABA force bias terms
  Z_B_W and Zplus_B_W are not only a function of the velocity dependent
  gyroscopic terms Ab_WB and Fb_B_W, but also of the externally applied forces
  and actuation.

  @note Even though we use H as the symbol for the "hinge matrix" ("joint's
  motion subspace matrix" in Featherstone) as introduced by Jain, this matrix is
  the **transpose** of Jain's matrix so that it acts as any other Jacobian. For
  instance in Drake we read `V_PB_W = H_PB_W * v_B` while H_PB_W would be
  transposed in Jain's book.

  Terms that are only a function of the configuration q such as P_B_W,
  Pplus_PB_W, D_B, and g_B_W are computed in the first pass of the ABA by
  CalcArticulatedBodyInertiaCache(). The second pass implemented in
  CalcArticulatedBodyForces() computes the acceleration bias Ab_WB_W,
  articulated body force biases Z_B_W, Zplus_B_W, and e_B. These terms are
  function of the full state including configuration and velocities and of the
  applied external forcing. Ab_WB_W is zero when velocities are zero. Z_B_W,
  Zplus_B_W, and e_B are zero when velocities and externally applied forces are
  zero.

  @note [Featherstone, 2008] and [Jain, 2010] use a different definition of
  Z_B_W. When comparing the two algorithms, we'll denote with Zj_B_W,
  Zjplus_B_W, and ej_B the bias terms as defined by Jain. They have different
  numerical values than those introduced by [Featherstone, 2008] (even after
  making the conversion from Plücker to Jain's spatial algebra.)
  A detailed analysis of the two reveals that: <pre>
    Zjplus_B_W = Zplus_B_W
    Zj_B_W = Z_B_W + P_B_W * Ab_WB
    ej_B = e_B - H_PB_Wᵀ * P_B_W * Ab_WB
  </pre>
  which then translates into the differences we observe with [Jain, 2010,
  Algorithm 7.2]: <pre>
    Zj_B_W = Fb_B_W - Fapp_B_W + Σᵢ Zplus_Cib_W + P_B_W * Ab_WB
    Z_B_W  = Fb_B_W - Fapp_B_W + Σᵢ Zplus_Cib_W,                    from Eq. (3)
  </pre>
  where the term with Ab_WB does not appear in our Eq. (3).
  <pre>
    Zjplus_B_W = Zj_B_W + g_B_W * ej_B
    Zplus_B_W  = Z_B_W  + g_B_W * e_B + Pplus_PB_W * Ab_WB,         from Eq. (9)
  </pre>
  where notice our Eq. (9) has the additional term Pplus_PB_W * Ab_WB. However,
  as mentioned above, the numerical values of Zplus_B_W and Zjplus_B_W are
  exactly the same given the difference cancels out through the additional terms
  present in ej_B, see below. Finally:
  <pre>
    ej_B = tau_B - H_PB_Wᵀ * Zj_B_W
    e_B  = tau_B - H_PB_Wᵀ * Z_B_W,                                from Eq. (10)
  </pre>
  which is deceivingly the same as our Eq. (10), however the result is
  different given it has Zj_B_W in it, which numerically differs from Z_B_W.
  This different definition of the force bias leads to a different expression
  for the computation of reaction forces in terms of the articulated body
  quantities: <pre>
    F_B = P_B * A_WB + Z_B,             [Featherstone, 2008, Eq. 7.25]
    F_B = P_B * (A_WB - Ab_WB) + Zj_B,  [Jain, 2010, Eq. 7.34]
  </pre>
  In Drake we prefer Featherstone's definition of the force bias terms given the
  parallelism of the joint reaction forces equation with the Newton-Euler
  equations.

  @anchor abi_computing_accelerations
  <h3> Computing Accelerations </h3>
  Once ABA inertias and force bias terms are computed according to Eqs.
  (2)-(10), the computation of accelerations is remarkably simple. The last base
  to tip pass of the algorithm stems from combining the following three
  equations: <pre>
    A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B                              (11a)
    F_B_W = P_B_W * A_WB + Z_B_W                                           (11b)
    tau_B = H_PB_Wᵀ * F_B_W                                                (11c)
  </pre>
  Equation (11a) is the motion constraint imposed by the body's mobilizer,
  where the spatial acceleration bias Ab_WB = Ac_WB + Ab_PB includes the
  centrifugal and Coriolis terms Ac_WB documented in
  SpatialAcceleration::ComposeWithMovingFrameAcceleration() and the bias
  term across the mobilizer Ab_PB (A_PB = H_PB * vdot_B + Ab_PB.)
  Equation (11b) is the articulated body force balance from Eq. (1) and Eq.
  (11c) projects the reaction force F_B to obtain the generalized forces tau_B.
  We substitute Eqs. (11a) and (11b) into (11c) to obtain: <pre>
    H_PB_Wᵀ*[P_B_W * (Aplus_WB + Ab_WB + H_PB_W * vdot_B) + Z_B_W] = tau_B  (12)
  </pre>
  we then factor out terms grouping vdot_B, acceleration biases and forcing:
  <pre>
    (H_PB_Wᵀ*P_B_W*H_PB_W) * vdot_B + (H_PB_Wᵀ*P_B_W) * (Aplus_WB + Ab_WB ) =
      tau_B - H_PB_Wᵀ * Z_B_W                                               (13)
  </pre>
  using the definitions in Eqs. (6)-(8), we can rewrite (13) as: <pre>
    D_B * vdot_B + U_B_W * (Aplus_WB + Ab_WB) = e_B                         (14)
  </pre>
  Therefore the last base-to-tip pass updates generalized accelerations and
  spatial accelerations according to: <pre>
    vdot_B = D_B⁻¹ * e_B - g_B_Wᵀ * (Aplus_WB + Ab_WB)                      (15)
    A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B                               (16)
  </pre>
  This is implemented in CalcArticulatedBodyAccelerations().

  @note Given the different definition of Z and Zplus used by [Featherstone,
  2008] and [Jain, 2010], the acceleration update used by Jain is: <pre>
    vdot_B = D_B⁻¹ * e_B  - g_B_Wᵀ * (Aplus_WB + Ab_WB),           from Eq. (15)
    vdot_B = D_B⁻¹ * ej_B - g_B_Wᵀ * Aplus_WB,            [Jain, 2010. Alg. 7.2]
  </pre>

  <h3> Additional Diagonal Inertias </h3>
  @anchor additional_diagonal_inertias

  We can model additional diagonal inertias by considering external generalized
  forces of the form: <pre>
    tau_B <-- -d_B * vdot_B + tau_B                                         (17)
  </pre>
  That is, we update tau_B to include the term -d_B * vdot_B, for the mobilities
  of each body B. Such form of the generalized forces can be used to model fluid
  virtual masses, reflected inertias and/or even effective inertias result of
  discrete time stepping schemes.

  When Eq. (17) is used into Eq. (14), the update for vdot_B in Eq. (15) remains
  exactly the same but with D_B updated to: <pre>
    D_B <-- D_B + d_B
  </pre>
  With this modification to D_B, the algorithm remains the same.

  We remark that the modeling of this term is equivalent to adding a diagonal
  term D = diag{d_B} (the concantenation of each d_B to form the diagonal matrix
  D) to the mass matrix. This can be seen by considering the Newton-Euler
  equations of motion: <pre>
    M⋅v̇ + C⋅v = τ − D⋅v̇
  </pre>
  which can then rewritten as: <pre>
    (M+D)⋅v̇ + C⋅v = τ
  </pre>
  resulting on the same Newton-Euler equations but with the updated mass matrix
  M+D.

  We have not made any assumptions on the sign of the coefficients of D.
  However, physical models (e.g. reflected rotor inertias) will typically lead
  to a positive D, making the effective mass matrix M+D more diagonally
  dominant.

  <h3> Notation </h3>
  @anchor forward_dynamics_notation

  Since we use the ABA algorithm as described in [Featherstone 2008], with
  spatial algebra and symbols from [Jain, 2010] and monogram notation as
  described in @ref multibody_notation, here we present a table that compares
  the different symbols across these three different sources. This is
  especially useful when studying the particulars of ABA as introduced in
  [Jain, 2010] and [Featherstone 2008] or as implemented in Drake. */

  // clang-format off
  /*
  Quantity                            |    Featherstone 2008 †    |              Jain 2010 ††              | Drake monogram †††
  ------------------------------------|:-------------------------:|:--------------------------------------:|:-------------------
  Body spatial acceleration           |  𝒂ᵢ                       |  α(k)                                  | A_WB
  Rigidly shifted acceleration        |  N/A                      |  α⁺(k) (6.9)                           | Aplus_WB
  Hinge matrix                        |  𝑺ᵢ (3.33)                |  H*(k)                                 | H_PB_W
  Acceleration bias                   |  𝒄ᵢ (7.31)                |  𝔞(k)   (5.21)                         | Ab _WB
  Outboard body spatial acceleration  | 𝒂ᵢ = 𝒂ₗ + 𝒄ᵢ + 𝑺ᵢq̈ᵢ (7.31) | α(k) = α⁺(k) + 𝔞(k) + H*(k)v(k) (5.21) | A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B
  Articulated Body Inertia (ABI)      |  𝑰ᴬᵢ                (7.2)  | P(k)   (6.6)                          | P_B_W
  ABI across the mobilizer            |  𝑰ᵃᵢ                (7.23) | P⁺(k)  (6.24)                         | Pplus_B_W
  ABA force bias                      |  𝒑ᴬᵢ                (7.2)  | 𝔷(k)   (6.6)                           | Z_B_W
  ABA force bias across the mobilizer |  𝒑ᵃᵢ                (7.24) | 𝔷⁺(k)  (6.33)                          | Zplus_B_W
  */
  // clang-format on

  /*
  † Featherstone's spatial vectors are Plücker vectors, see §2.

  †† Jain's spatial vectors are the concatenation of two ordinary
  three-dimensional vectors.

  ††† In Drake's source we often specify the expressed-in frame explicitly.
  However we often write derivations in vector form and omit the expressed-in
  frame.

  - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
                algorithms. Springer Science & Business Media, pp. 123-130.
  - [Featherstone 2008] Featherstone, R., 2008. Rigid body dynamics algorithms.
                        Springer.
   @{
  */

  // Performs a tip-to-base pass to compute the ArticulatedBodyInertia for each
  // body as a function of the configuration q stored in `context`. The
  // computation is stored in `abic` along with other Articulated Body
  // Algorithm (ABA) quantities.
  void CalcArticulatedBodyInertiaCache(
      const systems::Context<T>& context,
      ArticulatedBodyInertiaCache<T>* abic) const;

  // Performs a tip-to-base pass which essentially computes the force bias
  // terms in the ABA equations. These are a function of the full state
  // `x = [q; v]`, stored in `context`, and externally applied `forces`.
  // Refer to @ref abi_and_bias_force "Articulated Body Inertia and Force Bias"
  // for further details.
  void CalcArticulatedBodyForceCache(
      const systems::Context<T>& context, const MultibodyForces<T>& forces,
      ArticulatedBodyForceCache<T>* aba_force_cache) const;

  // Performs the final base-to-tip pass of ABA to compute the acceleration of
  // each body in the model into output `ac`.
  // Refer to @ref abi_computing_accelerations "Computing Accelerations" for
  // further details.
  void CalcArticulatedBodyAccelerations(
      const systems::Context<T>& context,
      const ArticulatedBodyForceCache<T>& aba_force_cache,
      AccelerationKinematicsCache<T>* ac) const;

  // For a body B, computes the spatial acceleration bias term `Ab_WB` as it
  // appears in the acceleration level motion constraint imposed by body B's
  // mobilizer `A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B`, with `Aplus_WB =
  // Φᵀ(p_PB) * A_WP` the rigidly shifted spatial acceleration of the inboard
  // body P and `H_PB_W` and `vdot_B` its mobilizer's hinge matrix and
  // mobilities, respectively. See @ref abi_computing_accelerations for further
  // details. On output `Ab_WB_all[mobod_index]`
  // contains `Ab_WB` for the body with node index `mobod_index`.
  void CalcSpatialAccelerationBias(
      const systems::Context<T>& context,
      std::vector<SpatialAcceleration<T>>* Ab_WB_all) const;

  // Computes the articulated body force bias `Zb_Bo_W = Pplus_PB_W * Ab_WB`
  // for each articulated body B. On output `Zb_Bo_W_all[mobod_index]`
  // contains `Zb_Bo_W` for the body B with node index `mobod_index`.
  void CalcArticulatedBodyForceBias(
      const systems::Context<T>& context,
      std::vector<SpatialForce<T>>* Zb_Bo_W_all) const;

  /*
  @anchor forward_dynamics_with_diagonal_terms_apis
  @name Alternative ABA Signatures to include diagonal inertia terms.
  These "advanced" level alternative APIs are provided so that we can
  include the modeling of additional diagonal inertias as discussed in @ref
  additional_diagonal_inertias. The additional inertias affect both articulated
  body inertias and bias terms, CalcArticulatedBodyInertiaCache() and
  CalcArticulatedBodyForceBias() respectively. These in turn must be included in
  CalcArticulatedBodyForceCache() to compute the ABA force cache needed to
  finally compute accelerations with CalcArticulatedBodyAccelerations(). */
  void CalcArticulatedBodyInertiaCache(
      const systems::Context<T>& context, const VectorX<T>& diagonal_inertias,
      ArticulatedBodyInertiaCache<T>* abic) const;

  void CalcArticulatedBodyForceBias(
      const systems::Context<T>& context,
      const ArticulatedBodyInertiaCache<T>& abic,
      std::vector<SpatialForce<T>>* Zb_Bo_W_all) const;

  void CalcArticulatedBodyForceCache(
      const systems::Context<T>& context,
      const ArticulatedBodyInertiaCache<T>& abic,
      const std::vector<SpatialForce<T>>& Zb_Bo_W_cache,
      const MultibodyForces<T>& forces,
      ArticulatedBodyForceCache<T>* aba_force_cache) const;

  void CalcArticulatedBodyAccelerations(
      const systems::Context<T>& context,
      const ArticulatedBodyInertiaCache<T>& abic,
      const ArticulatedBodyForceCache<T>& aba_force_cache,
      AccelerationKinematicsCache<T>* ac) const;
  // @}

  // @} Closes "Articulated Body Algorithm Forward Dynamics" Doxygen section.

  // @}
  // Closes "Computational methods" Doxygen section.

  // See MultibodyPlant method.
  MatrixX<double> MakeStateSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const;

  // Alternative signature to build a state selector matrix from a std::vector
  // of joint names.
  // See MakeStateSelectorMatrixFromJointNames(const std::vector<JointIndex>&)
  // for details.
  // `selected_joints` must not contain any duplicates.
  //
  // A user specifies the preferred order in the selected states vector xₛ via
  // `selected_joints`. The selected state is built such that selected
  // positions are followed by selected velocities, as in `xₛ = [qₛ, vₛ]`.
  // The positions in qₛ are a concatenation of the positions for each joint
  // in the order they appear in `selected_joints`. That is, the positions for
  // `selected_joints[0]` are first, followed by the positions for
  // `selected_joints[1]`, etc. Similarly for the selected velocities vₛ.
  //
  // @throws std::exception if there are any duplicates in `selected_joints`.
  // @throws std::exception if there is no joint in the model with a name
  // specified in `selected_joints`.
  MatrixX<double> MakeStateSelectorMatrixFromJointNames(
      const std::vector<std::string>& selected_joints) const;

  // See MultibodyPlant method.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointActuatorIndex>& user_to_actuator_index_map) const;

  // See MultibodyPlant method.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const;

  // See MultibodyPlant method.
  VectorX<double> GetPositionLowerLimits() const;

  // See MultibodyPlant method.
  VectorX<double> GetPositionUpperLimits() const;

  // See MultibodyPlant method.
  VectorX<double> GetVelocityLowerLimits() const;

  // See MultibodyPlant method.
  VectorX<double> GetVelocityUpperLimits() const;

  // See MultibodyPlant method.
  VectorX<double> GetAccelerationLowerLimits() const;

  // See MultibodyPlant method.
  VectorX<double> GetAccelerationUpperLimits() const;

  // See MultibodyPlant method.
  VectorX<double> GetEffortLowerLimits() const;

  // See MultibodyPlant method.
  VectorX<double> GetEffortUpperLimits() const;

  // @name Methods to retrieve multibody element variants
  //
  // Given two variants of the same %MultibodyTree, these methods map an
  // element in one variant, to its corresponding element in the other variant.
  //
  // A concrete case is the call to ToAutoDiffXd() to obtain a
  // %MultibodyTree variant templated on AutoDiffXd from a %MultibodyTree
  // templated on `double`. Typically, a user holding a `RigidBody<double>` (or
  // any other multibody element in the original variant templated on `double`)
  // would like to retrieve the corresponding `RigidBody<AutoDiffXd>` variant
  // from the new AutoDiffXd tree variant.
  //
  // Consider the following code example:
  // @code
  //   // The user creates a model.
  //   MultibodyTree<double> model;
  //   // User adds a body and keeps a reference to it.
  //   const RigidBody<double>& body = model.AddRigidBody(...);
  //   // User creates an AutoDiffXd variant. Variants on other scalar types
  //   // can be created with a call to CloneToScalar().
  //   std::unique_ptr<MultibodyTree<Tvariant>> variant_model =
  //       model.ToAutoDiffXd();
  //   // User retrieves the AutoDiffXd variant corresponding to the original
  //   // body added above.
  //   const RigidBody<AutoDiffXd>&
  //       variant_body = variant_model.get_variant(body);
  // @endcode
  //
  // MultibodyTree::get_variant() is templated on the multibody element
  // type which is deduced from its only input argument. The returned element
  // is templated on the scalar type T of the %MultibodyTree on which this
  // method is invoked.
  // @{

  // Overload for Frame<T> elements.
  template <typename Scalar>
  const Frame<T>& get_variant(const Frame<Scalar>& element) const {
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    const FrameIndex frame_index = element.index();
    return frames_.get_element(frame_index);
  }

  // Overload for RigidBody<T> elements.
  template <typename Scalar>
  const RigidBody<T>& get_variant(const RigidBody<Scalar>& element) const {
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    const BodyIndex body_index = element.index();
    return rigid_bodies_.get_element(body_index);
  }

  // Overload for Mobilizer<T> elements.
  template <typename Scalar>
  const Mobilizer<T>& get_variant(const Mobilizer<Scalar>& element) const {
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    MobodIndex mobilizer_index = element.index();
    DRAKE_DEMAND(mobilizer_index < num_mobilizers());
    const Mobilizer<T>* result = mobilizers_[mobilizer_index].get();
    DRAKE_DEMAND(result != nullptr);
    return *result;
  }

  // Overload for Mobilizer<T> elements (mutable).
  // TODO(russt): Add mutable accessors for other variants as needed.
  template <typename Scalar>
  Mobilizer<T>& get_mutable_variant(const Mobilizer<Scalar>& element) {
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    MobodIndex mobilizer_index = element.index();
    DRAKE_DEMAND(mobilizer_index < num_mobilizers());
    Mobilizer<T>* result = mobilizers_[mobilizer_index].get();
    DRAKE_DEMAND(result != nullptr);
    return *result;
  }

  // Overload for Joint<T> elements.
  template <typename Scalar>
  const Joint<T>& get_variant(const Joint<Scalar>& element) const {
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    const JointIndex joint_index = element.index();
    return joints_.get_element(joint_index);
  }
  // @}

  // Creates a deep copy of `this` %MultibodyTree templated on the same
  // scalar type T as `this` tree.
  std::unique_ptr<MultibodyTree<T>> Clone() const { return CloneToScalar<T>(); }

  // Creates a deep copy of `this` %MultibodyTree templated on AutoDiffXd.
  std::unique_ptr<MultibodyTree<AutoDiffXd>> ToAutoDiffXd() const {
    return CloneToScalar<AutoDiffXd>();
  }

  // Creates a deep copy of `this` %MultibodyTree templated on the scalar type
  // `ToScalar`.
  // The new deep copy is guaranteed to have exactly the same
  // MultibodyTreeTopology as the original tree the method is called on.
  // This method ensures the following cloning order:
  //
  //   - RigidBody objects, and their corresponding RigidBodyFrame objects.
  //   - Frame objects.
  //   - If a Frame is attached to another frame, its parent frame is
  //     guaranteed to be created first.
  //   - Mobilizer objects are created last and therefore clones of the
  //     original Frame objects are guaranteed to already be part of the cloned
  //     tree.
  //
  // Consider the following code example:
  // @code
  //   // The user creates a model.
  //   MultibodyTree<double> model;
  //   // User adds a body and keeps a reference to it.
  //   const RigidBody<double>& body = model.AddRigidBody(...);
  //   // User creates an AutoDiffXd variant, where ToScalar = AutoDiffXd.
  //   std::unique_ptr<MultibodyTree<AutoDiffXd>> model_autodiff =
  //       model.CloneToScalar<AutoDiffXd>();
  //   // User retrieves the AutoDiffXd variant corresponding to the original
  //   // body added above.
  //   const RigidBody<AutoDiffXd>&
  //       body_autodiff = model_autodiff.get_variant(body);
  // @endcode
  //
  // MultibodyTree::get_variant() is templated on the multibody element
  // type which is deduced from its only input argument. The returned element
  // is templated on the scalar type T of the %MultibodyTree on which this
  // method is invoked.
  // In the example above, the user could have also invoked the method
  // ToAutoDiffXd().
  //
  // @pre Finalize() must have already been called on this %MultibodyTree.
  template <typename ToScalar>
  std::unique_ptr<MultibodyTree<ToScalar>> CloneToScalar() const;

  // Evaluates frame body poses cached in context, updating all frames'
  // body poses if parameters have changed since last update.
  // @returns a reference to the now-up-to-date cache entry
  const FrameBodyPoseCache<T>& EvalFrameBodyPoses(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalFrameBodyPoses(context);
  }

  // Evaluates position kinematics cached in context.
  // @param context A Context whose position kinematics cache will be
  //                updated and returned.
  // @return Reference to the PositionKinematicsCache of context.
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalPositionKinematics(context);
  }

  // Evaluates velocity kinematics cached in context. This will also
  // force position kinematics to be updated if it hasn't already.
  // @param context A Context whose velocity kinematics cache will be
  //                updated and returned.
  // @return Reference to the VelocityKinematicsCache of context.
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalVelocityKinematics(context);
  }

  // Evaluates acceleration kinematics cached in context. This will also
  // force other cache entries to be updated if they haven't already.
  // @param context A Context whose acceleration kinematics cache will be
  //                updated and returned.
  // @return Reference to the AccelerationKinematicsCache of context.
  const AccelerationKinematicsCache<T>& EvalAccelerationKinematics(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalForwardDynamics(context);
  }

  // Evaluate the cache entry storing articulated body inertias in `context`.
  const ArticulatedBodyInertiaCache<T>& EvalArticulatedBodyInertiaCache(
      const systems::Context<T>& context) const {
    return tree_system_->EvalArticulatedBodyInertiaCache(context);
  }

  // Evaluate the cache entry storing the across node Jacobian H_PB_W in
  // `context`.
  const std::vector<Vector6<T>>& EvalAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context) const {
    return tree_system_->EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  }

  // @name                 State access methods
  // These methods use information in the MultibodyTree to determine how to
  // locate the tree's state variables in a given Context or State.
  //@{

  // Returns true if we are using discrete state for positions and velocities;
  // otherwise we're using continuous state.
  bool is_state_discrete() const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->is_discrete();
  }

  // Returns a const reference to the position state vector q stored in the
  // given Context as an `Eigen::VectorBlock<const VectorX<T>>`.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<const VectorX<T>> get_positions(
      const systems::Context<T>& context) const;

  // This is a mutable-Context version of `get_positions()`.
  // @note Invalidates all q-dependent cache entries. (May also invalidate
  //       v-dependent cache entries.)
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositions(
      systems::Context<T>* context) const;

  // This is a mutable-State version of `get_positions()`.
  // @note This does not cause cache invalidation so it is very fast.
  // @pre `state` is a valid multibody system State.
  Eigen::VectorBlock<VectorX<T>> get_mutable_positions(
      systems::State<T>* state) const;

  // Returns a const reference to the velocity state vector v stored in the
  // given Context as an `Eigen::VectorBlock<const VectorX<T>>`.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<const VectorX<T>> get_velocities(
      const systems::Context<T>& context) const;

  // This is a mutable-Context version of `get_velocities()`.
  // @note Invalidates all v-dependent cache entries. (May also invalidate
  //       q-dependent cache entries.)
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<VectorX<T>> GetMutableVelocities(
      systems::Context<T>* context) const;

  // This is a mutable-State version of `get_velocities()`.
  // @note This does not cause cache invalidation so it is very fast.
  // @pre `state` is a valid multibody system State.
  Eigen::VectorBlock<VectorX<T>> get_mutable_velocities(
      systems::State<T>* state) const;

  // Returns a const fixed-size Eigen::VectorBlock of `length` elements
  // referencing a segment in the state vector with its first element
  // at `start`.
  // @pre `context` is a valid multibody system Context.
  // @pre start, length >= 0 and start+length is in-bounds for qv state.
  template <int length>
  Eigen::VectorBlock<const VectorX<T>, length> get_state_segment(
      const systems::Context<T>& context, int start) const {
    Eigen::VectorBlock<const VectorX<T>> qv =
        get_positions_and_velocities(context);
    return make_block_segment<length>(qv, start);
  }

  // This is a mutable-Context version of `get_state_segment<count>(start)`.
  // @note Invalidates all q- or v-dependent cache entries.
  // @pre `context` is a valid multibody system Context.
  // @pre start, length >= 0 and start+length is in-bounds for qv state.
  template <int length>
  Eigen::VectorBlock<VectorX<T>, length> GetMutableStateSegment(
      systems::Context<T>* context, int start) const {
    Eigen::VectorBlock<VectorX<T>> qv =
        GetMutablePositionsAndVelocities(context);
    return make_mutable_block_segment<length>(&qv, start);
  }

  // This is a mutable-State version of `get_state_segment<count>(start)`.
  // @note This does not cause cache invalidation so it is very fast.
  // @pre `state` is a valid multibody system State.
  // @pre start, length >= 0 and start+length is in-bounds for qv state.
  template <int length>
  Eigen::VectorBlock<VectorX<T>, length> get_mutable_state_segment(
      systems::State<T>* state, int start) const {
    Eigen::VectorBlock<VectorX<T>> qv =
        get_mutable_positions_and_velocities(state);
    return make_mutable_block_segment<length>(&qv, start);
  }

  // Returns a const Eigen::VectorBlock of `length` elements referencing a
  // segment in the Context's state vector with its first element at `start`.
  // @pre `context` is a valid multibody system Context.
  // @pre start, length >= 0 and start+length is in-bounds for qv state.
  Eigen::VectorBlock<const VectorX<T>> get_state_segment(
      const systems::Context<T>& context, int start, int length) const {
    Eigen::VectorBlock<const VectorX<T>> qv =
        get_positions_and_velocities(context);
    return make_block_segment(qv, start, length);
  }

  // This is a mutable-Context version of `get_state_segment(start, length)`.
  // @note Invalidates all q- or v-dependent cache entries.
  // @pre `context` is a valid multibody system Context.
  // @pre start, length >= 0 and start+length is in-bounds for qv state.
  Eigen::VectorBlock<VectorX<T>> GetMutableStateSegment(
      systems::Context<T>* context, int start, int length) const {
    Eigen::VectorBlock<VectorX<T>> qv =
        GetMutablePositionsAndVelocities(context);
    return make_mutable_block_segment(&qv, start, length);
  }

  // This is a mutable-State version of `get_state_segment(start, length)`.
  // @note This does not cause cache invalidation.
  // @pre `state` is a valid multibody system State.
  // @pre start, length >= 0 and start+length is in-bounds for qv state.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_segment(
      systems::State<T>* state, int start, int length) const {
    Eigen::VectorBlock<VectorX<T>> qv =
        get_mutable_positions_and_velocities(state);
    return make_mutable_block_segment(&qv, start, length);
  }
  //@}

  // Returns the MultibodyTreeSystem that owns this MultibodyTree.
  // @pre There is an owning MultibodyTreeSystem.
  const MultibodyTreeSystem<T>& tree_system() const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return *tree_system_;
  }

  // (Internal use only) Informs the MultibodyTree how to access its resources
  // within a Context.
  void set_tree_system(MultibodyTreeSystem<T>* tree_system) {
    DRAKE_DEMAND(tree_system != nullptr && tree_system_ == nullptr);
    tree_system_ = tree_system;
  }

  // (Internal) For a body B, calculates the cache entry associated with
  // H_PB_W for each node, where H_PB_W is
  // the `6 x nm` hinge matrix that relates `V_PB_W` (body B's spatial
  // velocity in its parent body P, expressed in world W) to this node's `nm`
  // generalized velocities (or mobilities) `v_B` as `V_PB_W = H_PB_W * v_B`.
  // `H_PB_W_cache` stores the Jacobian matrices for all nodes in the tree as
  // a vector of the columns of these matrices. Therefore `H_PB_W_cache` has
  // as many entries as number of generalized velocities in the tree.
  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      std::vector<Vector6<T>>* H_PB_W_cache) const;

  // (Internal use only) Sets the discrete state index for the multibody
  // state.
  void set_discrete_state_index(systems::DiscreteStateIndex index) {
    DRAKE_DEMAND(is_state_discrete());
    discrete_state_index_ = index;
  }

  // (Internal use only) Returns the discrete state index for the multibody
  // state.
  systems::DiscreteStateIndex get_discrete_state_index() const {
    DRAKE_DEMAND(tree_system_ != nullptr);
    DRAKE_DEMAND(is_state_discrete());
    DRAKE_DEMAND(topology_is_valid());
    return discrete_state_index_;
  }

  // Calculates the total default mass of all bodies in a set of BodyIndex.
  // @param[in] body_indexes A set of BodyIndex.
  // @retval Total mass of all bodies in body_indexes or 0 if there is no mass.
  double CalcTotalDefaultMass(const std::vector<BodyIndex>& body_indexes) const;

  // In the set of bodies associated with BodyIndex, returns true if any of
  // the bodies have a NaN default rotational inertia.
  // @param[in] body_indexes A set of BodyIndex.
  bool IsAnyDefaultRotationalInertiaNaN(
      const std::vector<BodyIndex>& body_indexes) const;

  // In the set of bodies associated with BodyIndex, returns true if all the
  // bodies have a zero default rotational inertia.
  // @param[in] body_indexes A set of BodyIndex.
  bool AreAllDefaultRotationalInertiaZero(
      const std::vector<BodyIndex>& body_indexes) const;

  // Throw an exception if there are bodies whose default mass or inertia
  // properties will cause subsequent numerical problems.
  void ThrowDefaultMassInertiaError() const;

  // Helper method for throwing an exception within public methods that should
  // not be called post-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfFinalized(const char* source_method) const;

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const;

 private:
  // Make MultibodyTree templated on every other scalar type a friend of
  // MultibodyTree<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from MultibodyTree<T>.
  template <typename>
  friend class MultibodyTree;

  // Friend class to facilitate testing.
  friend class MultibodyTreeTester;

  // (Internal use only) Adds a Joint to the MultibodyPlant corresponding to
  // joints that were added to the LinkJointGraph during modeling (elements
  // added during modeling are called "ephemeral"). The joint connects the body
  // frames.
  template <template <typename> class JointType, typename... Args>
  const JointType<T>& AddEphemeralJoint(const std::string& name,
                                        const RigidBody<T>& parent,
                                        const RigidBody<T>& child,
                                        Args&&... args);

  // Helpers for getting the full qv discrete state once we know we are using
  // discrete state.
  Eigen::VectorBlock<const VectorX<T>> get_discrete_state_vector(
      const systems::Context<T>& context) const;

  // Invalidates q- or v-dependent cache entries.
  Eigen::VectorBlock<VectorX<T>> get_mutable_discrete_state_vector(
      systems::Context<T>* context) const;

  // Does no invalidation.
  Eigen::VectorBlock<VectorX<T>> get_mutable_discrete_state_vector(
      systems::State<T>* state) const;

  // @pre the VectorBase is a BasicVector.
  Eigen::VectorBlock<const VectorX<T>> extract_qv_from_continuous(
      const systems::VectorBase<T>& continuous_qvz) const;

  // @pre the VectorBase is non-null and is a BasicVector.
  Eigen::VectorBlock<VectorX<T>> extract_mutable_qv_from_continuous(
      systems::VectorBase<T>* continuous_qvz) const;

  // Given a VectorBlock and (start, length) segment specification relative
  // to that block, return a new block representing that segment relative to
  // the original nested expression.
  static Eigen::VectorBlock<const VectorX<T>> make_block_segment(
      const Eigen::VectorBlock<const VectorX<T>>& block, int start,
      int length) {
    DRAKE_ASSERT(start >= 0 && length >= 0);
    DRAKE_ASSERT(start + length <= block.rows());
    return block.nestedExpression().segment(block.startRow() + start, length);
  }

  // The mutable version of make_block_segment().
  static Eigen::VectorBlock<VectorX<T>> make_mutable_block_segment(
      Eigen::VectorBlock<VectorX<T>>* block, int start, int length) {
    DRAKE_ASSERT(block != nullptr);
    DRAKE_ASSERT(start >= 0 && length >= 0);
    DRAKE_ASSERT(start + length <= block->rows());
    return block->nestedExpression().segment(block->startRow() + start, length);
  }

  // Templatized versions of the above where the return type is a fixed-length
  // VectorBlock of the given VectorX.
  template <int length>
  static Eigen::VectorBlock<const VectorX<T>, length> make_block_segment(
      const Eigen::VectorBlock<const VectorX<T>>& block, int start) {
    static_assert(length >= 0,
                  "make_block_segment(): length must be non-negative");
    DRAKE_ASSERT(start >= 0);
    DRAKE_ASSERT(start + length <= block.rows());
    return block.nestedExpression().template segment<length>(block.startRow() +
                                                             start);
  }

  template <int length>
  static Eigen::VectorBlock<VectorX<T>, length> make_mutable_block_segment(
      Eigen::VectorBlock<VectorX<T>>* block, int start) {
    static_assert(length >= 0,
                  "make_mutable_block_segment(): length must be non-negative");
    DRAKE_ASSERT(block != nullptr);
    DRAKE_ASSERT(start >= 0);
    DRAKE_ASSERT(start + length <= block->rows());
    return block->nestedExpression().template segment<length>(
        block->startRow() + start);
  }

  // Takes ownership of `body` and adds it to this MultibodyTree. Returns a
  // constant reference to the body just added, which will remain valid for the
  // lifetime of this MultibodyTree. Public members AddRigidBody() end up here.
  const RigidBody<T>& AddRigidBodyImpl(std::unique_ptr<RigidBody<T>> body);

  const Joint<T>& GetJointByNameImpl(std::string_view,
                                     std::optional<ModelInstanceIndex>) const;

  [[noreturn]] void ThrowJointSubtypeMismatch(const Joint<T>&,
                                              std::string_view) const;

  // If X_BF is nullopt, returns the body frame of `body`. Otherwise, adds a
  // FixedOffsetFrame (named based on the joint_name and frame_suffix) to `body`
  // and returns it.
  const Frame<T>& AddOrGetJointFrame(
      const RigidBody<T>& body,
      const std::optional<math::RigidTransform<double>>& X_BF,
      ModelInstanceIndex joint_instance, std::string_view joint_name,
      std::string_view frame_suffix);

  // Finalizes the MultibodyTreeTopology of this tree in accordance with the
  // SpanningForest.
  void FinalizeTopology();

  // At Finalize(), this method performs all other finalization that is not
  // topological (i.e. performed by FinalizeTopology()). This includes for
  // instance the creation of BodyNode objects.
  // This method will throw a std::exception if FinalizeTopology() was not
  // previously called on this tree.
  void FinalizeInternals();

  // Helper method to add a QuaternionFreeMobilizer to all bodies that do not
  // have a mobilizer. The mobilizer is between each body and the world. To be
  // called at Finalize().
  // The world body is special in that it is the only body in the model with no
  // mobilizer, even after Finalize().
  void CreateJointImplementations();

  // For a frame Fp that is fixed/welded to a frame_F, this method computes
  // A_AFp_E, Fp's spatial acceleration in a body_A, expressed in a frame_E.
  // @param[in] context Contains the state of the multibody system.
  // @param[in] frame_F Frame Fp is fixed/welded to frame_F and frame_F is
  //  fixed/welded to a body_B.
  // @param[in] p_FoFp_F Position vector from Fo (frame_F's origin) to the
  //   the origin of frame_Fp, expressed in frame_F.
  // @param[in] body_A The rigid body whose body-frame measures A_AFp_E.
  // @param[in] frame_E The frame in which A_AFp_E is expressed on output.
  // @param[in] A_WB_W The spatial acceleration of body_B in world frame W,
  //   expressed in W (body_B is the body to which frame_F is fixed/welded).
  // @param[in] A_WA_W The spatial acceleration of body_A in world frame W,
  //   expressed in W.
  // @returns A_AFp_E Fp's spatial acceleration in body_A, expressed in frame_E.
  // @note To use this method for a bias spatial acceleration in world frame W,
  // expressed in W with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), instead pass
  // A𝑠Bias_WB_W (body_B's bias spatial acceleration in W expressed in W) and
  // A𝑠Bias_WA_W (body_A's bias spatial acceleration in W expressed in W),
  // in which case the method returns A𝑠Bias_AFp_E (Fp's bias spatial
  // acceleration in body_A, expressed in frame_E, with respect to speeds 𝑠.
  SpatialAcceleration<T> CalcSpatialAccelerationHelper(
      const systems::Context<T>& context, const Frame<T>& frame_F,
      const Eigen::Ref<const Vector3<T>>& p_FoFp_F, const RigidBody<T>& body_A,
      const Frame<T>& frame_E, const SpatialAcceleration<T>& A_WB_W,
      const SpatialAcceleration<T>& A_WA_W) const;

  // For a frame Bp fixed/welded to both a frame_B and a body_A, this method
  // shifts spatial acceleration in the world frame W from body_A to frame_Bp.
  // @param[in] frame_B A frame that is fixed/welded to body_A.
  //            frame_Bp is fixed to frame_B, shifted by p_BoBp_B.
  // @param[in] p_BoBp_B Position vector from Bo (frame_B's origin) to the
  //            origin of frame_Bp, expressed in frame_B.
  // @param[in] A_WA_W body_A's spatial acceleration in W, expressed in W.
  // @param[in] pc Contains the position kinematics for this multibody system.
  // @param[in] pc Contains the velocity kinematics for this multibody system.
  // @returns A_WBp_W Frame Bp's spatial acceleration in W, expressed in W.
  // @note To use this method for a bias spatial acceleration, instead pass
  // A𝑠Bias_WAo_W (body_A's bias spatial acceleration in W expressed in W with
  // respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v)). It then returns A𝑠Bias_WBp_W (point
  // Bp's bias spatial acceleration in W, expressed in W with respect to 𝑠).
  SpatialAcceleration<T> ShiftSpatialAccelerationInWorld(
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
      const SpatialAcceleration<T>& A_WA_W,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const;

  // For all bodies, calculate bias spatial acceleration in the world frame W.
  // @param[in] context The state of the multibody system.
  // @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  // JacobianWrtVariable::kV, indicating whether the spatial acceleration bias
  // is with respect to 𝑠 = q̇ or 𝑠 = v.
  // @param[out] AsBias_WB_all Each body's spatial acceleration bias in world
  // frame W, with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame W.
  // @throws std::exception if with_respect_to is not JacobianWrtVariable::kV
  // @throws std::exception if AsBias_WB_all is nullptr.
  // @throws std::exception if AsBias_WB_all.size() is not num_bodies().
  void CalcAllBodyBiasSpatialAccelerationsInWorld(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      std::vector<SpatialAcceleration<T>>* AsBias_WB_all) const;

  // This method returns the spatial momentum of a list of bodies in the
  // world frame W, about the world origin Wo, expressed in the world frame W.
  // @param[in] context Contains the state of the model.
  // @param[in] body_indexes Array of selected bodies.  This method does not
  //  distinguish between welded bodies, joint-connected bodies,
  //  floating bodies, the world_body(), or repeated bodies.
  // @throws std::exception if model_instances contains an invalid
  // ModelInstanceIndex.
  // @throws std::exception if body_indexes contains an invalid BodyIndex.
  SpatialMomentum<T> CalcBodiesSpatialMomentumInWorldAboutWo(
      const systems::Context<T>& context,
      const std::vector<BodyIndex>& body_indexes) const;

  // Helper method to access the mobilizer of a free body (that is, a
  // body connected to its parent by a 6-dof joint). If `body` is free, this
  // method will return the Mobilizer for the body, which will
  // be one of the 6-dof mobilizers. Otherwise this method will throw
  // std::exception. The Mobilizer API supports the ability to set the
  // mobilizer's state including both pose and spatial velocity; 6-dof
  // mobilizers have the unique property of being able to represent _any_ pose
  // and spatial velocity.
  // @throws std::exception if `body` is not a free body.
  // @throws std::exception if called pre-finalize.
  // @pre `body` is not World
  const Mobilizer<T>& GetFreeBodyMobilizerOrThrow(
      const RigidBody<T>& body) const;

  // Helper for ThrowDefaultMassInertiaError(): takes a terminal Link or a set
  // of Links forming a terminal composite, and complains if its default
  // mass properties are inappropriate for its active mobilizer.
  void ThrowIfTerminalBodyHasBadDefaultMassProperties(
      const std::vector<BodyIndex>& link_composites,
      MobodIndex active_mobilizer_index) const;

  // Evaluates the cache entry stored in context with the spatial inertias
  // M_Bo_W(q) for each body in the system. These will be updated as needed.
  const std::vector<SpatialInertia<T>>& EvalSpatialInertiaInWorldCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalSpatialInertiaInWorldCache(context);
  }

  // Evaluates the cache entry stored in context with the reflected inertia for
  // each degree of freedom in the system. These will be updated as needed.
  const VectorX<T>& EvalReflectedInertiaCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalReflectedInertiaCache(context);
  }

  // Evaluates the cache entry stored in context with the joint damping for each
  // degree of freedom in the system. These will be updated as needed.
  const VectorX<T>& EvalJointDampingCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalJointDampingCache(context);
  }

  const std::vector<SpatialInertia<T>>& EvalCompositeBodyInertiaInWorldCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalCompositeBodyInertiaInWorldCache(context);
  }

  // Evaluates the cache entry stored in context with the bias term
  // Fb_Bo_W(q, v) for each body. These will be updated as needed.
  const std::vector<SpatialForce<T>>& EvalDynamicBiasCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalDynamicBiasCache(context);
  }

  // See CalcSpatialAccelerationBiasCache() for details.
  const std::vector<SpatialAcceleration<T>>& EvalSpatialAccelerationBiasCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalSpatialAccelerationBiasCache(context);
  }

  // See CalcArticulatedBodyForceBiasCache() for details.
  const std::vector<SpatialForce<T>>& EvalArticulatedBodyForceBiasCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalArticulatedBodyForceBiasCache(context);
  }

  // Given the state of this model in `context` and a known vector
  // of generalized accelerations `known_vdot`, this method computes the
  // spatial acceleration `A_WB` for each body as measured and expressed in the
  // world frame W.
  //
  // Iff `ignore_velocities = true` velocity values stored in `context` are
  // ignored and are assumed to be zero. Therefore, Velocity kinematics and
  // velocity dependent terms that become zero (such as bias terms) are not
  // computed to avoid unnecessary work.
  void CalcSpatialAccelerationsFromVdot(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      bool ignore_velocities,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  // Helper method for Jacobian methods, namely CalcJacobianAngularVelocity(),
  // CalcJacobianTranslationalVelocity(), and CalcJacobianSpatialVelocity().
  // @param[in] context The state of the multibody system.
  // @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  // JacobianWrtVariable::kV, indicating whether Jacobians Js_w_WF and Js_v_WFpi
  // are partial derivatives with respect to 𝑠 = q̇ (time-derivatives of
  // generalized positions) or with respect to 𝑠 = v (generalized velocities).
  // If 𝑠 = q̇, `w_WF = Js_w_WF ⋅ q̇`  and  `v_WFpi = Js_v_WFpi ⋅ q̇`.
  // If 𝑠 = v, `w_WF = Js_w_WF ⋅ v`  and  `v_WFpi = Js_v_WFpi ⋅ v`.
  // @param[in] frame_F The frame on which point Fpi is fixed (e.g., welded).
  // @param[in] p_WoFpi_W A position vector or list of position vectors from
  // Wo (world frame W's origin) to points Fpi (regarded as fixed/welded to F),
  // where each position vector is expressed in frame W.
  // @param[out] Js_w_WF_W Frame B's angular velocity Jacobian in frame W with
  // respect to speeds 𝑠 (which is either q̇ or v), expressed in frame W.
  // `Js_w_WF_W` is either nullptr or a `3 x n` matrix, where n is the number of
  // elements in 𝑠.
  // @param[out] Js_v_WFpi_W Point Fpi's velocity Jacobian in world frame W with
  // respect to speeds 𝑠 (which is either q̇ or v), expressed in frame W.
  // `Js_v_WFpi_W` is either nullptr or a `3*p x n` matrix, where p is the
  // number of points in Fpi and n is the number of elements in 𝑠.
  // @throws std::exception if any of the following occurs:
  // - The size of `p_WoFpi_W' differs from `3 x n`.
  // - `Js_w_WF_W` and `Js_v_WFpi_W` are both nullptr.
  // - `Js_w_WF_W` is not nullptr and its size differs from `3 x n`.
  // - `Js_v_WFpi_W` is not nullptr and its size differs from `3*p x n`.
  void CalcJacobianAngularAndOrTranslationalVelocityInWorld(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_F, const Eigen::Ref<const Matrix3X<T>>& p_WoFpi_W,
      EigenPtr<Matrix3X<T>> Js_w_WF_W, EigenPtr<MatrixX<T>> Js_v_WFpi_W) const;

  // Helper method for CalcJacobianTranslationalVelocity().
  // @param[in] context The state of the multibody system.
  // @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  // JacobianWrtVariable::kV, indicating whether the Jacobian `Js_v_ABi` is
  // partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  // positions) or with respect to 𝑠 = v (generalized velocities).
  // @param[in] frame_B The frame on which point Bi is fixed (e.g., welded).
  // @param[in] p_WoBi_W A position vector or list of p position vectors from
  // Wo (world frame W's origin) to points Bi (regarded as fixed to B),
  // where each position vector is expressed in frame W.
  // @param[in] frame_A The frame that measures `v_ABi` (Bi's velocity in A).
  // @param[out] Js_v_ABi_W Point Bi's velocity Jacobian in frame A with
  // respect to speeds 𝑠 (which is either q̇ or v), expressed in world frame W.
  // `Js_v_ABi_W` is a `3*p x n` matrix, where p is the number of points Bi and
  // n is the number of elements in 𝑠.
  // @throws std::exception if `Js_v_ABi_W` is nullptr or not sized `3*p x n`.
  void CalcJacobianTranslationalVelocityHelper(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Matrix3X<T>>& p_WoBi_W,
      const Frame<T>& frame_A, EigenPtr<MatrixX<T>> Js_v_ABi_W) const;

  // Helper method to apply forces due to damping at the joints.
  // MultibodyTree treats damping forces separately from other ForceElement
  // forces for a quick simple solution. This allows clients of MBT (namely MBP)
  // to implement their own customized (implicit) time stepping schemes.
  // TODO(amcastro-tri): Consider updating ForceElement to also compute a
  //  Jacobian for general force models. That would allow us to implement
  //  implicit schemes for any forces using a more general infrastructure rather
  //  than having to deal with damping in a special way.
  void AddJointDampingForces(const systems::Context<T>& context,
                             MultibodyForces<T>* forces) const;

  void CreateBodyNode(MobodIndex mobod_index);

  void FinalizeModelInstances();

  // Helper method to create a clone of `frame` and add it to `this` tree.
  template <typename FromScalar>
  Frame<T>* CloneFrameAndAdd(const Frame<FromScalar>& frame);

  // Helper method to create a clone of `body` and add it to `this` tree.
  // Because this method is only invoked in a controlled manner from within
  // CloneToScalar(), it is guaranteed that the cloned body in this variant's
  // `rigid_bodies_` will occupy the same position as its corresponding
  // RigidBody in the source variant `body`.
  template <typename FromScalar>
  RigidBody<T>* CloneBodyAndAdd(const RigidBody<FromScalar>& body);

  // Helper method to create a clone of `mobilizer` and add it to `this` tree.
  template <typename FromScalar>
  Mobilizer<T>* CloneMobilizerAndAdd(const Mobilizer<FromScalar>& mobilizer);

  // Helper method to create a clone of `force_element` and add it to `this`
  // tree.
  template <typename FromScalar>
  void CloneForceElementAndAdd(const ForceElement<FromScalar>& force_element);

  // Helper method to create a clone of `joint` and add it to `this` tree.
  template <typename FromScalar>
  Joint<T>* CloneJointAndAdd(const Joint<FromScalar>& joint);

  // Helper method to create a clone of `actuator` (which is templated on
  // FromScalar) and add it to `this` tree (templated on T).
  template <typename FromScalar>
  void CloneActuatorAndAdd(const JointActuator<FromScalar>& actuator);

  // If there exists a unique base body (a body whose parent is the world body)
  // in the model given by `model_instance`, return the index of that body.
  // Otherwise return std::nullopt. In particular, if the given `model_instance`
  // is the world model instance, return `std::nullopt`.
  // @throws std::exception if `model_instance` is not valid.
  std::optional<BodyIndex> MaybeGetUniqueBaseBodyIndex(
      ModelInstanceIndex model_instance) const;

  // Helper function for GetDefaultFreeBodyPose().
  std::pair<Eigen::Quaternion<double>, Vector3<double>>
  GetDefaultFreeBodyPoseAsQuaternionVec3Pair(const RigidBody<T>& body) const;

  // TODO(amcastro-tri): In future PR's adding MBT computational methods, write
  //  a method that verifies the state of the topology with a signature similar
  //  to RoadGeometry::CheckHasRightSizeForModel().

  // These objects are defined via MultibodyPlant and are thus user-visible.
  const RigidBody<T>* world_rigid_body_{nullptr};
  // When we need to look up elements by name, we'll use an ElementCollection.
  // Otherwise, we'll just use a plain vector.
  ElementCollection<T, RigidBody, BodyIndex> rigid_bodies_;
  ElementCollection<T, Frame, FrameIndex> frames_;
  ElementCollection<T, Joint, JointIndex> joints_;
  std::vector<std::unique_ptr<ForceElement<T>>> force_elements_;
  ElementCollection<T, JointActuator, JointActuatorIndex> actuators_;

  // This is the internal representation of user-defined model instances.
  ElementCollection<T, internal::ModelInstance, ModelInstanceIndex>
      model_instances_;

  // A graph representing the user-specified Link/Joint topology of the
  // multibody plant, and containing the chosen SpanningForest we use
  // for computation.
  internal::LinkJointGraph link_joint_graph_;

  // These are internal data structures generated to implement the model.
  // Both arrays are the same length and are indexed by MobodIndex.
  std::vector<std::unique_ptr<Mobilizer<T>>> mobilizers_;
  std::vector<std::unique_ptr<internal::BodyNode<T>>> body_nodes_;

  // The gravity field force element.
  UniformGravityFieldElement<T>* gravity_field_{nullptr};

  // Mobilized body indexes (for Mobods, BodyNodes, Mobilizers) are ordered
  // by level (a.k.a depth) in the SpanningForest, starting with 0 for World.
  // body_node_levels_[i] contains the list of all MobodIndexes at level i.
  std::vector<std::vector<MobodIndex>> body_node_levels_;

  // Joint to Mobilizer map, of size num_joints(). For a joint with index
  // joint_index, mobilizer_index = joint_to_mobilizer_[joint_index] maps to the
  // mobilizer model of the joint, or an invalid index if the joint is modeled
  // with constraints instead.
  std::unordered_map<JointIndex, MobodIndex> joint_to_mobilizer_;

  // Maps the default body poses of all floating bodies AND bodies touched by
  // MultibodyPlant::SetDefaultFreeBodyPose(). During Finalize(), the default
  // pose of a floating body is converted to the joint index of the floating
  // joint connecting the world and the body. Post-finalize and the default
  // poses of such floating bodies can (and should) be retrieved via the joints'
  // default positions. The poses are stored as a quaternion-translation pair to
  // match the default positions stored in the quaternion floating joints
  // without any numerical conversions and thereby avoiding roundoff errors and
  // surprising discrepancies pre and post finalize.
  std::unordered_map<
      BodyIndex, std::variant<JointIndex, std::pair<Eigen::Quaternion<double>,
                                                    Vector3<double>>>>
      default_body_poses_;

  MultibodyTreeTopology topology_;

  // Back pointer to the owning MultibodyTreeSystem.
  const MultibodyTreeSystem<T>* tree_system_{};

  // TODO(sherm1) This should be split into separate position and velocity
  //  states for efficient use of the cache.
  // The discrete state index for the multibody state if the system is discrete.
  systems::DiscreteStateIndex discrete_state_index_;
};

}  // namespace internal

/// @cond
// Undef macros defined at the top of the file. From the GSG:
// "Exporting macros from headers (i.e. defining them in a header without
// #undefing them before the end of the header) is extremely strongly
// discouraged."
// This will require us to re-define them in the .cc file.
#undef DRAKE_MBT_THROW_IF_FINALIZED
#undef DRAKE_MBT_THROW_IF_NOT_FINALIZED
/// @endcond

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MultibodyTree);
