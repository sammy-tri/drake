#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/model_values.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/system_symbolic_inspector.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/value_checker.h"

namespace drake {
namespace systems {

/// A superclass template that extends System with some convenience utilities
/// that are not applicable to Diagrams.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class LeafSystem : public System<T> {
 public:
  // LeafSystem objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafSystem)

  ~LeafSystem() override {}

  /// Allocates a CompositeEventCollection object for this system.
  /// @sa System::AllocateCompositeEventCollection().
  std::unique_ptr<CompositeEventCollection<T>>
  AllocateCompositeEventCollection() const final;

  /// Shadows System<T>::AllocateContext to provide a more concrete return
  /// type LeafContext<T>.
  std::unique_ptr<LeafContext<T>> AllocateContext() const;

  // =========================================================================
  // Implementations of System<T> methods.

  /// @cond
  // The three methods below are hidden from doxygen, as described in
  // documentation for their corresponding methods in System.
  std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const override;

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const override;

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const override;
  /// @endcond

  std::unique_ptr<ContextBase> DoAllocateContext() const final;

  /// Default implementation: sets all continuous state to the model vector
  /// given in DeclareContinuousState (or zero if no model vector was given) and
  /// discrete states to zero. Overrides must not change the number of state
  /// variables.
  // TODO(sherm/russt): Initialize the discrete state from the model vector
  // pending resolution of #7058.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override;

  /// Default implementation: sets all numeric parameters to the model vector
  /// given to DeclareNumericParameter, or else if no model was provided sets
  /// the numeric parameter to one.  It sets all abstract parameters to the
  /// model value given to DeclareAbstractParameter.  Overrides must not change
  /// the number of parameters.
  void SetDefaultParameters(const Context<T>& context,
                            Parameters<T>* parameters) const override;

  /// Returns the AllocateContinuousState value, which must not be nullptr.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return AllocateContinuousState();
  }

  /// Returns the AllocateDiscreteState value, which must not be nullptr.
  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables()
      const override {
    return AllocateDiscreteState();
  }

  std::multimap<int, int> GetDirectFeedthroughs() const final;

  int get_num_continuous_states() const final {
    int total = num_generalized_positions_ +
                num_generalized_velocities_+
                num_misc_continuous_states_;
    return total;
  }

 protected:
  // Promote so we don't need "this->" in defaults which show up in Doxygen.
  using SystemBase::all_sources_ticket;

  /// Default constructor that declares no inputs, outputs, state, parameters,
  /// events, nor scalar-type conversion support (AutoDiff, etc.).  To enable
  /// AutoDiff support, use the SystemScalarConverter-based constructor.
  LeafSystem();

  /// Constructor that declares no inputs, outputs, state, parameters, or
  /// events, but allows subclasses to declare scalar-type conversion support
  /// (AutoDiff, etc.).
  ///
  /// The scalar-type conversion support will use @p converter.
  /// To enable scalar-type conversion support, pass a `SystemTypeTag<S>{}`
  /// where `S` must be the exact class of `this` being constructed.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  explicit LeafSystem(SystemScalarConverter converter);

  /// Provides a new instance of the leaf context for this system. Derived
  /// leaf systems with custom derived leaf system contexts should override this
  /// to provide a context of the appropriate type. The returned context should
  /// be "empty"; invoked by AllocateContext(), the caller will take the
  /// responsibility to initialize the core LeafContext data. The default
  /// implementation provides a default-constructed `LeafContext<T>`.
  virtual std::unique_ptr<LeafContext<T>> DoMakeLeafContext() const;

  /// Derived classes that impose restrictions on what resources are permitted
  /// should check those restrictions by implementing this. For example, a
  /// derived class might require a single input and single output. Note that
  /// the supplied Context will be complete except that input and output
  /// dependencies on peer and parent subcontexts will not yet have been set up,
  /// so you may not consider them for validation.
  /// The default implementation does nothing.
  virtual void DoValidateAllocatedLeafContext(
      const LeafContext<T>& context) const {
    unused(context);
  }

  // =========================================================================
  // Implementations of System<T> methods.

  T DoCalcWitnessValue(const Context<T>& context,
                       const WitnessFunction<T>& witness_func) const final {
    DRAKE_DEMAND(this == &witness_func.get_system());
    return witness_func.CalcWitnessValue(context);
  }

  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>* event,
      CompositeEventCollection<T>* events) const final;

  /// Computes the next update time based on the configured periodic events, for
  /// scalar types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic. Subclasses that require aperiodic events should override, but
  /// be sure to invoke the parent class implementation at the start of the
  /// override if you want periodic events to continue to be handled.
  ///
  /// @post `time` is set to a value greater than or equal to
  ///       `context.get_time()` on return.
  /// @warning If you override this method, think carefully before setting
  ///          `time` to `context.get_time()` on return, which can inadvertently
  ///          cause simulations of systems derived from %LeafSystem to loop
  ///          interminably. Such a loop will occur if, for example, the
  ///          event(s) does not modify the state.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            CompositeEventCollection<T>* events,
                            T* time) const override;

  /// Emits a graphviz fragment for this System. Leaf systems are visualized as
  /// records. For instance, a leaf system with 2 inputs and 1 output is:
  ///
  /// @verbatim
  /// 123456 [shape= record, label="name | {<u0> 0 |<y0> 0} | {<u1> 1 | }"];
  /// @endverbatim
  ///
  /// which looks like:
  ///
  /// @verbatim
  /// +------------+----+
  /// | name  | u0 | u1 |
  /// |       | y0 |    |
  /// +-------+----+----+
  /// @endverbatim
  void GetGraphvizFragment(int max_depth,
                           std::stringstream* dot) const override;

  void GetGraphvizInputPortToken(const InputPort<T>& port,
                                 int max_depth,
                                 std::stringstream *dot) const final;

  void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                  int max_depth,
                                  std::stringstream *dot) const final;

  // =========================================================================
  // New methods for subclasses to override

  /// Returns a ContinuousState used to implement both CreateDefaultContext and
  /// AllocateTimeDerivatives. Allocates the state configured with
  /// DeclareContinuousState, or none by default. Systems with continuous state
  /// variables may override (not recommended), but must ensure the
  /// ContinuousState vector is a subclass of BasicVector.
  virtual std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const;

  /// Reserves the discrete state as required by CreateDefaultContext. By
  /// default, clones the model values as provided in DeclareDiscreteState()
  /// calls. Alternatively, systems with discrete state can override this
  /// method (not recommended).
  virtual std::unique_ptr<DiscreteValues<T>> AllocateDiscreteState() const;

  /// Reserves the abstract state as required by CreateDefaultContext. By
  /// default, it clones the abstract states declared through
  /// DeclareAbstractState() calls. Derived systems may override for
  /// different behaviors (not recommended).
  virtual std::unique_ptr<AbstractValues> AllocateAbstractState() const;

  /// Reserves the parameters as required by CreateDefaultContext.  The default
  /// implementation in this class clones the model_vector for all parameters
  /// declared via DeclareNumericParameter(), as well as the model value for all
  /// parameters declared via DeclareAbstractParameter().  Subclasses can
  /// override this method if the default behavior is not sufficient.
  virtual std::unique_ptr<Parameters<T>> AllocateParameters() const;

  /// Returns true if there is direct-feedthrough from the given @p input_port
  /// to the given @p output_port, false if there is not direct-feedthrough, or
  /// nullopt if unknown (in which case SystemSymbolicInspector will attempt to
  /// measure the feedthrough using symbolic form).
  ///
  /// By default, %LeafSystem assumes there is direct feedthrough of values
  /// from every input to every output.
  /// This is a conservative assumption that ensures we detect and can prevent
  /// the formation of algebraic loops (implicit computations) in system
  /// Diagrams. Systems which do not have direct feedthrough may override that
  /// assumption in two ways:
  ///
  /// - Override DoToSymbolic, allowing %LeafSystem to infer the sparsity
  ///   from the symbolic equations. This method is typically preferred for
  ///   systems that have a symbolic form, but should be avoided in certain
  ///   corner cases where fully descriptive symbolic analysis is impossible,
  ///   e.g., when the symbolic form depends on C++ native conditionals. For
  ///   additional discussion, consult the documentation for
  ///   SystemSymbolicInspector.
  ///
  /// - Override this function directly, reporting manual sparsity. This method
  ///   is recommended when DoToSymbolic has not been implemented, or when
  ///   creating the symbolic form is too computationally expensive, or when its
  ///   output is not fully descriptive, as discussed above. Manually configured
  ///   sparsity must be conservative: if there is any Context for which an
  ///   input port is direct-feedthrough to an output port, this function must
  ///   return either true or nullopt for those two ports.
  virtual optional<bool> DoHasDirectFeedthrough(
      int input_port, int output_port) const {
    unused(input_port, output_port);
    return nullopt;
  }

  // =========================================================================
  // New methods for subclasses to use

  /// Declares a numeric parameter using the given @p model_vector.  This is
  /// the best way to declare LeafSystem numeric parameters.  LeafSystem's
  /// default implementation of AllocateParameters uses model_vector.Clone(),
  /// and the default implementation of SetDefaultParameters() will reset
  /// parameters to their model vectors.  If the @p model_vector declares any
  /// VectorBase::CalcInequalityConstraint() constraints, they will be
  /// re-declared as inequality constraints on this system (see
  /// DeclareInequalityConstraint()).  Returns the index of the new parameter.
  int DeclareNumericParameter(const BasicVector<T>& model_vector);

  /// Extracts the numeric parameters of type U from the @p context at @p index.
  /// Asserts if the context is not a LeafContext, or if it does not have a
  /// vector-valued parameter of type U at @p index.
  template <template <typename> class U = BasicVector>
  const U<T>& GetNumericParameter(const Context<T>& context, int index) const {
    static_assert(std::is_base_of<BasicVector<T>, U<T>>::value,
                  "U must be a subclass of BasicVector.");
    const auto& leaf_context =
        dynamic_cast<const systems::LeafContext<T>&>(context);
    const auto* const params =
        dynamic_cast<const U<T>*>(&leaf_context.get_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /// Extracts the numeric parameters of type U from the @p context at @p index.
  /// Asserts if the context is not a LeafContext, or if it does not have a
  /// vector-valued parameter of type U at @p index.
  template <template <typename> class U = BasicVector>
  U<T>& GetMutableNumericParameter(Context<T>* context, int index) const {
    static_assert(std::is_base_of<BasicVector<T>, U<T>>::value,
                  "U must be a subclass of BasicVector.");
    auto* leaf_context = dynamic_cast<systems::LeafContext<T>*>(context);
    DRAKE_ASSERT(leaf_context != nullptr);
    auto* const params = dynamic_cast<U<T>*>(
        &leaf_context->get_mutable_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /// Declares an abstract parameter using the given @p model_value.  This is
  /// the best way to declare LeafSystem abstract parameters.  LeafSystem's
  /// default implementation of AllocateParameters uses model_value.Clone(), and
  /// the default implementation of SetDefaultParameters() will reset parameters
  /// to their model values.  Returns the index of the new parameter.
  int DeclareAbstractParameter(const AbstractValue& model_value);

  // =========================================================================
  /// @anchor declare_periodic_events
  /// @name                  Declare periodic events
  /// Methods in the this group declare that this System has an event that
  /// is triggered periodically. The first periodic trigger will occur at
  /// t = `offset_sec`, and it will recur at every `period_sec` thereafter.
  /// Several signatures are provided to allow for a general Event object to be
  /// triggered or for simpler class member functions to be invoked instead.
  ///
  /// Reaching a designated time causes a periodic event to be dispatched
  /// to one of the three available types of event dispatcher: publish (read
  /// only), discrete update, and unrestricted update.
  ///
  /// @note If you want to generate timed events that are _not_ periodic
  /// (timers, alarms, etc.), overload DoCalcNextUpdateTime() rather than using
  /// the methods in this section.
  ///
  /// Template arguments to these methods are inferred from the argument lists
  /// and need not be specified explicitly.
  /// @pre `period_sec` > 0 and `offset_sec` ≥ 0.
  //@{

  /// Declares that a Publish event should occur periodically and that it should
  /// invoke the given event handler method. The handler should be a class
  /// member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyPublish(const Context<T>&) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_periodic_events "Declare periodic events" for more
  /// information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `publish` must not be null.
  ///
  /// @see DeclarePeriodicDiscreteUpdateEvent()
  /// @see DeclarePeriodicUnrestrictedUpdateEvent()
  /// @see DeclarePeriodicEvent()
  template <class MySystem>
  void DeclarePeriodicPublishEvent(
      double period_sec, double offset_sec,
      EventStatus (MySystem::*publish)(const Context<T>&) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(publish != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        PublishEvent<T>(TriggerType::kPeriodic, [this_ptr, publish](
                                                    const Context<T>& context,
                                                    const PublishEvent<T>&) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*publish)(context);  // Ignore return status for now.
        }));
  }

  /// Declares that a DiscreteUpdate event should occur periodically and that it
  /// should invoke the given event handler method. The handler should be a
  /// class member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyUpdate(const Context<T>&,
  ///                                  DiscreteValues<T>*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_periodic_events "Declare periodic events" for more
  /// information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `update` must not be null.
  ///
  /// @see DeclarePeriodicPublishEvent()
  /// @see DeclarePeriodicUnrestrictedUpdateEvent()
  /// @see DeclarePeriodicEvent()
  template <class MySystem>
  void DeclarePeriodicDiscreteUpdateEvent(
      double period_sec, double offset_sec,
      EventStatus (MySystem::*update)(const Context<T>&, DiscreteValues<T>*)
          const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        DiscreteUpdateEvent<T>(TriggerType::kPeriodic,
                               [this_ptr, update](const Context<T>& context,
                                                  const DiscreteUpdateEvent<T>&,
                                                  DiscreteValues<T>* xd) {
                                 // TODO(sherm1) Forward the return status.
                                 (this_ptr->*update)(
                                     context,
                                     &*xd);  // Ignore return status for now.
                               }));
  }

  /// Declares that an UnrestrictedUpdate event should occur periodically and
  /// that it should invoke the given event handler method. The handler should
  /// be a class member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyUpdate(const Context<T>&,
  ///                                  State<T>*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_periodic_events "Declare periodic events" for more
  /// information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `update` must not be null.
  ///
  /// @see DeclarePeriodicPublishEvent()
  /// @see DeclarePeriodicDiscreteUpdateEvent()
  /// @see DeclarePeriodicEvent()
  template <class MySystem>
  void DeclarePeriodicUnrestrictedUpdateEvent(
      double period_sec, double offset_sec,
      EventStatus (MySystem::*update)(const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        UnrestrictedUpdateEvent<T>(
            TriggerType::kPeriodic,
            [this_ptr, update](const Context<T>& context,
                               const UnrestrictedUpdateEvent<T>&, State<T>* x) {
              // TODO(sherm1) Forward the return status.
              (this_ptr->*update)(context,
                                  &*x);  // Ignore return status for now.
            }));
  }

  /// (Advanced) Declares that a particular Event object should be dispatched
  /// periodically. This is the most general form for declaring periodic events
  /// and most users should use one of the other methods in this group instead.
  ///
  /// @see DeclarePeriodicPublishEvent()
  /// @see DeclarePeriodicDiscreteUpdateEvent()
  /// @see DeclarePeriodicUnrestrictedUpdateEvent()
  ///
  /// See @ref declare_periodic_events "Declare periodic events" for more
  /// information.
  ///
  /// Depending on the type of `event`, when triggered it will be passed to
  /// the Publish, DiscreteUpdate, or UnrestrictedUpdate event dispatcher. If
  /// the `event` object contains a handler function, Drake's default
  /// dispatchers will invoke that handler. If not, then no further action is
  /// taken. Thus an `event` with no handler has no effect unless its dispatcher
  /// has been overridden. We strongly recommend that you _do not_ override the
  /// dispatcher and instead _do_ supply a handler.
  ///
  /// The given `event` object is deep-copied (cloned), and the copy is stored
  /// internally so you do not need to keep the object around after this call.
  ///
  /// @pre `event`'s associated trigger type must be TriggerType::kUnknown or
  /// already set to TriggerType::kPeriodic.
  template <typename EventType>
  void DeclarePeriodicEvent(double period_sec, double offset_sec,
                            const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
        event.get_trigger_type() == TriggerType::kPeriodic);
    PeriodicEventData periodic_data;
    periodic_data.set_period_sec(period_sec);
    periodic_data.set_offset_sec(offset_sec);
    auto event_copy = event.Clone();
    event_copy->set_trigger_type(TriggerType::kPeriodic);
    periodic_events_.emplace_back(
        std::make_pair(periodic_data, std::move(event_copy)));
  }

  /// (To be deprecated) Declares a periodic publish event that invokes the
  /// Publish() dispatcher but does not provide a handler function. This does
  /// guarantee that a Simulator step will end exactly at the publish time,
  /// but otherwise has no effect unless the DoPublish() dispatcher has been
  /// overloaded (not recommended).
  void DeclarePeriodicPublish(double period_sec, double offset_sec = 0);

  /// (To be deprecated) Declares a periodic discrete update event that invokes
  /// the DiscreteUpdate() dispatcher but does not provide a handler
  /// function. This does guarantee that a Simulator step will end exactly at
  /// the update time, but otherwise has no effect unless the
  /// DoDiscreteUpdate() dispatcher has been overloaded (not recommended).
  void DeclarePeriodicDiscreteUpdate(double period_sec, double offset_sec = 0);

  /// (To be deprecated) Declares a periodic unrestricted update event that
  /// invokes the UnrestrictedUpdate() dispatcher but does not provide a handler
  /// function. This does guarantee that a Simulator step will end exactly at
  /// the update time, but otherwise has no effect unless the
  /// DoUnrestrictedUpdate() dispatcher has been overloaded (not recommended).
  void DeclarePeriodicUnrestrictedUpdate(double period_sec,
                                         double offset_sec = 0);
  //@}

  // =========================================================================
  /// @anchor declare_per-step_events
  /// @name                 Declare per-step events
  /// These methods are used to declare events that are triggered whenever the
  /// Drake Simulator::StepTo() method takes a substep that advances the
  /// simulated trajectory. Note that each call to StepTo() typically generates
  /// many trajectory-advancing substeps of varying time intervals; per-step
  /// events are triggered for each of those substeps.
  ///
  /// Per-step events are useful for taking discrete action at every point of a
  /// simulated trajectory (generally spaced irregularly in time) without
  /// missing anything. For example, per-step events can be used to implement
  /// a high-accuracy signal delay by maintaining a buffer of past signal
  /// values, updated at each step. Because the steps are smaller in regions
  /// of rapid change, the interpolated signal retains the accuracy provided
  /// by the denser sampling. A periodic sampling would produce less-accurate
  /// interpolations.
  ///
  /// As with any Drake event trigger type, a per-step event is
  /// dispatched to one of the three available types of event dispatcher:
  /// publish (read only), discrete state update, and unrestricted state update.
  /// Several signatures are provided below to allow for a general Event object
  /// to be triggered, or simpler class member functions to be invoked instead.
  ///
  /// Per-step events are issued as follows: First, the Simulator::Initialize()
  /// method queries and records the set of declared per-step events, which set
  /// does not change during a simulation. Then every StepTo() internal substep
  /// dispatches unrestricted and discrete update events at the start of the
  /// step, and dispatches publish events at the end of the step (that is,
  /// after time advances). No per-step event is triggered during the
  /// Initialize() call.
  ///
  /// Template arguments to these methods are inferred from the argument lists
  /// and need not be specified explicitly.
  //@{

  /// Declares that a Publish event should occur every step and that it should
  /// invoke the given event handler method. The handler should be a class
  /// member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyPublish(const Context<T>&) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_per-step_events "Declare per-step events" for more
  /// information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `publish` must not be null.
  ///
  /// @see DeclarePerStepDiscreteUpdateEvent()
  /// @see DeclarePerStepUnrestrictedUpdateEvent()
  /// @see DeclarePerStepEvent()
  template <class MySystem>
  void DeclarePerStepPublishEvent(
      EventStatus (MySystem::*publish)(const Context<T>&) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(publish != nullptr);

    DeclarePerStepEvent<PublishEvent<T>>(PublishEvent<T>(
        TriggerType::kPerStep,
        [this_ptr, publish](const Context<T>& context, const PublishEvent<T>&) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*publish)(context);  // Ignore return status for now.
        }));
  }

  /// Declares that a DiscreteUpdate event should occur every step and that it
  /// should invoke the given event handler method. The handler should be a
  /// class member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyUpdate(const Context<T>&,
  ///                                  DiscreteValues<T>*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_per-step_events "Declare per-step events" for more
  /// information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `update` must not be null.
  ///
  /// @see DeclarePerStepPublishEvent()
  /// @see DeclarePerStepUnrestrictedUpdateEvent()
  /// @see DeclarePerStepEvent()
  template <class MySystem>
  void DeclarePerStepDiscreteUpdateEvent(EventStatus (MySystem::*update)(
      const Context<T>&, DiscreteValues<T>*) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclarePerStepEvent(DiscreteUpdateEvent<T>(
        TriggerType::kPerStep, [this_ptr, update](const Context<T>& context,
                                                  const DiscreteUpdateEvent<T>&,
                                                  DiscreteValues<T>* xd) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(context,
                              &*xd);  // Ignore return status for now.
        }));
  }

  /// Declares that an UnrestrictedUpdate event should occur every step and that
  /// it should invoke the given event handler method. The handler should be a
  /// class member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyUpdate(const Context<T>&,
  ///                                  State<T>*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_per-step_events "Declare per-step events" for more
  /// information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `update` must not be null.
  ///
  /// @see DeclarePerStepPublishEvent()
  /// @see DeclarePerStepDiscreteUpdateEvent()
  /// @see DeclarePerStepEvent()
  template <class MySystem>
  void DeclarePerStepUnrestrictedUpdateEvent(
      EventStatus (MySystem::*update)(const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclarePerStepEvent(UnrestrictedUpdateEvent<T>(
        TriggerType::kPerStep,
        [this_ptr, update](const Context<T>& context,
                           const UnrestrictedUpdateEvent<T>&, State<T>* x) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(context,
                              &*x);  // Ignore return status for now.
        }));
  }

  /// (Advanced) Declares that a particular Event object should be dispatched at
  /// every simulation step. This is the most general form for declaring
  /// per-step events and most users should use one of the other methods in this
  /// group instead.
  ///
  /// @see DeclarePerStepPublishEvent()
  /// @see DeclarePerStepDiscreteUpdateEvent()
  /// @see DeclarePerStepUnrestrictedUpdateEvent()
  ///
  /// See @ref declare_per-step_events "Declare per-step events" for more
  /// information.
  ///
  /// Depending on the type of `event`, at each step it will be passed to
  /// the Publish, DiscreteUpdate, or UnrestrictedUpdate event dispatcher. If
  /// the `event` object contains a handler function, Drake's default
  /// dispatchers will invoke that handler. If not, then no further action is
  /// taken. Thus an `event` with no handler has no effect unless its dispatcher
  /// has been overridden. We strongly recommend that you _do not_ override the
  /// dispatcher and instead _do_ supply a handler.
  ///
  /// The given `event` object is deep-copied (cloned), and the copy is stored
  /// internally so you do not need to keep the object around after this call.
  ///
  /// @pre `event`'s associated trigger type must be TriggerType::kUnknown or
  /// already set to TriggerType::kPerStep.
  template <typename EventType>
  void DeclarePerStepEvent(const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
        event.get_trigger_type() == TriggerType::kPerStep);
    event.AddToComposite(TriggerType::kPerStep, &per_step_events_);
  }
  //@}

  // =========================================================================
  /// @anchor declare_initialization_events
  /// @name                 Declare initialization events
  /// These methods are used to declare events that occur when the Drake
  /// Simulator::Initialize() method is invoked.
  ///
  /// During initialization, unrestricted update events are performed first for
  /// the whole Diagram, then discrete update events for the whole Diagram.
  /// Timed update events are not performed during initialization, even if they
  /// are scheduled for the initial time; in that case they are done at the
  /// beginning of the first Simulator::StepTo() call. On the other hand,
  /// initialization publish events and timed publish events that are scheduled
  /// for the initial time are dispatched together during initialization.
  /// They are ordered such that each subsystem sees its initialization publish
  /// events before its timed publish events.
  ///
  /// Template arguments to these methods are inferred from the argument lists
  /// and need not be specified explicitly.
  //@{

  /// Declares that a Publish event should occur at initialization and that it
  /// should invoke the given event handler method. The handler should be a
  /// class member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyPublish(const Context<T>&) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_initialization_events "Declare initialization events" for
  /// more information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `publish` must not be null.
  ///
  /// @see DeclareInitializationDiscreteUpdateEvent()
  /// @see DeclareInitializationUnrestrictedUpdateEvent()
  /// @see DeclareInitializationEvent()
  template <class MySystem>
  void DeclareInitializationPublishEvent(
      EventStatus(MySystem::*publish)(const Context<T>&) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(publish != nullptr);

    DeclareInitializationEvent<PublishEvent<T>>(PublishEvent<T>(
        TriggerType::kInitialization,
        [this_ptr, publish](const Context<T>& context,
                            const PublishEvent<T>&) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*publish)(context);  // Ignore return status for now.
        }));
  }

  /// Declares that a DiscreteUpdate event should occur at initialization
  /// and that it should invoke the given event handler method. The handler
  /// should be a class member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyUpdate(const Context<T>&,
  ///                                  DiscreteValues<T>*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_initialization_events "Declare initialization events" for
  /// more information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `update` must not be null.
  ///
  /// @see DeclareInitializationPublishEvent()
  /// @see DeclareInitializationUnrestrictedUpdateEvent()
  /// @see DeclareInitializationEvent()
  template <class MySystem>
  void DeclareInitializationDiscreteUpdateEvent(
      EventStatus(MySystem::*update)
          (const Context<T>&, DiscreteValues<T>*) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclareInitializationEvent(DiscreteUpdateEvent<T>(
        TriggerType::kInitialization,
        [this_ptr, update](const Context<T>& context,
                           const DiscreteUpdateEvent<T>&,
                           DiscreteValues<T>* xd) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(context,
                              &*xd);  // Ignore return status for now.
        }));
  }

  /// Declares that an UnrestrictedUpdate event should occur at initialization
  /// and that it should invoke the given event handler method. The handler
  /// should be a class member function (method) with this signature:
  /// @code
  ///   EventStatus MySystem::MyUpdate(const Context<T>&,
  ///                                  State<T>*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and the method
  /// name is arbitrary.
  ///
  /// See @ref declare_initialization_events "Declare initialization events" for
  /// more information.
  ///
  /// @pre `this` must be dynamic_cast-able to MySystem.
  /// @pre `update` must not be null.
  ///
  /// @see DeclareInitializationPublishEvent()
  /// @see DeclareInitializationDiscreteUpdateEvent()
  /// @see DeclareInitializationEvent()
  template <class MySystem>
  void DeclareInitializationUnrestrictedUpdateEvent(
      EventStatus(MySystem::*update)
          (const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclareInitializationEvent(UnrestrictedUpdateEvent<T>(
        TriggerType::kInitialization,
        [this_ptr, update](const Context<T>& context,
                           const UnrestrictedUpdateEvent<T>&, State<T>* x) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(context,
                              &*x);  // Ignore return status for now.
        }));
  }

  /// (Advanced) Declares that a particular Event object should be dispatched at
  /// initialization. This is the most general form for declaring initialization
  /// events and most users should use one of the other methods in this group
  /// instead.
  ///
  /// @see DeclareInitializationPublishEvent()
  /// @see DeclareInitializationDiscreteUpdateEvent()
  /// @see DeclareInitializationUnrestrictedUpdate()
  ///
  /// See @ref declare_initialization_events "Declare initialization events" for
  /// more information.
  ///
  /// Depending on the type of `event`, on initialization it will be passed to
  /// the Publish, DiscreteUpdate, or UnrestrictedUpdate event dispatcher. If
  /// the `event` object contains a handler function, Drake's default
  /// dispatchers will invoke that handler. If not, then no further action is
  /// taken. Thus an `event` with no handler has no effect unless its dispatcher
  /// has been overridden. We strongly recommend that you _do not_ override the
  /// dispatcher and instead _do_ supply a handler.
  ///
  /// The given `event` object is deep-copied (cloned), and the copy is stored
  /// internally so you do not need to keep the object around after this call.
  ///
  /// @pre `event`'s associated trigger type must be TriggerType::kUnknown or
  /// already set to TriggerType::kInitialization.
  template <typename EventType>
  void DeclareInitializationEvent(const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
        event.get_trigger_type() == TriggerType::kInitialization);
    event.AddToComposite(TriggerType::kInitialization, &initialization_events_);
  }
  //@}

  /// @name          Declare continuous state variables
  /// Continuous state consists of up to three kinds of variables: generalized
  /// coordinates q, generalized velocities v, and miscellaneous continuous
  /// variables z. Methods in this section provide different ways to declare
  /// these, and offer the ability to provide a `model_vector` to specify the
  /// initial values for these variables. Model values are useful when you want
  /// the values of these variables in a default Context to be something other
  /// than zero.
  ///
  /// If multiple calls are made to DeclareContinuousState() methods, only the
  /// last call has any effect. Also, these methods have no effect if
  /// AllocateContinuousState() is overridden (not recommended).
  //@{

  /// Declares that this System should reserve continuous state with
  /// @p num_state_variables state variables, which have no second-order
  /// structure.
  void DeclareContinuousState(int num_state_variables) {
    const int num_q = 0, num_v = 0;
    DeclareContinuousState(num_q, num_v, num_state_variables);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables.
  void DeclareContinuousState(int num_q, int num_v, int num_z) {
    const int n = num_q + num_v + num_z;
    DeclareContinuousState(BasicVector<T>(VectorX<T>::Zero(n)), num_q, num_v,
                           num_z);
  }

  /// Declares that this System should reserve continuous state with
  /// @p model_vector.size() miscellaneous state variables, stored in a
  /// vector cloned from @p model_vector.
  void DeclareContinuousState(const BasicVector<T>& model_vector) {
    const int num_q = 0, num_v = 0;
    const int num_z = model_vector.size();
    DeclareContinuousState(model_vector, num_q, num_v, num_z);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables, stored in a vector cloned from
  /// @p model_vector. Aborts if @p model_vector has the wrong size. If the
  /// @p model_vector declares any VectorBase::CalcInequalityConstraint()
  /// constraints, they will be re-declared as inequality constraints on this
  /// system (see DeclareInequalityConstraint()).
  void DeclareContinuousState(const BasicVector<T>& model_vector, int num_q,
                              int num_v, int num_z);

  //@}

  /// @name            Declare discrete state variables
  /// Discrete state consists of any number of discrete state "groups", each
  /// of which is a vector of discrete state variables. Methods in this section
  /// provide different ways to declare these, and offer the ability to provide
  /// a `model_vector` to specify the initial values for each group of
  /// variables. Model values are useful when you want the values of
  /// these variables in a default Context to be something other than zero.
  ///
  /// Each call to a DeclareDiscreteState() method produces another
  /// discrete state group, and the group index is returned. These methods have
  /// no effect if AllocateDiscreteState() is overridden (not recommended).
  //@{

  /// Declares a discrete state group with @p model_vector.size() state
  /// variables, stored in a vector cloned from @p model_vector (preserving the
  /// concrete type and value).
  DiscreteStateIndex DeclareDiscreteState(const BasicVector<T>& model_vector);

  /// Declares a discrete state group with @p vector.size() state variables,
  /// stored in a BasicVector initialized with the contents of @p vector.
  DiscreteStateIndex DeclareDiscreteState(
      const Eigen::Ref<const VectorX<T>>& vector) {
    return DeclareDiscreteState(BasicVector<T>(vector));
  }

  /// Declares a discrete state group with @p num_state_variables state
  /// variables, stored in a BasicVector initialized to be all-zero. If you want
  /// non-zero initial values, use an alternate DeclareDiscreteState() signature
  /// that accepts a `model_vector` parameter.
  /// @pre `num_state_variables` must be non-negative.
  DiscreteStateIndex DeclareDiscreteState(int num_state_variables) {
    DRAKE_DEMAND(num_state_variables >= 0);
    return DeclareDiscreteState(VectorX<T>::Zero(num_state_variables));
  }
  //@}

  /// @name            Declare abstract state variables
  /// Abstract state consists of any number of arbitrarily-typed variables, each
  /// represented by an AbstractValue. Each call to the DeclareAbstractState()
  /// method produces another abstract state variable, and the abstract state
  /// variable index is returned. This method has no effect if
  /// AllocateAbstractState() is overridden (not recommended).
  //@{

  /// Declares an abstract state.
  /// @param abstract_state The abstract state, its ownership is transferred.
  /// @return index of the declared abstract state.
  AbstractStateIndex DeclareAbstractState(
      std::unique_ptr<AbstractValue> abstract_state);
  //@}

  // =========================================================================
  /// @name                    Declare input ports
  /// Methods in this section are used by derived classes to declare their
  /// output ports, which may be vector valued or abstract valued.
  ///
  /// You should normally provide a meaningful name for any input port you
  /// create. Names must be unique for this system (passing in a duplicate
  /// name will throw std::logic_error). However, if you specify
  /// kUseDefaultName as the name, then a default name of e.g. "u2", where 2
  /// is the input port number will be provided. An empty name is not
  /// permitted.
  //@{

  /// Declares a vector-valued input port using the given @p model_vector.
  /// This is the best way to declare LeafSystem input ports that require
  /// subclasses of BasicVector.  The port's size and type will be the same as
  /// model_vector. If the port is intended to model a random noise or
  /// disturbance input, @p random_type can (optionally) be used to label it
  /// as such.  If the @p model_vector declares any
  /// VectorBase::CalcInequalityConstraint() constraints, they will be
  /// re-declared as inequality constraints on this system (see
  /// DeclareInequalityConstraint()).
  ///
  /// @see System::DeclareInputPort() for more information.
  const InputPort<T>& DeclareVectorInputPort(
      variant<std::string, UseDefaultName> name,
      const BasicVector<T>& model_vector,
      optional<RandomDistribution> random_type = nullopt);

  // Avoid shadowing out the no-arg DeclareAbstractInputPort().  (This line
  // should be removed when the deprecated base class methods disappear.)
  using System<T>::DeclareAbstractInputPort;

  /// Declares an abstract-valued input port using the given @p model_value.
  /// This is the best way to declare LeafSystem abstract input ports.
  ///
  /// Any port connected to this input, and any call to FixInputPort for this
  /// input, must provide for values whose type matches this @p model_value.
  ///
  /// @see System::DeclareInputPort() for more information.
  const InputPort<T>& DeclareAbstractInputPort(
      variant<std::string, UseDefaultName> name,
      const AbstractValue& model_value);

  //@}

  // =========================================================================
  /// @name          To-be-deprecated input port declarations
  /// Methods in this section leave out the name parameter and are the same
  /// as invoking the corresponding method with `kUseDefaultName` as the name.
  /// We intend to make specifying the name required and will deprecate these
  /// soon. Don't use them.
  //@{

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const InputPort<T>& DeclareVectorInputPort(
      const BasicVector<T>& model_vector,
      optional<RandomDistribution> random_type = nullopt) {
    return DeclareVectorInputPort(kUseDefaultName, model_vector, random_type);
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const InputPort<T>& DeclareAbstractInputPort(
      const AbstractValue& model_value) {
    return DeclareAbstractInputPort(kUseDefaultName, model_value);
  }
  //@}

  // =========================================================================
  /// @name                    Declare output ports
  /// @anchor DeclareLeafOutputPort_documentation
  ///
  /// Methods in this section are used by derived classes to declare their
  /// output ports, which may be vector valued or abstract valued. Every output
  /// port must have an _allocator_ function and
  /// a _calculator_ function. The allocator returns an object suitable for
  /// holding a value of the output port. The calculator uses the contents of
  /// a given Context to produce the output port's value, which is placed in
  /// an object of the type returned by the allocator.
  ///
  /// Although the allocator and calculator functions ultimately satisfy generic
  /// function signatures defined in LeafOutputPort, we provide a variety
  /// of `DeclareVectorOutputPort()` and `DeclareAbstractOutputPort()`
  /// signatures here for convenient specification, with mapping to the generic
  /// form handled invisibly. In particular, allocators are most easily defined
  /// by providing a model value that can be used to construct an
  /// allocator that copies the model when a new value object is needed.
  /// Alternatively a method can be provided that constructs a value object when
  /// invoked (those methods are conventionally, but not necessarily, named
  /// `MakeSomething()` where `Something` is replaced by the output port value
  /// type).
  ///
  /// Because output port values are ultimately stored in AbstractValue objects,
  /// the underlying types must be suitable. For vector ports, that means the
  /// type must be BasicVector or a class derived from BasicVector. For abstract
  /// ports, the type must be copy constructible or cloneable. For
  /// methods below that are not given an explicit model value or construction
  /// ("make") method, the underlying type must be default constructible.
  /// @see drake::systems::Value for more about abstract values.
  ///
  /// A list of prerequisites may be provided for the calculator function to
  /// avoid unnecessary recomputation. If no prerequisites are provided, the
  /// default is to assume the output port value is dependent on all possible
  /// sources. See @ref DeclareCacheEntry_documentation "DeclareCacheEntry"
  /// for more information about prerequisites.
  ///
  /// Output ports must have a name that is unique within the owning subsystem.
  /// Users can provide meaningful names or specify the name as
  /// `kUseDefaultName` in which case a name like "y3" is
  /// automatically provided, where the number is the output port index. An
  /// empty name is not permitted.
  //@{

  /// Declares a vector-valued output port by specifying (1) a model vector of
  /// type BasicVectorSubtype derived from BasicVector and initialized to the
  /// correct size and desired initial value, and (2) a calculator function that
  /// is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputVector(const Context<T>&,
  ///                                 BasicVectorSubtype*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>`. Template
  /// arguments will be deduced and do not need to be specified.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      variant<std::string, UseDefaultName> name,
      const BasicVectorSubtype& model_vector,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    static_assert(std::is_base_of<BasicVector<T>, BasicVectorSubtype>::value,
                  "Expected vector type derived from BasicVector.");
    // We need to obtain a `this` pointer of the right derived type to capture
    // in the calculator functor, so that it will be able to invoke the given
    // mmember function `calc()`.
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    // Currently all vector ports in Drake require a fixed size that is known
    // at the time the port is declared.
    auto& port = CreateVectorLeafOutputPort(
        NextOutputPortName(std::move(name)), model_vector.size(),
        // Allocator function just clones the given model vector.
        MakeAllocCallback<BasicVector<T>>(model_vector),
        // Calculator function downcasts to specific vector type and invokes
        // the given member function.
        [this_ptr, calc](const Context<T>& context, BasicVector<T>* result) {
          auto typed_result = dynamic_cast<BasicVectorSubtype*>(result);
          DRAKE_DEMAND(typed_result != nullptr);
          (this_ptr->*calc)(context, typed_result);
        },
        std::move(prerequisites_of_calc));
    // Caution: "name" is empty now.
    MaybeDeclareVectorBaseInequalityConstraint(
        "output " + std::to_string(int{port.get_index()}), model_vector,
        [&port, storage = std::shared_ptr<AbstractValue>{}](
            const Context<T>& context) mutable -> const VectorBase<T>& {
          // Because we must return a VectorBase by const reference, our lambda
          // object needs a member field to maintain storage for our result.
          // We must use a shared_ptr not because we share storage, but because
          // our lambda must be copyable.  This will go away once Eval works.
          storage = port.Allocate();
          // TODO(jwnimmer-tri) We should use port.Eval(), once it works.
          port.Calc(context, storage.get());
          return storage->GetValue<BasicVector<T>>();
        });
    return port;
  }

  /// Declares a vector-valued output port by specifying _only_ a calculator
  /// function that is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputVector(const Context<T>&,
  ///                                 BasicVectorSubtype*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and
  /// `BasicVectorSubtype` is derived from `BasicVector<T>` and has a suitable
  /// default constructor that allocates a vector of the expected size. This
  /// will use `BasicVectorSubtype{}` (that is, the default constructor) to
  /// produce a model vector for the output port's value.
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @note The default constructor will be called once immediately, and
  /// subsequent allocations will just copy the model value without invoking the
  /// constructor again. If you want the constructor invoked again at each
  /// allocation (not common), use one of the other signatures to explicitly
  /// provide a method for the allocator to call; that method can then invoke
  /// the `BasicVectorSubtype` default constructor.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      variant<std::string, UseDefaultName> name,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(
        std::is_default_constructible<BasicVectorSubtype>::value,
        "LeafSystem::DeclareVectorOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Invokes the previous method.
    return DeclareVectorOutputPort(NextOutputPortName(std::move(name)),
                                   BasicVectorSubtype{}, calc,
                                   std::move(prerequisites_of_calc));
  }

  /// (Advanced) Declares a vector-valued output port using the given
  /// `model_vector` and a function for calculating the port's value at runtime.
  /// The port's size will be model_vector.size(), and the default allocator for
  /// the port will be model_vector.Clone(). Note that this takes the calculator
  /// function in its most generic form; if you have a member function available
  /// use one of the other signatures.
  /// @see LeafOutputPort::CalcVectorCallback
  const OutputPort<T>& DeclareVectorOutputPort(
      variant<std::string, UseDefaultName> name,
      const BasicVector<T>& model_vector,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
        all_sources_ticket()});

  /// Declares an abstract-valued output port by specifying a model value of
  /// concrete type `OutputType` and a calculator function that is a class
  /// member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` must be a class derived from `LeafSystem<T>`.
  /// `OutputType` must be such that `Value<OutputType>` is permitted.
  /// Template arguments will be deduced and do not need to be specified.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      variant<std::string, UseDefaultName> name, const OutputType& model_value,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);

    auto& port = CreateAbstractLeafOutputPort(
        NextOutputPortName(std::move(name)), MakeAllocCallback(model_value),
        [this_ptr, calc](const Context<T>& context, AbstractValue* result) {
          OutputType& typed_result = result->GetMutableValue<OutputType>();
          (this_ptr->*calc)(context, &typed_result);
        },
        std::move(prerequisites_of_calc));
    return port;
  }

  /// Declares an abstract-valued output port by specifying only a calculator
  /// function that is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>`. `OutputType`
  /// is a concrete type such that `Value<OutputType>` is permitted, and
  /// must be default constructible, so that we can create a model value using
  /// `Value<OutputType>{}` (value initialized so numerical types will be
  /// zeroed in the model).
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @note The default constructor will be called once immediately, and
  /// subsequent allocations will just copy the model value without invoking the
  /// constructor again. If you want the constructor invoked again at each
  /// allocation (not common), use one of the other signatures to explicitly
  /// provide a method for the allocator to call; that method can then invoke
  /// the `OutputType` default constructor.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      variant<std::string, UseDefaultName> name,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(
        std::is_default_constructible<OutputType>::value,
        "LeafSystem::DeclareAbstractOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Note that value initialization {} is required here.
    return DeclareAbstractOutputPort(NextOutputPortName(std::move(name)),
                                     OutputType{}, calc,
                                     std::move(prerequisites_of_calc));
  }

  /// Declares an abstract-valued output port by specifying member functions to
  /// use both for the allocator and calculator. The signatures are:
  /// @code
  /// OutputType MySystem::MakeOutputValue() const;
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and `OutputType`
  /// may be any concrete type such that `Value<OutputType>` is permitted.
  /// See alternate signature if your allocator method needs a Context.
  /// Template arguments will be deduced and do not need to be specified.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      variant<std::string, UseDefaultName> name,
      OutputType (MySystem::*make)() const,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);

    auto& port = CreateAbstractLeafOutputPort(
        NextOutputPortName(std::move(name)),
        [this_ptr, make]() { return AbstractValue::Make((this_ptr->*make)()); },
        [this_ptr, calc](const Context<T>& context, AbstractValue* result) {
          OutputType& typed_result = result->GetMutableValue<OutputType>();
          (this_ptr->*calc)(context, &typed_result);
        },
        std::move(prerequisites_of_calc));
    return port;
  }

  /// (Advanced) Declares an abstract-valued output port using the given
  /// allocator and calculator functions provided in their most generic forms.
  /// If you have a member function available use one of the other signatures.
  /// @see LeafOutputPort::AllocCallback, LeafOutputPort::CalcCallback
  const OutputPort<T>& DeclareAbstractOutputPort(
      variant<std::string, UseDefaultName> name,
      typename LeafOutputPort<T>::AllocCallback alloc_function,
      typename LeafOutputPort<T>::CalcCallback calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
        all_sources_ticket()});
  //@}

  // =========================================================================
  /// @name          To-be-deprecated output port declarations
  /// Methods in this section leave out the name parameter and are the same
  /// as invoking the corresponding method with `kUseDefaultName` as the name.
  /// We intend to make specifying the name required and will deprecate these
  /// soon. Don't use them.
  //@{

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      const BasicVectorSubtype& model_vector,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(kUseDefaultName, model_vector, calc,
                                   std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(kUseDefaultName, calc,
                                   std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const OutputPort<T>& DeclareVectorOutputPort(
      const BasicVector<T>& model_vector,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(kUseDefaultName, model_vector,
                                   std::move(vector_calc_function),
                                   std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447. Note that the deprecated method is not available for
  /// `OutputType` std::string as that would create an ambiguity. In that
  /// case the name is required.
  template <class MySystem, typename OutputType>
  std::enable_if_t<!std::is_same<OutputType, std::string>::value,
                   const OutputPort<T>&>
  DeclareAbstractOutputPort(const OutputType& model_value,
                            void (MySystem::*calc)(const Context<T>&,
                                                   OutputType*) const,
                            std::set<DependencyTicket> prerequisites_of_calc = {
                                all_sources_ticket()}) {
    return DeclareAbstractOutputPort(kUseDefaultName, model_value, calc,
                                     std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareAbstractOutputPort<MySystem, OutputType>(
        kUseDefaultName, calc, std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      OutputType (MySystem::*make)() const,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareAbstractOutputPort(kUseDefaultName, make, calc,
                                     std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const OutputPort<T>& DeclareAbstractOutputPort(
      typename LeafOutputPort<T>::AllocCallback alloc_function,
      typename LeafOutputPort<T>::CalcCallback calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareAbstractOutputPort(kUseDefaultName, std::move(alloc_function),
                                     std::move(calc_function),
                                     std::move(prerequisites_of_calc));
  }
  //@}

  // =========================================================================
  /// @name                    Declare witness functions
  /// Methods in this section are used by derived classes to declare any
  /// witness functions useful for ensuring that integration ends a step upon
  /// entering particular times or states.
  ///
  /// In contrast to other declaration methods (e.g., DeclareVectorOutputPort(),
  /// for which the System class creates and stores the objects and returns
  /// references to them, the witness function declaration functions return
  /// heap-allocated objects that the subclass of leaf system owns. This
  /// facilitates returning pointers to these objects in
  /// System::DoGetWitnessFunctions().
  //@{

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator function; and
  /// with no event object.
  /// @note Constructing a witness function with no corresponding event forces
  ///       Simulator's integration of an ODE to end a step at the witness
  ///       isolation time. For example, isolating a function's minimum or
  ///       maximum values can be realized with a witness that triggers on a
  ///       sign change of the function's time derivative, ensuring that the
  ///       actual extreme value is present in the discretized trajectory.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const) const {
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc);
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator function; and
  /// with no event object.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      std::function<T(const Context<T>&)> calc) const {
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc);
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, calculator function, and
  /// publish event callback function for when this triggers.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*publish_callback)(
          const Context<T>&, const PublishEvent<T>&) const) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, publish_callback](
        const Context<T>& context, const PublishEvent<T>& publish_event) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr);
      return (system_ptr->*publish_callback)(context, publish_event);
    };
    PublishEvent<T> publish_event(fn);
    publish_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, publish_event.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, calculator function, and
  /// discrete update event callback function for when this triggers.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*du_callback)(const Context<T>&,
          const DiscreteUpdateEvent<T>&, DiscreteValues<T>*) const) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, du_callback](const Context<T>& context,
        const DiscreteUpdateEvent<T>& du_event, DiscreteValues<T>* values) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr);
      return (system_ptr->*du_callback)(context, du_event, values);
    };
    DiscreteUpdateEvent<T> du_event(fn);
    du_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, du_event.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, calculator function, and
  /// unrestricted update event callback function for when this triggers.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*uu_callback)(const Context<T>&,
          const UnrestrictedUpdateEvent<T>&, State<T>*) const) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, uu_callback](const Context<T>& context,
        const UnrestrictedUpdateEvent<T>& uu_event, State<T>* state) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr);
      return (system_ptr->*uu_callback)(context, uu_event, state);
    };
    UnrestrictedUpdateEvent<T> uu_event(fn);
    uu_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, uu_event.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator
  /// function, and with an object corresponding to the event that is to be
  /// dispatched when this witness function triggers. Example types of event
  /// objects are publish, discrete variable update, unrestricted update events.
  /// A clone of the event will be owned by the newly constructed
  /// WitnessFunction.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      const Event<T>& e) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, e.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator
  /// function, and with an object corresponding to the event that is to be
  /// dispatched when this witness function triggers. Example types of event
  /// objects are publish, discrete variable update, unrestricted update events.
  /// A clone of the event will be owned by the newly constructed
  /// WitnessFunction.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      std::function<T(const Context<T>&)> calc,
      const Event<T>& e) const {
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, e.Clone());
  }
  //@}

  /// Declares a system constraint of the form
  ///   f(context) = 0
  /// by specifying a member function to use to calculate the (VectorX)
  /// constraint value with a signature:
  /// @code
  /// void MySystem::CalcConstraint(const Context<T>&, VectorX<T>*) const;
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  template <class MySystem>
  SystemConstraintIndex DeclareEqualityConstraint(
      void (MySystem::*calc)(const Context<T>&, VectorX<T>*) const,
      int count, const std::string& description) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    return DeclareEqualityConstraint(
        [this_ptr, calc](const Context<T>& context, VectorX<T>* value) {
          DRAKE_DEMAND(value != nullptr);
          (this_ptr->*calc)(context, value);
        },
        count, description);
  }

  /// Declares a system constraint of the form
  ///   f(context) = 0
  /// by specifying a std::function to use to calculate the (Vector) constraint
  /// value with a signature:
  /// @code
  /// void CalcConstraint(const Context<T>&, VectorX<T>*);
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  SystemConstraintIndex DeclareEqualityConstraint(
      typename SystemConstraint<T>::CalcCallback calc, int count,
      const std::string& description) {
    return this->AddConstraint(std::make_unique<SystemConstraint<T>>(
        calc, count, SystemConstraintType::kEquality, description));
  }

  /// Declares a system constraint of the form
  ///   f(context) ≥ 0
  /// by specifying a member function to use to calculate the (VectorX)
  /// constraint value with a signature:
  /// @code
  /// void MySystem::CalcConstraint(const Context<T>&, VectorX<T>*) const;
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  template <class MySystem>
  SystemConstraintIndex DeclareInequalityConstraint(
      void (MySystem::*calc)(const Context<T>&, VectorX<T>*) const,
      int count, const std::string& description) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    return DeclareInequalityConstraint(
        [this_ptr, calc](const Context<T>& context, VectorX<T>* value) {
          DRAKE_DEMAND(value != nullptr);
          (this_ptr->*calc)(context, value);
        },
        count, description);
  }

  /// Declares a system constraint of the form
  ///   f(context) ≥ 0
  /// by specifying a std::function to use to calculate the (Vector) constraint
  /// value with a signature:
  /// @code
  /// void CalcConstraint(const Context<T>&, VectorX<T>*);
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  SystemConstraintIndex DeclareInequalityConstraint(
      typename SystemConstraint<T>::CalcCallback calc, int count,
      const std::string& description);

  /// Derived-class event dispatcher for all simultaneous publish events
  /// in @p events. Override this in your derived LeafSystem only if you require
  /// behavior other than the default dispatch behavior (not common).
  /// The default behavior is to traverse events in the arbitrary order they
  /// appear in @p events, and for each event that has a callback function,
  /// to invoke the callback with @p context and that event.
  ///
  /// Do not override this just to handle an event -- instead declare the event
  /// and a handler callback for it using one of the `Declare...PublishEvent()`
  /// methods.
  ///
  /// This method is called only from the virtual DispatchPublishHandler, which
  /// is only called from the public non-virtual Publish(), which will have
  /// already error-checked @p context so you may assume that it is valid.
  ///
  /// @param[in] context Const current context.
  /// @param[in] events All the publish events that need handling.
  virtual void DoPublish(
      const Context<T>& context,
      const std::vector<const PublishEvent<T>*>& events) const;

  /// Derived-class event dispatcher for all simultaneous discrete update
  /// events. Override this in your derived LeafSystem only if you require
  /// behavior other than the default dispatch behavior (not common).
  /// The default behavior is to traverse events in the arbitrary order they
  /// appear in @p events, and for each event that has a callback function,
  /// to invoke the callback with @p context, that event, and @p discrete_state.
  /// Note that the same (possibly modified) @p discrete_state is passed to
  /// subsequent callbacks.
  ///
  /// Do not override this just to handle an event -- instead declare the event
  /// and a handler callback for it using one of the
  /// `Declare...DiscreteUpdateEvent()` methods.
  ///
  /// This method is called only from the virtual
  /// DispatchDiscreteVariableUpdateHandler(), which is only called from
  /// the public non-virtual CalcDiscreteVariableUpdates(), which will already
  /// have error-checked the parameters so you don't have to. In particular,
  /// implementations may assume that @p context is valid; that
  /// @p discrete_state is non-null, and that the referenced object has the
  /// same constituent structure as was produced by AllocateDiscreteVariables().
  ///
  /// @param[in] context The "before" state.
  /// @param[in] events All the discrete update events that need handling.
  /// @param[in,out] discrete_state The current state of the system on input;
  /// the desired state of the system on return.
  virtual void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>& events,
      DiscreteValues<T>* discrete_state) const;

  /// Derived-class event dispatcher for all simultaneous unrestricted update
  /// events. Override this in your derived LeafSystem only if you require
  /// behavior other than the default dispatch behavior (not common).
  /// The default behavior is to traverse events in the arbitrary order they
  /// appear in @p events, and for each event that has a callback function,
  /// to invoke the callback with @p context, that event, and @p state.
  /// Note that the same (possibly modified) @p state is passed to subsequent
  /// callbacks.
  ///
  /// Do not override this just to handle an event -- instead declare the event
  /// and a handler callback for it using one of the
  /// `Declare...UnrestrictedUpdateEvent()` methods.
  ///
  /// This method is called only from the virtual
  /// DispatchUnrestrictedUpdateHandler(), which is only called from the
  /// non-virtual public CalcUnrestrictedUpdate(), which will already have
  /// error-checked the parameters so you don't have to. In particular,
  /// implementations may assume that the @p context is valid; that @p state
  /// is non-null, and that the referenced object has the same constituent
  /// structure as the state in @p context.
  ///
  /// @param[in]     context The "before" state that is to be used to calculate
  ///                        the returned state update.
  /// @param[in]     events All the unrestricted update events that need
  ///                       handling.
  /// @param[in,out] state   The current state of the system on input; the
  ///                        desired state of the system on return.
  // TODO(sherm1) Shouldn't require preloading of the output state; better to
  //              note just the changes since usually only a small subset will
  //              be changed by this method.
  virtual void DoCalcUnrestrictedUpdate(
      const Context<T>& context,
      const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
      State<T>* state) const;

 private:
  using SystemBase::NextInputPortName;
  using SystemBase::NextOutputPortName;

  // Either clones the model_value, or else for vector ports allocates a
  // BasicVector, or else for abstract ports throws an exception.
  std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>& input_port) const final;

  std::map<PeriodicEventData, std::vector<const Event<T>*>,
           PeriodicEventDataComparator> DoGetPeriodicEvents() const;

  // Calls DoPublish.
  // Assumes @param events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @param events is not empty. Aborts otherwise.
  void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& events) const final;

  // Calls DoCalcDiscreteVariableUpdates.
  // Assumes @param events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @param events is not empty. Aborts otherwise.
  void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state) const final;

  // Calls DoCalcUnrestrictedUpdate.
  // Assumes @param events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @param events is not empty. Aborts otherwise.
  void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const final;

  void DoGetPerStepEvents(
      const Context<T>&,
      CompositeEventCollection<T>* events) const override {
    events->SetFrom(per_step_events_);
  }

  void DoGetInitializationEvents(
      const Context<T>&,
      CompositeEventCollection<T>* events) const override {
    events->SetFrom(initialization_events_);
  }

  // Returns a SystemSymbolicInspector for this system, or nullptr if one
  // cannot be constructed because this System has no symbolic representation.
  std::unique_ptr<SystemSymbolicInspector> MakeSystemSymbolicInspector() const {
    // We use different implementations when T = Expression or not.
    return MakeSystemSymbolicInspectorImpl(*this);
  }

  // When T == Expression, we don't need to use scalar conversion.
  static std::unique_ptr<SystemSymbolicInspector>
  MakeSystemSymbolicInspectorImpl(const System<symbolic::Expression>& system) {
    return std::make_unique<SystemSymbolicInspector>(system);
  }

  // When T != Expression, attempt to use scalar conversion.
  template <typename T1>
  static
  typename std::enable_if_t<
    !std::is_same<T1, symbolic::Expression>::value,
    std::unique_ptr<SystemSymbolicInspector>>
  MakeSystemSymbolicInspectorImpl(const System<T1>& system) {
    using symbolic::Expression;
    std::unique_ptr<System<Expression>> converted = system.ToSymbolicMaybe();
    if (converted) {
      return MakeSystemSymbolicInspectorImpl(*converted);
    } else {
      return nullptr;
    }
  }

  // Creates a new cached, vector-valued LeafOutputPort in this LeafSystem and
  // returns a reference to it.
  LeafOutputPort<T>& CreateVectorLeafOutputPort(
      std::string name,
      int fixed_size,
      typename LeafOutputPort<T>::AllocCallback vector_allocator,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calculator,
      std::set<DependencyTicket> calc_prerequisites);

  // Creates a new cached, abstract-valued LeafOutputPort in this LeafSystem and
  // returns a reference to it.
  LeafOutputPort<T>& CreateAbstractLeafOutputPort(
      std::string name,
      typename LeafOutputPort<T>::AllocCallback allocator,
      typename LeafOutputPort<T>::CalcCallback calculator,
      std::set<DependencyTicket> calc_prerequisites);

  // Creates a new cached LeafOutputPort in this LeafSystem and returns a
  // reference to it. Pass fixed_size == 0 for abstract ports, or non-zero
  // for vector ports. Prerequisites list must not be empty.
  LeafOutputPort<T>& CreateCachedLeafOutputPort(
      std::string name, int fixed_size,
      typename CacheEntry::AllocCallback allocator,
      typename CacheEntry::CalcCallback calculator,
      std::set<DependencyTicket> calc_prerequisites);

  // Creates an abstract output port allocator function from an arbitrary type
  // model value.
  template <typename OutputType>
  static typename LeafOutputPort<T>::AllocCallback MakeAllocCallback(
      const OutputType& model_value) {
    // The given model value may have *either* a copy constructor or a Clone()
    // method, since it just has to be suitable for containing in an
    // AbstractValue. We need to create a functor that is copy constructible,
    // so need to wrap the model value to give it a copy constructor. Drake's
    // copyable_unique_ptr does just that, so is suitable for capture by the
    // allocator functor here.
    copyable_unique_ptr<AbstractValue> owned_model(
        new Value<OutputType>(model_value));
    return [model = std::move(owned_model)]() {
      return model->Clone();
    };
  }

  // If @p model_vector's CalcInequalityConstraint provides any constraints,
  // then declares inequality constraints on `this` using a calc function that
  // obtains a VectorBase from a Context using @p get_vector_from_context and
  // then delegates to the VectorBase::CalcInequalityConstraint.  Note that the
  // model vector is only used to determine how many constraints will appear;
  // it is not part of the ongoing constraint computations.
  void MaybeDeclareVectorBaseInequalityConstraint(
      const std::string& kind,
      const VectorBase<T>& model_vector,
      const std::function<const VectorBase<T>&(const Context<T>&)>&
      get_vector_from_context);

  // Periodic Update or Publish events registered on this system.
  std::vector<std::pair<PeriodicEventData,
                        std::unique_ptr<Event<T>>>>
      periodic_events_;

  // Update or Publish events registered on this system for every simulator
  // major time step.
  LeafCompositeEventCollection<T> per_step_events_;

  // Update or Publish events that need to be handled at system initialization.
  LeafCompositeEventCollection<T> initialization_events_;

  // A model continuous state to be used during Context allocation.
  std::unique_ptr<BasicVector<T>> model_continuous_state_vector_;
  int num_generalized_positions_{0};
  int num_generalized_velocities_{0};
  int num_misc_continuous_states_{0};

  // A model discrete state to be used during Context allocation.
  DiscreteValues<T> model_discrete_state_;

  // A model abstract state to be used during Context allocation.
  detail::ModelValues model_abstract_states_;

  // Model inputs to be used in AllocateInput{Vector,Abstract}.
  detail::ModelValues model_input_values_;

  // Model numeric parameters to be used during Context allocation.
  detail::ModelValues model_numeric_parameters_;

  // Model abstract parameters to be used during Context allocation.
  detail::ModelValues model_abstract_parameters_;
};

}  // namespace systems
}  // namespace drake
