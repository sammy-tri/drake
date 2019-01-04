#pragma once

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/diagram_discrete_values.h"
#include "drake/systems/framework/diagram_output_port.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_constraint.h"

namespace drake {
namespace systems {

template <typename T>
class DiagramBuilder;

/// Diagram is a System composed of one or more constituent Systems, arranged
/// in a directed graph where the vertices are the constituent Systems
/// themselves, and the edges connect the output of one constituent System
/// to the input of another. To construct a Diagram, use a DiagramBuilder.
///
/// Each System in the Diagram must have a unique, non-empty name.
///
/// @tparam T The mathematical scalar type. Must be a valid Eigen scalar.
template <typename T>
class Diagram : public System<T>, internal::SystemParentServiceInterface {
 public:
  // Diagram objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Diagram)

  using InputPortLocator = std::pair<const System<T>*, InputPortIndex>;
  using OutputPortLocator = std::pair<const System<T>*, OutputPortIndex>;

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Diagram(const Diagram<U>& other)
      : Diagram(other.template ConvertScalarType<T>()) {}

  ~Diagram() override {}

  /// Returns the list of contained Systems.
  std::vector<const systems::System<T>*> GetSystems() const;

  std::multimap<int, int> GetDirectFeedthroughs() const final;

  /// Allocates a DiagramEventCollection for this Diagram.
  /// @sa System::AllocateCompositeEventCollection().
  std::unique_ptr<CompositeEventCollection<T>>
  AllocateCompositeEventCollection() const final;

  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override;

  void SetDefaultParameters(const Context<T>& context,
                            Parameters<T>* params) const override;

  void SetRandomState(const Context<T>& context, State<T>* state,
                      RandomGenerator* generator) const override;

  void SetRandomParameters(const Context<T>& context, Parameters<T>* params,
                           RandomGenerator* generator) const override;

  /// @cond
  // The three methods below are hidden from doxygen, as described in
  // documentation for their corresponding methods in System.
  std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const final {
    return AllocateForcedEventCollection<PublishEvent<T>>(
        &System<T>::AllocateForcedPublishEventCollection);
  }

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const final {
    return AllocateForcedEventCollection<DiscreteUpdateEvent<T>>(
        &System<T>::AllocateForcedDiscreteUpdateEventCollection);
  }

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const final {
    return AllocateForcedEventCollection<UnrestrictedUpdateEvent<T>>(
        &System<T>::AllocateForcedUnrestrictedUpdateEventCollection);
  }
  /// @endcond

  /// Aggregates the time derivatives from each subsystem into a
  /// DiagramContinuousState.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override;

  /// Aggregates the discrete update variables from each subsystem into a
  /// DiagramDiscreteVariables.
  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables()
      const override;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

  /// Retrieves a reference to the subsystem with name @p name returned by
  /// get_name().
  /// @throws std::logic_error if a match cannot be found.
  /// @see System<T>::get_name()
  const System<T>& GetSubsystemByName(const std::string& name) const;

  /// Retrieves the state derivatives for a particular subsystem from the
  /// derivatives for the entire diagram. Aborts if @p subsystem is not
  /// actually a subsystem of this diagram. Returns a 0-length ContinuousState
  /// if @p subsystem has none.
  const ContinuousState<T>& GetSubsystemDerivatives(
      const System<T>& subsystem,
      const ContinuousState<T>& derivatives) const;

  DRAKE_DEPRECATED("Call GetSubsystemDerivatives(subsystem, derivatives) "
                   "instead.  This call site will be removed on 2/15/19.")
  const ContinuousState<T>& GetSubsystemDerivatives(
      const ContinuousState<T>& derivatives, const System<T>* subsystem)
      const {
    return GetSubsystemDerivatives(*subsystem, derivatives);
  }

  /// Retrieves the discrete state values for a particular subsystem from the
  /// discrete values for the entire diagram. Aborts if @p subsystem is not
  /// actually a subsystem of this diagram. Returns an empty DiscreteValues
  /// if @p subsystem has none.
  const DiscreteValues<T>& GetSubsystemDiscreteValues(
      const System<T>& subsystem,
      const DiscreteValues<T>& discrete_values) const;

  /// Returns a constant reference to the subcontext that corresponds to the
  /// system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the appropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  const Context<T>& GetSubsystemContext(const System<T>& subsystem,
                                        const Context<T>& context) const {
    auto ret = DoGetTargetSystemContext(subsystem, &context);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the subcontext that corresponds to the system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the appropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  Context<T>& GetMutableSubsystemContext(const System<T>& subsystem,
                                         Context<T>* context) const {
    auto ret = DoGetMutableTargetSystemContext(subsystem, context);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the const subsystem composite event collection from @p events
  /// that corresponds to @p subsystem. Aborts if @p subsystem is not a
  /// subsystem of this diagram.
  const CompositeEventCollection<T>&
  GetSubsystemCompositeEventCollection(const System<T>& subsystem,
      const CompositeEventCollection<T>& events) const {
    auto ret = DoGetTargetSystemCompositeEventCollection(subsystem, &events);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the mutable subsystem composite event collection that corresponds
  /// to @p subsystem. Aborts if @p subsystem is not a subsystem of this
  /// diagram.
  CompositeEventCollection<T>& GetMutableSubsystemCompositeEventCollection(
      const System<T>& subsystem, CompositeEventCollection<T>* events) const {
    auto ret = DoGetMutableTargetSystemCompositeEventCollection(
        subsystem, events);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  // TODO(david-german-tri): Provide finer-grained accessors for finer-grained
  // invalidation.
  /// Retrieves the state for a particular subsystem from the context for the
  /// entire diagram. Invalidates all entries in that subsystem's cache that
  /// depend on State. Aborts if @p subsystem is not actually a subsystem of
  /// this diagram.
  State<T>& GetMutableSubsystemState(const System<T>& subsystem,
                                     Context<T>* context) const {
    Context<T>& subcontext = GetMutableSubsystemContext(subsystem, context);
    return subcontext.get_mutable_state();
  }

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  State<T>& GetMutableSubsystemState(const System<T>& subsystem,
                                     State<T>* state) const {
    auto ret = DoGetMutableTargetSystemState(subsystem, state);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  const State<T>& GetSubsystemState(const System<T>& subsystem,
                                    const State<T>& state) const {
    auto ret = DoGetTargetSystemState(subsystem, &state);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  //----------------------------------------------------------------------------
  /// @name                      Graphviz methods
  //@{

  /// Returns a Graphviz fragment describing this Diagram. To obtain a complete
  /// Graphviz graph, call System<T>::GetGraphvizString.
  void GetGraphvizFragment(int max_depth,
                           std::stringstream* dot) const override;

  void GetGraphvizInputPortToken(const InputPort<T>& port,
                                 int max_depth,
                                 std::stringstream* dot) const final;

  void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                  int max_depth,
                                  std::stringstream* dot) const final;
  //@}

  /// Returns the index of the given @p sys in this diagram, or aborts if @p sys
  /// is not a member of the diagram.
  SubsystemIndex GetSystemIndexOrAbort(const System<T>* sys) const {
    auto it = system_index_map_.find(sys);
    DRAKE_DEMAND(it != system_index_map_.end());
    return it->second;
  }

  int get_num_continuous_states() const final {
    int num_states = 0;
    for (const auto& system : registered_systems_) {
      num_states += system->get_num_continuous_states();
    }

    return num_states;
  }

 protected:
  /// Constructs an uninitialized Diagram. Subclasses that use this constructor
  /// are obligated to call DiagramBuilder::BuildInto(this).  Provides scalar-
  /// type conversion support only if every contained subsystem provides the
  /// same support.
  Diagram();

  /// (Advanced) Constructs an uninitialized Diagram.  Subclasses that use this
  /// constructor are obligated to call DiagramBuilder::BuildInto(this).
  ///
  /// Declares scalar-type conversion support using @p converter.  Support for
  /// a given pair of types `T, U` to convert from and to will be enabled only
  /// if every contained subsystem supports that pair.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  explicit Diagram(SystemScalarConverter converter);

  /// For the subsystem associated with @p witness_func, gets its subcontext
  /// from @p context, passes the subcontext to @p witness_func' Evaulate
  /// method and returns the result. Aborts if the subsystem is not part of
  /// this Diagram.
  T DoCalcWitnessValue(const Context<T>& context,
                       const WitnessFunction<T>& witness_func) const final {
    const System<T>& system = witness_func.get_system();
    const Context<T>& subcontext = GetSubsystemContext(system, context);
    return witness_func.CalcWitnessValue(subcontext);
  }

  /// For the subsystem associated with `witness_func`, gets its mutable
  /// sub composite event collection from `events`, and passes it to
  /// `witness_func`'s AddEventToCollection method. This method also modifies
  /// `event` by updating the pointers to "diagram" continuous state to point to
  /// the ContinuousState pointers for the associated subsystem instead. Aborts
  /// if the subsystem is not part of this Diagram.
  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>* event,
      CompositeEventCollection<T>* events) const final;

  /// Provides witness functions of subsystems that are active at the beginning
  /// of a continuous time interval. The vector of witness functions is not
  /// ordered in a particular manner.
  void DoGetWitnessFunctions(
      const Context<T>& context,
      std::vector<const WitnessFunction<T>*>* witnesses) const final;

  /// Returns a pointer to mutable context if @p target_system is a sub system
  /// of this, nullptr is returned otherwise.
  Context<T>* DoGetMutableTargetSystemContext(
      const System<T>& target_system, Context<T>* context) const final;

  /// Returns a pointer to const context if @p target_system is a subsystem
  /// of this, nullptr is returned otherwise.
  const Context<T>* DoGetTargetSystemContext(
      const System<T>& target_system, const Context<T>* context) const final;

  /// Returns a pointer to mutable state if @p target_system is a subsystem
  /// of this, nullptr is returned otherwise.
  State<T>* DoGetMutableTargetSystemState(
      const System<T>& target_system, State<T>* state) const final;

  /// Returns a pointer to const state if @p target_system is a subsystem
  /// of this, nullptr is returned otherwise.
  const ContinuousState<T>* DoGetTargetSystemContinuousState(
      const System<T>& target_system,
      const ContinuousState<T>* xc) const final;

  /// Returns a pointer to const state if @p target_system is a subsystem
  /// of this, nullptr is returned otherwise.
  const State<T>* DoGetTargetSystemState(
      const System<T>& target_system, const State<T>* state) const final;

  /// Returns a pointer to mutable composite event collection if
  /// @p target_system is a subsystem of this, nullptr is returned otherwise.
  CompositeEventCollection<T>* DoGetMutableTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      CompositeEventCollection<T>* events) const final;

  /// Returns a pointer to const composite event collection if
  /// @p target_system is a subsystem of this, nullptr is returned otherwise.
  const CompositeEventCollection<T>* DoGetTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      const CompositeEventCollection<T>* events) const final;

  /// The @p generalized_velocity vector must have the same size and ordering as
  /// the generalized velocity in the ContinuousState that this Diagram reserves
  /// in its context.
  void DoMapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const override;

  /// The @p generalized_velocity vector must have the same size and ordering as
  /// the generalized velocity in the ContinuousState that this Diagram reserves
  /// in its context.
  void DoMapQDotToVelocity(const Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           VectorBase<T>* generalized_velocity) const override;

  /// Computes the next update time based on the configured actions, for scalar
  /// types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            CompositeEventCollection<T>* event_info,
                            T* time) const override;

 private:
  std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>& input_port) const final;

  // Allocates a default-constructed diagram context containing the complete
  // diagram substructure of default-constructed subcontexts.
  std::unique_ptr<ContextBase> DoAllocateContext() const final;

  // Evaluates the value of the specified subsystem input
  // port in the given context. The port has already been determined _not_ to
  // be a fixed port, so it must be connected either
  // - to the output port of a peer subsystem, or
  // - to an input port of this Diagram,
  // - or not connected at all in which case we return null.
  const AbstractValue* EvalConnectedSubsystemInputPort(
      const ContextBase& context_base,
      const InputPortBase& input_port_base) const final;

  std::string GetParentPathname() const final {
    return this->GetSystemPathname();
  }

  // Returns true if there might be direct feedthrough from the given
  // @p input_port of the Diagram to the given @p output_port of the Diagram.
  bool DoHasDirectFeedthrough(int input_port, int output_port) const;

  template <typename EventType>
  std::unique_ptr<EventCollection<EventType>> AllocateForcedEventCollection(
      std::function<
      std::unique_ptr<EventCollection<EventType>>(const System<T>*)>
  allocater_func) const {
    const int num_systems = num_subsystems();
    auto ret = std::make_unique<DiagramEventCollection<EventType>>(num_systems);
    for (SubsystemIndex i(0); i < num_systems; ++i) {
      std::unique_ptr<EventCollection<EventType>> subevent_collection =
          allocater_func(registered_systems_[i].get());
      ret->set_and_own_subevent_collection(i, std::move(subevent_collection));
    }
    return ret;
  }

  // For each subsystem, if there is a publish event in its corresponding
  // subevent collection, calls its Publish method with the appropriate
  // subcontext and subevent collection.
  void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& event_info) const final;

  // For each subsystem, if there is a discrete update event in its
  // corresponding subevent collection, calls its CalcDiscreteVariableUpdates
  // method with the appropriate subcontext, subevent collection and
  // substate.
  void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& event_info,
      DiscreteValues<T>* discrete_state) const final;

  // For each subsystem, if there is an unrestricted update event in its
  // corresponding subevent collection, calls its CalcUnrestrictedUpdate
  // method with the appropriate subcontext, subevent collection and substate.
  void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& event_info,
      State<T>* state) const final;

  // Tries to recursively find @p target_system's BaseStuff
  // (context / state / etc). nullptr is returned if @p target_system is not
  // a subsystem of this diagram. This template function should only be used
  // to reduce code repetition for DoGetMutableTargetSystemContext(),
  // DoGetTargetSystemContext(), DoGetMutableTargetSystemState(), and
  // DoGetTargetSystemState().
  // @param target_system The subsystem of interest.
  // @param my_stuff BaseStuff that's associated with this diagram.
  // @param recursive_getter A member function of System that returns sub
  // context or state. Should be one of the four functions listed above.
  // @param get_child_stuff A member function of DiagramContext or DiagramState
  // that returns context or state given the index of the subsystem.
  //
  // @tparam BaseStuff Can be Context<T>, const Context<T>, State<T> and
  // const State<T>.
  // @tparam DerivedStuff Can be DiagramContext<T>,
  // const DiagramContext<T>, DiagramState<T> and const DiagramState<T>.
  //
  // @pre @p target_system cannot be `this`. The caller should check for this
  // edge case.
  template <typename BaseStuff, typename DerivedStuff>
  BaseStuff* GetSubsystemStuff(
      const System<T>& target_system, BaseStuff* my_stuff,
      std::function<BaseStuff*(const System<T>*, const System<T>&, BaseStuff*)>
          recursive_getter,
      std::function<BaseStuff&(DerivedStuff*, SubsystemIndex)> get_child_stuff)
      const {
    static_assert(
        std::is_same<BaseStuff,
                     typename std::remove_pointer<BaseStuff>::type>::value,
        "BaseStuff cannot be a pointer");
    static_assert(
        std::is_same<DerivedStuff,
                     typename std::remove_pointer<DerivedStuff>::type>::value,
        "DerivedStuff cannot be a pointer");

    DRAKE_DEMAND(my_stuff != nullptr);
    DRAKE_DEMAND(&target_system != this);
    DerivedStuff& my_stuff_as_derived = dynamic_cast<DerivedStuff&>(*my_stuff);

    SubsystemIndex index(0);
    for (const auto& child : registered_systems_) {
      BaseStuff& child_stuff = get_child_stuff(&my_stuff_as_derived, index);

      BaseStuff* const target_stuff =
          recursive_getter(child.get(), target_system, &child_stuff);

      if (target_stuff != nullptr) {
        return target_stuff;
      }
      ++index;
    }

    return nullptr;
  }

  // Uses this Diagram<T> to manufacture a Diagram<NewType>::Blueprint,
  // using system scalar conversion.
  //
  // @tparam NewType The scalar type to which to convert.
  template <typename NewType>
  std::unique_ptr<typename Diagram<NewType>::Blueprint> ConvertScalarType()
      const {
    std::vector<std::unique_ptr<System<NewType>>> new_systems;
    // Recursively convert all the subsystems.
    std::map<const System<T>*, const System<NewType>*> old_to_new_map;
    for (const auto& old_system : registered_systems_) {
      // Convert old_system to new_system using the old_system's converter.
      std::unique_ptr<System<NewType>> new_system =
          old_system->get_system_scalar_converter().
          template Convert<NewType>(*old_system);
      DRAKE_DEMAND(new_system != nullptr);

      // Update our mapping and take ownership.
      old_to_new_map[old_system.get()] = new_system.get();
      new_systems.push_back(std::move(new_system));
    }

    // Set up the blueprint.
    auto blueprint = std::make_unique<typename Diagram<NewType>::Blueprint>();
    // Make all the inputs and outputs.
    for (const InputPortLocator& id : input_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const InputPortIndex port = id.second;
      blueprint->input_port_ids.emplace_back(new_system, port);
    }
    for (InputPortIndex i{0}; i < this->get_num_input_ports(); i++) {
      blueprint->input_port_names.emplace_back(
          this->get_input_port(i).get_name());
    }
    for (const OutputPortLocator& id : output_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const OutputPortIndex port = id.second;
      blueprint->output_port_ids.emplace_back(new_system, port);
    }
    for (OutputPortIndex i{0}; i < this->get_num_output_ports(); i++) {
      blueprint->output_port_names.emplace_back(
          this->get_output_port(i).get_name());
    }
    // Make all the connections.
    for (const auto& edge : connection_map_) {
      const InputPortLocator& old_dest = edge.first;
      const System<NewType>* const dest_system = old_to_new_map[old_dest.first];
      const InputPortIndex dest_port = old_dest.second;
      const typename Diagram<NewType>::InputPortLocator new_dest{dest_system,
                                                                 dest_port};

      const OutputPortLocator& old_src = edge.second;
      const System<NewType>* const src_system = old_to_new_map[old_src.first];
      const OutputPortIndex src_port = old_src.second;
      const typename Diagram<NewType>::OutputPortLocator new_src{src_system,
                                                                 src_port};

      blueprint->connection_map[new_dest] = new_src;
    }
    // Move the new systems into the blueprint.
    blueprint->systems = std::move(new_systems);

    return blueprint;
  }

  std::map<PeriodicEventData, std::vector<const Event<T>*>,
           PeriodicEventDataComparator> DoGetPeriodicEvents() const override;

  void DoGetPerStepEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* event_info) const override;

  void DoGetInitializationEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* event_info) const override;

  // A structural outline of a Diagram, produced by DiagramBuilder.
  struct Blueprint {
    // The ordered subsystem ports that are inputs to the entire diagram.
    std::vector<InputPortLocator> input_port_ids;
    // The names should be the same length and ordering as the ids.
    std::vector<std::string> input_port_names;

    // The ordered subsystem ports that are outputs of the entire diagram.
    std::vector<OutputPortLocator> output_port_ids;
    // The names should be the same length and ordering as the ids.
    std::vector<std::string> output_port_names;

    // A map from the input ports of constituent systems to the output ports
    // on which they depend. This graph is possibly cyclic, but must not
    // contain an algebraic loop.
    std::map<InputPortLocator, OutputPortLocator> connection_map;
    // All of the systems to be included in the diagram.
    std::vector<std::unique_ptr<System<T>>> systems;
  };

  // Constructs a Diagram from the Blueprint that a DiagramBuilder produces.
  // This constructor is private because only DiagramBuilder calls it. The
  // constructor takes the systems from the blueprint.
  explicit Diagram(std::unique_ptr<Blueprint> blueprint) : Diagram() {
    Initialize(std::move(blueprint));
  }

  // Validates the given @p blueprint and sets up the Diagram accordingly.
  void Initialize(std::unique_ptr<Blueprint> blueprint);

  // Exposes the given port as an input of the Diagram.
  void ExportInput(const InputPortLocator& port, std::string name);

  // Exposes the given subsystem output port as an output of the Diagram.
  void ExportOutput(const OutputPortLocator& port, std::string name);

  // Returns a reference to the value in the given context, of the specified
  // output port of one of this Diagram's immediate subsystems, recalculating
  // if necessary to bring the value up to date.
  const AbstractValue& EvalSubsystemOutputPort(
      const DiagramContext<T>& context, const OutputPortLocator& id) const;

  // Converts an InputPortLocator to a DiagramContext::InputPortIdentifier.
  // The DiagramContext::InputPortIdentifier contains the index of the System in
  // the diagram, instead of an actual pointer to the System.
  // TODO(sherm1) Should just use the (SystemIndex,PortIndex) form everywhere.
  typename DiagramContext<T>::InputPortIdentifier
  ConvertToContextPortIdentifier(const InputPortLocator& locator) const {
    typename DiagramContext<T>::InputPortIdentifier identifier;
    identifier.first = GetSystemIndexOrAbort(locator.first);
    identifier.second = locator.second;
    return identifier;
  }

  // Converts an OutputPortLocator to a DiagramContext::OutputPortIdentifier.
  // The DiagramContext::OutputPortIdentifier contains the index of the System
  // in the diagram, instead of an actual pointer to the System.
  typename DiagramContext<T>::OutputPortIdentifier
  ConvertToContextPortIdentifier(const OutputPortLocator& locator) const {
    typename DiagramContext<T>::OutputPortIdentifier identifier;
    identifier.first = GetSystemIndexOrAbort(locator.first);
    identifier.second = locator.second;
    return identifier;
  }

  // Returns true if every port mentioned in the connection map exists.
  bool PortsAreValid() const;

  // Returns true if every subsystem has a unique, non-empty name.
  // O(N * log(N)) in the number of subsystems.
  bool NamesAreUniqueAndNonEmpty() const;

  int num_subsystems() const {
    return static_cast<int>(registered_systems_.size());
  }

  // A map from the input ports of constituent systems, to the output ports of
  // the systems from which they get their values.
  std::map<InputPortLocator, OutputPortLocator> connection_map_;

  // The Systems in this Diagram, which are owned by this Diagram, in the order
  // they were registered. Index by SubsystemIndex.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // Map to quickly satisfy "What is the subsystem index of the child system?"
  std::map<const System<T>*, SubsystemIndex> system_index_map_;

  // The ordered inputs and outputs of this Diagram. Index by InputPortIndex
  // and OutputPortIndex.
  std::vector<InputPortLocator> input_port_ids_;
  std::vector<OutputPortLocator> output_port_ids_;

  // For all T, Diagram<T> considers DiagramBuilder<T> a friend, so that the
  // builder can set the internal state correctly.
  friend class DiagramBuilder<T>;

  // For any T1 & T2, Diagram<T1> considers Diagram<T2> a friend, so that
  // Diagram can provide transmogrification methods across scalar types.
  // See Diagram<T>::ConvertScalarType.
  template <typename> friend class Diagram;
};

}  // namespace systems
}  // namespace drake
