#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/supervector.h"

namespace drake {
namespace systems {

// TODO(sherm1) This should be in its own file.
/// DiagramState is a State, annotated with pointers to all the mutable
/// substates that it spans.
template <typename T>
class DiagramState : public State<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramState)

  /// Constructs a DiagramState consisting of @p size substates.
  explicit DiagramState<T>(int size) :
      State<T>(),
      substates_(size),
      owned_substates_(size) {}

  /// Sets the substate at @p index to @p substate, or aborts if @p index is
  /// out of bounds. Does not take ownership of @p substate, which must live
  /// as long as this object.
  void set_substate(int index, State<T>* substate) {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    substates_[index] = substate;
  }

  /// Sets the substate at @p index to @p substate, or aborts if @p index is
  /// out of bounds.
  void set_and_own_substate(int index, std::unique_ptr<State<T>> substate) {
    set_substate(index, substate.get());
    owned_substates_[index] = std::move(substate);
  }

  /// Returns the substate at @p index.
  const State<T>& get_substate(int index) const {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    return *substates_[index];
  }

  /// Returns the substate at @p index.
  State<T>& get_mutable_substate(int index) {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    return *substates_[index];
  }

  /// Finalizes this state as a span of all the constituent substates.
  void Finalize();

 private:
  int num_substates() const {
    return static_cast<int>(substates_.size());
  }

  bool finalized_{false};
  std::vector<State<T>*> substates_;
  std::vector<std::unique_ptr<State<T>>> owned_substates_;
};

/// The DiagramContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a Diagram. Specifically, a
/// DiagramContext contains Context objects for all its constituent Systems.
/// @see Context for more information.
///
/// In general, users should not need to interact with a DiagramContext
/// directly. Use the accessors on Diagram instead.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class DiagramContext final : public Context<T> {
 public:
  /// @name  Does not allow copy, move, or assignment.
  //@{
  // Copy constructor is protected for use in implementing Clone().
  DiagramContext(DiagramContext&&) = delete;
  DiagramContext& operator=(const DiagramContext&) = delete;
  DiagramContext& operator=(DiagramContext&&) = delete;
  //@}

  /// Identifies a child subsystem's input port.
  using InputPortIdentifier = std::pair<SubsystemIndex, InputPortIndex>;
  /// Identifies a child subsystem's output port.
  using OutputPortIdentifier = std::pair<SubsystemIndex, OutputPortIndex>;

  /// Constructs a DiagramContext with the given @p num_subcontexts, which is
  /// final: you cannot resize a DiagramContext after construction. The
  /// number and ordering of subcontexts is identical to the number and
  /// ordering of subsystems in the corresponding Diagram.
  explicit DiagramContext(int num_subcontexts)
      : contexts_(num_subcontexts),
        state_(std::make_unique<DiagramState<T>>(num_subcontexts)) {}

  /// Declares a new subsystem in the DiagramContext. Subsystems are identified
  /// by number. If the subsystem has already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void AddSystem(SubsystemIndex index, std::unique_ptr<Context<T>> context) {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] == nullptr);
    ContextBase::set_parent(context.get(), this);
    contexts_[index] = std::move(context);
  }

  /// (Internal use only) Declares that a particular input port of a child
  /// subsystem is an input to the entire Diagram that allocates this Context.
  /// Sets up tracking of the child port's dependency on the parent
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeExportedInputPortToDiagramPort(
    InputPortIndex input_port_index,
    const InputPortIdentifier& subsystem_input_port);

  /// (Internal use only) Declares that a particular output port of this
  /// diagram is simply forwarded from an output port of one of its child
  /// subsystems. Sets up tracking of the diagram port's dependency on the child
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeDiagramPortToExportedOutputPort(
      OutputPortIndex output_port_index,
      const OutputPortIdentifier& subsystem_output_port);

  /// (Internal use only) Declares that a connection exists between a peer
  /// output port and input port in this Diagram, and registers the input port's
  /// dependency tracker with the output port's dependency tracker. By "peer"
  /// we mean that both ports belong to immediate child subsystems of this
  /// Diagram (it is also possible for both ports to belong to the same
  /// subsystem).
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeInputPortToOutputPort(const OutputPortIdentifier& output_port,
                                      const InputPortIdentifier& input_port);

  /// (Internal use only) Makes the diagram state, parameter, and composite
  /// cache entry trackers subscribe to the corresponding constituent trackers
  /// in the child subcontexts.
  // Diagrams don't provide diagram-level tickets for individual
  // discrete or abstract state or individual numerical or abstract parameters.
  // That means we need only subscribe the aggregate trackers xd, xa, pn, pa
  // to their children's xd, xa, pn, pa, resp.
  void SubscribeDiagramCompositeTrackersToChildrens();

  /// (Internal use only) Generates the state vector for the entire diagram by
  /// wrapping the states of all the constituent diagrams.
  void MakeState();

  /// (Internal use only) Generates the parameters for the entire diagram by
  /// wrapping the parameters of all the constituent Systems. The wrapper simply
  /// holds pointers to the parameters in the subsystem Contexts. It does not
  /// make a copy, or take ownership.
  void MakeParameters();

  // TODO(david-german-tri): Rename to get_subsystem_context.
  /// Returns the context structure for a given constituent system @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  const Context<T>& GetSubsystemContext(SubsystemIndex index) const {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return *contexts_[index].get();
  }

  // TODO(david-german-tri): Rename to get_mutable_subsystem_context.
  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  Context<T>& GetMutableSubsystemContext(SubsystemIndex index) {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return *contexts_[index].get();
  }

 protected:
  /// Protected copy constructor takes care of the local data members and
  /// all base class members, but doesn't update base class pointers so is
  /// not a complete copy.
  DiagramContext(const DiagramContext& source)
      : Context<T>(source),
        contexts_(source.num_subcontexts()),
        state_(std::make_unique<DiagramState<T>>(source.num_subcontexts())) {
    // Clone all the subsystem contexts.
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      DRAKE_DEMAND(source.contexts_[i] != nullptr);
      AddSystem(i, Context<T>::CloneWithoutPointers(*source.contexts_[i]));
    }

    // Build a superstate over the subsystem contexts.
    MakeState();

    // Build superparameters over the subsystem contexts.
    MakeParameters();

    // Everything else was handled by the Context<T> copy constructor.
  }

 private:
  friend class DiagramContextTest;
  using ContextBase::AddInputPort;    // For DiagramContextTest.
  using ContextBase::AddOutputPort;

  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::unique_ptr<ContextBase>(new DiagramContext<T>(*this));
  }

  std::unique_ptr<State<T>> DoCloneState() const final {
    auto clone = std::make_unique<DiagramState<T>>(num_subcontexts());

    for (SubsystemIndex i(0); i < num_subcontexts(); i++) {
      Context<T>* context = contexts_[i].get();
      clone->set_and_own_substate(i, context->CloneState());
    }

    clone->Finalize();
    return clone;
  }

  // Print summary information for the diagram context and recurse into
  // the (non-empty) subcontexts.
  std::string do_to_string() const final;

  // Returns the number of immediate child subcontexts in this DiagramContext.
  int num_subcontexts() const {
    return static_cast<int>(contexts_.size());
  }

  const State<T>& do_access_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& do_access_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  // Recursively sets the time on all subcontexts.
  void DoPropagateTimeChange(const T& time_sec, int64_t change_event) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      Context<T>::PropagateTimeChange(&*subcontext, time_sec, change_event);
    }
  }

  // Recursively sets the accuracy on all subcontexts.
  void DoPropagateAccuracyChange(const optional<double>& accuracy,
                                 int64_t change_event) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      Context<T>::PropagateAccuracyChange(&*subcontext, accuracy, change_event);
    }
  }

  // Recursively notifies subcontexts of bulk changes.
  void DoPropagateBulkChange(
      int64_t change_event,
      void (ContextBase::*note_bulk_change)(int64_t change_event)) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      ContextBase::PropagateBulkChange(&*subcontext, change_event,
                                       note_bulk_change);
    }
  }

  // Recursively notifies subcontexts of some caching behavior change.
  void DoPropagateCachingChange(
      void (Cache::*caching_change)()) const final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      ContextBase::PropagateCachingChange(*subcontext, caching_change);
    }
  }

  // For this method `this` is the source being copied into `clone`.
  void DoPropagateBuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const final {
    auto& clone_diagram = dynamic_cast<const DiagramContext<T>&>(clone);
    DRAKE_DEMAND(clone_diagram.contexts_.size() == contexts_.size());
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      ContextBase::BuildTrackerPointerMap(
          *contexts_[i], *clone_diagram.contexts_[i], &*tracker_map);
    }
  }

  // For this method, `this` is the clone copied from `source`.
  void DoPropagateFixContextPointers(
      const ContextBase& source,
      const DependencyTracker::PointerMap& tracker_map) final {
    auto& source_diagram = dynamic_cast<const DiagramContext<T>&>(source);
    DRAKE_DEMAND(contexts_.size() == source_diagram.contexts_.size());
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      ContextBase::FixContextPointers(*source_diagram.contexts_[i], tracker_map,
                                      &*contexts_[i]);
    }
  }

  // The contexts are stored in SubsystemIndex order, and contexts_ is equal in
  // length to the number of subsystems specified at construction time.
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // The internal state of the Diagram, which includes all its subsystem states.
  std::unique_ptr<DiagramState<T>> state_;
};

}  // namespace systems
}  // namespace drake
