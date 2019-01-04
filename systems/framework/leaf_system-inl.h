#pragma once

/// @file
/// Template method implementations for system.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/pointer_cast.h"

/* clang-format off to disable clang-format-includes */
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system-inl.h"
/* clang-format on */

namespace drake {
namespace systems {

namespace leaf_system_detail {

// Returns the next sample time for the given @p attribute.
template <typename T>
static T GetNextSampleTime(
    const PeriodicEventData& attribute,
    const T& current_time_sec) {
  const double period = attribute.period_sec();
  DRAKE_ASSERT(period > 0);
  const double offset = attribute.offset_sec();
  DRAKE_ASSERT(offset >= 0);

  // If the first sample time hasn't arrived yet, then that is the next
  // sample time.
  if (current_time_sec < offset) {
    return offset;
  }

  // Compute the index in the sequence of samples for the next time to sample,
  // which should be greater than the present time.
  using std::ceil;
  const T offset_time = current_time_sec - offset;
  const T next_k = ceil(offset_time / period);
  T next_t = offset + next_k * period;
  if (next_t <= current_time_sec) {
    next_t = offset + (next_k + 1) * period;
  }
  DRAKE_ASSERT(next_t > current_time_sec);
  return next_t;
}

}  // namespace leaf_system_detail

template <typename T>
std::unique_ptr<CompositeEventCollection<T>>
LeafSystem<T>::AllocateCompositeEventCollection() const {
  return std::make_unique<LeafCompositeEventCollection<T>>();
}

template <typename T>
std::unique_ptr<LeafContext<T>> LeafSystem<T>::AllocateContext() const {
  return dynamic_pointer_cast_or_throw<LeafContext<T>>(
      System<T>::AllocateContext());
}

template <typename T>
std::unique_ptr<EventCollection<PublishEvent<T>>>
LeafSystem<T>::AllocateForcedPublishEventCollection() const {
  return LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection();
}

template <typename T>
std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
LeafSystem<T>::AllocateForcedDiscreteUpdateEventCollection() const {
  return LeafEventCollection<
    DiscreteUpdateEvent<T>>::MakeForcedEventCollection();
}

template <typename T>
std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
LeafSystem<T>::AllocateForcedUnrestrictedUpdateEventCollection() const {
  return LeafEventCollection<
    UnrestrictedUpdateEvent<T>>::MakeForcedEventCollection();
}

template <typename T>
std::unique_ptr<ContextBase> LeafSystem<T>::DoAllocateContext() const {
  std::unique_ptr<LeafContext<T>> context = DoMakeLeafContext();
  this->InitializeContextBase(&*context);

  // Reserve parameters via delegation to subclass.
  context->init_parameters(this->AllocateParameters());

  // Reserve state via delegation to subclass.
  context->init_continuous_state(this->AllocateContinuousState());
  context->init_discrete_state(this->AllocateDiscreteState());
  context->init_abstract_state(this->AllocateAbstractState());

  // At this point this LeafContext is complete except possibly for
  // inter-Context dependencies involving port connections to peers or
  // parent. We can now perform some final sanity checks.

  // The numeric vectors used for parameters and state must be contiguous,
  // i.e., valid BasicVectors. In general, a Context's numeric vectors can be
  // any kind of VectorBase including scatter-gather implementations like
  // Supervector. But for a LeafContext, we only allow BasicVectors, which are
  // guaranteed to have a contiguous storage layout.

  // If xc is not BasicVector, the dynamic_cast will yield nullptr, and the
  // invariant-checker will complain.
  const VectorBase<T>* const xc = &context->get_continuous_state_vector();
  detail::CheckBasicVectorInvariants(dynamic_cast<const BasicVector<T>*>(xc));

  // The discrete state must all be valid BasicVectors.
  for (const BasicVector<T>* group :
           context->get_state().get_discrete_state().get_data()) {
    detail::CheckBasicVectorInvariants(group);
  }

  // The numeric parameters must all be valid BasicVectors.
  const int num_numeric_parameters =
      context->num_numeric_parameter_groups();
  for (int i = 0; i < num_numeric_parameters; ++i) {
    const BasicVector<T>& group = context->get_numeric_parameter(i);
    detail::CheckBasicVectorInvariants(&group);
  }

  // Allow derived LeafSystem to validate allocated Context.
  DoValidateAllocatedLeafContext(*context);

  // TODO(sherm1) Remove this line and the corresponding one in
  // Diagram to enable caching by default in Drake.
  context->DisableCaching();

  return context;
}

template <typename T>
void LeafSystem<T>::SetDefaultState(const Context<T>& context,
                                    State<T>* state) const {
  unused(context);
  DRAKE_DEMAND(state != nullptr);
  ContinuousState<T>& xc = state->get_mutable_continuous_state();
  if (model_continuous_state_vector_ != nullptr) {
    xc.SetFromVector(model_continuous_state_vector_->get_value());
  } else {
    xc.SetFromVector(VectorX<T>::Zero(xc.size()));
  }

  DiscreteValues<T>& xd = state->get_mutable_discrete_state();

  // Check that _if_ we have models, there is one for each group.
  DRAKE_DEMAND(model_discrete_state_.num_groups() == 0 ||
               model_discrete_state_.num_groups() == xd.num_groups());

  if (model_discrete_state_.num_groups() > 0) {
    xd.CopyFrom(model_discrete_state_);
  } else {
    // With no model vector, we just zero all the discrete variables.
    for (int i = 0; i < xd.num_groups(); i++) {
      BasicVector<T>& s = xd.get_mutable_vector(i);
      s.SetFromVector(VectorX<T>::Zero(s.size()));
    }
  }

  AbstractValues& xa = state->get_mutable_abstract_state();
  xa.CopyFrom(AbstractValues(model_abstract_states_.CloneAllModels()));
}

template <typename T>
void LeafSystem<T>::SetDefaultParameters(const Context<T>& context,
                                         Parameters<T>* parameters) const {
  unused(context);
  for (int i = 0; i < parameters->num_numeric_parameter_groups(); i++) {
    BasicVector<T>& p = parameters->get_mutable_numeric_parameter(i);
    auto model_vector = model_numeric_parameters_.CloneVectorModel<T>(i);
    if (model_vector != nullptr) {
      p.SetFrom(*model_vector);
    } else {
      p.SetFromVector(VectorX<T>::Constant(p.size(), 1.0));
    }
  }
  for (int i = 0; i < parameters->num_abstract_parameters(); i++) {
    AbstractValue& p = parameters->get_mutable_abstract_parameter(i);
    auto model_value = model_abstract_parameters_.CloneModel(i);
    p.SetFrom(*model_value);
  }
}

template <typename T>
std::multimap<int, int> LeafSystem<T>::GetDirectFeedthroughs() const {
  // A helper object that is latch-initialized the first time it is needed,
  // but not before.  The optional<> wrapper represents whether or not the
  // latch-init has been attempted; the unique_ptr's non-nullness represents
  // whether or not symbolic form is supported.
  optional<std::unique_ptr<SystemSymbolicInspector>> inspector;

  // This predicate answers a feedthrough query using symbolic form, or
  // returns "true" if symbolic form is unavailable.  It is lazy, in that it
  // will not create the symbolic form until the first time it is invoked.
  auto inspect_symbolic_feedthrough = [this, &inspector](int u, int v) {
    // The very first time we are called, latch-initialize the inspector.
    if (!inspector) { inspector = MakeSystemSymbolicInspector(); }

    // If we have an inspector, delegate to it.  Otherwise, be conservative.
    if (SystemSymbolicInspector* inspector_value = inspector.value().get()) {
      return inspector_value->IsConnectedInputToOutput(u, v);
    } else {
      return true;
    }
  };

  // Iterate all input-output pairs, populating the map with the "true" terms.
  std::multimap<int, int> pairs;
  for (int u = 0; u < this->get_num_input_ports(); ++u) {
    for (int v = 0; v < this->get_num_output_ports(); ++v) {
      // Ask our subclass whether it wants to directly express feedthrough.
      const optional<bool> overridden_feedthrough =
          DoHasDirectFeedthrough(u, v);
      // If our subclass didn't provide an answer, use symbolic form instead.
      const bool direct_feedthrough =
          overridden_feedthrough ? overridden_feedthrough.value() :
          inspect_symbolic_feedthrough(u, v);
      if (direct_feedthrough) {
        pairs.emplace(u, v);
      }
    }
  }
  return pairs;
}

template <typename T>
LeafSystem<T>::LeafSystem() : LeafSystem(SystemScalarConverter{}) {}

template <typename T>
LeafSystem<T>::LeafSystem(SystemScalarConverter converter)
    : System<T>(std::move(converter)) {
  this->set_forced_publish_events(
      LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection());
  this->set_forced_discrete_update_events(
      LeafEventCollection<
      DiscreteUpdateEvent<T>>::MakeForcedEventCollection());
  this->set_forced_unrestricted_update_events(
      LeafEventCollection<
      UnrestrictedUpdateEvent<T>>::MakeForcedEventCollection());
}

template <typename T>
std::unique_ptr<LeafContext<T>> LeafSystem<T>::DoMakeLeafContext() const {
  return std::make_unique<LeafContext<T>>();
}

template <typename T>
void LeafSystem<T>::AddTriggeredWitnessFunctionToCompositeEventCollection(
    Event<T>* event,
    CompositeEventCollection<T>* events) const {
  DRAKE_DEMAND(event);
  DRAKE_DEMAND(event->get_event_data());
  DRAKE_DEMAND(dynamic_cast<const WitnessTriggeredEventData<T>*>(
      event->get_event_data()));
  DRAKE_DEMAND(events);
  event->AddToComposite(events);
}

template <typename T>
void LeafSystem<T>::DoCalcNextUpdateTime(
    const Context<T>& context,
    CompositeEventCollection<T>* events,
    T* time) const {
  T min_time = std::numeric_limits<double>::infinity();

  if (periodic_events_.empty()) {
    *time = min_time;
    return;
  }

  // Find the minimum next sample time across all registered events, and
  // the set of registered events that will occur at that time.
  std::vector<const Event<T>*> next_events;
  for (const auto& event_pair : periodic_events_) {
    const PeriodicEventData& event_data = event_pair.first;
    const Event<T>* const event = event_pair.second.get();
    const T t =
        leaf_system_detail::GetNextSampleTime(event_data, context.get_time());
    if (t < min_time) {
      min_time = t;
      next_events = {event};
    } else if (t == min_time) {
      next_events.push_back(event);
    }
  }

  // Write out the events that fire at min_time.
  *time = min_time;
  for (const Event<T>* event : next_events) {
    event->AddToComposite(events);
  }
}

template <typename T>
void LeafSystem<T>::GetGraphvizFragment(int max_depth,
                                        std::stringstream* dot) const {
  unused(max_depth);

  // Use the this pointer as a unique ID for the node in the dotfile.
  const int64_t id = this->GetGraphvizId();
  std::string name = this->get_name();
  if (name.empty()) {
    name = this->GetMemoryObjectName();
  }

  // Open the attributes and label.
  *dot << id << " [shape=record, label=\"" << name << "|{";

  // Append input ports to the label.
  *dot << "{";
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    if (i != 0) *dot << "|";
    *dot << "<u" << i << ">" << this->get_input_port(i).get_name();
  }
  *dot << "}";

  // Append output ports to the label.
  *dot << " | {";
  for (int i = 0; i < this->get_num_output_ports(); ++i) {
    if (i != 0) *dot << "|";
    *dot << "<y" << i << ">" << this->get_output_port(i).get_name();
  }
  *dot << "}";

  // Close the label and attributes.
  *dot << "}\"];" << std::endl;
}

template <typename T>
void LeafSystem<T>::GetGraphvizInputPortToken(const InputPort<T>& port,
                                              int max_depth,
                                              std::stringstream *dot) const {
  unused(max_depth);
  DRAKE_DEMAND(port.get_system() == this);
  *dot << this->GetGraphvizId() << ":u" << port.get_index();
}

template <typename T>
void LeafSystem<T>::GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                               int max_depth,
                                               std::stringstream *dot) const {
  unused(max_depth);
  DRAKE_DEMAND(&port.get_system() == this);
  *dot << this->GetGraphvizId() << ":y" << port.get_index();
}

template <typename T>
std::unique_ptr<ContinuousState<T>>
LeafSystem<T>::AllocateContinuousState() const {
  if (model_continuous_state_vector_ != nullptr) {
    return std::make_unique<ContinuousState<T>>(
        model_continuous_state_vector_->Clone(), num_generalized_positions_,
        num_generalized_velocities_, num_misc_continuous_states_);
  }
  return std::make_unique<ContinuousState<T>>();
}

template <typename T>
std::unique_ptr<DiscreteValues<T>>
LeafSystem<T>::AllocateDiscreteState() const {
  return model_discrete_state_.Clone();
}

template <typename T>
std::unique_ptr<AbstractValues>
LeafSystem<T>::AllocateAbstractState() const {
  return std::make_unique<AbstractValues>(
      std::move(model_abstract_states_.CloneAllModels()));
}

template <typename T>
std::unique_ptr<Parameters<T>> LeafSystem<T>::AllocateParameters() const {
  std::vector<std::unique_ptr<BasicVector<T>>> numeric_params;
  numeric_params.reserve(model_numeric_parameters_.size());
  for (int i = 0; i < model_numeric_parameters_.size(); ++i) {
    auto param = model_numeric_parameters_.CloneVectorModel<T>(i);
    DRAKE_ASSERT(param != nullptr);
    numeric_params.emplace_back(std::move(param));
  }
  std::vector<std::unique_ptr<AbstractValue>> abstract_params;
  abstract_params.reserve(model_abstract_parameters_.size());
  for (int i = 0; i < model_abstract_parameters_.size(); ++i) {
    auto param = model_abstract_parameters_.CloneModel(i);
    DRAKE_ASSERT(param != nullptr);
    abstract_params.emplace_back(std::move(param));
  }
  return std::make_unique<Parameters<T>>(std::move(numeric_params),
                                         std::move(abstract_params));
}

template <typename T>
int LeafSystem<T>::DeclareNumericParameter(const BasicVector<T>& model_vector) {
  const NumericParameterIndex index(model_numeric_parameters_.size());
  model_numeric_parameters_.AddVectorModel(index, model_vector.Clone());
  MaybeDeclareVectorBaseInequalityConstraint(
      "parameter " + std::to_string(index), model_vector,
      [index](const Context<T>& context) -> const VectorBase<T>& {
        const BasicVector<T>& result = context.get_numeric_parameter(index);
        return result;
      });
  this->AddNumericParameter(index);
  return index;
}

template <typename T>
int LeafSystem<T>::DeclareAbstractParameter(const AbstractValue& model_value) {
  const AbstractParameterIndex index(model_abstract_parameters_.size());
  model_abstract_parameters_.AddModel(index, model_value.Clone());
  this->AddAbstractParameter(index);
  return index;
}

template <typename T>
void LeafSystem<T>::DeclarePeriodicPublish(
    double period_sec, double offset_sec) {
  DeclarePeriodicEvent(period_sec, offset_sec, PublishEvent<T>());
}

template <typename T>
void LeafSystem<T>::DeclarePeriodicDiscreteUpdate(
    double period_sec, double offset_sec) {
  DeclarePeriodicEvent(period_sec, offset_sec, DiscreteUpdateEvent<T>());
}

template <typename T>
void LeafSystem<T>::DeclarePeriodicUnrestrictedUpdate(
    double period_sec, double offset_sec) {
  DeclarePeriodicEvent(period_sec, offset_sec, UnrestrictedUpdateEvent<T>());
}

template <typename T>
void LeafSystem<T>::DeclareContinuousState(
    const BasicVector<T>& model_vector, int num_q,
    int num_v, int num_z) {
  DRAKE_DEMAND(model_vector.size() == num_q + num_v + num_z);
  model_continuous_state_vector_ = model_vector.Clone();
  num_generalized_positions_ = num_q;
  num_generalized_velocities_ = num_v;
  num_misc_continuous_states_ = num_z;
  MaybeDeclareVectorBaseInequalityConstraint(
      "continuous state", model_vector,
      [](const Context<T>& context) -> const VectorBase<T>& {
        const ContinuousState<T>& state = context.get_continuous_state();
        return state.get_vector();
      });
}

template <typename T>
DiscreteStateIndex LeafSystem<T>::DeclareDiscreteState(
    const BasicVector<T>& model_vector) {
  const DiscreteStateIndex index(model_discrete_state_.num_groups());
  model_discrete_state_.AppendGroup(model_vector.Clone());
  this->AddDiscreteStateGroup(index);
  MaybeDeclareVectorBaseInequalityConstraint(
      "discrete state", model_vector,
      [index](const Context<T>& context) -> const VectorBase<T>& {
        const BasicVector<T>& state = context.get_discrete_state(index);
        return state;
      });
  return index;
}

template <typename T>
AbstractStateIndex LeafSystem<T>::DeclareAbstractState(
    std::unique_ptr<AbstractValue> abstract_state) {
  const AbstractStateIndex index(model_abstract_states_.size());
  model_abstract_states_.AddModel(index, std::move(abstract_state));
  this->AddAbstractState(index);
  return index;
}

template <typename T>
const InputPort<T>& LeafSystem<T>::DeclareVectorInputPort(
    variant<std::string, UseDefaultName> name,
    const BasicVector<T>& model_vector,
    optional<RandomDistribution> random_type) {
  const int size = model_vector.size();
  const int index = this->get_num_input_ports();
  model_input_values_.AddVectorModel(index, model_vector.Clone());
  MaybeDeclareVectorBaseInequalityConstraint(
      "input " + std::to_string(index), model_vector,
      [this, index](const Context<T>& context) -> const VectorBase<T>& {
        const BasicVector<T>* input = this->EvalVectorInput(context, index);
        DRAKE_DEMAND(input != nullptr);
        return *input;
      });
  return this->DeclareInputPort(NextInputPortName(std::move(name)),
                                kVectorValued, size, random_type);
}

template <typename T>
const InputPort<T>& LeafSystem<T>::DeclareAbstractInputPort(
    variant<std::string, UseDefaultName> name,
    const AbstractValue& model_value) {
  const int next_index = this->get_num_input_ports();
  model_input_values_.AddModel(next_index, model_value.Clone());
  return this->DeclareInputPort(NextInputPortName(std::move(name)),
                                kAbstractValued, 0 /* size */);
}

template <typename T>
const OutputPort<T>& LeafSystem<T>::DeclareVectorOutputPort(
    variant<std::string, UseDefaultName> name,
    const BasicVector<T>& model_vector,
    typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function,
    std::set<DependencyTicket> prerequisites_of_calc) {
  auto& port = CreateVectorLeafOutputPort(NextOutputPortName(std::move(name)),
      model_vector.size(), MakeAllocCallback(model_vector),
      std::move(vector_calc_function), std::move(prerequisites_of_calc));
  return port;
}


template <typename T>
const OutputPort<T>& LeafSystem<T>::DeclareAbstractOutputPort(
    variant<std::string, UseDefaultName> name,
    typename LeafOutputPort<T>::AllocCallback alloc_function,
    typename LeafOutputPort<T>::CalcCallback calc_function,
    std::set<DependencyTicket> prerequisites_of_calc) {
  auto& port = CreateAbstractLeafOutputPort(
      NextOutputPortName(std::move(name)), std::move(alloc_function),
      std::move(calc_function), std::move(prerequisites_of_calc));
  return port;
}

template <typename T>
SystemConstraintIndex LeafSystem<T>::DeclareInequalityConstraint(
    typename SystemConstraint<T>::CalcCallback calc, int count,
    const std::string& description) {
  return this->AddConstraint(std::make_unique<SystemConstraint<T>>(
      calc, count, SystemConstraintType::kInequality, description));
}

template <typename T>
void LeafSystem<T>::DoPublish(
    const Context<T>& context,
    const std::vector<const PublishEvent<T>*>& events) const {
  for (const PublishEvent<T>* event : events) {
    event->handle(context);
  }
}

template <typename T>
void LeafSystem<T>::DoCalcDiscreteVariableUpdates(
    const Context<T>& context,
    const std::vector<const DiscreteUpdateEvent<T>*>& events,
    DiscreteValues<T>* discrete_state) const {
  for (const DiscreteUpdateEvent<T>* event : events) {
    event->handle(context, discrete_state);
  }
}

template <typename T>
void LeafSystem<T>::DoCalcUnrestrictedUpdate(
    const Context<T>& context,
    const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
    State<T>* state) const {
  for (const UnrestrictedUpdateEvent<T>* event : events) {
    event->handle(context, state);
  }
}

template <typename T>
std::unique_ptr<AbstractValue> LeafSystem<T>::DoAllocateInput(
    const InputPort<T>& input_port) const {
  std::unique_ptr<AbstractValue> model_result =
      model_input_values_.CloneModel(input_port.get_index());
  if (model_result) {
    return model_result;
  }
  if (input_port.get_data_type() == kVectorValued) {
    return std::make_unique<Value<BasicVector<T>>>(input_port.size());
  }
  throw std::logic_error(fmt::format(
      "System::AllocateInputAbstract(): a System with abstract input ports "
      "must pass a model_value to DeclareAbstractInputPort; the port[{}] "
      "named '{}' did not do so (System {})",
      input_port.get_index(), input_port.get_name(),
      this->GetSystemPathname()));
}

template <typename T>
std::map<PeriodicEventData, std::vector<const Event<T>*>,
         PeriodicEventDataComparator>
LeafSystem<T>::DoGetPeriodicEvents() const {
  std::map<PeriodicEventData, std::vector<const Event<T>*>,
           PeriodicEventDataComparator> periodic_events_map;
  for (const auto& i : periodic_events_) {
    periodic_events_map[i.first].push_back(i.second.get());
  }
  return periodic_events_map;
}

template <typename T>
void LeafSystem<T>::DispatchPublishHandler(
    const Context<T>& context,
    const EventCollection<PublishEvent<T>>& events) const {
  const LeafEventCollection<PublishEvent<T>>& leaf_events =
      dynamic_cast<const LeafEventCollection<PublishEvent<T>>&>(events);
  // Only call DoPublish if there are publish events.
  DRAKE_DEMAND(leaf_events.HasEvents());
  this->DoPublish(context, leaf_events.get_events());
}

template <typename T>
void LeafSystem<T>::DispatchDiscreteVariableUpdateHandler(
    const Context<T>& context,
    const EventCollection<DiscreteUpdateEvent<T>>& events,
    DiscreteValues<T>* discrete_state) const {
  const LeafEventCollection<DiscreteUpdateEvent<T>>& leaf_events =
      dynamic_cast<const LeafEventCollection<DiscreteUpdateEvent<T>>&>(
          events);
  // TODO(siyuan): should have a API level CopyFrom for DiscreteValues.
  discrete_state->CopyFrom(context.get_discrete_state());
  // Only call DoCalcDiscreteVariableUpdates if there are discrete update
  // events.
  DRAKE_DEMAND(leaf_events.HasEvents());
  this->DoCalcDiscreteVariableUpdates(context, leaf_events.get_events(),
                                      discrete_state);
}

template <typename T>
void LeafSystem<T>::DispatchUnrestrictedUpdateHandler(
    const Context<T>& context,
    const EventCollection<UnrestrictedUpdateEvent<T>>& events,
    State<T>* state) const {
  const LeafEventCollection<UnrestrictedUpdateEvent<T>>& leaf_events =
      dynamic_cast<const LeafEventCollection<UnrestrictedUpdateEvent<T>>&>(
          events);
  // Only call DoCalcUnrestrictedUpdate if there are unrestricted update
  // events.
  DRAKE_DEMAND(leaf_events.HasEvents());
  this->DoCalcUnrestrictedUpdate(context, leaf_events.get_events(), state);
}

template <typename T>
LeafOutputPort<T>& LeafSystem<T>::CreateVectorLeafOutputPort(
    std::string name,
    int fixed_size,
    typename LeafOutputPort<T>::AllocCallback vector_allocator,
    typename LeafOutputPort<T>::CalcVectorCallback vector_calculator,
    std::set<DependencyTicket> calc_prerequisites) {
  // Construct a suitable type-erased cache calculator from the given
  // BasicVector<T> calculator function.
  auto cache_calc_function = [vector_calculator](
      const ContextBase& context_base, AbstractValue* abstract) {
    auto& context = dynamic_cast<const Context<T>&>(context_base);

    // The abstract value must be a Value<BasicVector<T>>, even if the
    // underlying object is a more-derived vector type.
    auto value = dynamic_cast<Value<BasicVector<T>>*>(abstract);

    // TODO(sherm1) Make this error message more informative by capturing
    // system and port index info.
    if (value == nullptr) {
      throw std::logic_error(fmt::format(
          "An output port calculation required a {} object for its result "
          "but got a {} object instead.",
          NiceTypeName::Get<Value<BasicVector<T>>>(),
          abstract->GetNiceTypeName()));
    }
    vector_calculator(context, &value->get_mutable_value());
  };

  // The allocator function is identical between output port and cache.
  return CreateCachedLeafOutputPort(
      std::move(name), fixed_size, std::move(vector_allocator),
      std::move(cache_calc_function), std::move(calc_prerequisites));
}

template <typename T>
LeafOutputPort<T>& LeafSystem<T>::CreateAbstractLeafOutputPort(
    std::string name,
    typename LeafOutputPort<T>::AllocCallback allocator,
    typename LeafOutputPort<T>::CalcCallback calculator,
    std::set<DependencyTicket> calc_prerequisites) {
  // Construct a suitable type-erased cache calculator from the given
  // type-T calculator function.
  auto cache_calc_function = [calculator](
      const ContextBase& context_base, AbstractValue* result) {
    const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
    return calculator(context, result);
  };

  return CreateCachedLeafOutputPort(
      std::move(name), 0 /* size */, std::move(allocator),
      std::move(cache_calc_function), std::move(calc_prerequisites));
}

template <typename T>
LeafOutputPort<T>& LeafSystem<T>::CreateCachedLeafOutputPort(
    std::string name, int fixed_size,
    typename CacheEntry::AllocCallback allocator,
    typename CacheEntry::CalcCallback calculator,
    std::set<DependencyTicket> calc_prerequisites) {
  DRAKE_DEMAND(!calc_prerequisites.empty());
  // Create a cache entry for this output port.
  const OutputPortIndex oport_index(this->get_num_output_ports());
  const CacheEntry& cache_entry = this->DeclareCacheEntry(
      "output port " + std::to_string(oport_index) + "(" + name + ") cache",
      std::move(allocator), std::move(calculator),
      std::move(calc_prerequisites));

  // Create and install the port. Note that it has a separate ticket from
  // the cache entry; the port's tracker will be subscribed to the cache
  // entry's tracker when a Context is created.
  // TODO(sherm1) Use implicit_cast when available (from abseil).
  auto port = std::make_unique<LeafOutputPort<T>>(
      this,  // implicit_cast<const System<T>*>(this)
      this,  // implicit_cast<const SystemBase*>(this)
      std::move(name),
      oport_index, this->assign_next_dependency_ticket(),
      fixed_size == 0 ? kAbstractValued : kVectorValued, fixed_size,
      &cache_entry);
  LeafOutputPort<T>* const port_ptr = port.get();
  this->AddOutputPort(std::move(port));
  return *port_ptr;
}

template <typename T>
void LeafSystem<T>::MaybeDeclareVectorBaseInequalityConstraint(
    const std::string& kind,
    const VectorBase<T>& model_vector,
    const std::function<const VectorBase<T>&(const Context<T>&)>&
    get_vector_from_context) {
  VectorX<T> dummy_value;
  model_vector.CalcInequalityConstraint(&dummy_value);
  const int count = dummy_value.size();
  if (count == 0) {
    return;
  }
  this->DeclareInequalityConstraint(
      [get_vector_from_context](const Context<T>& con, VectorX<T>* value) {
        get_vector_from_context(con).CalcInequalityConstraint(value);
      },
      count,
      kind + " of type " + NiceTypeName::Get(model_vector));
}

}  // namespace systems
}  // namespace drake
