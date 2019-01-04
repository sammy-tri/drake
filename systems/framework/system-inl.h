#pragma once

/// @file
/// Template method implementations for system.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

#include <map>
#include <memory>
#include <string>
#include <utility>

/* clang-format off to disable clang-format-includes */
#include "drake/systems/framework/system.h"
/* clang-format on */

namespace drake {
namespace systems {

template <typename T>
std::unique_ptr<Context<T>> System<T>::AllocateContext() const {
  return dynamic_pointer_cast_or_throw<Context<T>>(
      SystemBase::AllocateContext());
}

template <typename T>
std::unique_ptr<BasicVector<T>> System<T>::AllocateInputVector(
    const InputPort<T>& input_port) const {
  DRAKE_THROW_UNLESS(input_port.get_data_type() == kVectorValued);
  const int index = input_port.get_index();
  DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
  DRAKE_ASSERT(get_input_port(index).get_data_type() == kVectorValued);
  std::unique_ptr<AbstractValue> value = DoAllocateInput(input_port);
  return value->GetMutableValue<BasicVector<T>>().Clone();
}

template <typename T>
std::unique_ptr<AbstractValue> System<T>::AllocateInputAbstract(
    const InputPort<T>& input_port) const {
  const int index = input_port.get_index();
  DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
  return DoAllocateInput(input_port);
}

template <typename T>
std::unique_ptr<SystemOutput<T>> System<T>::AllocateOutput() const {
  // make_unique can't invoke this private constructor.
  auto output = std::unique_ptr<SystemOutput<T>>(new SystemOutput<T>());
  for (int i = 0; i < this->get_num_output_ports(); ++i) {
    const OutputPort<T>& port = this->get_output_port(i);
    output->add_port(port.Allocate());
  }
  return output;
}

template <typename T>
std::unique_ptr<Context<T>> System<T>::CreateDefaultContext() const {
  std::unique_ptr<Context<T>> context = AllocateContext();
  SetDefaultContext(context.get());
  return context;
}

template <typename T>
void System<T>::SetDefaultContext(Context<T>* context) const {
  // Set the default state, checking that the number of state variables does
  // not change.
  const int n_xc = context->get_continuous_state().size();
  const int n_xd = context->get_num_discrete_state_groups();
  const int n_xa = context->get_num_abstract_states();

  SetDefaultState(*context, &context->get_mutable_state());

  DRAKE_DEMAND(n_xc == context->get_continuous_state().size());
  DRAKE_DEMAND(n_xd == context->get_num_discrete_state_groups());
  DRAKE_DEMAND(n_xa == context->get_num_abstract_states());

  // Set the default parameters, checking that the number of parameters does
  // not change.
  const int num_params = context->num_numeric_parameter_groups();
  SetDefaultParameters(*context, &context->get_mutable_parameters());
  DRAKE_DEMAND(num_params == context->num_numeric_parameter_groups());
}

template <typename T>
void System<T>::SetRandomContext(
    Context<T>* context, RandomGenerator* generator) const {
  // Set the default state, checking that the number of state variables does
  // not change.
  const int n_xc = context->get_continuous_state().size();
  const int n_xd = context->get_num_discrete_state_groups();
  const int n_xa = context->get_num_abstract_states();

  SetRandomState(*context, &context->get_mutable_state(), generator);

  DRAKE_DEMAND(n_xc == context->get_continuous_state().size());
  DRAKE_DEMAND(n_xd == context->get_num_discrete_state_groups());
  DRAKE_DEMAND(n_xa == context->get_num_abstract_states());

  // Set the default parameters, checking that the number of parameters does
  // not change.
  const int num_params = context->num_numeric_parameter_groups();
  SetRandomParameters(*context, &context->get_mutable_parameters(),
                      generator);
  DRAKE_DEMAND(num_params == context->num_numeric_parameter_groups());
}

template <typename T>
void System<T>::AllocateFixedInputs(Context<T>* context) const {
  for (InputPortIndex i(0); i < get_num_input_ports(); ++i) {
    const InputPort<T>& port = get_input_port(i);
    if (port.get_data_type() == kVectorValued) {
      context->FixInputPort(port.get_index(), AllocateInputVector(port));
    } else {
      DRAKE_DEMAND(port.get_data_type() == kAbstractValued);
      context->FixInputPort(port.get_index(), AllocateInputAbstract(port));
    }
  }
}

template <typename T>
bool System<T>::HasDirectFeedthrough(int output_port) const {
  std::multimap<int, int> pairs = GetDirectFeedthroughs();
  for (const auto& pair : pairs) {
    if (pair.second == output_port) return true;
  }
  return false;
}

template <typename T>
bool System<T>::HasDirectFeedthrough(int input_port, int output_port) const {
  std::multimap<int, int> pairs = GetDirectFeedthroughs();
  auto range = pairs.equal_range(input_port);
    for (auto i = range.first; i != range.second; ++i) {
      if (i->second == output_port) return true;
    }
    return false;
}

template <typename T>
const ContinuousState<T>& System<T>::EvalTimeDerivatives(
    const Context<T>& context) const {
  const CacheEntry& entry =
      this->get_cache_entry(time_derivatives_cache_index_);
  return entry.Eval<ContinuousState<T>>(context);
}

template <typename T>
const T& System<T>::EvalPotentialEnergy(const Context<T>& context) const {
  const CacheEntry& entry =
      this->get_cache_entry(potential_energy_cache_index_);
  return entry.Eval<T>(context);
}

template <typename T>
const T& System<T>::EvalKineticEnergy(const Context<T>& context) const {
  const CacheEntry& entry =
      this->get_cache_entry(kinetic_energy_cache_index_);
  return entry.Eval<T>(context);
}

template <typename T>
const T& System<T>::EvalConservativePower(const Context<T>& context) const {
  const CacheEntry& entry =
      this->get_cache_entry(conservative_power_cache_index_);
  return entry.Eval<T>(context);
}

template <typename T>
const T& System<T>::EvalNonConservativePower(const Context<T>& context) const {
  const CacheEntry& entry =
      this->get_cache_entry(nonconservative_power_cache_index_);
  return entry.Eval<T>(context);
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> System<T>::EvalEigenVectorInput(
    const Context<T>& context, int port_index) const {
  if (port_index < 0)
    ThrowNegativePortIndex(__func__, port_index);
  const InputPortIndex port(port_index);

  const BasicVector<T>* const basic_value =
      EvalBasicVectorInputImpl(__func__, context, port);
  if (basic_value == nullptr)
    ThrowCantEvaluateInputPort(__func__, port);

  return basic_value->get_value();
}

template <typename T>
optional<PeriodicEventData>
System<T>::GetUniquePeriodicDiscreteUpdateAttribute() const {
  optional<PeriodicEventData> saved_attr;
  auto periodic_events = GetPeriodicEvents();
  for (const auto& saved_attr_and_vector : periodic_events) {
    for (const auto& event : saved_attr_and_vector.second) {
      if (event->is_discrete_update()) {
        if (saved_attr)
          return nullopt;
        saved_attr = saved_attr_and_vector.first;
        break;
      }
    }
  }

  return saved_attr;
}

template <typename T>
void System<T>::CalcOutput(
    const Context<T>& context, SystemOutput<T>* outputs) const {
  DRAKE_DEMAND(outputs != nullptr);
  DRAKE_ASSERT_VOID(CheckValidContext(context));
  DRAKE_ASSERT_VOID(CheckValidOutput(outputs));
  for (OutputPortIndex i(0); i < get_num_output_ports(); ++i) {
    // TODO(sherm1) Would be better to use Eval() here but we don't have
    // a generic abstract assignment capability that would allow us to
    // copy into existing memory in `outputs` (rather than clone). User
    // code depends on memory stability in SystemOutput.
    get_output_port(i).Calc(context, outputs->GetMutableData(i));
  }
}

template <typename T>
void System<T>::CalcUnrestrictedUpdate(
    const Context<T>& context,
    const EventCollection<UnrestrictedUpdateEvent<T>>& events,
    State<T>* state) const {
  DRAKE_ASSERT_VOID(CheckValidContext(context));
  const int continuous_state_dim = state->get_continuous_state().size();
  const int discrete_state_dim = state->get_discrete_state().num_groups();
  const int abstract_state_dim = state->get_abstract_state().size();

  // Copy current state to the passed-in state, as specified in the
  // documentation for DoCalcUnrestrictedUpdate().
  state->CopyFrom(context.get_state());

  DispatchUnrestrictedUpdateHandler(context, events, state);

  if (continuous_state_dim != state->get_continuous_state().size() ||
      discrete_state_dim != state->get_discrete_state().num_groups() ||
      abstract_state_dim != state->get_abstract_state().size())
    throw std::logic_error(
        "State variable dimensions cannot be changed "
        "in CalcUnrestrictedUpdate().");
}

template <typename T>
std::string System<T>::GetMemoryObjectName() const {
  return SystemImpl::GetMemoryObjectName(NiceTypeName::Get(*this),
                                         GetGraphvizId());
}

template <typename T>
const InputPort<T>& System<T>::get_input_port(int port_index) const {
  return dynamic_cast<const InputPort<T>&>(
      this->GetInputPortBaseOrThrow(__func__, port_index));
}

template <typename T>
const InputPort<T>& System<T>::GetInputPort(
    const std::string& port_name) const {
  for (InputPortIndex i{0}; i < get_num_input_ports(); i++) {
    if (port_name == get_input_port_base(i).get_name()) {
      return get_input_port(i);
    }
  }
  throw std::logic_error("System " + GetSystemName() +
                         " does not have an input port named " +
                         port_name);
}

template <typename T>
const OutputPort<T>& System<T>::get_output_port(int port_index) const {
  return dynamic_cast<const OutputPort<T>&>(
      this->GetOutputPortBaseOrThrow(__func__, port_index));
}

template <typename T>
const OutputPort<T>& System<T>::GetOutputPort(
    const std::string& port_name) const {
  for (OutputPortIndex i{0}; i < get_num_output_ports(); i++) {
    if (port_name == get_output_port_base(i).get_name()) {
      return get_output_port(i);
    }
  }
  throw std::logic_error("System " + GetSystemName() +
                         " does not have an output port named " +
                         port_name);
}

template <typename T>
const SystemConstraint<T>& System<T>::get_constraint(
    SystemConstraintIndex constraint_index) const {
  if (constraint_index < 0 || constraint_index >= get_num_constraints()) {
    throw std::out_of_range("System " + get_name() + ": Constraint index " +
                            std::to_string(constraint_index) +
                            " is out of range. There are only " +
                            std::to_string(get_num_constraints()) +
                            " constraints.");
  }
  return *constraints_[constraint_index];
}

template <typename T>
boolean<T> System<T>::CheckSystemConstraintsSatisfied(
    const Context<T>& context, double tol) const {
  DRAKE_DEMAND(tol >= 0.0);
  boolean<T> result{true};
  for (const auto& constraint : constraints_) {
    result = result && constraint->CheckSatisfied(context, tol);
    // If T is a real number (not a symbolic expression), we can bail out
    // early with a diagnostic when the first constraint fails.
    if (scalar_predicate<T>::is_bool && !result) {
      SPDLOG_DEBUG(drake::log(),
                   "Context fails to satisfy SystemConstraint {}",
                   constraint->description());
      return result;
    }
  }
  return result;
}

template <typename T>
void System<T>::CheckValidOutput(const SystemOutput<T>* output) const {
  DRAKE_THROW_UNLESS(output != nullptr);

  // Checks that the number of output ports in the system output is consistent
  // with the number of output ports declared by the System.
  DRAKE_THROW_UNLESS(output->get_num_ports() == get_num_output_ports());

  // Checks the validity of each output port.
  for (int i = 0; i < get_num_output_ports(); ++i) {
    // TODO(sherm1): consider adding (very expensive) validation of the
    // abstract ports also.
    if (get_output_port(i).get_data_type() == kVectorValued) {
      const BasicVector<T>* output_vector = output->get_vector_data(i);
      DRAKE_THROW_UNLESS(output_vector != nullptr);
      DRAKE_THROW_UNLESS(output_vector->size() == get_output_port(i).size());
    }
  }
}

template <typename T>
std::string System<T>::GetGraphvizString(int max_depth) const {
  DRAKE_DEMAND(max_depth >= 0);
  std::stringstream dot;
  dot << "digraph _" << this->GetGraphvizId() << " {" << std::endl;
  dot << "rankdir=LR" << std::endl;
  GetGraphvizFragment(max_depth, &dot);
  dot << "}" << std::endl;
  return dot.str();
}

template <typename T>
std::unique_ptr<System<AutoDiffXd>> System<T>::ToAutoDiffXd() const {
  return System<T>::ToAutoDiffXd(*this);
}

template <typename T>
std::unique_ptr<System<AutoDiffXd>> System<T>::ToAutoDiffXdMaybe() const {
  return system_scalar_converter_.Convert<AutoDiffXd, T>(*this);
}

template <typename T>
std::unique_ptr<System<symbolic::Expression>> System<T>::ToSymbolic() const {
  return System<T>::ToSymbolic(*this);
}

template <typename T>
std::unique_ptr<System<symbolic::Expression>>
System<T>::ToSymbolicMaybe() const {
  return system_scalar_converter_.Convert<symbolic::Expression, T>(*this);
}

template <typename T>
void System<T>::FixInputPortsFrom(const System<double>& other_system,
                                  const Context<double>& other_context,
                                  Context<T>* target_context) const {
  DRAKE_ASSERT_VOID(CheckValidContextT(other_context));
  DRAKE_ASSERT_VOID(CheckValidContext(*target_context));
  DRAKE_ASSERT_VOID(other_system.CheckValidContext(other_context));
  DRAKE_ASSERT_VOID(other_system.CheckValidContextT(*target_context));

  for (int i = 0; i < get_num_input_ports(); ++i) {
    const auto& input_port = get_input_port(i);

    if (input_port.get_data_type() == kVectorValued) {
      // For vector-valued input ports, we placewise initialize a fixed input
      // vector using the explicit conversion from double to T.
      const BasicVector<double>* other_vec =
          other_system.EvalVectorInput(other_context, i);
      if (other_vec == nullptr) continue;
      auto our_vec = this->AllocateInputVector(input_port);
      for (int j = 0; j < our_vec->size(); ++j) {
        our_vec->SetAtIndex(j, T(other_vec->GetAtIndex(j)));
      }
      target_context->FixInputPort(i, *our_vec);
    } else if (input_port.get_data_type() == kAbstractValued) {
      // For abstract-valued input ports, we just clone the value and fix
      // it to the port.
      const AbstractValue* other_value =
          other_system.EvalAbstractInput(other_context, i);
      if (other_value == nullptr) continue;
      target_context->FixInputPort(i, other_value->Clone());
    } else {
      DRAKE_ABORT_MSG("Unknown input port type.");
    }
  }
}

template <typename T>
System<T>::System(SystemScalarConverter converter)
    : system_scalar_converter_(std::move(converter)) {
  // Note that configuration and kinematics tickets also include dependence
  // on parameters and accuracy, but not time or input ports.

  // Potential and kinetic energy, and conservative power that measures
  // the transfer between them, must _not_ be (explicitly) time dependent.
  // See API documentation above for Eval{Potential|Kinetic}Energy() and
  // EvalConservativePower() to see why.

  // TODO(sherm1) Due to issue #9171 we cannot always recognize which
  // variables contribute to configuration so we'll invalidate on all changes.
  // Use configuration, kinematics, and mass tickets when #9171 is resolved.
  potential_energy_cache_index_ =
      DeclareCacheEntry(
          "potential energy",
          &System<T>::CalcPotentialEnergy,
          {all_sources_ticket()})  // After #9171: configuration + mass.
      .cache_index();

  kinetic_energy_cache_index_ =
      DeclareCacheEntry(
          "kinetic energy",
          &System<T>::CalcKineticEnergy,
          {all_sources_ticket()})  // After #9171: kinematics + mass.
      .cache_index();

  conservative_power_cache_index_ =
      DeclareCacheEntry(
          "conservative power",
          &System<T>::CalcConservativePower,
          {all_sources_ticket()})  // After #9171: kinematics + mass.
      .cache_index();

  // Only non-conservative power can have an explicit time or input
  // port dependence. See API documentation above for
  // EvalNonConservativePower() to see why.
  nonconservative_power_cache_index_ =
      DeclareCacheEntry("non-conservative power",
                        &System<T>::CalcNonConservativePower,
                        {all_sources_ticket()})  // This is correct.
      .cache_index();

  // For the time derivative cache we need to use the general form for
  // cache creation because we're dealing with pre-defined allocator and
  // calculator method signatures.
  CacheEntry::AllocCallback alloc_derivatives = [this]() {
    return std::make_unique<Value<ContinuousState<T>>>(
        this->AllocateTimeDerivatives());
  };
  CacheEntry::CalcCallback calc_derivatives = [this](
      const ContextBase& context_base, AbstractValue* result) {
    DRAKE_DEMAND(result != nullptr);
    ContinuousState<T>& state = result->GetMutableValue<ContinuousState<T>>();
    const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
    CalcTimeDerivatives(context, &state);
  };

  // We must assume that time derivatives can depend on *any* context source.
  time_derivatives_cache_index_ =
      this->DeclareCacheEntryWithKnownTicket(
          xcdot_ticket(), "time derivatives",
          std::move(alloc_derivatives), std::move(calc_derivatives),
          {all_sources_ticket()})
      .cache_index();

  // TODO(sherm1) Allocate and use discrete update cache.
}

template <typename T>
const InputPort<T>& System<T>::DeclareInputPort(
    variant<std::string, UseDefaultName> name, PortDataType type, int size,
    optional<RandomDistribution> random_type) {
  const InputPortIndex port_index(get_num_input_ports());

  const DependencyTicket port_ticket(this->assign_next_dependency_ticket());
  this->AddInputPort(std::make_unique<InputPort<T>>(
      this, this, NextInputPortName(std::move(name)), port_index, port_ticket,
      type, size, random_type));
  return get_input_port(port_index);
}

template <typename T>
void System<T>::DoMapQDotToVelocity(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    VectorBase<T>* generalized_velocity) const {
  unused(context);
  // In the particular case where generalized velocity and generalized
  // configuration are not even the same size, we detect this error and abort.
  // This check will thus not identify cases where the generalized velocity
  // and time derivative of generalized configuration are identically sized
  // but not identical!
  const int n = qdot.size();
  // You need to override System<T>::DoMapQDottoVelocity!
  DRAKE_THROW_UNLESS(generalized_velocity->size() == n);
  generalized_velocity->SetFromVector(qdot);
}

template <typename T>
void System<T>::DoMapVelocityToQDot(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    VectorBase<T>* qdot) const {
  unused(context);
  // In the particular case where generalized velocity and generalized
  // configuration are not even the same size, we detect this error and abort.
  // This check will thus not identify cases where the generalized velocity
  // and time derivative of generalized configuration are identically sized
  // but not identical!
  const int n = generalized_velocity.size();
  // You need to override System<T>::DoMapVelocityToQDot!
  DRAKE_THROW_UNLESS(qdot->size() == n);
  qdot->SetFromVector(generalized_velocity);
}

template <typename T>
void System<T>::DoCheckValidContext(const ContextBase& context_base) const {
  const Context<T>* context = dynamic_cast<const Context<T>*>(&context_base);
  DRAKE_THROW_UNLESS(context != nullptr);
  CheckValidContextT(*context);
}

template <typename T>
std::function<void(const AbstractValue&)>
System<T>::MakeFixInputPortTypeChecker(
    InputPortIndex port_index) const {
  const InputPort<T>& port = this->get_input_port(port_index);
  const std::string pathname = this->GetSystemPathname();

  // Note that our lambdas below will capture all necessary items by-value,
  // so that they do not rely on this System still being alive.  (We do not
  // allow a Context and System to have pointers to each other.)
  switch (port.get_data_type()) {
    case kAbstractValued: {
      // For abstract inputs, we only need to ensure that both runtime values
      // share the same base T in the Value<T>. Even if the System declared a
      // model_value that was a subtype of T, there is no EvalInputValue
      // sugar that allows the System to evaluate the input by downcasting to
      // that subtype, so here we should not insist that some dynamic_cast
      // would succeed. If the user writes the downcast on their own, it's
      // fine to let them also handle detailed error reporting on their own.
      const std::type_info& expected_type =
          this->AllocateInputAbstract(port)->static_type_info();
      return [&expected_type, port_index, pathname](
          const AbstractValue& actual) {
        if (actual.static_type_info() != expected_type) {
          SystemBase::ThrowInputPortHasWrongType(
              "FixInputPortTypeCheck", pathname, port_index,
              NiceTypeName::Get(expected_type),
              NiceTypeName::Get(actual.type_info()));
        }
      };
    }
    case kVectorValued: {
      // For vector inputs, check that the size is the same.
      // TODO(jwnimmer-tri) We should type-check the vector, eventually.
      const std::unique_ptr<BasicVector<T>> model_vector =
          this->AllocateInputVector(port);
      const int expected_size = model_vector->size();
      return [expected_size, port_index, pathname](
          const AbstractValue& actual) {
        const BasicVector<T>* const actual_vector =
            actual.MaybeGetValue<BasicVector<T>>();
        if (actual_vector == nullptr) {
          SystemBase::ThrowInputPortHasWrongType(
              "FixInputPortTypeCheck", pathname, port_index,
              NiceTypeName::Get<Value<BasicVector<T>>>(),
              NiceTypeName::Get(actual));
        }
        // Check that vector sizes match.
        if (actual_vector->size() != expected_size) {
          SystemBase::ThrowInputPortHasWrongType(
              "FixInputPortTypeCheck", pathname, port_index,
              fmt::format("{} with size={}",
                          NiceTypeName::Get<BasicVector<T>>(),
                          expected_size),
              fmt::format("{} with size={}",
                          NiceTypeName::Get(*actual_vector),
                          actual_vector->size()));
        }
      };
    }
  }
  DRAKE_ABORT();
}

template <typename T>
const BasicVector<T>* System<T>::EvalBasicVectorInputImpl(
    const char* func, const Context<T>& context,
    InputPortIndex port_index) const {
  // Make sure this is the right kind of port before worrying about whether
  // it is connected up properly.
  const InputPortBase& port = GetInputPortBaseOrThrow(func, port_index);
  if (port.get_data_type() != kVectorValued)
    ThrowNotAVectorInputPort(func, port_index);

  // If there is no value at all, the port is not connected which is not
  // a problem here.
  const AbstractValue* const abstract_value =
      EvalAbstractInputImpl(func, context, port_index);
  if (abstract_value == nullptr) {
    return nullptr;
  }

  // We have a vector port with a value, it better be a BasicVector!
  const BasicVector<T>* const basic_value =
      abstract_value->MaybeGetValue<BasicVector<T>>();
  DRAKE_DEMAND(basic_value != nullptr);

  // Shouldn't have been possible to create this vector-valued port with
  // the wrong size.
  DRAKE_DEMAND(basic_value->size() == port.size());

  return basic_value;
}

}  // namespace systems
}  // namespace drake
