#pragma once

/// @file
/// Template method implementations for diagram.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

/* clang-format off to disable clang-format-includes */
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system-inl.h"
/* clang-format on */

namespace drake {
namespace systems {

template <typename T>
std::vector<const systems::System<T>*> Diagram<T>::GetSystems() const {
  std::vector<const systems::System<T>*> result;
  result.reserve(registered_systems_.size());
  for (const auto& system : registered_systems_) {
    result.push_back(system.get());
  }
  return result;
}

template <typename T>
std::multimap<int, int> Diagram<T>::GetDirectFeedthroughs() const {
  std::multimap<int, int> pairs;
  for (InputPortIndex u(0); u < this->get_num_input_ports(); ++u) {
    for (OutputPortIndex v(0); v < this->get_num_output_ports(); ++v) {
      if (DoHasDirectFeedthrough(u, v)) {
        pairs.emplace(u, v);
      }
    }
  }
  return pairs;
}

template <typename T>
std::unique_ptr<CompositeEventCollection<T>>
Diagram<T>::AllocateCompositeEventCollection() const {
  const int num_systems = num_subsystems();
  std::vector<std::unique_ptr<CompositeEventCollection<T>>> subevents(
      num_systems);
  for (SubsystemIndex i(0); i < num_systems; ++i) {
    subevents[i] = registered_systems_[i]->AllocateCompositeEventCollection();
  }

  return std::make_unique<DiagramCompositeEventCollection<T>>(
      std::move(subevents));
}

template <typename T>
void Diagram<T>::SetDefaultState(const Context<T>& context,
                                 State<T>* state) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
  DRAKE_DEMAND(diagram_state != nullptr);

  // Set default state of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);
    auto& substate = diagram_state->get_mutable_substate(i);
    registered_systems_[i]->SetDefaultState(subcontext, &substate);
  }
}

template <typename T>
void Diagram<T>::SetDefaultParameters(const Context<T>& context,
                                      Parameters<T>* params) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  int numeric_parameter_offset = 0;
  int abstract_parameter_offset = 0;

  // Set default parameters of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);

    if (!subcontext.num_numeric_parameter_groups() &&
        !subcontext.num_abstract_parameters()) {
      // Then there is no work to do for this subcontext.
      continue;
    }

    // Make a new Parameters<T> structure with pointers to the mutable
    // subsystem parameter values.  This does not make a copy of the
    // underlying data.
    // TODO(russt): Consider implementing a DiagramParameters, analogous to
    // DiagramState, to avoid these dynamic allocations if they prove
    // expensive.

    std::vector<BasicVector<T>*> numeric_params;
    for (int j = 0; j < subcontext.num_numeric_parameter_groups(); ++j) {
      numeric_params.push_back(&params->get_mutable_numeric_parameter(
          numeric_parameter_offset + j));
    }
    numeric_parameter_offset += subcontext.num_numeric_parameter_groups();

    std::vector<AbstractValue*> abstract_params;
    for (int j = 0; j < subcontext.num_abstract_parameters(); ++j) {
      abstract_params.push_back(&params->get_mutable_abstract_parameter(
          abstract_parameter_offset + j));
    }
    abstract_parameter_offset += subcontext.num_abstract_parameters();

    Parameters<T> subparameters;
    subparameters.set_numeric_parameters(
        std::make_unique<DiscreteValues<T>>(numeric_params));
    subparameters.set_abstract_parameters(
        std::make_unique<AbstractValues>(abstract_params));

    registered_systems_[i]->SetDefaultParameters(subcontext, &subparameters);
  }
}

template <typename T>
void Diagram<T>::SetRandomState(
    const Context<T>& context, State<T>* state,
    RandomGenerator* generator) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
  DRAKE_DEMAND(diagram_state != nullptr);

  // Set state of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);
    auto& substate = diagram_state->get_mutable_substate(i);
    registered_systems_[i]->SetRandomState(subcontext, &substate, generator);
  }
}

template <typename T>
void Diagram<T>::SetRandomParameters(
    const Context<T>& context, Parameters<T>* params,
    RandomGenerator* generator) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  int numeric_parameter_offset = 0;
  int abstract_parameter_offset = 0;

  // Set parameters of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);

    if (!subcontext.num_numeric_parameter_groups() &&
        !subcontext.num_abstract_parameters()) {
      // Then there is no work to do for this subcontext.
      continue;
    }

    // Make a new Parameters<T> structure with pointers to the mutable
    // subsystem parameter values.  This does not make a copy of the
    // underlying data.
    // TODO(russt): This code is duplicated from SetDefaultParameters.
    // Consider extracting it to a helper method (waiting for the rule of
    // three).

    std::vector<BasicVector<T>*> numeric_params;
    std::vector<AbstractValue*> abstract_params;
    for (int j = 0; j < subcontext.num_numeric_parameter_groups(); ++j) {
      numeric_params.push_back(&params->get_mutable_numeric_parameter(
          numeric_parameter_offset + j));
    }
    numeric_parameter_offset += subcontext.num_numeric_parameter_groups();
    for (int j = 0; j < subcontext.num_abstract_parameters(); ++j) {
      abstract_params.push_back(&params->get_mutable_abstract_parameter(
          abstract_parameter_offset + j));
    }
    abstract_parameter_offset += subcontext.num_abstract_parameters();
    Parameters<T> subparameters;
    subparameters.set_numeric_parameters(
        std::make_unique<DiscreteValues<T>>(numeric_params));
    subparameters.set_abstract_parameters(
        std::make_unique<AbstractValues>(abstract_params));

    registered_systems_[i]->SetRandomParameters(subcontext, &subparameters,
                                                generator);
  }
}

template <typename T>
std::unique_ptr<ContinuousState<T>> Diagram<T>::AllocateTimeDerivatives()
    const {
  std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
  for (const auto& system : registered_systems_) {
    sub_derivatives.push_back(system->AllocateTimeDerivatives());
  }
  return std::make_unique<DiagramContinuousState<T>>(
      std::move(sub_derivatives));
}

template <typename T>
std::unique_ptr<DiscreteValues<T>> Diagram<T>::AllocateDiscreteVariables()
    const {
  std::vector<std::unique_ptr<DiscreteValues<T>>> sub_discretes;
  for (const auto& system : registered_systems_) {
    sub_discretes.push_back(system->AllocateDiscreteVariables());
  }
  return std::make_unique<DiagramDiscreteValues<T>>(std::move(sub_discretes));
}

template <typename T>
void Diagram<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  auto diagram_derivatives =
      dynamic_cast<DiagramContinuousState<T>*>(derivatives);
  DRAKE_DEMAND(diagram_derivatives != nullptr);
  const int n = diagram_derivatives->num_substates();
  DRAKE_DEMAND(num_subsystems() == n);

  // Evaluate the derivatives of each constituent system.
  for (SubsystemIndex i(0); i < n; ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    ContinuousState<T>& subderivatives =
        diagram_derivatives->get_mutable_substate(i);
    registered_systems_[i]->CalcTimeDerivatives(subcontext, &subderivatives);
  }
}

template <typename T>
const System<T>& Diagram<T>::GetSubsystemByName(
    const std::string& name) const {
  for (const auto& child : registered_systems_) {
    if (child->get_name() == name) {
      return *child;
    }
  }
  throw std::logic_error("System " + this->GetSystemName() +
                         " does not have a subsystem named " + name);
}

template <typename T>
const ContinuousState<T>& Diagram<T>::GetSubsystemDerivatives(
    const System<T>& subsystem,
    const ContinuousState<T>& derivatives) const {
  auto diagram_derivatives =
      dynamic_cast<const DiagramContinuousState<T>*>(&derivatives);
  DRAKE_DEMAND(diagram_derivatives != nullptr);
  const SubsystemIndex i = GetSystemIndexOrAbort(&subsystem);
  return diagram_derivatives->get_substate(i);
}

template <typename T>
const DiscreteValues<T>& Diagram<T>::GetSubsystemDiscreteValues(
    const System<T>& subsystem,
    const DiscreteValues<T>& discrete_values) const {
  auto diagram_discrete_state =
      dynamic_cast<const DiagramDiscreteValues<T>*>(&discrete_values);
  DRAKE_DEMAND(diagram_discrete_state != nullptr);
  const SubsystemIndex i = GetSystemIndexOrAbort(&subsystem);
  return diagram_discrete_state->get_subdiscrete(i);
}

template <typename T>
void Diagram<T>::GetGraphvizFragment(int max_depth,
                                     std::stringstream* dot) const {
  const int64_t id = this->GetGraphvizId();
  std::string name = this->get_name();
  if (name.empty()) name = std::to_string(id);

  if (max_depth == 0) {
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

    return;
  }

  // Open the Diagram.
  *dot << "subgraph cluster" << id << "diagram" " {" << std::endl;
  *dot << "color=black" << std::endl;
  *dot << "concentrate=true" << std::endl;
  *dot << "label=\"" << name << "\";" << std::endl;

  // Add a cluster for the input port nodes.
  *dot << "subgraph cluster" << id << "inputports" << " {" << std::endl;
  *dot << "rank=same" << std::endl;
  *dot << "color=lightgrey" << std::endl;
  *dot << "style=filled" << std::endl;
  *dot << "label=\"input ports\"" << std::endl;
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    this->GetGraphvizInputPortToken(this->get_input_port(i), max_depth,
                                    dot);
    *dot << "[color=blue, label=\"u" << i << "\"];" << std::endl;
  }
  *dot << "}" << std::endl;

  // Add a cluster for the output port nodes.
  *dot << "subgraph cluster" << id << "outputports" << " {" << std::endl;
  *dot << "rank=same" << std::endl;
  *dot << "color=lightgrey" << std::endl;
  *dot << "style=filled" << std::endl;
  *dot << "label=\"output ports\"" << std::endl;
  for (int i = 0; i < this->get_num_output_ports(); ++i) {
    this->GetGraphvizOutputPortToken(this->get_output_port(i), max_depth,
                                     dot);
    *dot << "[color=green, label=\"y" << i << "\"];" << std::endl;
  }
  *dot << "}" << std::endl;

  // Add a cluster for the subsystems.
  *dot << "subgraph cluster" << id << "subsystems" << " {" << std::endl;
  *dot << "color=white" << std::endl;
  *dot << "label=\"\"" << std::endl;
  // -- Add the subsystems themselves.
  for (const auto& subsystem : registered_systems_) {
    subsystem->GetGraphvizFragment(max_depth - 1, dot);
  }
  // -- Add the connections as edges.
  for (const auto& edge : connection_map_) {
    const OutputPortLocator& src = edge.second;
    const System<T>* src_sys = src.first;
    const InputPortLocator& dest = edge.first;
    const System<T>* dest_sys = dest.first;
    src_sys->GetGraphvizOutputPortToken(src_sys->get_output_port(src.second),
                                        max_depth - 1, dot);
    *dot << " -> ";
    dest_sys->GetGraphvizInputPortToken(dest_sys->get_input_port(dest.second),
                                        max_depth - 1, dot);
    *dot << ";" << std::endl;
  }

  // -- Add edges from the input and output port nodes to the subsystems that
  //    actually service that port.  These edges are highlighted in blue
  //    (input) and green (output), matching the port nodes.
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    const auto& port_id = input_port_ids_[i];
    this->GetGraphvizInputPortToken(this->get_input_port(i), max_depth,
                                    dot);
    *dot << " -> ";
    port_id.first->GetGraphvizInputPortToken(
        port_id.first->get_input_port(port_id.second), max_depth - 1, dot);
    *dot << " [color=blue];" << std::endl;
  }

  for (int i = 0; i < this->get_num_output_ports(); ++i) {
    const auto& port_id = output_port_ids_[i];
    port_id.first->GetGraphvizOutputPortToken(
        port_id.first->get_output_port(port_id.second), max_depth - 1, dot);
    *dot << " -> ";
    this->GetGraphvizOutputPortToken(this->get_output_port(i),
                                     max_depth, dot);
    *dot << " [color=green];" << std::endl;
  }
  *dot << "}" << std::endl;

  // Close the diagram.
  *dot << "}" << std::endl;
}

template <typename T>
void Diagram<T>::GetGraphvizInputPortToken(const InputPort<T>& port,
                                           int max_depth,
                                           std::stringstream* dot) const {
  DRAKE_DEMAND(port.get_system() == this);
  // Note: ports are rendered in a fundamentally different way depending on
  // max_depth.
  if (max_depth > 0) {
    // Ports are rendered as nodes in the "input ports" subgraph.
    *dot << "_" << this->GetGraphvizId() << "_u" << port.get_index();
  } else {
    // Ports are rendered as a part of the system label.
    *dot << this->GetGraphvizId() << ":u" << port.get_index();
  }
}

template <typename T>
void Diagram<T>::GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                            int max_depth,
                                            std::stringstream* dot) const {
  DRAKE_DEMAND(&port.get_system() == this);
  // Note: ports are rendered in a fundamentally different way depending on
  // max_depth.
  if (max_depth > 0) {
    // Ports are rendered as nodes in the "input ports" subgraph.
    *dot << "_" << this->GetGraphvizId() << "_y" << port.get_index();
  } else {
    // Ports are rendered as a part of the system label.
    *dot << this->GetGraphvizId() << ":y" << port.get_index();
  }
}

template <typename T>
Diagram<T>::Diagram() : System<T>(
    SystemScalarConverter(
        SystemTypeTag<systems::Diagram>{},
        SystemScalarConverter::GuaranteedSubtypePreservation::kDisabled)) {}

template <typename T>
Diagram<T>::Diagram(SystemScalarConverter converter)
    : System<T>(std::move(converter)) {}

template <typename T>
void Diagram<T>::AddTriggeredWitnessFunctionToCompositeEventCollection(
    Event<T>* event,
    CompositeEventCollection<T>* events) const {
  DRAKE_DEMAND(events);
  DRAKE_DEMAND(event);
  DRAKE_DEMAND(event->get_event_data());

  // Get the event data- it will need to be modified.
  auto data = dynamic_cast<WitnessTriggeredEventData<T>*>(
      event->get_mutable_event_data());
  DRAKE_DEMAND(data);

  // Get the vector of events corresponding to the subsystem.
  const System<T>& subsystem = data->triggered_witness()->get_system();
  CompositeEventCollection<T>& subevents =
      GetMutableSubsystemCompositeEventCollection(subsystem, events);

  // Get the continuous states at both window endpoints.
  auto diagram_xc0 = dynamic_cast<const DiagramContinuousState<T>*>(
      data->xc0());
  DRAKE_DEMAND(diagram_xc0 != nullptr);
  auto diagram_xcf = dynamic_cast<const DiagramContinuousState<T>*>(
      data->xcf());
  DRAKE_DEMAND(diagram_xcf != nullptr);

  // Modify the pointer to the event data to point to the sub-system
  // continuous states.
  data->set_xc0(DoGetTargetSystemContinuousState(subsystem, diagram_xc0));
  data->set_xcf(DoGetTargetSystemContinuousState(subsystem, diagram_xcf));

  // Add the event to the collection.
  event->AddToComposite(&subevents);
}

template <typename T>
void Diagram<T>::DoGetWitnessFunctions(
    const Context<T>& context,
    std::vector<const WitnessFunction<T>*>* witnesses) const {
  // A temporary vector is necessary since the vector of witnesses is
  // declared to be empty on entry to DoGetWitnessFunctions().
  std::vector<const WitnessFunction<T>*> temp_witnesses;

  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  SubsystemIndex index(0);

  for (const auto& system : registered_systems_) {
    DRAKE_ASSERT(index == GetSystemIndexOrAbort(system.get()));
    temp_witnesses.clear();
    system->GetWitnessFunctions(diagram_context->GetSubsystemContext(index),
                                &temp_witnesses);
    witnesses->insert(witnesses->end(), temp_witnesses.begin(),
                      temp_witnesses.end());
    ++index;
  }
}

template <typename T>
Context<T>* Diagram<T>::DoGetMutableTargetSystemContext(
    const System<T>& target_system, Context<T>* context) const {
  if (&target_system == this)
    return context;

  return GetSubsystemStuff<Context<T>, DiagramContext<T>>(
      target_system, context,
      &System<T>::DoGetMutableTargetSystemContext,
      &DiagramContext<T>::GetMutableSubsystemContext);
}

template <typename T>
const Context<T>* Diagram<T>::DoGetTargetSystemContext(
    const System<T>& target_system, const Context<T>* context) const {
  if (&target_system == this)
    return context;

  return GetSubsystemStuff<const Context<T>, const DiagramContext<T>>(
      target_system, context,
      &System<T>::DoGetTargetSystemContext,
      &DiagramContext<T>::GetSubsystemContext);
}

template <typename T>
State<T>* Diagram<T>::DoGetMutableTargetSystemState(
    const System<T>& target_system, State<T>* state) const {
  if (&target_system == this)
    return state;

  return GetSubsystemStuff<State<T>, DiagramState<T>>(
      target_system, state,
      &System<T>::DoGetMutableTargetSystemState,
      &DiagramState<T>::get_mutable_substate);
}

template <typename T>
const ContinuousState<T>* Diagram<T>::DoGetTargetSystemContinuousState(
    const System<T>& target_system,
    const ContinuousState<T>* xc) const {
  if (&target_system == this)
    return xc;

  return GetSubsystemStuff<const ContinuousState<T>,
                           const DiagramContinuousState<T>>(
      target_system, xc,
      &System<T>::DoGetTargetSystemContinuousState,
      &DiagramContinuousState<T>::get_substate);
}

template <typename T>
const State<T>* Diagram<T>::DoGetTargetSystemState(
    const System<T>& target_system, const State<T>* state) const {
  if (&target_system == this)
    return state;

  return GetSubsystemStuff<const State<T>, const DiagramState<T>>(
      target_system, state,
      &System<T>::DoGetTargetSystemState,
      &DiagramState<T>::get_substate);
}

template <typename T>
CompositeEventCollection<T>*
Diagram<T>::DoGetMutableTargetSystemCompositeEventCollection(
    const System<T>& target_system,
    CompositeEventCollection<T>* events) const {
  if (&target_system == this)
    return events;

  return GetSubsystemStuff<CompositeEventCollection<T>,
                           DiagramCompositeEventCollection<T>>(
      target_system, events,
      &System<T>::DoGetMutableTargetSystemCompositeEventCollection,
      &DiagramCompositeEventCollection<T>::get_mutable_subevent_collection);
}

template <typename T>
const CompositeEventCollection<T>*
Diagram<T>::DoGetTargetSystemCompositeEventCollection(
    const System<T>& target_system,
    const CompositeEventCollection<T>* events) const {
  if (&target_system == this)
    return events;

  return GetSubsystemStuff<const CompositeEventCollection<T>,
                           const DiagramCompositeEventCollection<T>>(
      target_system, events,
      &System<T>::DoGetTargetSystemCompositeEventCollection,
      &DiagramCompositeEventCollection<T>::get_subevent_collection);
}

template <typename T>
void Diagram<T>::DoMapVelocityToQDot(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    VectorBase<T>* qdot) const {
  // Check that the dimensions of the continuous state in the context match
  // the dimensions of the provided generalized velocity and configuration
  // derivatives.
  const ContinuousState<T>& xc = context.get_continuous_state();
  const int nq = xc.get_generalized_position().size();
  const int nv = xc.get_generalized_velocity().size();
  DRAKE_DEMAND(nq == qdot->size());
  DRAKE_DEMAND(nv == generalized_velocity.size());

  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  // Iterate over the subsystems, asking each subsystem to map its subslice of
  // velocity to configuration derivatives. This approach is valid because the
  // DiagramContinuousState guarantees that the subsystem states are
  // concatenated in order.
  int v_index = 0;  // The next index to read in generalized_velocity.
  int q_index = 0;  // The next index to write in qdot.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    // Find the continuous state of subsystem i.
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    const ContinuousState<T>& sub_xc = subcontext.get_continuous_state();

    // Select the chunk of generalized_velocity belonging to subsystem i.
    const int num_v = sub_xc.get_generalized_velocity().size();
    const Eigen::Ref<const VectorX<T>>& v_slice =
        generalized_velocity.segment(v_index, num_v);

    // Select the chunk of qdot belonging to subsystem i.
    const int num_q = sub_xc.get_generalized_position().size();
    Subvector<T> dq_slice(qdot, q_index, num_q);

    // Delegate the actual mapping to subsystem i itself.
    registered_systems_[i]->MapVelocityToQDot(subcontext, v_slice, &dq_slice);

    // Advance the indices.
    v_index += num_v;
    q_index += num_q;
  }
}

template <typename T>
void Diagram<T>::DoMapQDotToVelocity(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    VectorBase<T>* generalized_velocity) const {
  // Check that the dimensions of the continuous state in the context match
  // the dimensions of the provided generalized velocity and configuration
  // derivatives.
  const ContinuousState<T>& xc = context.get_continuous_state();
  const int nq = xc.get_generalized_position().size();
  const int nv = xc.get_generalized_velocity().size();
  DRAKE_DEMAND(nq == qdot.size());
  DRAKE_DEMAND(nv == generalized_velocity->size());

  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  // Iterate over the subsystems, asking each subsystem to map its subslice of
  // configuration derivatives to velocity. This approach is valid because the
  // DiagramContinuousState guarantees that the subsystem states are
  // concatenated in order.
  int q_index = 0;  // The next index to read in qdot.
  int v_index = 0;  // The next index to write in generalized_velocity.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    // Find the continuous state of subsystem i.
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    const ContinuousState<T>& sub_xc = subcontext.get_continuous_state();

    // Select the chunk of qdot belonging to subsystem i.
    const int num_q = sub_xc.get_generalized_position().size();
    const Eigen::Ref<const VectorX<T>>& dq_slice =
        qdot.segment(q_index, num_q);

    // Select the chunk of generalized_velocity belonging to subsystem i.
    const int num_v = sub_xc.get_generalized_velocity().size();
    Subvector<T> v_slice(generalized_velocity, v_index, num_v);

    // Delegate the actual mapping to subsystem i itself.
    registered_systems_[i]->MapQDotToVelocity(subcontext, dq_slice, &v_slice);

    // Advance the indices.
    v_index += num_v;
    q_index += num_q;
  }
}

template <typename T>
void Diagram<T>::DoCalcNextUpdateTime(const Context<T>& context,
                                      CompositeEventCollection<T>* event_info,
                                      T* time) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
  DRAKE_DEMAND(diagram_context != nullptr);
  DRAKE_DEMAND(info != nullptr);

  *time = std::numeric_limits<double>::infinity();

  // Iterate over the subsystems, and harvest the most imminent updates.
  std::vector<T> times(num_subsystems());
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    CompositeEventCollection<T>& subinfo =
        info->get_mutable_subevent_collection(i);
    const T sub_time =
        registered_systems_[i]->CalcNextUpdateTime(subcontext, &subinfo);
    times[i] = sub_time;

    if (sub_time < *time) {
      *time = sub_time;
    }
  }

  // For all the subsystems whose next update time is bigger than *time,
  // clear their event collections.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    if (times[i] > *time)
      info->get_mutable_subevent_collection(i).Clear();
  }
}

template <typename T>
std::unique_ptr<AbstractValue> Diagram<T>::DoAllocateInput(
    const InputPort<T>& input_port) const {
  // Ask the subsystem to perform the allocation.
  const InputPortLocator& id = input_port_ids_[input_port.get_index()];
  const System<T>* subsystem = id.first;
  const InputPortIndex subindex = id.second;
  return subsystem->AllocateInputAbstract(
      subsystem->get_input_port(subindex));
}

template <typename T>
std::unique_ptr<ContextBase> Diagram<T>::DoAllocateContext() const {
  // Reserve inputs as specified during Diagram initialization.
  auto context = std::make_unique<DiagramContext<T>>(num_subsystems());
  this->InitializeContextBase(&*context);

  // Recursively construct each constituent system and its subsystems,
  // then add to this diagram Context.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const System<T>& system = *registered_systems_[i];
    auto subcontext = dynamic_pointer_cast_or_throw<Context<T>>(
        system.AllocateContext());
    context->AddSystem(i, std::move(subcontext));
  }

  // Creates this diagram's composite data structures that collect its
  // subsystems' resources, which must have already been allocated above.
  // No dependencies are set up in these two calls.
  context->MakeParameters();
  context->MakeState();

  // Subscribe each of the Diagram's composite-entity dependency trackers to
  // the trackers for the corresponding constituent entities in the child
  // subsystems. This ensures that changes made at the subcontext level are
  // propagated correctly to the diagram context level. That includes state
  // and parameter changes, as well as local changes that affect composite
  // diagram-level computations like xcdot and pe.
  context->SubscribeDiagramCompositeTrackersToChildrens();

  // Peer-to-peer connections wire a child subsystem's input port to a
  // child subsystem's output port. Subscribe each child input port to the
  // child output port on which it depends.
  for (const auto& connection : connection_map_) {
    const OutputPortLocator& src = connection.second;
    const InputPortLocator& dest = connection.first;
    context->SubscribeInputPortToOutputPort(
        ConvertToContextPortIdentifier(src),
        ConvertToContextPortIdentifier(dest));
  }

  // Diagram-external input ports are exported from child subsystems (meaning
  // the Diagram input is fed into the input of one of its children.
  // Subscribe the child subsystem's input port to its parent Diagram's input
  // port on which it depends.
  for (InputPortIndex i(0); i < this->get_num_input_ports(); ++i) {
    const InputPortLocator& id = input_port_ids_[i];
    context->SubscribeExportedInputPortToDiagramPort(
        i, ConvertToContextPortIdentifier(id));
  }

  // Diagram-external output ports are exported from child subsystem output
  // ports. Subscribe each Diagram-level output to the child-level output on
  // which it depends.
  for (OutputPortIndex i(0); i < this->get_num_output_ports(); ++i) {
    const OutputPortLocator& id = output_port_ids_[i];
    context->SubscribeDiagramPortToExportedOutputPort(
        i, ConvertToContextPortIdentifier(id));
  }

  // TODO(sherm1) Remove this line and the corresponding one in
  // LeafSystem to enable caching by default in Drake (issue #9205).
  context->DisableCaching();

  return context;
}

template <typename T>
const AbstractValue* Diagram<T>::EvalConnectedSubsystemInputPort(
    const ContextBase& context_base,
    const InputPortBase& input_port_base) const {
  auto& diagram_context =
      dynamic_cast<const DiagramContext<T>&>(context_base);
  auto& system =
      dynamic_cast<const System<T>&>(input_port_base.get_system_base());
  const InputPortLocator id{&system, input_port_base.get_index()};

  // Find if this input port is exported (connected to an input port of this
  // containing diagram).
  // TODO(sherm1) Fix this. Shouldn't have to search.
  const auto external_it =
      std::find(input_port_ids_.begin(), input_port_ids_.end(), id);
  const bool is_exported = (external_it != input_port_ids_.end());

  // Find if this input port is connected to an output port.
  // TODO(sherm1) Fix this. Shouldn't have to search.
  const auto upstream_it = connection_map_.find(id);
  const bool is_connected = (upstream_it != connection_map_.end());

  if (!(is_exported || is_connected))
    return nullptr;

  DRAKE_DEMAND(is_exported ^ is_connected);

  if (is_exported) {
    // The upstream source is an input to this whole Diagram; evaluate that
    // input port and use the result as the value for this one.
    const InputPortIndex i(external_it - input_port_ids_.begin());
    return this->EvalAbstractInput(diagram_context, i);
  }

  // The upstream source is an output port of one of this Diagram's child
  // subsystems; evaluate it.
  // TODO(david-german-tri): Add online algebraic loop detection here.
  DRAKE_ASSERT(is_connected);
  const OutputPortLocator& prerequisite = upstream_it->second;
  return &this->EvalSubsystemOutputPort(diagram_context, prerequisite);
}

template <typename T>
bool Diagram<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_ASSERT(input_port >= 0);
  DRAKE_ASSERT(input_port < this->get_num_input_ports());
  DRAKE_ASSERT(output_port >= 0);
  DRAKE_ASSERT(output_port < this->get_num_output_ports());

  const InputPortLocator& target_input_id = input_port_ids_[input_port];

  // Search the graph for a direct-feedthrough connection from the output_port
  // back to the input_port. Maintain a set of the output port identifiers
  // that are known to have a direct-feedthrough path to the output_port.
  std::set<OutputPortLocator> active_set;
  active_set.insert(output_port_ids_[output_port]);
  while (!active_set.empty()) {
    const OutputPortLocator current_output_id = *active_set.begin();
    size_t removed_count = active_set.erase(current_output_id);
    DRAKE_ASSERT(removed_count == 1);
    const System<T>* sys = current_output_id.first;
    for (InputPortIndex i(0); i < sys->get_num_input_ports(); ++i) {
      if (sys->HasDirectFeedthrough(i, current_output_id.second)) {
        const InputPortLocator curr_input_id(sys, i);
        if (curr_input_id == target_input_id) {
          // We've found a direct-feedthrough path to the input_port.
          return true;
        } else {
          // We've found an intermediate input port has a direct-feedthrough
          // path to the output_port. Add the upstream output port (if there
          // is one) to the active set.
          auto it = connection_map_.find(curr_input_id);
          if (it != connection_map_.end()) {
            const OutputPortLocator& upstream_output = it->second;
            active_set.insert(upstream_output);
          }
        }
      }
    }
  }
  // If there are no intermediate output ports with a direct-feedthrough path
  // to the output_port, there is no direct feedthrough to it from the
  // input_port.
  return false;
}

template <typename T>
void Diagram<T>::DispatchPublishHandler(
    const Context<T>& context,
    const EventCollection<PublishEvent<T>>& event_info) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context);
  const DiagramEventCollection<PublishEvent<T>>& info =
      dynamic_cast<const DiagramEventCollection<PublishEvent<T>>&>(
          event_info);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<PublishEvent<T>>& subinfo =
        info.get_subevent_collection(i);

    if (subinfo.HasEvents()) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      registered_systems_[i]->Publish(subcontext, subinfo);
    }
  }
}

template <typename T>
void Diagram<T>::DispatchDiscreteVariableUpdateHandler(
    const Context<T>& context,
    const EventCollection<DiscreteUpdateEvent<T>>& event_info,
    DiscreteValues<T>* discrete_state) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context);
  auto diagram_discrete =
      dynamic_cast<DiagramDiscreteValues<T>*>(discrete_state);
  DRAKE_DEMAND(diagram_discrete);

  // As a baseline, initialize all the discrete variables to their
  // current values.
  diagram_discrete->CopyFrom(context.get_discrete_state());

  const DiagramEventCollection<DiscreteUpdateEvent<T>>& info =
      dynamic_cast<const DiagramEventCollection<DiscreteUpdateEvent<T>>&>(
          event_info);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<DiscreteUpdateEvent<T>>& subinfo =
        info.get_subevent_collection(i);

    if (subinfo.HasEvents()) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      DiscreteValues<T>& subdiscrete =
          diagram_discrete->get_mutable_subdiscrete(i);

      registered_systems_[i]->CalcDiscreteVariableUpdates(subcontext, subinfo,
                                                          &subdiscrete);
    }
  }
}

template <typename T>
void Diagram<T>::DispatchUnrestrictedUpdateHandler(
    const Context<T>& context,
    const EventCollection<UnrestrictedUpdateEvent<T>>& event_info,
    State<T>* state) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context);
  auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
  DRAKE_DEMAND(diagram_state != nullptr);

  // No need to set state to context's state, since it has already been done
  // in System::CalcUnrestrictedUpdate().

  const DiagramEventCollection<UnrestrictedUpdateEvent<T>>& info =
      dynamic_cast<const DiagramEventCollection<UnrestrictedUpdateEvent<T>>&>(
          event_info);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<UnrestrictedUpdateEvent<T>>& subinfo =
        info.get_subevent_collection(i);

    if (subinfo.HasEvents()) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      State<T>& substate = diagram_state->get_mutable_substate(i);

      registered_systems_[i]->CalcUnrestrictedUpdate(subcontext, subinfo,
                                                     &substate);
    }
  }
}

template <typename T>
std::map<PeriodicEventData, std::vector<const Event<T>*>,
         PeriodicEventDataComparator>
Diagram<T>::DoGetPeriodicEvents() const {
  std::map<PeriodicEventData,
           std::vector<const Event<T>*>,
           PeriodicEventDataComparator> periodic_events_map;

  for (int i = 0; i < num_subsystems(); ++i) {
    auto sub_map = registered_systems_[i]->GetPeriodicEvents();
    for (const auto& sub_attr_events : sub_map) {
      const auto& sub_vec = sub_attr_events.second;
      auto& vec = periodic_events_map[sub_attr_events.first];
      vec.insert(vec.end(), sub_vec.begin(), sub_vec.end());
    }
  }

  return periodic_events_map;
}

template <typename T>
void Diagram<T>::DoGetPerStepEvents(
    const Context<T>& context,
    CompositeEventCollection<T>* event_info) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
  DRAKE_DEMAND(diagram_context != nullptr);
  DRAKE_DEMAND(info != nullptr);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    CompositeEventCollection<T>& subinfo =
        info->get_mutable_subevent_collection(i);

    registered_systems_[i]->GetPerStepEvents(subcontext, &subinfo);
  }
}

template <typename T>
void Diagram<T>::DoGetInitializationEvents(
    const Context<T>& context,
    CompositeEventCollection<T>* event_info) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
  DRAKE_DEMAND(diagram_context != nullptr);
  DRAKE_DEMAND(info != nullptr);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    CompositeEventCollection<T>& subinfo =
        info->get_mutable_subevent_collection(i);

    registered_systems_[i]->GetInitializationEvents(subcontext, &subinfo);
  }
}

template <typename T>
void Diagram<T>::Initialize(std::unique_ptr<Blueprint> blueprint) {
  // We must be given something to own.
  DRAKE_DEMAND(!blueprint->systems.empty());
  // We must not already own any subsystems.
  DRAKE_DEMAND(registered_systems_.empty());

  // Move the data from the blueprint into private member variables.
  connection_map_ = std::move(blueprint->connection_map);
  input_port_ids_ = std::move(blueprint->input_port_ids);
  output_port_ids_ = std::move(blueprint->output_port_ids);
  registered_systems_ = std::move(blueprint->systems);

  // Generate a map from the System pointer to its index in the registered
  // order.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    system_index_map_[registered_systems_[i].get()] = i;
    SystemBase::set_parent_service(registered_systems_[i].get(), this);
  }

  // Generate constraints for the diagram from the constraints on the
  // subsystems.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const auto sys = registered_systems_[i].get();
    for (SystemConstraintIndex j(0); j < sys->get_num_constraints(); ++j) {
      const auto c = &(sys->get_constraint(j));
      typename SystemConstraint<T>::CalcCallback diagram_calc =
          [this, sys, c](const Context<T>& context, VectorX<T>* value) {
        c->Calc(this->GetSubsystemContext(*sys, context), value);
      };
      this->AddConstraint(std::make_unique<SystemConstraint<T>>(
          diagram_calc, c->size(), c->type(),
          sys->get_name() + ":" + c->description()));
    }
  }

  // Every system must appear exactly once.
  DRAKE_DEMAND(registered_systems_.size() == system_index_map_.size());
  // Every port named in the connection_map_ must actually exist.
  DRAKE_ASSERT(PortsAreValid());
  // Every subsystem must have a unique name.
  DRAKE_THROW_UNLESS(NamesAreUniqueAndNonEmpty());

  // Add the inputs to the Diagram topology, and check their invariants.
  DRAKE_DEMAND(input_port_ids_.size() ==
               blueprint->input_port_names.size());
  std::vector<std::string>::iterator name_iter =
      blueprint->input_port_names.begin();
  for (const InputPortLocator& id : input_port_ids_) {
    ExportInput(id, *name_iter++);
  }
  DRAKE_DEMAND(output_port_ids_.size() ==
               blueprint->output_port_names.size());
  name_iter = blueprint->output_port_names.begin();
  for (const OutputPortLocator& id : output_port_ids_) {
    ExportOutput(id, *name_iter++);
  }

  // Identify the intersection of the subsystems' scalar conversion support.
  // Remove all conversions that at least one subsystem did not support.
  SystemScalarConverter& this_scalar_converter =
      SystemImpl::get_mutable_system_scalar_converter(this);
  for (const auto& system : registered_systems_) {
    this_scalar_converter.RemoveUnlessAlsoSupportedBy(
        system->get_system_scalar_converter());
  }

  this->set_forced_publish_events(
      AllocateForcedEventCollection<PublishEvent<T>>(
          &System<T>::AllocateForcedPublishEventCollection));
  this->set_forced_discrete_update_events(
      AllocateForcedEventCollection<DiscreteUpdateEvent<T>>(
          &System<T>::AllocateForcedDiscreteUpdateEventCollection));
  this->set_forced_unrestricted_update_events(
      AllocateForcedEventCollection<UnrestrictedUpdateEvent<T>>(
          &System<T>::AllocateForcedUnrestrictedUpdateEventCollection));
}

template <typename T>
void Diagram<T>::ExportInput(
    const InputPortLocator& port, std::string name) {
  const System<T>* const sys = port.first;
  const int port_index = port.second;
  // Fail quickly if this system is not part of the diagram.
  GetSystemIndexOrAbort(sys);

  // Add this port to our externally visible topology.
  const auto& subsystem_input_port = sys->get_input_port(port_index);
  this->DeclareInputPort(
      std::move(name), subsystem_input_port.get_data_type(),
      subsystem_input_port.size(), subsystem_input_port.get_random_type());
}

template <typename T>
void Diagram<T>::ExportOutput(
    const OutputPortLocator& port, std::string name) {
  const System<T>* const sys = port.first;
  const int port_index = port.second;
  const auto& source_output_port = sys->get_output_port(port_index);
  // TODO(sherm1) Use implicit_cast when available (from abseil).
  auto diagram_port = std::make_unique<DiagramOutputPort<T>>(
      this,  // implicit_cast<const System<T>*>(this)
      this,  // implicit_cast<SystemBase*>(this)
      std::move(name), OutputPortIndex(this->get_num_output_ports()),
      this->assign_next_dependency_ticket(), &source_output_port,
      GetSystemIndexOrAbort(&source_output_port.get_system()));
  this->AddOutputPort(std::move(diagram_port));
}

template <typename T>
const AbstractValue& Diagram<T>::EvalSubsystemOutputPort(
    const DiagramContext<T>& context, const OutputPortLocator& id) const {
  const System<T>* const system = id.first;
  const OutputPortIndex port_index(id.second);
  const OutputPort<T>& port = system->get_output_port(port_index);
  const SubsystemIndex i = GetSystemIndexOrAbort(system);
  SPDLOG_TRACE(log(), "Evaluating output for subsystem {}, port {}",
               system->GetSystemPathname(), port_index);
  const Context<T>& subsystem_context = context.GetSubsystemContext(i);
  return port.EvalAbstract(subsystem_context);
}

template <typename T>
bool Diagram<T>::PortsAreValid() const {
  for (const auto& entry : connection_map_) {
    const InputPortLocator& dest = entry.first;
    const OutputPortLocator& src = entry.second;
    if (dest.second < 0 || dest.second >= dest.first->get_num_input_ports()) {
      return false;
    }
    if (src.second < 0 || src.second >= src.first->get_num_output_ports()) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool Diagram<T>::NamesAreUniqueAndNonEmpty() const {
  std::set<std::string> names;
  for (const auto& system : registered_systems_) {
    const std::string& name = system->get_name();
    if (name.empty()) {
      // This can only happen if someone blanks out the name *after* adding
      // it to DiagramBuilder; if an empty name is given to DiagramBuilder,
      // a default non-empty name is automatically assigned.
      log()->error("Subsystem of type {} has no name",
                   NiceTypeName::Get(*system));
      // We skip names.insert here, so that the return value will be false.
      continue;
    }
    if (names.find(name) != names.end()) {
      log()->error("Non-unique name \"{}\" for subsystem of type {}", name,
                   NiceTypeName::Get(*system));
    }
    names.insert(name);
  }
  return names.size() == registered_systems_.size();
}

}  // namespace systems
}  // namespace drake
