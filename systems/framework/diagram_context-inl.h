#pragma once

/// @file
/// Template method implementations for diagram.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

#include <memory>
#include <string>
#include <utility>
#include <vector>

/* clang-format off to disable clang-format-includes */
#include "drake/systems/framework/diagram_context.h"
/* clang-format on */

namespace drake {
namespace systems {

template <typename T>
void DiagramState<T>::Finalize() {
  DRAKE_DEMAND(!finalized_);
  finalized_ = true;
  std::vector<ContinuousState<T>*> sub_xcs;
  sub_xcs.reserve(num_substates());
  std::vector<BasicVector<T>*> sub_xds;
  std::vector<AbstractValue*> sub_xas;
  for (State<T>* substate : substates_) {
    // Continuous
    sub_xcs.push_back(&substate->get_mutable_continuous_state());
    // Discrete
    const std::vector<BasicVector<T>*>& xd_data =
        substate->get_mutable_discrete_state().get_data();
    sub_xds.insert(sub_xds.end(), xd_data.begin(), xd_data.end());
    // Abstract
    AbstractValues& xa = substate->get_mutable_abstract_state();
    for (int i_xa = 0; i_xa < xa.size(); ++i_xa) {
      sub_xas.push_back(&xa.get_mutable_value(i_xa));
    }
  }

  // This State consists of a continuous, discrete, and abstract state, each
  // of which is a spanning vector over the continuous, discrete, and abstract
  // parts of the constituent states.  The spanning vectors do not own any
  // of the actual memory that contains state variables. They just hold
  // pointers to that memory.
  this->set_continuous_state(
      std::make_unique<DiagramContinuousState<T>>(sub_xcs));
  this->set_discrete_state(std::make_unique<DiscreteValues<T>>(sub_xds));
  this->set_abstract_state(std::make_unique<AbstractValues>(sub_xas));
}

template <typename T>
void DiagramContext<T>::SubscribeExportedInputPortToDiagramPort(
    InputPortIndex input_port_index,
    const InputPortIdentifier& subsystem_input_port) {
  // Identify and validate the destination input port.
  const SubsystemIndex subsystem_index = subsystem_input_port.first;
  const InputPortIndex subsystem_iport_index = subsystem_input_port.second;
  Context<T>& subcontext = GetMutableSubsystemContext(subsystem_index);
  DRAKE_DEMAND(0 <= subsystem_iport_index &&
               subsystem_iport_index < subcontext.get_num_input_ports());

  // Get this Diagram's input port that serves as the source.
  const DependencyTicket iport_ticket =
      this->input_port_ticket(input_port_index);
  DependencyTracker& iport_tracker = this->get_mutable_tracker(iport_ticket);
  const DependencyTicket subcontext_iport_ticket =
      subcontext.input_port_ticket(subsystem_iport_index);
  DependencyTracker& subcontext_iport_tracker =
      subcontext.get_mutable_tracker(subcontext_iport_ticket);
  subcontext_iport_tracker.SubscribeToPrerequisite(&iport_tracker);
}

template <typename T>
void DiagramContext<T>::SubscribeDiagramPortToExportedOutputPort(
    OutputPortIndex output_port_index,
    const OutputPortIdentifier& subsystem_output_port) {
  // Identify and validate the source output port.
  const SubsystemIndex subsystem_index = subsystem_output_port.first;
  const OutputPortIndex subsystem_oport_index = subsystem_output_port.second;
  Context<T>& subcontext = GetMutableSubsystemContext(subsystem_index);
  DRAKE_DEMAND(0 <= subsystem_oport_index &&
               subsystem_oport_index < subcontext.get_num_output_ports());

  // Get the child subsystem's output port tracker that serves as the source.
  const DependencyTicket subcontext_oport_ticket =
      subcontext.output_port_ticket(subsystem_oport_index);
  DependencyTracker& subcontext_oport_tracker =
      subcontext.get_mutable_tracker(subcontext_oport_ticket);

  // Get the diagram's output port tracker that is the destination.
  const DependencyTicket oport_ticket =
      this->output_port_ticket(output_port_index);
  DependencyTracker& oport_tracker = this->get_mutable_tracker(oport_ticket);

  oport_tracker.SubscribeToPrerequisite(&subcontext_oport_tracker);
}

template <typename T>
void DiagramContext<T>::SubscribeInputPortToOutputPort(
    const OutputPortIdentifier& output_port,
    const InputPortIdentifier& input_port) {
  // Identify and validate the source output port.
  const SubsystemIndex oport_system_index = output_port.first;
  const OutputPortIndex oport_index = output_port.second;
  Context<T>& oport_context = GetMutableSubsystemContext(oport_system_index);
  DRAKE_DEMAND(oport_index >= 0);
  DRAKE_DEMAND(oport_index < oport_context.get_num_output_ports());

  // Identify and validate the destination input port.
  const SubsystemIndex iport_system_index = input_port.first;
  const InputPortIndex iport_index = input_port.second;
  Context<T>& iport_context = GetMutableSubsystemContext(iport_system_index);
  DRAKE_DEMAND(iport_index >= 0);
  DRAKE_DEMAND(iport_index < iport_context.get_num_input_ports());

  // Dig out the dependency trackers for both ports so we can subscribe the
  // input port tracker to the output port tracker.
  const DependencyTicket oport_ticket =
      oport_context.output_port_ticket(oport_index);
  const DependencyTicket iport_ticket =
      iport_context.input_port_ticket(iport_index);
  DependencyTracker& oport_tracker =
      oport_context.get_mutable_tracker(oport_ticket);
  DependencyTracker& iport_tracker =
      iport_context.get_mutable_tracker(iport_ticket);
  iport_tracker.SubscribeToPrerequisite(&oport_tracker);
}

template <typename T>
void DiagramContext<T>::SubscribeDiagramCompositeTrackersToChildrens() {
  std::vector<internal::BuiltInTicketNumbers> composites{
    internal::kQTicket,  // Value sources.
        internal::kVTicket,
        internal::kZTicket,
        internal::kXdTicket,
        internal::kXaTicket,
        internal::kPnTicket,
        internal::kPaTicket,
        internal::kXcdotTicket,  // Cache entries.
        internal::kPeTicket,
        internal::kKeTicket,
        internal::kPcTicket,
        internal::kPncTicket};

  // Validate the claim above that Diagrams do not have tickets for individual
  // variables and parameters.
  DRAKE_DEMAND(!this->owns_any_variables_or_parameters());

  // Collect the diagram trackers for each composite item above.
  DependencyGraph& graph = this->get_mutable_dependency_graph();
  std::vector<DependencyTracker*> diagram_trackers;
  for (auto ticket : composites) {
    diagram_trackers.push_back(
        &graph.get_mutable_tracker(DependencyTicket(ticket)));
  }

  // Subscribe those trackers to the corresponding subcontext trackers.
  for (auto& subcontext : contexts_) {
    DependencyGraph& subgraph = subcontext->get_mutable_dependency_graph();
    for (size_t i = 0; i < composites.size(); ++i) {
      auto& sub_tracker =
          subgraph.get_mutable_tracker(DependencyTicket(composites[i]));
      diagram_trackers[i]->SubscribeToPrerequisite(&sub_tracker);
    }
  }
}

template <typename T>
void DiagramContext<T>::MakeState() {
  auto state = std::make_unique<DiagramState<T>>(num_subcontexts());
  for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
    Context<T>& subcontext = *contexts_[i].get();
    // Using `access` here to avoid sending invalidations.
    state->set_substate(i, &Context<T>::access_mutable_state(&subcontext));
  }
  state->Finalize();
  state_ = std::move(state);
}

template <typename T>
void DiagramContext<T>::MakeParameters() {
  std::vector<BasicVector<T>*> numeric_params;
  std::vector<AbstractValue*> abstract_params;
  for (auto& subcontext : contexts_) {
    // Using `access` here to avoid sending invalidations.
    Parameters<T>& subparams =
        Context<T>::access_mutable_parameters(&*subcontext);
    for (int i = 0; i < subparams.num_numeric_parameter_groups(); ++i) {
      numeric_params.push_back(&subparams.get_mutable_numeric_parameter(i));
    }
    for (int i = 0; i < subparams.num_abstract_parameters(); ++i) {
      abstract_params.push_back(&subparams.get_mutable_abstract_parameter(i));
    }
  }
  auto params = std::make_unique<Parameters<T>>();
  params->set_numeric_parameters(
      std::make_unique<DiscreteValues<T>>(numeric_params));
  params->set_abstract_parameters(
      std::make_unique<AbstractValues>(abstract_params));
  this->init_parameters(std::move(params));
}

template <typename T>
std::string DiagramContext<T>::do_to_string() const {
  std::ostringstream os;

  os << this->GetSystemPathname() << " Context (of a Diagram)\n";
  os << std::string(this->GetSystemPathname().size() + 24, '-') << "\n";
  if (this->get_continuous_state().size())
    os << this->get_continuous_state().size() << " total continuous states\n";
  if (this->get_num_discrete_state_groups()) {
    int num_discrete_states = 0;
    for (int i = 0; i < this->get_num_discrete_state_groups(); i++) {
      num_discrete_states += this->get_discrete_state(i).size();
    }
    os << num_discrete_states << " total discrete states in "
       << this->get_num_discrete_state_groups() << " groups\n";
  }
  if (this->get_num_abstract_states())
    os << this->get_num_abstract_states() << " total abstract states\n";

  if (this->num_numeric_parameter_groups()) {
    int num_numeric_parameters = 0;
    for (int i = 0; i < this->num_numeric_parameter_groups(); i++) {
      num_numeric_parameters += this->get_numeric_parameter(i).size();
    }
    os << num_numeric_parameters << " total numeric parameters in "
       << this->num_numeric_parameter_groups() << " groups\n";
  }
  if (this->num_abstract_parameters())
    os << this->num_abstract_parameters() << " total abstract parameters\n";

  for (systems::SubsystemIndex i{0}; i < num_subcontexts(); i++) {
    const Context<T>& subcontext = this->GetSubsystemContext(i);
    // Only print this context if it has something useful to print.
    if (subcontext.get_continuous_state_vector().size() ||
        subcontext.get_num_discrete_state_groups() ||
        subcontext.get_num_abstract_states() ||
        subcontext.num_numeric_parameter_groups() ||
        subcontext.num_abstract_parameters()) {
      os << "\n" << subcontext.to_string();
    }
  }

  return os.str();
}

}  // namespace systems
}  // namespace drake
