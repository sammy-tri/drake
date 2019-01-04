#pragma once

/// @file
/// Template method implementations for diagram_builder.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

/* clang-format off to disable clang-format-includes */
#include "drake/systems/framework/diagram_builder.h"
/* clang-format on */

namespace drake {
namespace systems {

template <typename T>
void DiagramBuilder<T>::Connect(const OutputPort<T>& src,
                                const InputPort<T>& dest) {
  InputPortLocator dest_id{dest.get_system(), dest.get_index()};
  OutputPortLocator src_id{&src.get_system(), src.get_index()};
  ThrowIfSystemNotRegistered(&src.get_system());
  ThrowIfSystemNotRegistered(dest.get_system());
  ThrowIfInputAlreadyWired(dest_id);
  if (src.get_data_type() != dest.get_data_type()) {
    throw std::logic_error(fmt::format(
        "DiagramBuilder::Connect: Cannot mix vector-valued and abstract-"
        "valued ports while connecting output port {} of System {} to "
        "input port {} of System {}",
        src.get_name(), src.get_system().get_name(),
        dest.get_name(), dest.get_system()->get_name()));
  }
  if ((src.get_data_type() != kAbstractValued) &&
      (src.size() != dest.size())) {
    throw std::logic_error(fmt::format(
        "DiagramBuilder::Connect: Mismatched vector sizes while connecting "
        "output port {} of System {} (size {}) to "
        "input port {} of System {} (size {})",
        src.get_name(), src.get_system().get_name(), src.size(),
        dest.get_name(), dest.get_system()->get_name(), dest.size()));
  }
  if (src.get_data_type() == kAbstractValued) {
    auto model_output = src.Allocate();
    auto model_input = dest.get_system()->AllocateInputAbstract(dest);
    const std::type_info& output_type = model_output->static_type_info();
    const std::type_info& input_type = model_input->static_type_info();
    if (output_type != input_type) {
      throw std::logic_error(fmt::format(
          "DiagramBuilder::Connect: Mismatched value types while connecting "
          "output port {} of System {} (type {}) to "
          "input port {} of System {} (type {})",
          src.get_name(), src.get_system().get_name(),
          NiceTypeName::Get(output_type),
          dest.get_name(), dest.get_system()->get_name(),
          NiceTypeName::Get(input_type)));
    }
  }
  connection_map_[dest_id] = src_id;
}

template <typename T>
InputPortIndex DiagramBuilder<T>::ExportInput(
    const InputPort<T>& input,
    variant<std::string, UseDefaultName> name) {
  InputPortLocator id{input.get_system(), input.get_index()};
  ThrowIfInputAlreadyWired(id);
  ThrowIfSystemNotRegistered(input.get_system());
  InputPortIndex return_id(input_port_ids_.size());
  input_port_ids_.push_back(id);

  // The requirement that subsystem names are unique guarantees uniqueness
  // of the port names.
  std::string port_name =
      name == kUseDefaultName
      ? input.get_system()->get_name() + "_" + input.get_name()
      : get<std::string>(std::move(name));
  DRAKE_DEMAND(!port_name.empty());
  input_port_names_.emplace_back(std::move(port_name));

  diagram_input_set_.insert(id);
  return return_id;
}

template <typename T>
OutputPortIndex DiagramBuilder<T>::ExportOutput(
    const OutputPort<T>& output,
    variant<std::string, UseDefaultName> name) {
  ThrowIfSystemNotRegistered(&output.get_system());
  OutputPortIndex return_id(output_port_ids_.size());
  output_port_ids_.push_back(
      OutputPortLocator{&output.get_system(), output.get_index()});

  // The requirement that subsystem names are unique guarantees uniqueness
  // of the port names.
  std::string port_name =
      name == kUseDefaultName
      ? output.get_system().get_name() + "_" + output.get_name()
      : get<std::string>(std::move(name));
  DRAKE_DEMAND(!port_name.empty());
  output_port_names_.emplace_back(std::move(port_name));

  return return_id;
}

template <typename T>
void DiagramBuilder<T>::ThrowIfAlgebraicLoopsExist() const {
  // Each port in the diagram is a node in a graph.
  // An edge exists from node u to node v if:
  //  1. output u is connected to input v (via Connect(u, v) method), or
  //  2. a direct feedthrough from input u to output v is reported.
  // A depth-first search of the graph should produce a forest of valid trees
  // if there are no algebraic loops. Otherwise, at least one link moving
  // *up* the tree will exist.

  // Build the graph.
  // Generally, the nodes of the graph would be the set of all defined ports
  // (input and output) of each subsystem. However, we only need to
  // consider the input/output ports that have a diagram level output-to-input
  // connection (ports that are not connected in this manner cannot contribute
  // to an algebraic loop).

  // Track *all* of the nodes involved in a diagram-level connection as
  // described above.
  std::set<PortIdentifier> nodes;
  // A map from node u, to the set of edges { (u, v_i) }. In normal cases,
  // not every node in `nodes` will serve as a key in `edges` (as that is a
  // necessary condition for there to be no algebraic loop).
  std::map<PortIdentifier, std::set<PortIdentifier>> edges;

  // In order to store PortIdentifiers for both input and output ports in the
  // same set, I need to encode the ports. The identifier for the first input
  // port and output port look identical (same system pointer, same port
  // id 0). So, to distinguish them, I'll modify the output ports to use the
  // negative half of the int space. The function below provides a utility for
  // encoding an output port id.
  auto output_to_key = [](int port_id) { return -(port_id + 1); };

  // Populate the node set from the connections (and define the edges implied
  // by those connections).
  for (const auto& connection : connection_map_) {
    // Dependency graph is a mapping from the destination of the connection
    // to what it *depends on* (the source).
    const PortIdentifier& src = connection.second;
    const PortIdentifier& dest = connection.first;
    PortIdentifier encoded_src{src.first, output_to_key(src.second)};
    nodes.insert(encoded_src);
    nodes.insert(dest);
    edges[encoded_src].insert(dest);
  }

  // Populate more edges based on direct feedthrough.
  for (const auto& system : registered_systems_) {
    for (const auto& pair : system->GetDirectFeedthroughs()) {
      PortIdentifier src_port{system.get(), pair.first};
      PortIdentifier dest_port{system.get(), output_to_key(pair.second)};
      if (nodes.count(src_port) > 0 && nodes.count(dest_port) > 0) {
        // Track direct feedthrough only on port pairs where *both* ports are
        // connected to other ports at the diagram level.
        edges[src_port].insert(dest_port);
      }
    }
  }

  // Evaluate the graph for cycles.
  std::set<PortIdentifier> visited;
  std::vector<PortIdentifier> stack;
  for (const auto& node : nodes) {
    if (visited.count(node) == 0) {
      if (HasCycleRecurse(node, edges, &visited, &stack)) {
        std::stringstream ss;

        auto port_to_stream = [&ss](const auto& id) {
          ss << "  " << id.first->get_name() << ":";
          if (id.second < 0)
            ss << "Out(";
          else
            ss << "In(";
          ss << (id.second >= 0 ? id.second : -id.second - 1) << ")";
        };

        ss << "Algebraic loop detected in DiagramBuilder:\n";
        for (size_t i = 0; i < stack.size() - 1; ++i) {
          port_to_stream(stack[i]);
          ss << " depends on\n";
        }
        port_to_stream(stack.back());

        throw std::runtime_error(ss.str());
      }
    }
  }
}

}  // namespace systems
}  // namespace drake
