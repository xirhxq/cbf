#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace gf {

using NodeId = std::uint16_t;
using TopologyVersion = std::uint64_t;

struct DirectedEdge {
    NodeId reference;
    NodeId owner;

    DirectedEdge(NodeId reference_id, NodeId owner_id)
        : reference(reference_id), owner(owner_id) {
        if (reference == owner) {
            throw std::invalid_argument(
                "directed reference edge endpoints must be distinct");
        }
    }

    std::string id() const {
        return std::to_string(reference) + "->" + std::to_string(owner);
    }

    bool operator==(const DirectedEdge& other) const {
        return reference == other.reference && owner == other.owner;
    }
};

struct UndirectedEdge {
    NodeId first;
    NodeId second;

    static UndirectedEdge canonical(NodeId a, NodeId b) {
        if (a == b) {
            throw std::invalid_argument(
                "undirected measurement edge endpoints must be distinct");
        }
        return a < b ? UndirectedEdge{a, b} : UndirectedEdge{b, a};
    }

    std::string id() const {
        return std::to_string(first) + "--" + std::to_string(second);
    }

private:
    UndirectedEdge(NodeId first_id, NodeId second_id)
        : first(first_id), second(second_id) {}
};

enum class SolverProfile {
    Gurobi,
    OpenSource
};

}  // namespace gf
