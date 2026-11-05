#ifndef CBF_BARRIER_EDGE_REGISTRY_HPP
#define CBF_BARRIER_EDGE_REGISTRY_HPP

#include <algorithm>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace cbf2026 {

enum class EdgeKind {
    Localization,
    Collision
};

struct EdgeId {
    EdgeKind kind;
    int low;
    int high;
    int baseId;
};

inline bool operator==(const EdgeId& lhs, const EdgeId& rhs) {
    return lhs.kind == rhs.kind
           && lhs.low == rhs.low
           && lhs.high == rhs.high
           && lhs.baseId == rhs.baseId;
}

inline bool operator!=(const EdgeId& lhs, const EdgeId& rhs) {
    return !(lhs == rhs);
}

inline bool operator<(const EdgeId& lhs, const EdgeId& rhs) {
    return std::tie(lhs.kind, lhs.low, lhs.high, lhs.baseId)
           < std::tie(rhs.kind, rhs.low, rhs.high, rhs.baseId);
}

struct EdgeIdHash {
    std::size_t operator()(const EdgeId& edge) const {
        std::size_t seed = std::hash<int>{}(
            static_cast<int>(edge.kind)
        );
        const auto combine = [&seed](int value) {
            seed ^= std::hash<int>{}(value)
                    + 0x9e3779b9U
                    + (seed << 6U)
                    + (seed >> 2U);
        };
        combine(edge.low);
        combine(edge.high);
        combine(edge.baseId);
        return seed;
    }
};

inline EdgeId canonicalUavEdge(EdgeKind kind, int first, int second) {
    if ((kind != EdgeKind::Localization
         && kind != EdgeKind::Collision)
        || first <= 0 || second <= 0 || first == second) {
        throw std::invalid_argument(
            "UAV edge kind and endpoints are invalid"
        );
    }
    return {
        kind,
        std::min(first, second),
        std::max(first, second),
        -1
    };
}

inline EdgeId canonicalBaseLocalizationEdge(int uavId, int baseId) {
    if (uavId <= 0 || baseId < 0) {
        throw std::invalid_argument(
            "base localization edge IDs are invalid"
        );
    }
    return {EdgeKind::Localization, uavId, uavId, baseId};
}

inline void validateCanonicalEdge(const EdgeId& edge) {
    if (edge.kind != EdgeKind::Localization
        && edge.kind != EdgeKind::Collision) {
        throw std::invalid_argument("edge kind is unsupported");
    }
    if (edge.low <= 0 || edge.high <= 0) {
        throw std::invalid_argument("edge UAV IDs must be positive");
    }
    if (edge.baseId >= 0) {
        if (edge.kind != EdgeKind::Localization
            || edge.low != edge.high) {
            throw std::invalid_argument("base edge is not canonical");
        }
        return;
    }
    if (edge.baseId != -1 || edge.low >= edge.high) {
        throw std::invalid_argument("UAV edge is not canonical");
    }
}

struct FixedLocalizationReference {
    int owner;
    int reference;
    bool hoveringBase;
};

class BarrierEdgeRegistry {
public:
    BarrierEdgeRegistry(
        int uavCount,
        const std::vector<FixedLocalizationReference>& references
    ) : uavCount_(uavCount) {
        if (uavCount <= 0) {
            throw std::invalid_argument("UAV count must be positive");
        }
        for (const auto& reference : references) {
            if (reference.owner <= 0 || reference.owner > uavCount) {
                throw std::invalid_argument(
                    "fixed localization owner is outside the mission"
                );
            }
            EdgeId edge;
            if (reference.hoveringBase) {
                edge = canonicalBaseLocalizationEdge(
                    reference.owner, reference.reference
                );
            } else {
                if (reference.reference <= 0
                    || reference.reference > uavCount
                    || reference.reference >= reference.owner) {
                    throw std::invalid_argument(
                        "fixed UAV reference is not a lower-index predecessor"
                    );
                }
                edge = canonicalUavEdge(
                    EdgeKind::Localization,
                    reference.owner,
                    reference.reference
                );
            }
            if (std::find(
                    fixedLocalizationEdges_.begin(),
                    fixedLocalizationEdges_.end(),
                    edge
                ) != fixedLocalizationEdges_.end()) {
                throw std::invalid_argument(
                    "fixed localization edge is duplicated"
                );
            }
            fixedLocalizationEdges_.push_back(edge);
        }
        std::sort(
            fixedLocalizationEdges_.begin(),
            fixedLocalizationEdges_.end()
        );
        for (int low = 1; low <= uavCount; ++low) {
            for (int high = low + 1; high <= uavCount; ++high) {
                collisionEdges_.push_back(canonicalUavEdge(
                    EdgeKind::Collision, low, high
                ));
            }
        }
    }

    const std::vector<EdgeId>& fixedLocalizationEdges() const {
        return fixedLocalizationEdges_;
    }

    const std::vector<EdgeId>& collisionEdges() const {
        return collisionEdges_;
    }

    std::vector<EdgeId> incidentEdges(int uavId) const {
        if (uavId <= 0 || uavId > uavCount_) {
            throw std::invalid_argument(
                "incident UAV ID is outside the mission"
            );
        }
        std::vector<EdgeId> incident;
        auto appendIncident = [uavId, &incident](
            const std::vector<EdgeId>& edges
        ) {
            for (const auto& edge : edges) {
                if (edge.low == uavId || edge.high == uavId) {
                    incident.push_back(edge);
                }
            }
        };
        appendIncident(fixedLocalizationEdges_);
        appendIncident(collisionEdges_);
        std::sort(incident.begin(), incident.end());
        return incident;
    }

private:
    int uavCount_;
    std::vector<EdgeId> fixedLocalizationEdges_;
    std::vector<EdgeId> collisionEdges_;
};

}

#endif
