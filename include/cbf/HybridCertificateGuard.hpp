#ifndef CBF_HYBRID_CERTIFICATE_GUARD_HPP
#define CBF_HYBRID_CERTIFICATE_GUARD_HPP

#include "cbf/AllocatedPairwiseCBF.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace cbf2026 {

enum class ResetCause {
    ActiveReferenceChange,
    EstimateInterfaceReset,
    CertificateDiscontinuity,
    HardEdgeSnapshotChange
};

enum class GuardStatus { Accepted, Rejected };

using ActiveDag = std::map<int, std::vector<int>>;

struct HardConstraintRow {
    EdgeId edge = {EdgeKind::Localization, 0, 0, -1};
    int owner = 0;
    std::string name;
    Eigen::VectorXd coefficients;
    double constant = 0.0;
    double postResetBarrier = 0.0;
    std::uint64_t snapshotVersion = 0;
    std::uint64_t allocationVersion = 0;
};

struct HardInputBound {
    int controlIndex = 0;
    double coefficient = 0.0;
    double limit = 0.0;
};

struct HardConstraintProblem {
    int owner = 0;
    int controlSize = 0;
    double planarComponentMax = 0.0;
    std::uint64_t snapshotVersion = 0;
    std::uint64_t allocationVersion = 0;
    std::vector<HardConstraintRow> rows;
    double yawRateMax = 0.35;
    std::vector<HardInputBound> bounds;
};

struct FeasibilityResult {
    bool feasible = false;
    std::string status;
    double minimumResidual = -std::numeric_limits<double>::infinity();
    std::string hardProblemHash;
};

struct LocalHardQpResetRecord {
    int nodeId = 0;
    HardConstraintProblem problem;
    FeasibilityResult feasibility;
};

struct NodeResetRecord {
    int nodeId = 0;
    int topologicalIndex = 0;
    std::uint64_t snapshotVersion = 0;
    std::optional<EndpointCertificateSnapshot> proposedSnapshot;
    Eigen::Vector2d deltaEstimate = Eigen::Vector2d::Zero();
    double deltaEpsilon = 0.0;
    std::vector<int> preActiveReferences;
    std::vector<int> postActiveReferences;
};

struct HardEdgeResetRecord {
    EdgeId edge = {EdgeKind::Localization, 0, 0, -1};
    double threshold = 0.0;
    Eigen::Vector2d basePosition = Eigen::Vector2d::Zero();
    double preBarrier = 0.0;
    std::vector<EndpointRow> endpointRows;
    double classKCoefficient = 1.0;
    int classKPower = 1;
    double postBarrier = std::numeric_limits<double>::quiet_NaN();
};

struct ResetProposal {
    double simulationTime = 0.0;
    ResetCause cause = ResetCause::CertificateDiscontinuity;
    std::uint64_t predecessorVersion = 0;
    std::uint64_t proposedVersion = 0;
    std::vector<int> changedNodes;
    ActiveDag activeDag;
    std::vector<int> descendantClosure;
    std::vector<NodeResetRecord> nodes;
    std::map<int, EndpointCertificateSnapshot> candidateEndpoints;
    std::vector<EdgeId> requiredHardEdges;
    std::vector<HardEdgeResetRecord> hardEdges;
    std::vector<int> requiredUavNodes;
    std::vector<LocalHardQpResetRecord> localHardQps;
    std::uint64_t frameIndex = std::numeric_limits<std::uint64_t>::max();
    std::vector<ResetCause> causes;
    std::map<int, NodeRateCertificate> candidateCertificates;
};

struct ResetTransaction {
    std::uint64_t predecessorVersion = 0;
    std::uint64_t proposedVersion = 0;
    std::vector<int> changedNodes;
    std::vector<int> descendantClosure;
    std::vector<NodeResetRecord> nodes;
    GuardStatus status = GuardStatus::Rejected;
    std::string reason;
    std::map<int, HardConstraintProblem> checkedHardProblems;
    double simulationTime = 0.0;
    std::uint64_t frameIndex = std::numeric_limits<std::uint64_t>::max();
    std::vector<ResetCause> causes;
    std::vector<HardEdgeResetRecord> hardEdges;
    std::vector<LocalHardQpResetRecord> localHardQps;
};

struct GuardDecision {
    GuardStatus status = GuardStatus::Rejected;
    std::string reason;
};

struct HardEdgeAuthority {
    EdgeId edge = {EdgeKind::Localization, 0, 0, -1};
    double threshold = 0.0;
    Eigen::Vector2d basePosition = Eigen::Vector2d::Zero();
    double classKCoefficient = 1.0;
    int classKPower = 1;
};

struct GuardAuthority {
    std::vector<int> requiredUavNodes;
    std::vector<HardEdgeAuthority> hardEdges;
    std::uint64_t allocationVersion = 1;
};

using HardQpVerifier = std::function<FeasibilityResult(
    int,
    const HardConstraintProblem&
)>;

struct CommittedCertificateState {
    std::uint64_t version = 0;
    std::map<int, std::uint64_t> nodeVersions;
    bool valid = false;
    std::map<int, EndpointCertificateSnapshot> endpoints;
    std::map<int, HardConstraintProblem> hardProblems;
    std::map<int, NodeRateCertificate> certificates;
};

inline const char* resetCauseName(ResetCause cause) {
    switch (cause) {
        case ResetCause::ActiveReferenceChange:
            return "active-reference-change";
        case ResetCause::EstimateInterfaceReset:
            return "estimate-interface-reset";
        case ResetCause::CertificateDiscontinuity:
            return "certificate-discontinuity";
        case ResetCause::HardEdgeSnapshotChange:
            return "hard-edge-snapshot-change";
    }
    throw std::invalid_argument("reset cause is unsupported");
}

inline std::vector<ResetCause> coalescedResetCauses(
    ResetCause primary,
    const std::vector<ResetCause>& additional
) {
    std::vector<ResetCause> causes = {primary};
    for (const auto cause : additional) {
        if (std::find(causes.begin(), causes.end(), cause) == causes.end()) {
            causes.push_back(cause);
        }
    }
    std::sort(
        causes.begin(), causes.end(),
        [](ResetCause lhs, ResetCause rhs) {
            return static_cast<int>(lhs) < static_cast<int>(rhs);
        }
    );
    return causes;
}

inline nlohmann::json serializeResetTransaction(
    const ResetTransaction& transaction
) {
    nlohmann::json causes = nlohmann::json::array();
    for (const auto cause : transaction.causes) {
        causes.push_back(resetCauseName(cause));
    }
    nlohmann::json nodes = nlohmann::json::array();
    for (const auto& node : transaction.nodes) {
        nodes.push_back({
            {"node_id", node.nodeId},
            {"topological_index", node.topologicalIndex},
            {"snapshot_version", node.snapshotVersion},
            {"delta_estimate", {
                node.deltaEstimate.x(), node.deltaEstimate.y()
            }},
            {"delta_epsilon", node.deltaEpsilon},
            {"pre_active_references", node.preActiveReferences},
            {"post_active_references", node.postActiveReferences}
        });
    }
    nlohmann::json hardEdges = nlohmann::json::array();
    for (const auto& edge : transaction.hardEdges) {
        nlohmann::json rows = nlohmann::json::array();
        for (const auto& row : edge.endpointRows) {
            rows.push_back({
                {"owner", row.owner},
                {"coefficient", {
                    row.coefficient.x(), row.coefficient.y()
                }},
                {"constant", row.constant},
                {"allocation", row.allocation},
                {"snapshot_version", row.snapshotVersion},
                {"allocation_version", row.allocationVersion}
            });
        }
        hardEdges.push_back({
            {"kind", edge.edge.kind == EdgeKind::Localization
                ? "localization" : "collision"},
            {"low", edge.edge.low},
            {"high", edge.edge.high},
            {"base_id", edge.edge.baseId},
            {"threshold", edge.threshold},
            {"base_position", {
                edge.basePosition.x(), edge.basePosition.y()
            }},
            {"pre_barrier", edge.preBarrier},
            {"post_barrier", edge.postBarrier},
            {"class_k_coefficient", edge.classKCoefficient},
            {"class_k_power", edge.classKPower},
            {"endpoint_rows", rows}
        });
    }
    nlohmann::json localQps = nlohmann::json::array();
    for (const auto& localQp : transaction.localHardQps) {
        localQps.push_back({
            {"node_id", localQp.nodeId},
            {"snapshot_version", localQp.problem.snapshotVersion},
            {"allocation_version", localQp.problem.allocationVersion},
            {"hard_row_count", localQp.problem.rows.size()},
            {"bound_row_count", localQp.problem.bounds.size()},
            {"feasible", localQp.feasibility.feasible},
            {"solver_status", localQp.feasibility.status},
            {"minimum_residual", localQp.feasibility.minimumResidual},
            {"hard_problem_hash", localQp.feasibility.hardProblemHash}
        });
    }
    return {
        {"simulation_time", transaction.simulationTime},
        {"frame_index", transaction.frameIndex},
        {"predecessor_version", transaction.predecessorVersion},
        {"proposed_version", transaction.proposedVersion},
        {"causes", causes},
        {"nodes", nodes},
        {"hard_edges", hardEdges},
        {"local_hard_qps", localQps},
        {"status", transaction.status == GuardStatus::Accepted
            ? "accepted" : "rejected"},
        {"reason", transaction.reason}
    };
}

inline GuardDecision validateResetHistory(
    const std::vector<ResetTransaction>& transactions
) {
    std::optional<std::uint64_t> previousAcceptedVersion;
    std::set<std::uint64_t> attemptedFrames;
    std::set<double> acceptedTimes;
    for (const auto& transaction : transactions) {
        if (!std::isfinite(transaction.simulationTime)
            || transaction.frameIndex
               == std::numeric_limits<std::uint64_t>::max()
            || transaction.simulationTime
               != 0.5 * static_cast<double>(transaction.frameIndex)
            || transaction.causes.empty()
            || !attemptedFrames.insert(transaction.frameIndex).second) {
            return {
                GuardStatus::Rejected,
                "reset lifecycle retries or duplicates a finite mission frame"
            };
        }
        if (transaction.status != GuardStatus::Accepted) {
            continue;
        }
        if (!acceptedTimes.insert(transaction.simulationTime).second) {
            return {
                GuardStatus::Rejected,
                "accepted resets share a simulation timestamp"
            };
        }
        if (transaction.proposedVersion
                != transaction.predecessorVersion + 1U
            || (previousAcceptedVersion.has_value()
                && transaction.predecessorVersion
                   != *previousAcceptedVersion)) {
            return {
                GuardStatus::Rejected,
                "accepted reset versions are not strictly linked"
            };
        }
        previousAcceptedVersion = transaction.proposedVersion;
    }
    return {GuardStatus::Accepted, "accepted"};
}

inline std::vector<HardInputBound> theoremInputBounds(
    double planarComponentMax = 25.0,
    double yawRateMax = 0.35
) {
    return {
        {0, 1.0, planarComponentMax},
        {0, -1.0, planarComponentMax},
        {1, 1.0, planarComponentMax},
        {1, -1.0, planarComponentMax},
        {2, 1.0, yawRateMax},
        {2, -1.0, yawRateMax}
    };
}

inline std::string canonicalHardConstraintProblemHash(
    const HardConstraintProblem& problem
) {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto appendBytes = [&hash](const void* data, std::size_t size) {
        const auto* bytes = static_cast<const unsigned char*>(data);
        for (std::size_t index = 0; index < size; ++index) {
            hash ^= static_cast<std::uint64_t>(bytes[index]);
            hash *= 1099511628211ULL;
        }
    };
    const auto appendInteger = [&appendBytes](auto value) {
        appendBytes(&value, sizeof(value));
    };
    const auto appendDouble = [&appendBytes](double value) {
        std::uint64_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        appendBytes(&bits, sizeof(bits));
    };
    const auto appendString = [&appendBytes, &appendInteger](
        const std::string& value
    ) {
        const std::uint64_t size = value.size();
        appendInteger(size);
        appendBytes(value.data(), value.size());
    };

    appendInteger(problem.owner);
    appendInteger(problem.controlSize);
    appendDouble(problem.planarComponentMax);
    appendDouble(problem.yawRateMax);
    appendInteger(problem.snapshotVersion);
    appendInteger(problem.allocationVersion);
    appendInteger(static_cast<std::uint64_t>(problem.bounds.size()));
    for (const auto& bound : problem.bounds) {
        appendInteger(bound.controlIndex);
        appendDouble(bound.coefficient);
        appendDouble(bound.limit);
    }
    appendInteger(static_cast<std::uint64_t>(problem.rows.size()));
    for (const auto& row : problem.rows) {
        appendInteger(static_cast<int>(row.edge.kind));
        appendInteger(row.edge.low);
        appendInteger(row.edge.high);
        appendInteger(row.edge.baseId);
        appendInteger(row.owner);
        appendString(row.name);
        appendInteger(static_cast<std::uint64_t>(row.coefficients.size()));
        for (Eigen::Index index = 0; index < row.coefficients.size(); ++index) {
            appendDouble(row.coefficients[index]);
        }
        appendDouble(row.constant);
        appendDouble(row.postResetBarrier);
        appendInteger(row.snapshotVersion);
        appendInteger(row.allocationVersion);
    }
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(16) << hash;
    return stream.str();
}

inline bool sameHardConstraintProblem(
    const HardConstraintProblem& lhs,
    const HardConstraintProblem& rhs
) {
    if (lhs.owner != rhs.owner
        || lhs.controlSize != rhs.controlSize
        || lhs.planarComponentMax != rhs.planarComponentMax
        || lhs.yawRateMax != rhs.yawRateMax
        || lhs.snapshotVersion != rhs.snapshotVersion
        || lhs.allocationVersion != rhs.allocationVersion
        || lhs.bounds.size() != rhs.bounds.size()
        || lhs.rows.size() != rhs.rows.size()) {
        return false;
    }
    for (std::size_t index = 0; index < lhs.bounds.size(); ++index) {
        const auto& left = lhs.bounds[index];
        const auto& right = rhs.bounds[index];
        if (left.controlIndex != right.controlIndex
            || left.coefficient != right.coefficient
            || left.limit != right.limit) {
            return false;
        }
    }
    for (std::size_t index = 0; index < lhs.rows.size(); ++index) {
        const auto& left = lhs.rows[index];
        const auto& right = rhs.rows[index];
        if (left.edge != right.edge
            || left.owner != right.owner
            || left.name != right.name
            || left.coefficients != right.coefficients
            || left.constant != right.constant
            || left.postResetBarrier != right.postResetBarrier
            || left.snapshotVersion != right.snapshotVersion
            || left.allocationVersion != right.allocationVersion) {
            return false;
        }
    }
    return true;
}

inline bool sameEndpointCertificateSnapshot(
    const EndpointCertificateSnapshot& lhs,
    const EndpointCertificateSnapshot& rhs
) {
    return lhs.robotId == rhs.robotId
           && lhs.estimate == rhs.estimate
           && lhs.covariance == rhs.covariance
           && lhs.covarianceRateBound == rhs.covarianceRateBound
           && lhs.epsilon == rhs.epsilon
           && lhs.barNu == rhs.barNu
           && lhs.snapshotVersion == rhs.snapshotVersion
           && lhs.allocationVersion == rhs.allocationVersion;
}

inline bool sameNodeRateCertificateExceptVersion(
    const NodeRateCertificate& lhs,
    const NodeRateCertificate& rhs
) {
    if (lhs.robotId != rhs.robotId
        || lhs.information != rhs.information
        || lhs.covariance != rhs.covariance
        || lhs.epsilon != rhs.epsilon
        || lhs.informationRateBound != rhs.informationRateBound
        || lhs.covarianceRateBound != rhs.covarianceRateBound
        || lhs.epsilonRateBound != rhs.epsilonRateBound
        || lhs.frozenReferences.size() != rhs.frozenReferences.size()) {
        return false;
    }
    for (std::size_t index = 0;
         index < lhs.frozenReferences.size(); ++index) {
        const auto& left = lhs.frozenReferences[index];
        const auto& right = rhs.frozenReferences[index];
        if (left.referenceId != right.referenceId
            || left.direction != right.direction
            || left.effectiveVariance != right.effectiveVariance) {
            return false;
        }
    }
    return true;
}

inline std::vector<int> canonicalFrozenReferenceIds(
    const NodeRateCertificate& certificate
) {
    std::vector<int> references;
    references.reserve(certificate.frozenReferences.size());
    for (const auto& reference : certificate.frozenReferences) {
        references.push_back(reference.referenceId);
    }
    std::sort(references.begin(), references.end());
    if (std::adjacent_find(references.begin(), references.end())
        != references.end()) {
        throw std::invalid_argument(
            "frozen certificate reference ID is duplicated"
        );
    }
    return references;
}

inline bool frozenReferenceSetChanged(
    const NodeRateCertificate& predecessor,
    const NodeRateCertificate& proposed
) {
    return canonicalFrozenReferenceIds(predecessor)
           != canonicalFrozenReferenceIds(proposed);
}

inline std::vector<int> transitiveDescendants(
    const std::vector<int>& changedNodes,
    const ActiveDag& activeDag
) {
    if (changedNodes.empty()) {
        throw std::invalid_argument("changed-node set must not be empty");
    }

    std::unordered_map<int, std::size_t> indegree;
    for (const auto& [node, children] : activeDag) {
        if (node < 0) {
            throw std::invalid_argument("active DAG contains an unknown node ID");
        }
        if (!indegree.emplace(node, 0U).second) {
            throw std::invalid_argument("active DAG node is duplicated");
        }
        std::unordered_set<int> uniqueChildren;
        for (int child : children) {
            if (child <= 0
                || child == node
                || (node > 0 && child <= node)
                || !uniqueChildren.insert(child).second) {
                throw std::invalid_argument("active DAG contains a self-loop or duplicate edge");
            }
        }
    }
    for (const auto& [node, children] : activeDag) {
        (void)node;
        for (int child : children) {
            auto childIt = indegree.find(child);
            if (childIt == indegree.end()) {
                throw std::invalid_argument("active DAG edge names an absent child");
            }
            ++childIt->second;
        }
    }

    std::set<int> ready;
    for (const auto& [node, degree] : indegree) {
        if (degree == 0U) {
            ready.insert(node);
        }
    }
    std::vector<int> topologicalOrder;
    topologicalOrder.reserve(activeDag.size());
    while (!ready.empty()) {
        const int node = *ready.begin();
        ready.erase(ready.begin());
        topologicalOrder.push_back(node);
        for (int child : activeDag.at(node)) {
            auto& degree = indegree.at(child);
            --degree;
            if (degree == 0U) {
                ready.insert(child);
            }
        }
    }
    if (topologicalOrder.size() != activeDag.size()) {
        throw std::invalid_argument("active localization graph is cyclic");
    }

    std::unordered_set<int> affected;
    std::deque<int> pending;
    for (int changed : changedNodes) {
        if (changed <= 0 || activeDag.find(changed) == activeDag.end()) {
            throw std::invalid_argument("changed node is absent from active DAG");
        }
        if (!affected.insert(changed).second) {
            throw std::invalid_argument("changed node is duplicated");
        }
        pending.push_back(changed);
    }
    while (!pending.empty()) {
        const int node = pending.front();
        pending.pop_front();
        for (int child : activeDag.at(node)) {
            if (affected.insert(child).second) {
                pending.push_back(child);
            }
        }
    }

    std::vector<int> closure;
    closure.reserve(affected.size());
    for (int node : topologicalOrder) {
        if (affected.count(node) != 0U) {
            closure.push_back(node);
        }
    }
    return closure;
}

inline GuardDecision validateResetTransaction(const ResetProposal& proposal) {
    if (!std::isfinite(proposal.simulationTime)) {
        return {GuardStatus::Rejected, "reset time is nonfinite"};
    }
    if (proposal.predecessorVersion
            == std::numeric_limits<std::uint64_t>::max()
        || proposal.proposedVersion != proposal.predecessorVersion + 1U) {
        return {GuardStatus::Rejected, "reset version is not the exact successor"};
    }

    std::vector<int> requiredClosure;
    try {
        requiredClosure = transitiveDescendants(
            proposal.changedNodes, proposal.activeDag
        );
    } catch (const std::invalid_argument& error) {
        return {GuardStatus::Rejected, error.what()};
    }
    if (proposal.descendantClosure != requiredClosure) {
        return {GuardStatus::Rejected, "reset omits or reorders a transitive descendant"};
    }
    if (proposal.nodes.size() != requiredClosure.size()) {
        return {GuardStatus::Rejected, "reset does not contain one record per descendant"};
    }
    int previousTopologicalIndex = std::numeric_limits<int>::min();
    for (std::size_t index = 0; index < proposal.nodes.size(); ++index) {
        const auto& node = proposal.nodes[index];
        if (node.nodeId != requiredClosure[index]
            || node.topologicalIndex <= previousTopologicalIndex) {
            return {GuardStatus::Rejected, "node records are not in bases-first topological order"};
        }
        if (node.snapshotVersion != proposal.proposedVersion) {
            return {GuardStatus::Rejected, "node record mixes certificate versions"};
        }
        previousTopologicalIndex = node.topologicalIndex;
    }
    return {GuardStatus::Accepted, "accepted"};
}

inline GuardDecision validateResetTransaction(
    const ResetProposal& proposal,
    const CommittedCertificateState& committed
) {
    const auto structural = validateResetTransaction(proposal);
    if (structural.status != GuardStatus::Accepted) {
        return structural;
    }
    if (!committed.valid
        || committed.version != proposal.predecessorVersion) {
        return {
            GuardStatus::Rejected,
            "committed predecessor version is unavailable"
        };
    }

    std::map<int, EndpointCertificateSnapshot> proposedEndpoints;
    if (committed.endpoints.empty() && proposal.candidateEndpoints.empty()) {
        proposedEndpoints.clear();
    } else {
        if (proposal.candidateEndpoints.size() != committed.endpoints.size()) {
            return {
                GuardStatus::Rejected,
                "candidate endpoint state does not cover every committed UAV"
            };
        }
        for (const auto& [nodeId, predecessor] : committed.endpoints) {
            (void)predecessor;
            const auto candidate = proposal.candidateEndpoints.find(nodeId);
            if (candidate == proposal.candidateEndpoints.end()) {
                return {
                    GuardStatus::Rejected,
                    "candidate endpoint state omits a committed UAV"
                };
            }
        }
        proposedEndpoints = proposal.candidateEndpoints;
    }
    std::optional<std::uint64_t> allocationVersion;
    for (const auto& [nodeId, endpoint] : proposedEndpoints) {
        try {
            validateEndpointCertificateSnapshot(nodeId, endpoint);
        } catch (const std::invalid_argument& error) {
            return {GuardStatus::Rejected, error.what()};
        }
        if (endpoint.snapshotVersion != proposal.proposedVersion) {
            return {
                GuardStatus::Rejected,
                "proposed endpoint snapshot mixes certificate versions"
            };
        }
        if (!allocationVersion.has_value()) {
            allocationVersion = endpoint.allocationVersion;
        } else if (*allocationVersion != endpoint.allocationVersion) {
            return {
                GuardStatus::Rejected,
                "candidate endpoint state mixes allocation versions"
            };
        }
    }
    if (!committed.certificates.empty()
        || !proposal.candidateCertificates.empty()) {
        if (committed.certificates.size() != committed.endpoints.size()
            || proposal.candidateCertificates.size()
               != proposedEndpoints.size()) {
            return {
                GuardStatus::Rejected,
                "candidate full-certificate state is incomplete"
            };
        }
        for (const auto& [nodeId, endpoint] : proposedEndpoints) {
            const auto certificate =
                proposal.candidateCertificates.find(nodeId);
            if (certificate == proposal.candidateCertificates.end()
                || certificate->second.robotId != nodeId
                || certificate->second.snapshotVersion
                   != proposal.proposedVersion
                || certificate->second.covariance != endpoint.covariance
                || certificate->second.covarianceRateBound
                   != endpoint.covarianceRateBound
                || certificate->second.epsilon != endpoint.epsilon
                || certificate->second.epsilonRateBound
                   != endpoint.barNu) {
                return {
                    GuardStatus::Rejected,
                    "candidate endpoint and full certificate disagree"
                };
            }
        }
    }
    const std::set<int> affectedNodes(
        proposal.descendantClosure.begin(),
        proposal.descendantClosure.end()
    );
    for (const auto& [nodeId, predecessor] : committed.endpoints) {
        if (affectedNodes.count(nodeId) != 0U) {
            continue;
        }
        const auto& candidate = proposedEndpoints.at(nodeId);
        if (candidate.covariance != predecessor.covariance
            || candidate.covarianceRateBound
               != predecessor.covarianceRateBound
            || candidate.epsilon != predecessor.epsilon
            || candidate.barNu != predecessor.barNu
            || candidate.allocationVersion
               != predecessor.allocationVersion) {
            return {
                GuardStatus::Rejected,
                "unaffected endpoint certificate was not cloned and re-versioned"
            };
        }
        if (!committed.certificates.empty()) {
            const auto previousCertificate =
                committed.certificates.find(nodeId);
            const auto candidateCertificate =
                proposal.candidateCertificates.find(nodeId);
            if (previousCertificate == committed.certificates.end()
                || candidateCertificate
                   == proposal.candidateCertificates.end()
                || !sameNodeRateCertificateExceptVersion(
                    previousCertificate->second,
                    candidateCertificate->second
                )) {
                return {
                    GuardStatus::Rejected,
                    "unaffected full certificate was not cloned and re-versioned"
                };
            }
        }
    }
    for (const auto& node : proposal.nodes) {
        if (!node.proposedSnapshot.has_value()) {
            return {
                GuardStatus::Rejected,
                "affected node is missing its proposed endpoint snapshot"
            };
        }
        const auto candidate = proposedEndpoints.find(node.nodeId);
        if (candidate == proposedEndpoints.end()
            || !sameEndpointCertificateSnapshot(
                candidate->second, *node.proposedSnapshot
            )) {
            return {
                GuardStatus::Rejected,
                "affected node record disagrees with candidate endpoint state"
            };
        }
        const auto predecessor = committed.endpoints.find(node.nodeId);
        if (predecessor == committed.endpoints.end()) {
            return {
                GuardStatus::Rejected,
                "affected node has no committed predecessor endpoint"
            };
        }
        if (node.deltaEstimate
                != node.proposedSnapshot->estimate
                   - predecessor->second.estimate
            || node.deltaEpsilon
               != node.proposedSnapshot->epsilon
                  - predecessor->second.epsilon) {
            return {
                GuardStatus::Rejected,
                "affected-node reset deltas do not match endpoint values"
            };
        }
        if (!committed.certificates.empty()) {
            const auto previousCertificate =
                committed.certificates.find(node.nodeId);
            const auto proposedCertificate =
                proposal.candidateCertificates.find(node.nodeId);
            if (previousCertificate == committed.certificates.end()
                || proposedCertificate
                   == proposal.candidateCertificates.end()) {
                return {
                    GuardStatus::Rejected,
                    "affected node is missing exact active-reference data"
                };
            }
            try {
                if (node.preActiveReferences
                        != canonicalFrozenReferenceIds(
                            previousCertificate->second
                        )
                    || node.postActiveReferences
                       != canonicalFrozenReferenceIds(
                           proposedCertificate->second
                       )) {
                    return {
                        GuardStatus::Rejected,
                        "node reset record does not preserve exact active-reference identities"
                    };
                }
            } catch (const std::invalid_argument& error) {
                return {GuardStatus::Rejected, error.what()};
            }
        }
    }

    std::vector<EdgeId> requiredEdges = proposal.requiredHardEdges;
    try {
        for (const auto& edge : requiredEdges) {
            validateCanonicalEdge(edge);
        }
    } catch (const std::invalid_argument& error) {
        return {GuardStatus::Rejected, error.what()};
    }
    std::sort(requiredEdges.begin(), requiredEdges.end());
    if (std::adjacent_find(requiredEdges.begin(), requiredEdges.end())
        != requiredEdges.end()) {
        return {GuardStatus::Rejected, "required hard edge is duplicated"};
    }

    std::vector<EdgeId> providedEdges;
    providedEdges.reserve(proposal.hardEdges.size());
    for (const auto& edge : proposal.hardEdges) {
        providedEdges.push_back(edge.edge);
    }
    std::sort(providedEdges.begin(), providedEdges.end());
    if (providedEdges != requiredEdges) {
        return {
            GuardStatus::Rejected,
            "post-reset hard edges are incomplete or duplicated"
        };
    }

    std::map<std::pair<EdgeId, int>, std::pair<EndpointRow, double>>
        expectedRowsByOwner;
    for (const auto& hardEdge : proposal.hardEdges) {
        if (!std::isfinite(hardEdge.threshold)
            || hardEdge.threshold < 0.0
            || !std::isfinite(hardEdge.preBarrier)
            || !std::isfinite(hardEdge.classKCoefficient)
            || hardEdge.classKCoefficient < 0.0
            || hardEdge.classKPower <= 0
            || hardEdge.classKPower % 2 == 0) {
            return {
                GuardStatus::Rejected,
                "hard-edge threshold or predecessor barrier is nonfinite"
            };
        }
        const auto first = proposedEndpoints.find(hardEdge.edge.low);
        if (first == proposedEndpoints.end()) {
            return {
                GuardStatus::Rejected,
                "post-reset hard edge is missing its first endpoint"
            };
        }
        if (first->second.snapshotVersion != proposal.proposedVersion) {
            return {
                GuardStatus::Rejected,
                "post-reset hard edge mixes endpoint versions"
            };
        }

        Eigen::Vector2d secondPosition = hardEdge.basePosition;
        double separation = 0.0;
        double epsilonJ = 0.0;
        double nuJ = 0.0;
        if (hardEdge.edge.baseId >= 0) {
            if (!hardEdge.basePosition.allFinite()) {
                return {
                    GuardStatus::Rejected,
                    "hard-edge base position is nonfinite"
                };
            }
            separation = (
                first->second.estimate - hardEdge.basePosition
            ).norm();
        } else {
            const auto second = proposedEndpoints.find(hardEdge.edge.high);
            if (second == proposedEndpoints.end()) {
                return {
                    GuardStatus::Rejected,
                    "post-reset hard edge is missing its second endpoint"
                };
            }
            if (second->second.snapshotVersion != proposal.proposedVersion
                || second->second.allocationVersion
                   != first->second.allocationVersion) {
                return {
                    GuardStatus::Rejected,
                    "post-reset hard edge mixes endpoint or allocation versions"
                };
            }
            secondPosition = second->second.estimate;
            separation = (
                first->second.estimate - second->second.estimate
            ).norm();
            epsilonJ = second->second.epsilon;
            nuJ = second->second.barNu;
        }
        if (!std::isfinite(separation)
            || separation <= kBarrierEdgeSeparationTolerance) {
            return {
                GuardStatus::Rejected,
                "post-reset hard-edge separation is singular"
            };
        }
        const double postBarrier =
            hardEdge.edge.kind == EdgeKind::Localization
            ? hardEdge.threshold - separation
              - first->second.epsilon - epsilonJ
            : separation - hardEdge.threshold
              - first->second.epsilon - epsilonJ;
        if (!std::isfinite(postBarrier) || postBarrier < 0.0) {
            return {
                GuardStatus::Rejected,
                "post-reset hard barrier is negative"
            };
        }
        const auto predecessorFirst =
            committed.endpoints.find(hardEdge.edge.low);
        if (predecessorFirst == committed.endpoints.end()) {
            return {
                GuardStatus::Rejected,
                "pre-reset hard edge is missing its first endpoint"
            };
        }
        Eigen::Vector2d predecessorSecondPosition = hardEdge.basePosition;
        double predecessorEpsilonJ = 0.0;
        if (hardEdge.edge.baseId < 0) {
            const auto predecessorSecond =
                committed.endpoints.find(hardEdge.edge.high);
            if (predecessorSecond == committed.endpoints.end()) {
                return {
                    GuardStatus::Rejected,
                    "pre-reset hard edge is missing its second endpoint"
                };
            }
            predecessorSecondPosition =
                predecessorSecond->second.estimate;
            predecessorEpsilonJ = predecessorSecond->second.epsilon;
        }
        const double predecessorSeparation = (
            predecessorFirst->second.estimate
            - predecessorSecondPosition
        ).norm();
        if (!std::isfinite(predecessorSeparation)
            || predecessorSeparation
               <= kBarrierEdgeSeparationTolerance) {
            return {
                GuardStatus::Rejected,
                "pre-reset hard-edge separation is singular"
            };
        }
        const double exactPreBarrier =
            hardEdge.edge.kind == EdgeKind::Localization
            ? hardEdge.threshold - predecessorSeparation
              - predecessorFirst->second.epsilon
              - predecessorEpsilonJ
            : predecessorSeparation - hardEdge.threshold
              - predecessorFirst->second.epsilon
              - predecessorEpsilonJ;
        if (!std::isfinite(exactPreBarrier)
            || exactPreBarrier < 0.0
            || hardEdge.preBarrier != exactPreBarrier
            || hardEdge.postBarrier != postBarrier) {
            return {
                GuardStatus::Rejected,
                "reset hard-edge record does not contain exact pre/post barriers"
            };
        }
        const double alphaValue = hardEdge.classKCoefficient
            * std::pow(postBarrier, hardEdge.classKPower);
        EdgeSnapshot exactSnapshot;
        try {
            exactSnapshot = makeEdgeSnapshot(
                hardEdge.edge,
                first->second.estimate,
                secondPosition,
                first->second.barNu,
                nuJ,
                alphaValue,
                proposal.proposedVersion,
                first->second.allocationVersion
            );
        } catch (const std::invalid_argument& error) {
            return {GuardStatus::Rejected, error.what()};
        }
        std::vector<EndpointRow> expectedRows;
        try {
            expectedRows = allocatedRows(
                exactSnapshot,
                hardEdge.edge.baseId >= 0 ? 1.0 : 0.5,
                hardEdge.edge.baseId >= 0 ? 0.0 : 0.5
            );
        } catch (const std::invalid_argument& error) {
            return {GuardStatus::Rejected, error.what()};
        }
        if (hardEdge.endpointRows.size() != expectedRows.size()) {
            return {
                GuardStatus::Rejected,
                "post-reset hard edge is missing a required endpoint row"
            };
        }
        const auto rowMatches = [](const EndpointRow& actual,
                                   const EndpointRow& expected) {
            return actual.edge == expected.edge
                   && actual.owner == expected.owner
                   && actual.coefficient.isApprox(
                       expected.coefficient, 1e-12
                   )
                   && std::abs(actual.constant - expected.constant) <= 1e-12
                   && std::abs(actual.allocation - expected.allocation)
                      <= kAllocationSumTolerance
                   && actual.snapshotVersion == expected.snapshotVersion
                   && actual.allocationVersion
                      == expected.allocationVersion;
        };
        for (const auto& expectedRow : expectedRows) {
            const auto match = std::find_if(
                hardEdge.endpointRows.begin(),
                hardEdge.endpointRows.end(),
                [&](const EndpointRow& actual) {
                    return rowMatches(actual, expectedRow);
                }
            );
            if (match == hardEdge.endpointRows.end()) {
                return {
                    GuardStatus::Rejected,
                    "post-reset endpoint row does not match its exact edge snapshot"
                };
            }
            if (!expectedRowsByOwner.emplace(
                    std::make_pair(expectedRow.edge, expectedRow.owner),
                    std::make_pair(expectedRow, postBarrier)
                ).second) {
                return {
                    GuardStatus::Rejected,
                    "post-reset endpoint row owner is duplicated"
                };
            }
        }
    }

    std::vector<int> requiredUavNodes = proposal.requiredUavNodes;
    std::sort(requiredUavNodes.begin(), requiredUavNodes.end());
    std::vector<int> candidateUavNodes;
    candidateUavNodes.reserve(proposedEndpoints.size());
    for (const auto& [nodeId, endpoint] : proposedEndpoints) {
        (void)endpoint;
        candidateUavNodes.push_back(nodeId);
    }
    if (requiredUavNodes != candidateUavNodes) {
        return {
            GuardStatus::Rejected,
            "required local hard-QP universe does not cover every candidate UAV"
        };
    }
    if (requiredUavNodes.empty()
        && !proposal.localHardQps.empty()) {
        return {
            GuardStatus::Rejected,
            "local hard QPs are present without a required UAV universe"
        };
    }
    if (std::adjacent_find(
            requiredUavNodes.begin(), requiredUavNodes.end()
        ) != requiredUavNodes.end()
        || (!requiredUavNodes.empty() && requiredUavNodes.front() <= 0)) {
        return {
            GuardStatus::Rejected,
            "required UAV hard-QP universe is invalid"
        };
    }
    std::vector<int> providedUavNodes;
    providedUavNodes.reserve(proposal.localHardQps.size());
    for (const auto& localQp : proposal.localHardQps) {
        providedUavNodes.push_back(localQp.nodeId);
    }
    std::sort(providedUavNodes.begin(), providedUavNodes.end());
    if (providedUavNodes != requiredUavNodes) {
        return {
            GuardStatus::Rejected,
            "post-reset local hard-QP checks are incomplete or duplicated"
        };
    }
    for (const auto& localQp : proposal.localHardQps) {
        const auto& problem = localQp.problem;
        const auto& feasibility = localQp.feasibility;
        if (problem.owner != localQp.nodeId
            || problem.controlSize != 3
            || !std::isfinite(problem.planarComponentMax)
            || problem.planarComponentMax <= 0.0
            || !std::isfinite(problem.yawRateMax)
            || problem.yawRateMax != 0.35
            || problem.snapshotVersion != proposal.proposedVersion) {
            return {
                GuardStatus::Rejected,
                "local hard-QP problem is not same-version bounded data"
            };
        }
        const auto requiredBounds = theoremInputBounds(
            problem.planarComponentMax, problem.yawRateMax
        );
        if (problem.bounds.size() != requiredBounds.size()) {
            return {
                GuardStatus::Rejected,
                "local hard-QP problem does not contain six frozen bounds"
            };
        }
        for (std::size_t index = 0; index < requiredBounds.size(); ++index) {
            if (problem.bounds[index].controlIndex
                    != requiredBounds[index].controlIndex
                || problem.bounds[index].coefficient
                   != requiredBounds[index].coefficient
                || problem.bounds[index].limit
                   != requiredBounds[index].limit) {
                return {
                    GuardStatus::Rejected,
                    "local hard-QP input bounds are not the frozen six-row set"
                };
            }
        }
        std::vector<std::pair<EndpointRow, double>> expectedLocalRows;
        for (const auto& [key, rowAndBarrier] : expectedRowsByOwner) {
            if (key.second == localQp.nodeId) {
                expectedLocalRows.push_back(rowAndBarrier);
            }
        }
        if (problem.rows.size() != expectedLocalRows.size()) {
            return {
                GuardStatus::Rejected,
                "local hard-QP problem omits or duplicates an endpoint row"
            };
        }
        for (std::size_t index = 0; index < problem.rows.size(); ++index) {
            const auto& actual = problem.rows[index];
            const auto& expected = expectedLocalRows[index].first;
            const double expectedBarrier = expectedLocalRows[index].second;
            if (actual.edge != expected.edge
                || actual.owner != expected.owner
                || actual.name.empty()
                || actual.coefficients.size() != problem.controlSize
                || !actual.coefficients.head<2>().isApprox(
                    expected.coefficient, 1e-12
                )
                || !actual.coefficients.tail(
                    problem.controlSize - 2
                ).isZero(0.0)
                || std::abs(actual.constant - expected.constant) > 1e-12
                || actual.postResetBarrier != expectedBarrier
                || actual.snapshotVersion != proposal.proposedVersion
                || actual.allocationVersion
                   != problem.allocationVersion) {
                return {
                    GuardStatus::Rejected,
                    "local hard-QP row disagrees with exact endpoint data"
                };
            }
        }
        const std::string expectedHash =
            canonicalHardConstraintProblemHash(problem);
        if (feasibility.hardProblemHash != expectedHash) {
            return {
                GuardStatus::Rejected,
                "local hard-QP feasibility result hashes different rows"
            };
        }
        if (!feasibility.feasible
            || feasibility.status != "optimal"
            || !std::isfinite(feasibility.minimumResidual)
            || feasibility.minimumResidual < -1e-7) {
            return {
                GuardStatus::Rejected,
                "post-reset bounded local hard QP is infeasible for UAV "
                + std::to_string(localQp.nodeId)
                + " (" + feasibility.status + ")"
            };
        }
    }
    return {GuardStatus::Accepted, "accepted"};
}

namespace detail {

inline ResetTransaction commitValidatedResetTransaction(
    const ResetProposal& proposal,
    CommittedCertificateState& committed
) {
    const auto decision = validateResetTransaction(proposal, committed);
    auto transaction = ResetTransaction{
        proposal.predecessorVersion,
        proposal.proposedVersion,
        proposal.changedNodes,
        proposal.descendantClosure,
        proposal.nodes,
        decision.status,
        decision.reason
    };
    transaction.simulationTime = proposal.simulationTime;
    transaction.frameIndex = proposal.frameIndex;
    transaction.causes = coalescedResetCauses(
        proposal.cause, proposal.causes
    );
    transaction.hardEdges = proposal.hardEdges;
    transaction.localHardQps = proposal.localHardQps;
    if (decision.status != GuardStatus::Accepted) {
        return transaction;
    }
    CommittedCertificateState next = committed;
    for (auto& [nodeId, version] : next.nodeVersions) {
        (void)nodeId;
        if (version != proposal.predecessorVersion) {
            transaction.status = GuardStatus::Rejected;
            transaction.reason = "node predecessor version is unavailable";
            return transaction;
        }
        version = proposal.proposedVersion;
    }
    if (!proposal.candidateEndpoints.empty()) {
        next.endpoints = proposal.candidateEndpoints;
    }
    if (!proposal.candidateCertificates.empty()) {
        next.certificates = proposal.candidateCertificates;
    }
    next.hardProblems.clear();
    for (const auto& localQp : proposal.localHardQps) {
        next.hardProblems.emplace(localQp.nodeId, localQp.problem);
        transaction.checkedHardProblems.emplace(
            localQp.nodeId, localQp.problem
        );
    }
    next.version = proposal.proposedVersion;
    committed = std::move(next);
    return transaction;
}

}

inline GuardDecision validateGuardAuthority(
    const ResetProposal& proposal,
    const GuardAuthority& authority
) {
    if (authority.allocationVersion == 0) {
        return {
            GuardStatus::Rejected,
            "runtime allocation authority is invalid"
        };
    }
    std::vector<int> expectedNodes = authority.requiredUavNodes;
    std::sort(expectedNodes.begin(), expectedNodes.end());
    if (std::adjacent_find(expectedNodes.begin(), expectedNodes.end())
            != expectedNodes.end()
        || expectedNodes != proposal.requiredUavNodes) {
        return {
            GuardStatus::Rejected,
            "proposal UAV universe disagrees with runtime authority"
        };
    }
    for (const auto& [nodeId, endpoint] : proposal.candidateEndpoints) {
        (void)nodeId;
        if (endpoint.allocationVersion != authority.allocationVersion) {
            return {
                GuardStatus::Rejected,
                "candidate endpoint disagrees with frozen allocation version"
            };
        }
    }
    for (const auto& localQp : proposal.localHardQps) {
        if (localQp.problem.allocationVersion
            != authority.allocationVersion) {
            return {
                GuardStatus::Rejected,
                "candidate hard QP disagrees with frozen allocation version"
            };
        }
    }
    std::map<EdgeId, HardEdgeAuthority> expectedEdges;
    try {
        for (const auto& edge : authority.hardEdges) {
            validateCanonicalEdge(edge.edge);
            if (!expectedEdges.emplace(edge.edge, edge).second) {
                return {
                    GuardStatus::Rejected,
                    "runtime hard-edge authority is duplicated"
                };
            }
        }
    } catch (const std::invalid_argument& error) {
        return {GuardStatus::Rejected, error.what()};
    }
    std::vector<EdgeId> expectedEdgeIds;
    expectedEdgeIds.reserve(expectedEdges.size());
    for (const auto& [edge, definition] : expectedEdges) {
        (void)definition;
        expectedEdgeIds.push_back(edge);
    }
    std::vector<EdgeId> proposedEdgeIds = proposal.requiredHardEdges;
    std::sort(proposedEdgeIds.begin(), proposedEdgeIds.end());
    if (proposedEdgeIds != expectedEdgeIds
        || proposal.hardEdges.size() != expectedEdges.size()) {
        return {
            GuardStatus::Rejected,
            "proposal hard-edge universe disagrees with runtime authority"
        };
    }
    for (const auto& record : proposal.hardEdges) {
        const auto expected = expectedEdges.find(record.edge);
        if (expected == expectedEdges.end()
            || record.threshold != expected->second.threshold
            || record.basePosition != expected->second.basePosition
            || record.classKCoefficient
               != expected->second.classKCoefficient
            || record.classKPower != expected->second.classKPower) {
            return {
                GuardStatus::Rejected,
                "proposal hard-edge data disagrees with frozen runtime configuration"
            };
        }
    }
    return {GuardStatus::Accepted, "accepted"};
}

inline GuardDecision validateResetTransaction(
    const ResetProposal& proposal,
    const CommittedCertificateState& committed,
    const GuardAuthority& authority,
    const HardQpVerifier& verifier
) {
    const auto authorityDecision = validateGuardAuthority(
        proposal, authority
    );
    if (authorityDecision.status != GuardStatus::Accepted) {
        return authorityDecision;
    }
    if (!verifier) {
        return {
            GuardStatus::Rejected,
            "runtime hard-QP verifier is unavailable"
        };
    }
    ResetProposal preflight = proposal;
    for (auto& localQp : preflight.localHardQps) {
        localQp.feasibility = {
            true,
            "optimal",
            0.0,
            canonicalHardConstraintProblemHash(localQp.problem)
        };
    }
    const auto preflightDecision = validateResetTransaction(
        preflight, committed
    );
    if (preflightDecision.status != GuardStatus::Accepted) {
        return preflightDecision;
    }
    ResetProposal verified = proposal;
    for (auto& localQp : verified.localHardQps) {
        try {
            localQp.feasibility = verifier(
                localQp.nodeId, localQp.problem
            );
        } catch (const std::exception& error) {
            return {GuardStatus::Rejected, error.what()};
        } catch (...) {
            return {
                GuardStatus::Rejected,
                "runtime hard-QP verifier failed"
            };
        }
    }
    return validateResetTransaction(verified, committed);
}

inline ResetTransaction commitResetTransaction(
    const ResetProposal& proposal,
    CommittedCertificateState& committed,
    const GuardAuthority& authority,
    const HardQpVerifier& verifier
) {
    const auto authorityDecision = validateGuardAuthority(
        proposal, authority
    );
    if (authorityDecision.status != GuardStatus::Accepted || !verifier) {
        ResetTransaction transaction{
            proposal.predecessorVersion,
            proposal.proposedVersion,
            proposal.changedNodes,
            proposal.descendantClosure,
            proposal.nodes,
            GuardStatus::Rejected,
            authorityDecision.status != GuardStatus::Accepted
                ? authorityDecision.reason
                : "runtime hard-QP verifier is unavailable"
        };
        transaction.simulationTime = proposal.simulationTime;
        transaction.frameIndex = proposal.frameIndex;
        transaction.causes = coalescedResetCauses(
            proposal.cause, proposal.causes
        );
        transaction.hardEdges = proposal.hardEdges;
        transaction.localHardQps = proposal.localHardQps;
        return transaction;
    }
    ResetProposal preflight = proposal;
    for (auto& localQp : preflight.localHardQps) {
        localQp.feasibility = {
            true,
            "optimal",
            0.0,
            canonicalHardConstraintProblemHash(localQp.problem)
        };
    }
    const auto preflightDecision = validateResetTransaction(
        preflight, committed
    );
    if (preflightDecision.status != GuardStatus::Accepted) {
        ResetTransaction transaction{
            proposal.predecessorVersion,
            proposal.proposedVersion,
            proposal.changedNodes,
            proposal.descendantClosure,
            proposal.nodes,
            GuardStatus::Rejected,
            preflightDecision.reason
        };
        transaction.simulationTime = proposal.simulationTime;
        transaction.frameIndex = proposal.frameIndex;
        transaction.causes = coalescedResetCauses(
            proposal.cause, proposal.causes
        );
        transaction.hardEdges = proposal.hardEdges;
        transaction.localHardQps = proposal.localHardQps;
        return transaction;
    }
    ResetProposal verified = proposal;
    for (auto& localQp : verified.localHardQps) {
        try {
            localQp.feasibility = verifier(
                localQp.nodeId, localQp.problem
            );
        } catch (const std::exception& error) {
            localQp.feasibility = {
                false,
                error.what(),
                -std::numeric_limits<double>::infinity(),
                canonicalHardConstraintProblemHash(localQp.problem)
            };
        } catch (...) {
            localQp.feasibility = {
                false,
                "runtime hard-QP verifier failed",
                -std::numeric_limits<double>::infinity(),
                canonicalHardConstraintProblemHash(localQp.problem)
            };
        }
    }
    return detail::commitValidatedResetTransaction(verified, committed);
}

inline bool validateQualifiedMaterializedConfig(
    const nlohmann::json& config,
    const std::vector<std::string>& availableBackends
) {
    const auto exactNumber = [](const nlohmann::json& value,
                                const double expected) {
        return value.is_number()
               && std::isfinite(value.get<double>())
               && value.get<double>() == expected;
    };
    const auto exactFloat = [](const nlohmann::json& value,
                               const double expected) {
        return value.is_number_float()
               && std::isfinite(value.get<double>())
               && value.get<double>() == expected;
    };
    const auto exactInteger = [](const nlohmann::json& value,
                                 const std::int64_t expected) {
        return value.is_number_integer()
               && value.get<std::int64_t>() == expected;
    };
    const auto exactBoolean = [](const nlohmann::json& value,
                                 const bool expected) {
        return value.is_boolean() && value.get<bool>() == expected;
    };
    const auto exactString = [](const nlohmann::json& value,
                                const std::string& expected) {
        return value.is_string() && value.get<std::string>() == expected;
    };
    const auto exactNumericArray = [&exactNumber](
        const nlohmann::json& value,
        const std::vector<double>& expected
    ) {
        if (!value.is_array() || value.size() != expected.size()) {
            return false;
        }
        for (std::size_t index = 0; index < expected.size(); ++index) {
            if (!exactNumber(value.at(index), expected.at(index))) {
                return false;
            }
        }
        return true;
    };
    const auto exactPointArray = [&exactNumericArray](
        const nlohmann::json& value,
        const std::vector<std::vector<double>>& expected
    ) {
        if (!value.is_array() || value.size() != expected.size()) {
            return false;
        }
        for (std::size_t index = 0; index < expected.size(); ++index) {
            if (!exactNumericArray(value.at(index), expected.at(index))) {
                return false;
            }
        }
        return true;
    };
    try {
        const auto& estimator = config.at("qualified-estimator");
        if (!estimator.is_object() || estimator.size() != 4
            || !exactNumber(estimator.at("mode-tolerance-m"), 0.001)
            || !exactNumericArray(
                estimator.at("sensitivity-tolerances-m"),
                {0.0005, 0.001, 0.002}
            )) {
            return false;
        }
        const auto& deployment = estimator.at("deployment");
        const auto& anchorIds = deployment.at("anchor-ids");
        if (!deployment.is_object() || deployment.size() != 8
            || !anchorIds.is_array() || anchorIds.size() != 2
            || !exactInteger(anchorIds.at(0), 0)
            || !exactInteger(anchorIds.at(1), 2)
            || !exactPointArray(
                deployment.at("anchor-coordinates"),
                {{-1550.0, -300.0}, {-1550.0, 300.0}}
            )
            || !exactPointArray(
                deployment.at("deployment-vertices"),
                {
                    {-1490.0, -200.0},
                    {-1370.0, -200.0},
                    {-1370.0, 200.0},
                    {-1490.0, 200.0}
                }
            )
            || !exactNumericArray(
                deployment.at("unit-normal"), {1.0, 0.0}
            )
            || !exactNumber(deployment.at("offset"), 1550.0)
            || !exactInteger(deployment.at("ocean-side"), 1)
            || !exactNumber(deployment.at("margin-m"), 1.0)
            || !exactString(
                deployment.at("domain-version"), "ocean-side-v1"
            )) {
            return false;
        }
        const Eigen::Vector2d normal(
            deployment.at("unit-normal").at(0).get<double>(),
            deployment.at("unit-normal").at(1).get<double>()
        );
        if (!normal.allFinite() || std::abs(normal.norm() - 1.0) > 1e-12) {
            return false;
        }
        const double offset = deployment.at("offset").get<double>();
        for (const auto& anchor : deployment.at("anchor-coordinates")) {
            const Eigen::Vector2d point(
                anchor.at(0).get<double>(), anchor.at(1).get<double>()
            );
            if (!point.allFinite()
                || std::abs(normal.dot(point) + offset) > 1e-12) {
                return false;
            }
        }
        for (const auto& vertex : deployment.at("deployment-vertices")) {
            const Eigen::Vector2d point(
                vertex.at(0).get<double>(), vertex.at(1).get<double>()
            );
            if (!point.allFinite()
                || normal.dot(point) + offset
                   < deployment.at("margin-m").get<double>()) {
                return false;
            }
        }
        const auto& history = estimator.at("history");
        if (!history.is_object() || history.size() != 4
            || !exactNumber(
                history.at("q-threshold"), 11.829007011943707
            )
            || !exactNumericArray(
                history.at("process-noise-diagonal"), {0.25, 0.25}
            )
            || !exactInteger(
                history.at("public-max-age-frames"), 2
            )
            || !exactString(
                history.at("private-max-age-policy"),
                "mission-frames-minus-one"
            )) {
            return false;
        }

        const auto& covariance = config.at("position_covariance");
        if (!covariance.at("reference-selection").is_string()) {
            return false;
        }
        const std::string referenceSelection =
            covariance.at("reference-selection").get<std::string>();
        if ((referenceSelection != "dynamic-lower-index"
             && referenceSelection != "fixed-cbf-only")
            || !exactNumber(covariance.at("ranging_sigma"), 0.5)
            || !exactNumber(
                covariance.at("singular-distance-tolerance-m"), 1e-8
            )
            || !exactNumber(
                covariance.at("relative-spectral-threshold"), 1e-12
            )
            || !exactBoolean(covariance.at("enable"), true)
            || !exactString(
                covariance.at("uncertainty-type"), "max_eigenvalue"
            )) {
            return false;
        }

        if (!exactString(config.at("model"), "SingleIntegrate2D")) {
            return false;
        }
        const auto& cbfs = config.at("cbfs");
        const bool hasController = config.contains("qualified-controller");
        std::string controllerSchema;
        if (hasController) {
            const auto& marker = config.at("qualified-controller");
            if (!marker.is_object() || marker.size() != 1
                || !marker.at("schema-version").is_string()) {
                return false;
            }
            controllerSchema = marker.at("schema-version").get<std::string>();
            if (controllerSchema != "hard-interior-v2"
                && controllerSchema != "hard-interior-v3") {
                return false;
            }
        }
        const bool hasInteriorPolicy = cbfs.contains(
            "hard-interior-selection"
        );
        if (hasController != hasInteriorPolicy) {
            return false;
        }
        if (!exactString(
                cbfs.at("uncertainty-rate").at("mode"),
                "analytic-topological"
            )
            || !exactBoolean(cbfs.at("input-limits").at("on"), true)
            || !cbfs.at("input-limits").at("planar-component-max").is_number()
            || !std::isfinite(
                cbfs.at("input-limits").at("planar-component-max")
                    .get<double>()
            )
            || cbfs.at("input-limits").at("planar-component-max")
                   .get<double>()
               <= 0.0
            || !exactNumber(
                cbfs.at("input-limits").at("yaw-rate-max"), 0.35
            )) {
            return false;
        }
        if (hasController) {
            const std::string policyMode = controllerSchema
                == "hard-interior-v2"
                ? "planar-chebyshev-fraction-cap-v1"
                : "planar-chebyshev-fraction-cap-v2";
            const double fraction = controllerSchema
                == "hard-interior-v2" ? 0.1 : 0.131;
            const auto& policy = cbfs.at("hard-interior-selection");
            if (!policy.is_object() || policy.size() != 4
                || !exactString(policy.at("mode"), policyMode)
                || !exactFloat(policy.at("fraction"), fraction)
                || !exactFloat(policy.at("cap-mps"), 0.1)
                || !exactFloat(
                    policy.at("feasibility-tolerance-mps"), 1e-9
                )) {
                return false;
            }
        }
        const auto& hard = cbfs.at("without-slack");
        if (!exactString(hard.at("method"), "all")
            || !exactBoolean(hard.at("safety").at("on"), true)
            || !exactString(
                hard.at("safety").at("mode"), "allocated-pairwise"
            )
            || !exactBoolean(
                hard.at("safety").at("consider-uncertainty"), true
            )
            || !exactBoolean(hard.at("comm-fixed").at("on"), true)
            || !exactString(
                hard.at("comm-fixed").at("mode"), "allocated-pairwise"
            )
            || !exactBoolean(
                hard.at("comm-fixed").at("consider-uncertainty"), true
            )
            || !exactBoolean(hard.at("energy").at("on"), false)
            || !exactBoolean(hard.at("comm-auto").at("on"), false)) {
            return false;
        }
        if (hasController) {
            const auto exactHardAlpha = [&exactNumber, &exactInteger](
                const nlohmann::json& hardClass
            ) {
                if (!hardClass.contains("alpha")) {
                    return false;
                }
                const auto& alpha = hardClass.at("alpha");
                return alpha.is_object() && alpha.size() == 2
                    && exactNumber(alpha.at("coe"), 0.1)
                    && exactInteger(alpha.at("pow"), 1);
            };
            if (!exactHardAlpha(hard.at("safety"))
                || !exactHardAlpha(hard.at("comm-fixed"))) {
                return false;
            }
        }
        const auto& execute = config.at("execute");
        if (!exactString(
                execute.at("execution-mode"), "distributed"
            )
            || !exactNumber(execute.at("time-step"), 0.5)) {
            return false;
        }
        if (!config.at("optimiser").is_string()) {
            return false;
        }
        const std::string backend = config.at("optimiser").get<std::string>();
        return backend == "Gurobi" && std::find(
            availableBackends.begin(), availableBackends.end(), backend
        ) != availableBackends.end();
    } catch (...) {
        return false;
    }
}

}

#endif
