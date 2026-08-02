#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "cbf/HybridCertificateGuard.hpp"
#include "Robot.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

namespace {

cbf2026::ActiveDag chainDag() {
    return {
        {0, {1, 4}},
        {1, {2}},
        {2, {3}},
        {3, {}},
        {4, {}}
    };
}

cbf2026::ResetProposal proposalWith(
    std::vector<int> closure,
    std::vector<cbf2026::NodeResetRecord> nodes
) {
    return {
        2.0,
        cbf2026::ResetCause::ActiveReferenceChange,
        7,
        8,
        {1},
        chainDag(),
        std::move(closure),
        std::move(nodes)
    };
}

cbf2026::EndpointCertificateSnapshot endpoint(
    int robotId,
    std::uint64_t version,
    double x,
    double y
) {
    return {
        robotId,
        Eigen::Vector2d(x, y),
        Eigen::Matrix2d::Identity(),
        0.0,
        3.0,
        0.0,
        version,
        1
    };
}

cbf2026::HardConstraintRow hardRow(
    const cbf2026::EndpointRow& endpointRow,
    double barrier,
    const std::string& name
) {
    cbf2026::HardConstraintRow row;
    row.edge = endpointRow.edge;
    row.owner = endpointRow.owner;
    row.name = name;
    row.coefficients = cbf2026::endpointRowToModelControl(endpointRow, 3);
    row.constant = endpointRow.constant;
    row.postResetBarrier = barrier;
    row.snapshotVersion = endpointRow.snapshotVersion;
    row.allocationVersion = endpointRow.allocationVersion;
    return row;
}

cbf2026::HardConstraintProblem hardProblem(
    int owner,
    const cbf2026::HardConstraintRow& row
) {
    cbf2026::HardConstraintProblem problem;
    problem.owner = owner;
    problem.controlSize = 3;
    problem.planarComponentMax = 25.0;
    problem.snapshotVersion = 8;
    problem.allocationVersion = 1;
    problem.rows = {row};
    problem.yawRateMax = 0.35;
    problem.bounds = cbf2026::theoremInputBounds();
    return problem;
}

json robotSettings() {
    return {
        {"world", {
            {"boundary", json::array({
                json::array({-100.0, -100.0}),
                json::array({100.0, -100.0}),
                json::array({100.0, 100.0}),
                json::array({-100.0, 100.0})
            })},
            {"charge", json::array()},
            {"spacing", 1.0}
        }},
        {"num", 2},
        {"all", json::array({1, 2})},
        {"dim", 2},
        {"formation", {
            {"parts", 1},
            {"bases-id", json::array({json::array({0, 1})})}
        }},
        {"bases", json::array({
            json::array({-20.0, 0.0}),
            json::array({0.0, -20.0})
        })},
        {"initial", {
            {"position", {
                {"method", "specified"},
                {"positions", json::array({
                    json::array({0.0, 0.0}),
                    json::array({10.0, 0.0})
                })}
            }},
            {"battery", {{"min", 4000.0}, {"max", 4000.0}}},
            {"yawDeg", 0.0}
        }},
        {"model", "SingleIntegrate2D"},
        {"optimiser", "Gurobi"},
        {"position_covariance", {
            {"enable", true},
            {"ranging_sigma", 0.5},
            {"uncertainty-type", "max_eigenvalue"},
            {"reference-selection", "fixed-cbf-only"}
        }},
        {"cbfs", {
            {"objective-function", {{"k_delta", 1.0}}},
            {"uncertainty-rate", {{"mode", "analytic-topological"}}},
            {"input-limits", {
                {"on", true},
                {"planar-component-max", 25.0},
                {"yaw-rate-max", 0.35}
            }},
            {"with-slack", {
                {"cvt", {{"on", false}}},
                {"cvt-yaw", {{"on", false}}},
                {"target-yaw", {{"on", false}}}
            }},
            {"without-slack", {
                {"method", "all"},
                {"comm-fixed", {
                    {"on", true},
                    {"mode", "allocated-pairwise"},
                    {"max-range", 20.0},
                    {"k", 1.0},
                    {"compensate-velocity", false},
                    {"consider-uncertainty", true},
                    {"min-neighbour-id-offset", -2},
                    {"max-neighbour-id-offset", 0},
                    {"alpha", {{"coe", 1.0}, {"pow", 1}}}
                }},
                {"comm-auto", {{"on", false}}},
                {"safety", {
                    {"on", false},
                    {"mode", "allocated-pairwise"},
                    {"safe-distance", 1.0},
                    {"consider-uncertainty", true},
                    {"alpha", {{"coe", 1.0}, {"pow", 1}}}
                }},
                {"energy", {{"on", false}}}
            }}
        }},
        {"debug", {{"opt-cbc", false}}},
        {"execute", {{"time-step", 0.5}}}
    };
}

json loadMaterializedConfig(const std::string& path) {
    std::ifstream stream(path);
    if (!stream.good()) {
        throw std::runtime_error("unable to load diagnostic config");
    }
    return json::parse(stream);
}

void mergeDiagnosticOverlay(json& materialized, const json& overlay) {
    for (const auto& [key, value] : overlay.items()) {
        if (value.is_object()
            && materialized.contains(key)
            && materialized.at(key).is_object()) {
            mergeDiagnosticOverlay(materialized[key], value);
        } else {
            materialized[key] = value;
        }
    }
}

json materializedDiagnosticConfig(const std::string& overlay) {
    json materialized = loadMaterializedConfig("config/config.json");
    mergeDiagnosticOverlay(materialized, loadMaterializedConfig(overlay));
    return materialized;
}

}

TEST_CASE("changed node expands to its exact topological descendant closure") {
    const auto closure = cbf2026::transitiveDescendants({1}, chainDag());

    CHECK(closure == std::vector<int>({1, 2, 3}));
}

TEST_CASE("same-cardinality base identity swap changes the exact active set") {
    auto predecessor = cbf2026::baseRateCertificate(1, 7);
    predecessor.frozenReferences = {{
        cbf2026::canonicalBaseReferenceId(0),
        Eigen::Vector2d(1.0, 0.0),
        1.0
    }};
    auto proposed = predecessor;
    proposed.snapshotVersion = 8;
    proposed.frozenReferences.front().referenceId =
        cbf2026::canonicalBaseReferenceId(2);

    CHECK(cbf2026::canonicalFrozenReferenceIds(predecessor)
          == std::vector<int>({-1}));
    CHECK(cbf2026::canonicalFrozenReferenceIds(proposed)
          == std::vector<int>({-3}));
    CHECK(cbf2026::frozenReferenceSetChanged(
        predecessor, proposed
    ));
}

TEST_CASE("descendant closure excludes bases and rejects backward predecessor direction") {
    CHECK_THROWS_AS(
        cbf2026::transitiveDescendants({0}, chainDag()),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::transitiveDescendants(
            {2},
            {{0, {2}}, {1, {}}, {2, {1}}}
        ),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::transitiveDescendants({1, 1}, chainDag()),
        std::invalid_argument
    );
}

TEST_CASE("missing descendant and mixed snapshot versions reject before commit") {
    const auto missing = cbf2026::validateResetTransaction(proposalWith(
        {1, 2},
        {{1, 1, 8}, {2, 2, 8}}
    ));
    CHECK(missing.status == cbf2026::GuardStatus::Rejected);

    const auto mixed = cbf2026::validateResetTransaction(proposalWith(
        {1, 2, 3},
        {{1, 1, 8}, {2, 2, 7}, {3, 3, 8}}
    ));
    CHECK(mixed.status == cbf2026::GuardStatus::Rejected);
}

TEST_CASE("valid successor transaction commits every descendant atomically") {
    cbf2026::CommittedCertificateState committed{
        7,
        {{1, 7}, {2, 7}, {3, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)},
            {3, endpoint(3, 7, 20.0, 0.0)}
        }
    };
    auto proposal = proposalWith(
        {1, 2, 3},
        {{1, 1, 8}, {2, 2, 8}, {3, 3, 8}}
    );
    for (auto& node : proposal.nodes) {
        node.proposedSnapshot = endpoint(
            node.nodeId, 8, 10.0 * (node.nodeId - 1), 0.0
        );
        proposal.candidateEndpoints.emplace(
            node.nodeId, *node.proposedSnapshot
        );
    }
    proposal.requiredUavNodes = {1, 2, 3};
    for (int nodeId : proposal.requiredUavNodes) {
        cbf2026::HardConstraintProblem problem;
        problem.owner = nodeId;
        problem.controlSize = 3;
        problem.planarComponentMax = 25.0;
        problem.snapshotVersion = 8;
        problem.allocationVersion = 1;
        problem.yawRateMax = 0.35;
        problem.bounds = cbf2026::theoremInputBounds();
        proposal.localHardQps.push_back({
            nodeId,
            problem,
            {
                true,
                "optimal",
                0.0,
                cbf2026::canonicalHardConstraintProblemHash(problem)
            }
        });
    }

    const cbf2026::GuardAuthority authority{
        {1, 2, 3},
        {}
    };
    const auto verifier = [](
        int,
        const cbf2026::HardConstraintProblem& problem
    ) {
        return cbf2026::FeasibilityResult{
            true,
            "optimal",
            0.0,
            cbf2026::canonicalHardConstraintProblemHash(problem)
        };
    };
    const auto transaction = cbf2026::commitResetTransaction(
        proposal, committed, authority, verifier
    );

    CHECK(transaction.status == cbf2026::GuardStatus::Accepted);
    CHECK(committed.version == 8);
    CHECK(committed.nodeVersions == std::map<int, std::uint64_t>({
        {1, 8}, {2, 8}, {3, 8}
    }));
}

TEST_CASE("runtime authority rejects forged universe config and feasibility") {
    cbf2026::CommittedCertificateState committed{
        7,
        {{1, 7}, {2, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)}
        }
    };
    cbf2026::ResetProposal proposal{
        2.0,
        cbf2026::ResetCause::ActiveReferenceChange,
        7,
        8,
        {1},
        {{0, {1}}, {1, {2}}, {2, {}}},
        {1, 2},
        {{1, 1, 8}, {2, 2, 8}}
    };
    proposal.nodes[0].proposedSnapshot = endpoint(1, 8, 0.0, 0.0);
    proposal.nodes[1].proposedSnapshot = endpoint(2, 8, 10.0, 0.0);
    proposal.candidateEndpoints = {
        {1, *proposal.nodes[0].proposedSnapshot},
        {2, *proposal.nodes[1].proposedSnapshot}
    };
    const auto edge = cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Localization, 1, 2
    );
    auto snapshot = cbf2026::makeEdgeSnapshot(
        edge,
        proposal.candidateEndpoints.at(1).estimate,
        proposal.candidateEndpoints.at(2).estimate,
        0.0,
        0.0,
        4.0,
        8,
        1
    );
    const auto rows = cbf2026::allocatedRows(snapshot, 0.5, 0.5);
    proposal.requiredHardEdges = {edge};
    proposal.hardEdges = {{
        edge,
        20.0,
        Eigen::Vector2d::Zero(),
        4.0,
        rows,
        1.0,
        1,
        4.0
    }};
    proposal.requiredUavNodes = {1, 2};
    for (const auto& row : rows) {
        auto problem = hardProblem(
            row.owner,
            hardRow(row, 4.0, "authoritative-row")
        );
        proposal.localHardQps.push_back({
            row.owner,
            problem,
            {
                true,
                "optimal",
                0.0,
                cbf2026::canonicalHardConstraintProblemHash(problem)
            }
        });
    }
    const cbf2026::GuardAuthority authority{
        {1, 2},
        {{
            edge,
            20.0,
            Eigen::Vector2d::Zero(),
            1.0,
            1
        }}
    };

    auto wrongThreshold = authority;
    wrongThreshold.hardEdges.front().threshold = 21.0;
    auto preserved = committed;
    CHECK(cbf2026::commitResetTransaction(
        proposal,
        preserved,
        wrongThreshold,
        [](int, const cbf2026::HardConstraintProblem& problem) {
            return cbf2026::FeasibilityResult{
                true,
                "optimal",
                0.0,
                cbf2026::canonicalHardConstraintProblemHash(problem)
            };
        }
    ).status == cbf2026::GuardStatus::Rejected);
    CHECK(preserved.version == 7);

    auto malformed = proposal;
    malformed.localHardQps.front()
        .problem.bounds.front().controlIndex = 99;
    bool verifierCalled = false;
    preserved = committed;
    CHECK(cbf2026::commitResetTransaction(
        malformed,
        preserved,
        authority,
        [&verifierCalled](
            int,
            const cbf2026::HardConstraintProblem& problem
        ) {
            verifierCalled = true;
            return cbf2026::FeasibilityResult{
                true,
                "optimal",
                0.0,
                cbf2026::canonicalHardConstraintProblemHash(problem)
            };
        }
    ).status == cbf2026::GuardStatus::Rejected);
    CHECK_FALSE(verifierCalled);
    CHECK(preserved.version == 7);

    auto wrongAllocation = authority;
    wrongAllocation.allocationVersion = 2;
    preserved = committed;
    CHECK(cbf2026::commitResetTransaction(
        proposal,
        preserved,
        wrongAllocation,
        [](int, const cbf2026::HardConstraintProblem& problem) {
            return cbf2026::FeasibilityResult{
                true,
                "optimal",
                0.0,
                cbf2026::canonicalHardConstraintProblemHash(problem)
            };
        }
    ).status == cbf2026::GuardStatus::Rejected);
    CHECK(preserved.version == 7);

    for (const bool tamperPostBarrier : {false, true}) {
        auto tamperedBarrier = proposal;
        if (tamperPostBarrier) {
            tamperedBarrier.hardEdges.front().postBarrier += 1.0;
        } else {
            tamperedBarrier.hardEdges.front().preBarrier += 1.0;
        }
        verifierCalled = false;
        preserved = committed;
        CHECK(cbf2026::commitResetTransaction(
            tamperedBarrier,
            preserved,
            authority,
            [&verifierCalled](
                int,
                const cbf2026::HardConstraintProblem& problem
            ) {
                verifierCalled = true;
                return cbf2026::FeasibilityResult{
                    true,
                    "optimal",
                    0.0,
                    cbf2026::canonicalHardConstraintProblemHash(problem)
                };
            }
        ).status == cbf2026::GuardStatus::Rejected);
        CHECK_FALSE(verifierCalled);
        CHECK(preserved.version == 7);
    }

    auto missingUav = authority;
    missingUav.requiredUavNodes = {1};
    preserved = committed;
    CHECK(cbf2026::commitResetTransaction(
        proposal,
        preserved,
        missingUav,
        [](int, const cbf2026::HardConstraintProblem& problem) {
            return cbf2026::FeasibilityResult{
                true,
                "optimal",
                0.0,
                cbf2026::canonicalHardConstraintProblemHash(problem)
            };
        }
    ).status == cbf2026::GuardStatus::Rejected);
    CHECK(preserved.version == 7);

    preserved = committed;
    CHECK(cbf2026::commitResetTransaction(
        proposal,
        preserved,
        authority,
        [](int, const cbf2026::HardConstraintProblem& problem) {
            return cbf2026::FeasibilityResult{
                false,
                "infeasible",
                -std::numeric_limits<double>::infinity(),
                cbf2026::canonicalHardConstraintProblemHash(problem)
            };
        }
    ).status == cbf2026::GuardStatus::Rejected);
    CHECK(preserved.version == 7);
}

TEST_CASE("negative reconstructed localization barrier rejects atomically") {
    cbf2026::CommittedCertificateState committed{
        7,
        {{1, 7}, {2, 7}, {3, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)},
            {3, endpoint(3, 7, 20.0, 0.0)}
        }
    };
    auto proposal = proposalWith(
        {1, 2, 3},
        {{1, 1, 8}, {2, 2, 8}, {3, 3, 8}}
    );
    for (auto& node : proposal.nodes) {
        node.proposedSnapshot = endpoint(
            node.nodeId, 8, 10.0 * (node.nodeId - 1), 0.0
        );
        proposal.candidateEndpoints.emplace(
            node.nodeId, *node.proposedSnapshot
        );
    }
    const auto edge = cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Localization, 1, 2
    );
    proposal.requiredHardEdges = {edge};
    proposal.hardEdges = {{edge, 5.0, Eigen::Vector2d::Zero(), 84.0, {}}};
    const auto originalVersions = committed.nodeVersions;

    const auto transaction = cbf2026::validateResetTransaction(
        proposal, committed
    );

    CHECK(transaction.status == cbf2026::GuardStatus::Rejected);
    CHECK(committed.version == 7);
    CHECK(committed.nodeVersions == originalVersions);
}

TEST_CASE("infeasible bounded local hard QP rejects atomically") {
    cbf2026::CommittedCertificateState committed{
        7,
        {{1, 7}, {2, 7}, {3, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)},
            {3, endpoint(3, 7, 20.0, 0.0)}
        }
    };
    auto proposal = proposalWith(
        {1, 2, 3},
        {{1, 1, 8}, {2, 2, 8}, {3, 3, 8}}
    );
    for (auto& node : proposal.nodes) {
        node.proposedSnapshot = endpoint(
            node.nodeId, 8, 10.0 * (node.nodeId - 1), 0.0
        );
        proposal.candidateEndpoints.emplace(
            node.nodeId, *node.proposedSnapshot
        );
    }
    const auto edge = cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Localization, 1, 2
    );
    proposal.requiredHardEdges = {edge};
    proposal.hardEdges = {{
        edge, 20.0, Eigen::Vector2d::Zero(), 4.0, {}, 1.0, 1, 4.0
    }};
    proposal.requiredUavNodes = {1, 2, 3};
    for (int nodeId : proposal.requiredUavNodes) {
        cbf2026::HardConstraintProblem problem{
            nodeId, 3, 25.0, 8, 1, {}
        };
        proposal.localHardQps.push_back({
            nodeId,
            problem,
            {
                nodeId != 2,
                nodeId == 2 ? "infeasible" : "optimal",
                nodeId == 2 ? -1.0 : 0.0,
                "fixture"
            }
        });
    }
    const auto originalVersions = committed.nodeVersions;

    const auto transaction = cbf2026::validateResetTransaction(
        proposal, committed
    );

    CHECK(transaction.status == cbf2026::GuardStatus::Rejected);
    CHECK(committed.version == 7);
    CHECK(committed.nodeVersions == originalVersions);
}

TEST_CASE("missing required endpoint row rejects before commit") {
    cbf2026::CommittedCertificateState committed{
        7,
        {{1, 7}, {2, 7}, {3, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)},
            {3, endpoint(3, 7, 20.0, 0.0)}
        }
    };
    auto proposal = proposalWith(
        {1, 2, 3},
        {{1, 1, 8}, {2, 2, 8}, {3, 3, 8}}
    );
    for (auto& node : proposal.nodes) {
        node.proposedSnapshot = endpoint(
            node.nodeId, 8, 10.0 * (node.nodeId - 1), 0.0
        );
        proposal.candidateEndpoints.emplace(
            node.nodeId, *node.proposedSnapshot
        );
    }
    const auto edge = cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Localization, 1, 2
    );
    proposal.requiredHardEdges = {edge};
    proposal.hardEdges = {{
        edge, 20.0, Eigen::Vector2d::Zero(), 4.0, {}, 1.0, 1, 4.0
    }};
    proposal.requiredUavNodes = {1, 2, 3};
    for (int nodeId : proposal.requiredUavNodes) {
        proposal.localHardQps.push_back({
            nodeId,
            {nodeId, 3, 25.0, 8, 1, {}},
            {true, "optimal", 0.0, "fixture"}
        });
    }

    const auto transaction = cbf2026::validateResetTransaction(
        proposal, committed
    );

    CHECK(transaction.status == cbf2026::GuardStatus::Rejected);
    CHECK(committed.version == 7);
}

TEST_CASE("candidate endpoint state must clone and re-version unaffected UAVs") {
    cbf2026::CommittedCertificateState committed{
        7,
        {{1, 7}, {2, 7}, {3, 7}, {4, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)},
            {3, endpoint(3, 7, 20.0, 0.0)},
            {4, endpoint(4, 7, 0.0, 20.0)}
        }
    };
    auto proposal = proposalWith(
        {1, 2, 3},
        {{1, 1, 8}, {2, 2, 8}, {3, 3, 8}}
    );
    for (auto& node : proposal.nodes) {
        node.proposedSnapshot = endpoint(
            node.nodeId, 8, 10.0 * (node.nodeId - 1), 0.0
        );
        proposal.candidateEndpoints.emplace(
            node.nodeId, *node.proposedSnapshot
        );
    }

    const auto transaction = cbf2026::validateResetTransaction(
        proposal, committed
    );

    CHECK(transaction.status == cbf2026::GuardStatus::Rejected);
    CHECK(committed.version == 7);
}

TEST_CASE("accepted transaction retains the exact checked hard problems") {
    cbf2026::CommittedCertificateState committed{
        7,
        {{1, 7}, {2, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)}
        },
        {}
    };
    cbf2026::ResetProposal proposal{
        2.0,
        cbf2026::ResetCause::ActiveReferenceChange,
        7,
        8,
        {1},
        {{0, {1}}, {1, {2}}, {2, {}}},
        {1, 2},
        {{1, 1, 8}, {2, 2, 8}}
    };
    proposal.nodes[0].proposedSnapshot = endpoint(1, 8, 0.0, 0.0);
    proposal.nodes[1].proposedSnapshot = endpoint(2, 8, 10.0, 0.0);
    proposal.candidateEndpoints = {
        {1, *proposal.nodes[0].proposedSnapshot},
        {2, *proposal.nodes[1].proposedSnapshot}
    };
    const auto edge = cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Localization, 1, 2
    );
    auto snapshot = cbf2026::makeEdgeSnapshot(
        edge,
        proposal.candidateEndpoints.at(1).estimate,
        proposal.candidateEndpoints.at(2).estimate,
        0.0,
        0.0,
        4.0,
        8,
        1
    );
    const auto endpointRows = cbf2026::allocatedRows(snapshot, 0.5, 0.5);
    proposal.requiredHardEdges = {edge};
    proposal.hardEdges = {{
        edge,
        20.0,
        Eigen::Vector2d::Zero(),
        4.0,
        endpointRows,
        1.0,
        1,
        4.0
    }};
    proposal.requiredUavNodes = {1, 2};
    for (const auto& endpointRow : endpointRows) {
        auto problem = hardProblem(
            endpointRow.owner,
            hardRow(
                endpointRow,
                4.0,
                "fixedCommCBF(#"
                    + std::to_string(
                        endpointRow.owner == 1 ? 2 : 1
                    )
                    + ")"
            )
        );
        const auto digest =
            cbf2026::canonicalHardConstraintProblemHash(problem);
        proposal.localHardQps.push_back({
            endpointRow.owner,
            problem,
            {true, "optimal", 0.0, digest}
        });
    }

    for (const double invalidResidual : {
             -1e-6,
             -std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN()
         }) {
        auto tamperedProposal = proposal;
        tamperedProposal.localHardQps.front()
            .feasibility.minimumResidual = invalidResidual;
        auto preserved = committed;
        const auto rejected = cbf2026::validateResetTransaction(
            tamperedProposal, preserved
        );
        CHECK(rejected.status == cbf2026::GuardStatus::Rejected);
        CHECK(preserved.version == 7);
        CHECK(preserved.nodeVersions == committed.nodeVersions);
    }

    const cbf2026::GuardAuthority authority{
        {1, 2},
        {{
            edge,
            20.0,
            Eigen::Vector2d::Zero(),
            1.0,
            1
        }}
    };
    const auto transaction = cbf2026::commitResetTransaction(
        proposal,
        committed,
        authority,
        [](int, const cbf2026::HardConstraintProblem& problem) {
            return cbf2026::FeasibilityResult{
                true,
                "optimal",
                0.0,
                cbf2026::canonicalHardConstraintProblemHash(problem)
            };
        }
    );

    REQUIRE(transaction.status == cbf2026::GuardStatus::Accepted);
    REQUIRE(committed.hardProblems.size() == 2);
    REQUIRE(transaction.hardEdges.size() == 1);
    CHECK(transaction.hardEdges.front().preBarrier == 4.0);
    CHECK(transaction.hardEdges.front().postBarrier == 4.0);
    REQUIRE(transaction.localHardQps.size() == 2);
    const auto serialized = cbf2026::serializeResetTransaction(transaction);
    CHECK(serialized.at("nodes").size() == 2);
    CHECK(serialized.at("hard_edges").size() == 1);
    CHECK(serialized.at("hard_edges").front().at("pre_barrier") == 4.0);
    CHECK(serialized.at("hard_edges").front().at("post_barrier") == 4.0);
    CHECK(serialized.at("hard_edges").front()
          .at("class_k_coefficient") == 1.0);
    CHECK(serialized.at("hard_edges").front()
          .at("class_k_power") == 1);
    CHECK(serialized.at("hard_edges").front()
          .at("endpoint_rows").front().at("allocation") == 0.5);
    CHECK(serialized.at("local_hard_qps").size() == 2);
    CHECK(serialized.at("local_hard_qps").front()
          .at("allocation_version") == 1);
    CHECK(serialized.at("local_hard_qps").front().at("feasible") == true);
    CHECK(serialized.at("local_hard_qps").front()
          .at("solver_status") == "optimal");
    CHECK(serialized.at("local_hard_qps").front()
          .at("minimum_residual") == 0.0);
    for (const auto& checked : proposal.localHardQps) {
        const auto& retained = committed.hardProblems.at(checked.nodeId);
        CHECK(cbf2026::sameHardConstraintProblem(
            retained, checked.problem
        ));
        CHECK(cbf2026::canonicalHardConstraintProblemHash(retained)
              == checked.feasibility.hardProblemHash);
    }
}

TEST_CASE("fresh hard-only feasibility precheck leaves live Robot state unchanged") {
    json settings = robotSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 100.0;
    Robot robot(1, settings);
    robot.model->setControlInput(
        (Eigen::Vector3d() << 0.25, -0.5, 0.1).finished()
    );
    robot.opt = {{"marker", 17}};
    robot.certificateAvailable = false;
    robot.certificateSnapshotVersion = 71;
    robot.certificateAllocationVersion = 19;
    robot.localCertificateSnapshot = endpoint(1, 8, 0.0, 0.0);
    robot.previousUncertainty = 0.75;
    robot.currentUncertainty = 1.25;
    robot.uncertaintyRate = 1.0;
    robot.hasUncertaintyHistory = true;
    robot.myFormation = {{"marker", "formation"}};
    robot.myCovarianceFormation = {{"marker", "covariance"}};
    robot.comm->receivePosition2D(2, Point(91.0, 92.0));
    robot.comm->receivePositionCovariance(2, Eigen::Matrix2d::Identity());
    robot.comm->receiveUncertaintyRate(2, 1.25);
    robot.comm->receiveVelocity2D(
        2, (Eigen::Vector2d() << 2.0, -3.0).finished()
    );
    robot.comm->receiveYawRad(2, 0.25);
    robot.comm->receiveBatteryLevel(2, 4000.0);
    robot.comm->receiveEndpointCertificateSnapshot(
        2, endpoint(2, 8, 10.0, 0.0)
    );
    robot.committedCertificateState = {
        7,
        {{1, 7}, {2, 7}},
        true,
        {
            {1, endpoint(1, 7, 0.0, 0.0)},
            {2, endpoint(2, 7, 10.0, 0.0)}
        }
    };
    cbf2026::CommittedCertificateState state{
        8,
        {{1, 8}, {2, 8}},
        true,
        {
            {1, endpoint(1, 8, 0.0, 0.0)},
            {2, endpoint(2, 8, 10.0, 0.0)}
        },
        {}
    };
    const Eigen::VectorXd controlBefore = robot.model->getControlInput();
    const Eigen::VectorXd stateBefore = robot.model->getX();
    const json optBefore = robot.opt;
    const json optimiserBefore = robot.optimiser->getStatus();
    const json settingsBefore = robot.settings;
    const double runtimeBefore = robot.runtime;
    const Eigen::Matrix2d localCovarianceBefore = robot.positionCovariance;
    const double previousUncertaintyBefore = robot.previousUncertainty;
    const double uncertaintyRateBefore = robot.uncertaintyRate;
    const bool historyBefore = robot.hasUncertaintyHistory;
    const auto localEndpointBefore = robot.localCertificateSnapshot;
    const auto snapshotVersionBefore = robot.certificateSnapshotVersion;
    const auto allocationVersionBefore = robot.certificateAllocationVersion;
    const json formationBefore = robot.myFormation;
    const json covarianceFormationBefore = robot.myCovarianceFormation;
    const std::size_t hardCbfCountBefore = robot.cbfNoSlack.cbfs.size();
    const std::size_t softCbfCountBefore = robot.cbfSlack.size();
    const auto positionsBefore = robot.comm->_othersPos;
    const auto velocitiesBefore = robot.comm->_othersVel;
    const auto yawsBefore = robot.comm->_othersYawRad;
    const auto batteriesBefore = robot.comm->_othersBatteryLevel;
    const auto covariancesBefore =
        robot.comm->_othersPositionCovariance;
    const auto uncertaintyRatesBefore =
        robot.comm->_othersUncertaintyRate;
    const auto endpointsBefore =
        robot.comm->_othersEndpointCertificateSnapshots;
    const auto epsilonsBefore = robot.comm->_othersEpsilon;
    const auto barNusBefore = robot.comm->_othersBarNu;
    const auto covarianceRatesBefore =
        robot.comm->_othersCovarianceRateBound;
    const auto snapshotVersionsBefore =
        robot.comm->_othersCertificateSnapshotVersion;
    const auto allocationVersionsBefore =
        robot.comm->_othersAllocationVersion;
    const auto certificateBefore = robot.rateCertificate;
    const auto committedBefore = robot.committedCertificateState;
    const double currentUncertaintyBefore = robot.currentUncertainty;

    const auto problem = robot.buildHardConstraintProblem(state);
    const auto result = robot.checkLocalHardQpFeasibility(problem);
    auto infeasibleProblem = problem;
    auto infeasibleRow = problem.rows.front();
    infeasibleRow.name = "infeasible-purity-row";
    infeasibleRow.coefficients = Eigen::Vector3d(1.0, 0.0, 0.0);
    infeasibleRow.constant = -30.0;
    infeasibleProblem.rows = {infeasibleRow};
    const auto infeasibleResult =
        robot.checkLocalHardQpFeasibility(infeasibleProblem);

    CHECK(problem.owner == 1);
    CHECK(problem.rows.size() == 3);
    CHECK(problem.bounds.size() == 6);
    CHECK(result.feasible);
    CHECK(result.status == "optimal");
    CHECK(result.hardProblemHash
          == cbf2026::canonicalHardConstraintProblemHash(problem));
    CHECK_FALSE(infeasibleResult.feasible);
    CHECK(infeasibleResult.status == "infeasible");
    CHECK(robot.model->getControlInput() == controlBefore);
    CHECK(robot.model->getX() == stateBefore);
    CHECK(robot.opt == optBefore);
    CHECK(robot.optimiser->getStatus() == optimiserBefore);
    CHECK(robot.settings == settingsBefore);
    CHECK(robot.runtime == runtimeBefore);
    CHECK(robot.positionCovariance == localCovarianceBefore);
    CHECK(robot.previousUncertainty == previousUncertaintyBefore);
    CHECK(robot.uncertaintyRate == uncertaintyRateBefore);
    CHECK(robot.hasUncertaintyHistory == historyBefore);
    CHECK(robot.comm->_othersPos == positionsBefore);
    CHECK(robot.comm->_othersVel.at(2) == velocitiesBefore.at(2));
    CHECK(robot.comm->_othersYawRad == yawsBefore);
    CHECK(robot.comm->_othersBatteryLevel == batteriesBefore);
    CHECK(robot.comm->_othersPositionCovariance.at(2)
          == covariancesBefore.at(2));
    CHECK(robot.comm->_othersUncertaintyRate
          == uncertaintyRatesBefore);
    CHECK(cbf2026::sameEndpointCertificateSnapshot(
        robot.comm->_othersEndpointCertificateSnapshots.at(2),
        endpointsBefore.at(2)
    ));
    CHECK(robot.comm->_othersEpsilon == epsilonsBefore);
    CHECK(robot.comm->_othersBarNu == barNusBefore);
    CHECK(robot.comm->_othersCovarianceRateBound
          == covarianceRatesBefore);
    CHECK(robot.comm->_othersCertificateSnapshotVersion
          == snapshotVersionsBefore);
    CHECK(robot.comm->_othersAllocationVersion
          == allocationVersionsBefore);
    CHECK(robot.certificateAvailable == false);
    CHECK(robot.certificateSnapshotVersion == snapshotVersionBefore);
    CHECK(robot.certificateAllocationVersion == allocationVersionBefore);
    REQUIRE(robot.localCertificateSnapshot.has_value());
    REQUIRE(localEndpointBefore.has_value());
    CHECK(cbf2026::sameEndpointCertificateSnapshot(
        *robot.localCertificateSnapshot, *localEndpointBefore
    ));
    CHECK(cbf2026::sameNodeRateCertificateExceptVersion(
        robot.rateCertificate, certificateBefore
    ));
    CHECK(robot.rateCertificate.snapshotVersion
          == certificateBefore.snapshotVersion);
    CHECK(robot.committedCertificateState.version
          == committedBefore.version);
    CHECK(robot.committedCertificateState.valid == committedBefore.valid);
    CHECK(robot.committedCertificateState.nodeVersions
          == committedBefore.nodeVersions);
    CHECK(robot.committedCertificateState.endpoints.size()
          == committedBefore.endpoints.size());
    CHECK(cbf2026::sameEndpointCertificateSnapshot(
        robot.committedCertificateState.endpoints.at(1),
        committedBefore.endpoints.at(1)
    ));
    CHECK(cbf2026::sameEndpointCertificateSnapshot(
        robot.committedCertificateState.endpoints.at(2),
        committedBefore.endpoints.at(2)
    ));
    CHECK(robot.committedCertificateState.hardProblems.size()
          == committedBefore.hardProblems.size());
    CHECK(robot.committedCertificateState.certificates.size()
          == committedBefore.certificates.size());
    CHECK(robot.currentUncertainty == currentUncertaintyBefore);
    CHECK(robot.myFormation == formationBefore);
    CHECK(robot.myCovarianceFormation == covarianceFormationBefore);
    CHECK(robot.cbfNoSlack.cbfs.size() == hardCbfCountBefore);
    CHECK(robot.cbfSlack.size() == softCbfCountBefore);
}

TEST_CASE("theorem hard rows exclude live commands and descriptive rate history") {
    json settings = robotSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 100.0;
    Robot robot(1, settings);
    cbf2026::CommittedCertificateState state{
        8,
        {{1, 8}, {2, 8}},
        true,
        {
            {1, endpoint(1, 8, 0.0, 0.0)},
            {2, endpoint(2, 8, 10.0, 0.0)}
        }
    };
    const auto before = robot.buildHardConstraintProblem(state);
    robot.model->setControlInput(
        (Eigen::Vector3d() << 24.0, -23.0, 0.3).finished()
    );
    robot.comm->receiveVelocity2D(
        2, (Eigen::Vector2d() << 99.0, -87.0).finished()
    );
    robot.comm->receiveUncertaintyRate(2, 12345.0);
    robot.uncertaintyRate = 54321.0;

    const auto after = robot.buildHardConstraintProblem(state);

    CHECK(cbf2026::sameHardConstraintProblem(before, after));
    CHECK(cbf2026::canonicalHardConstraintProblemHash(before)
          == cbf2026::canonicalHardConstraintProblemHash(after));
}

TEST_CASE("normal optimise consumes the exact hard problem checked by the guard") {
    json settings = robotSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 100.0;
    Robot robot(1, settings);
    cbf2026::CommittedCertificateState state{
        8,
        {{1, 8}, {2, 8}},
        true,
        {
            {1, endpoint(1, 8, 0.0, 0.0)},
            {2, endpoint(2, 8, 10.0, 0.0)}
        },
        {}
    };
    const auto checked = robot.buildHardConstraintProblem(state);
    state.hardProblems.emplace(1, checked);
    robot.committedCertificateState = state;

    robot.optimise();

    REQUIRE(robot.lastConsumedHardConstraintProblem.has_value());
    CHECK(cbf2026::sameHardConstraintProblem(
        *robot.lastConsumedHardConstraintProblem, checked
    ));
    CHECK(robot.lastConsumedHardProblemHash
          == cbf2026::canonicalHardConstraintProblemHash(checked));
    REQUIRE(robot.opt.at("cbfNoSlack").size() == checked.rows.size());
    for (std::size_t index = 0; index < checked.rows.size(); ++index) {
        CHECK(robot.opt.at("cbfNoSlack").at(index).at("name")
              == checked.rows[index].name);
    }
}

TEST_CASE("theorem charging frame still solves the committed hard problem") {
    json settings = robotSettings();
    settings["world"]["charge"] = json::array({{
        {"position", json::array({0.0, 0.0})},
        {"radius", 5.0},
        {"charge-rate", 1.0}
    }});
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 100.0;
    Robot robot(1, settings);
    cbf2026::CommittedCertificateState state{
        8,
        {{1, 8}, {2, 8}},
        true,
        {
            {1, endpoint(1, 8, 0.0, 0.0)},
            {2, endpoint(2, 8, 10.0, 0.0)}
        },
        {}
    };
    const auto checked = robot.buildHardConstraintProblem(state);
    state.hardProblems.emplace(1, checked);
    robot.committedCertificateState = state;

    robot.optimise();

    CHECK(robot.opt.value("status", "missing") == "success");
    CHECK(robot.opt.at("cbfNoSlack").size() == checked.rows.size());
    CHECK(robot.lastConsumedHardProblemHash
          == cbf2026::canonicalHardConstraintProblemHash(checked));
}

TEST_CASE("fixed FIM ablation excludes optional dynamic references") {
    json dynamicSettings = robotSettings();
    dynamicSettings["bases"].push_back(json::array({10.0, 10.0}));
    dynamicSettings["formation"]["bases-id"] =
        json::array({json::array({0, 1, 2})});
    dynamicSettings["position_covariance"]["reference-selection"] =
        "dynamic-lower-index";
    json fixedSettings = dynamicSettings;
    fixedSettings["position_covariance"]["reference-selection"] =
        "fixed-cbf-only";
    Robot dynamicRobot(1, dynamicSettings);
    Robot fixedRobot(1, fixedSettings);

    const auto dynamicCertificate = dynamicRobot.proposeRateCertificate(
        8, 1, {}
    );
    const auto fixedCertificate = fixedRobot.proposeRateCertificate(
        8, 1, {}
    );

    CHECK(dynamicCertificate.frozenReferences.size() == 3);
    CHECK(fixedCertificate.frozenReferences.size() == 2);
    CHECK(dynamicCertificate.snapshotVersion == 8);
    CHECK(fixedCertificate.snapshotVersion == 8);
}

TEST_CASE("qualified materialized guard accepts only matched v3 policy identities") {
    json primary = materializedDiagnosticConfig(
        "config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json"
    );
    json ablation = materializedDiagnosticConfig(
        "config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v3.json"
    );
    CHECK(cbf2026::validateQualifiedMaterializedConfig(primary, {"Gurobi"}));
    CHECK(cbf2026::validateQualifiedMaterializedConfig(ablation, {"Gurobi"}));

    json crossVersion = primary;
    crossVersion["qualified-controller"]["schema-version"] = "hard-interior-v2";
    CHECK_FALSE(cbf2026::validateQualifiedMaterializedConfig(
        crossVersion, {"Gurobi"}
    ));
    crossVersion = primary;
    crossVersion["cbfs"]["hard-interior-selection"]["mode"] =
        "planar-chebyshev-fraction-cap-v1";
    CHECK_FALSE(cbf2026::validateQualifiedMaterializedConfig(
        crossVersion, {"Gurobi"}
    ));
    crossVersion = primary;
    crossVersion["cbfs"]["hard-interior-selection"]["fraction"] = 0.13;
    CHECK_FALSE(cbf2026::validateQualifiedMaterializedConfig(
        crossVersion, {"Gurobi"}
    ));
}

TEST_CASE("reset lifecycle coalesces causes and admits one transaction per frame") {
    cbf2026::ResetTransaction first;
    first.predecessorVersion = 7;
    first.proposedVersion = 8;
    first.status = cbf2026::GuardStatus::Accepted;
    first.simulationTime = 2.0;
    first.frameIndex = 4;
    first.causes = cbf2026::coalescedResetCauses(
        cbf2026::ResetCause::ActiveReferenceChange,
        {
            cbf2026::ResetCause::CertificateDiscontinuity,
            cbf2026::ResetCause::ActiveReferenceChange
        }
    );
    cbf2026::ResetTransaction second = first;
    second.predecessorVersion = 8;
    second.proposedVersion = 9;
    second.simulationTime = 2.5;
    second.frameIndex = 5;

    const auto decision = cbf2026::validateResetHistory({first, second});
    const auto serialized = cbf2026::serializeResetTransaction(first);

    CHECK(decision.status == cbf2026::GuardStatus::Accepted);
    CHECK(first.causes.size() == 2);
    CHECK(serialized.at("simulation_time") == 2.0);
    CHECK(serialized.at("frame_index") == 4);
    CHECK(serialized.at("predecessor_version") == 7);
    CHECK(serialized.at("proposed_version") == 8);
    CHECK(serialized.at("causes").size() == 2);
}

TEST_CASE("reset lifecycle rejects same-frame retry and repeated accepted time") {
    cbf2026::ResetTransaction first;
    first.predecessorVersion = 7;
    first.proposedVersion = 8;
    first.status = cbf2026::GuardStatus::Rejected;
    first.simulationTime = 2.0;
    first.frameIndex = 4;
    first.causes = {cbf2026::ResetCause::CertificateDiscontinuity};
    cbf2026::ResetTransaction retry = first;
    CHECK(cbf2026::validateResetHistory({first, retry}).status
          == cbf2026::GuardStatus::Rejected);

    first.status = cbf2026::GuardStatus::Accepted;
    retry.status = cbf2026::GuardStatus::Accepted;
    retry.predecessorVersion = 8;
    retry.proposedVersion = 9;
    retry.frameIndex = 5;
    CHECK(cbf2026::validateResetHistory({first, retry}).status
          == cbf2026::GuardStatus::Rejected);

    retry = first;
    retry.frameIndex = 5;
    retry.simulationTime = 2.25;
    CHECK(cbf2026::validateResetHistory({retry}).status
          == cbf2026::GuardStatus::Rejected);
}
