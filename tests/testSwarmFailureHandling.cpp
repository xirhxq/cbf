#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif
#include "doctest/doctest.h"

#include "Swarm.hpp"

#include <exception>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

json loadJson(const std::string& path) {
    std::ifstream stream(path);
    REQUIRE(stream.good());
    return json::parse(stream);
}

void mergeOverlay(json& materialized, const json& overlay) {
    for (const auto& [key, value] : overlay.items()) {
        if (value.is_object()
            && materialized.contains(key)
            && materialized.at(key).is_object()) {
            mergeOverlay(materialized[key], value);
        } else {
            materialized[key] = value;
        }
    }
}

json qualifiedSettings() {
    json settings = loadJson("config/config.json");
    mergeOverlay(
        settings,
        loadJson(
            "config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json"
        )
    );
    settings["initial"]["position"]["method"] = "specified";
    settings["initial"]["position"]["positions"] = json::array({
        json::array({-1450.0, -300.0}),
        json::array({-1250.0, -300.0}),
        json::array({-1050.0, -300.0}),
        json::array({-850.0, -300.0}),
        json::array({-1450.0, -100.0}),
        json::array({-1250.0, -100.0}),
        json::array({-1050.0, -100.0}),
        json::array({-850.0, -100.0}),
        json::array({-1450.0, 100.0}),
        json::array({-1250.0, 100.0}),
        json::array({-1050.0, 100.0}),
        json::array({-850.0, 100.0}),
        json::array({-1450.0, 300.0}),
        json::array({-1250.0, 300.0})
    });
    settings["cbfs"]["without-slack"]["comm-fixed"]["alpha"]["coe"] = 10.0;
    settings["cbfs"]["without-slack"]["safety"]["alpha"]["coe"] = 10.0;
    return settings;
}

}

TEST_CASE("FailureRecoveryPreservesOriginalWhenCleanupThrows") {
    const std::exception_ptr original =
        std::make_exception_ptr(std::runtime_error("original loop failure"));
    int updateAttempts = 0;
    int logAttempts = 0;
    std::ostringstream warnings;

    const std::exception_ptr recovered = recoverFailedIteration(
        original,
        false,
        [&]() {
            ++updateAttempts;
            throw std::runtime_error("cleanup update failed");
        },
        [&]() {
            ++logAttempts;
        },
        warnings
    );

    CHECK(updateAttempts == 1);
    CHECK(logAttempts == 1);
    CHECK(warnings.str().find("cleanup update failed") != std::string::npos);
    try {
        std::rethrow_exception(recovered);
        FAIL("the original exception must be rethrown");
    } catch (const std::runtime_error& error) {
        CHECK(std::string(error.what()) == "original loop failure");
    }
}

TEST_CASE("FailureRecoveryDoesNotRepeatAnAttemptedFrameLog") {
    const std::exception_ptr original =
        std::make_exception_ptr(std::runtime_error("failure after log"));
    int logAttempts = 0;
    std::ostringstream warnings;

    const std::exception_ptr recovered = recoverFailedIteration(
        original,
        true,
        []() {},
        [&]() {
            ++logAttempts;
        },
        warnings
    );

    CHECK(recovered == original);
    CHECK(logAttempts == 0);
    CHECK(warnings.str().empty());
}

TEST_CASE("SimulationErrorGateHandlesNonStandardExceptions") {
    int attempts = 0;
    std::ostringstream errors;

    const int returnCode = runSimulationWithErrorGate(
        [&]() {
            ++attempts;
            throw 7;
        },
        errors
    );

    CHECK(attempts == 1);
    CHECK(returnCode == 1);
    CHECK(errors.str() == "[SIMULATION_ERROR] Unknown error\n");
}

TEST_CASE("qualified reset commits one complete same-version global hard state") {
    json settings = qualifiedSettings();
    Swarm swarm(settings);

    const auto transaction = swarm.refreshTheoremCertificateFrame(
        0,
        swarm.all_ids,
        {cbf2026::ResetCause::CertificateDiscontinuity}
    );

    REQUIRE(transaction.status == cbf2026::GuardStatus::Accepted);
    CHECK(transaction.predecessorVersion == 0);
    CHECK(transaction.proposedVersion == 1);
    CHECK(transaction.frameIndex == 0);
    CHECK(transaction.simulationTime == 0.0);
    CHECK(swarm.committedCertificateState.valid);
    CHECK(swarm.committedCertificateState.endpoints.size() == 14);
    CHECK(swarm.committedCertificateState.hardProblems.size() == 14);
    CHECK(swarm.certificateResetHistory.size() == 1);
    CHECK(swarm.certificateUnavailableReason.has_value() == false);

    std::vector<std::size_t> rowCounts;
    std::size_t totalRows = 0;
    std::size_t localizationRows = 0;
    std::size_t collisionRows = 0;
    std::set<cbf2026::EdgeId> localizationEdges;
    std::set<cbf2026::EdgeId> collisionEdges;
    for (int robotId : swarm.all_ids) {
        const auto& problem =
            swarm.committedCertificateState.hardProblems.at(robotId);
        rowCounts.push_back(problem.rows.size());
        totalRows += problem.rows.size();
        for (const auto& row : problem.rows) {
            if (row.edge.kind == cbf2026::EdgeKind::Localization) {
                ++localizationRows;
                localizationEdges.insert(row.edge);
            } else {
                ++collisionRows;
                collisionEdges.insert(row.edge);
            }
        }
        CHECK(problem.snapshotVersion == 1);
        CHECK(problem.bounds.size() == 6);
        const auto& robot = swarm.robots.at(robotId - 1);
        CHECK(robot->certificateAvailable);
        CHECK(robot->certificateSnapshotVersion == 1);
        CHECK(robot->committedCertificateState.version == 1);
        CHECK(robot->comm->_othersEndpointCertificateSnapshots.size()
              == 14);
        CHECK(robot->uncertaintyRate == 0.0);
        CHECK(robot->comm->_othersUncertaintyRate.at(robotId) == 0.0);
        CHECK(robot->comm->_othersBarNu.at(robotId)
              == swarm.committedCertificateState.endpoints.at(robotId).barNu);
    }
    CHECK(rowCounts == std::vector<std::size_t>({
        17, 17, 17, 17, 17, 16, 15,
        17, 17, 17, 17, 17, 16, 15
    }));
    CHECK(totalRows == 232);
    CHECK(localizationEdges.size() == 28);
    CHECK(collisionEdges.size() == 91);
    CHECK(localizationRows == 50);
    CHECK(collisionRows == 182);
}

TEST_CASE("qualified continuous flow keeps the version and creates no reset record") {
    json settings = qualifiedSettings();
    Swarm swarm(settings);
    swarm.refreshTheoremCertificateFrame(
        0,
        swarm.all_ids,
        {cbf2026::ResetCause::CertificateDiscontinuity}
    );

    swarm.refreshTheoremFlowFrame(1);

    CHECK(swarm.committedCertificateState.version == 1);
    CHECK(swarm.certificateResetHistory.size() == 1);
    CHECK(swarm.certificateResetHistory.front().frameIndex == 0);
    CHECK(cbf2026::validateResetHistory(
        swarm.certificateResetHistory
    ).status == cbf2026::GuardStatus::Accepted);
}

TEST_CASE("rejected bootstrap leaves no live predecessor certificate") {
    json settings = qualifiedSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["alpha"]["coe"] = 0.1;
    settings["cbfs"]["without-slack"]["safety"]["alpha"]["coe"] = 0.1;
    Swarm swarm(settings);

    CHECK_THROWS_AS(
        swarm.refreshTheoremCertificateFrame(
            0,
            swarm.all_ids,
            {cbf2026::ResetCause::CertificateDiscontinuity}
        ),
        std::runtime_error
    );

    CHECK_FALSE(swarm.committedCertificateState.valid);
    CHECK(swarm.committedCertificateState.version == 0);
    REQUIRE(swarm.certificateUnavailableReason.has_value());
    REQUIRE(swarm.certificateResetHistory.size() == 1);
    CHECK(swarm.certificateResetHistory.front().status
          == cbf2026::GuardStatus::Rejected);
    for (const auto& robot : swarm.robots) {
        CHECK_FALSE(robot->certificateAvailable);
        CHECK_FALSE(robot->committedCertificateState.valid);
    }
}

TEST_CASE("early certificate construction failure records one rejected envelope") {
    json settings = qualifiedSettings();
    Swarm swarm(settings);
    swarm.robots.front()->model->setPosition2D(Point(-1550.0, 0.0));

    CHECK_THROWS(
        swarm.refreshTheoremCertificateFrame(
            0,
            swarm.all_ids,
            {cbf2026::ResetCause::CertificateDiscontinuity}
        )
    );

    CHECK_FALSE(swarm.committedCertificateState.valid);
    REQUIRE(swarm.certificateResetHistory.size() == 1);
    const auto& rejected = swarm.certificateResetHistory.front();
    CHECK(rejected.status == cbf2026::GuardStatus::Rejected);
    CHECK(rejected.frameIndex == 0);
    CHECK(rejected.simulationTime == 0.0);
    CHECK(rejected.predecessorVersion == 0);
    CHECK(rejected.proposedVersion == 1);
    CHECK(rejected.causes == std::vector<cbf2026::ResetCause>({
        cbf2026::ResetCause::CertificateDiscontinuity
    }));
    CHECK(cbf2026::serializeResetTransaction(rejected).at("status")
          == "rejected");
}

TEST_CASE("partial reset uses committed certificates and current unaffected estimates") {
    json settings = qualifiedSettings();
    Swarm swarm(settings);
    swarm.refreshTheoremCertificateFrame(
        0,
        swarm.all_ids,
        {cbf2026::ResetCause::CertificateDiscontinuity}
    );
    const auto previousEndpoint =
        swarm.committedCertificateState.endpoints.at(14);
    const auto previousCertificate =
        swarm.committedCertificateState.certificates.at(14);
    const Point moved(
        previousEndpoint.estimate.x() + 1.0,
        previousEndpoint.estimate.y()
    );
    swarm.robots.at(13)->model->setPosition2D(moved);
    swarm.robots.at(13)->rateCertificate.epsilon = 999.0;
    std::map<int, std::vector<int>> frozenReferenceIds;
    for (int robotId : swarm.all_ids) {
        frozenReferenceIds.emplace(
            robotId,
            cbf2026::canonicalFrozenReferenceIds(
                swarm.committedCertificateState.certificates.at(robotId)
            )
        );
    }
    std::map<int, cbf2026::EndpointCertificateSnapshot>
        expectedCurrentPreReset;
    for (int robotId : swarm.all_ids) {
        const auto certificate = swarm.robots.at(robotId - 1)
            ->proposeRateCertificate(
                1,
                1,
                expectedCurrentPreReset,
                frozenReferenceIds.at(robotId)
            );
        const Point position = swarm.robots.at(robotId - 1)->model->xy();
        expectedCurrentPreReset.emplace(
            robotId,
            cbf2026::makeEndpointCertificateSnapshot(
                certificate,
                Eigen::Vector2d(position.x, position.y),
                1
            )
        );
    }
    for (auto& robot : swarm.robots) {
        robot->runtime = 0.5;
    }

    const auto transaction = swarm.refreshTheoremCertificateFrame(
        1,
        {1},
        {cbf2026::ResetCause::EstimateInterfaceReset}
    );

    REQUIRE(transaction.status == cbf2026::GuardStatus::Accepted);
    CHECK(swarm.committedCertificateState.version == 2);
    const auto& endpoint =
        swarm.committedCertificateState.endpoints.at(14);
    const auto& certificate =
        swarm.committedCertificateState.certificates.at(14);
    CHECK(endpoint.estimate == Eigen::Vector2d(moved.x, moved.y));
    CHECK(endpoint.covariance
          == expectedCurrentPreReset.at(14).covariance);
    CHECK(endpoint.epsilon == expectedCurrentPreReset.at(14).epsilon);
    CHECK(certificate.epsilon == expectedCurrentPreReset.at(14).epsilon);
    CHECK(certificate.epsilon != previousCertificate.epsilon);
    CHECK(certificate.epsilon != 999.0);
    CHECK(certificate.snapshotVersion == 2);
    CHECK(swarm.certificateResetHistory.size() == 2);
}

TEST_CASE("same-instant reset brackets a motion-triggered base identity change") {
    json settings = qualifiedSettings();
    settings["initial"]["position"]["positions"][0] =
        json::array({-1450.0, -543.0});
    Swarm swarm(settings);
    swarm.refreshTheoremCertificateFrame(
        0,
        swarm.all_ids,
        {cbf2026::ResetCause::CertificateDiscontinuity}
    );
    const auto dagBefore = swarm.committedActiveDag;
    const auto staleAffected =
        swarm.committedCertificateState.endpoints.at(1);
    std::map<int, std::vector<int>> frozenReferenceIds;
    for (int robotId : swarm.all_ids) {
        frozenReferenceIds.emplace(
            robotId,
            cbf2026::canonicalFrozenReferenceIds(
                swarm.committedCertificateState.certificates.at(robotId)
            )
        );
    }
    CHECK(std::find(
        frozenReferenceIds.at(1).begin(),
        frozenReferenceIds.at(1).end(),
        cbf2026::canonicalBaseReferenceId(2)
    ) != frozenReferenceIds.at(1).end());
    const Point movedAffected(
        staleAffected.estimate.x(),
        staleAffected.estimate.y() - 3.0
    );
    const auto staleUnaffected =
        swarm.committedCertificateState.endpoints.at(14);
    const Point movedUnaffected(
        staleUnaffected.estimate.x() + 1.0,
        staleUnaffected.estimate.y()
    );
    swarm.robots.at(0)->model->setPosition2D(movedAffected);
    swarm.robots.at(13)->model->setPosition2D(movedUnaffected);

    std::map<int, cbf2026::EndpointCertificateSnapshot>
        expectedCurrentPreReset;
    for (int robotId : swarm.all_ids) {
        const auto certificate = swarm.robots.at(robotId - 1)
            ->proposeRateCertificate(
                1,
                1,
                expectedCurrentPreReset,
                frozenReferenceIds.at(robotId)
            );
        const Point position = swarm.robots.at(robotId - 1)->model->xy();
        expectedCurrentPreReset.emplace(
            robotId,
            cbf2026::makeEndpointCertificateSnapshot(
                certificate,
                Eigen::Vector2d(position.x, position.y),
                1
            )
        );
    }
    for (auto& robot : swarm.robots) {
        robot->runtime = 0.5;
    }

    swarm.refreshTheoremFlowFrame(1);

    CHECK(swarm.committedCertificateState.version == 2);
    CHECK(swarm.certificateResetHistory.size() == 2);
    const auto& reset = swarm.certificateResetHistory.back();
    CHECK(reset.frameIndex == 1);
    CHECK(reset.causes
          == std::vector<cbf2026::ResetCause>({
              cbf2026::ResetCause::ActiveReferenceChange
          }));
    CHECK(swarm.committedActiveDag == dagBefore);
    CHECK(cbf2026::canonicalFrozenReferenceIds(
        swarm.committedCertificateState.certificates.at(1)
    ) != frozenReferenceIds.at(1));

    const auto node = std::find_if(
        reset.nodes.begin(),
        reset.nodes.end(),
        [](const cbf2026::NodeResetRecord& record) {
            return record.nodeId == 1;
        }
    );
    REQUIRE(node != reset.nodes.end());
    CHECK(node->deltaEstimate.isZero(0.0));

    const auto baseEdge = std::find_if(
        reset.hardEdges.begin(),
        reset.hardEdges.end(),
        [](const cbf2026::HardEdgeResetRecord& record) {
            return record.edge.kind == cbf2026::EdgeKind::Localization
                   && record.edge.low == 1
                   && record.edge.baseId >= 0;
        }
    );
    REQUIRE(baseEdge != reset.hardEdges.end());
    const auto& expectedPre = expectedCurrentPreReset.at(1);
    const double expectedPreBarrier = baseEdge->threshold
        - (expectedPre.estimate - baseEdge->basePosition).norm()
        - expectedPre.epsilon;
    const double stalePriorFrameBarrier = baseEdge->threshold
        - (staleAffected.estimate - baseEdge->basePosition).norm()
        - staleAffected.epsilon;
    CHECK(baseEdge->preBarrier == expectedPreBarrier);
    CHECK(baseEdge->preBarrier != stalePriorFrameBarrier);
    CHECK(swarm.committedCertificateState.endpoints.at(14).estimate
          == Eigen::Vector2d(movedUnaffected.x, movedUnaffected.y));
}
