#ifndef CBF_SWARM_HPP
#define CBF_SWARM_HPP

#include "Robot.hpp"

#include <exception>
#include <utility>

template<typename UpdateAction, typename LogAction>
std::exception_ptr recoverFailedIteration(
    std::exception_ptr originalFailure,
    bool frameLogAttempted,
    UpdateAction&& updateAction,
    LogAction&& logAction,
    std::ostream& warnings = std::cerr
) noexcept {
    auto runBestEffort = [&](const char* operation, auto&& action) {
        try {
            action();
        } catch (const std::exception& error) {
            try {
                warnings
                    << "[Swarm::run] Warning: failure recovery "
                    << operation << " failed: " << error.what() << std::endl;
            } catch (...) {
            }
        } catch (...) {
            try {
                warnings
                    << "[Swarm::run] Warning: failure recovery "
                    << operation << " failed: Unknown error" << std::endl;
            } catch (...) {
            }
        }
    };

    runBestEffort("updateGridWorld", std::forward<UpdateAction>(updateAction));
    if (!frameLogAttempted) {
        runBestEffort("logOnce", std::forward<LogAction>(logAction));
    }
    return originalFailure;
}

template<typename SimulationAction>
int runSimulationWithErrorGate(
    SimulationAction&& simulationAction,
    std::ostream& errors = std::cerr
) {
    try {
        simulationAction();
        return 0;
    } catch (const std::exception& error) {
        errors << "[SIMULATION_ERROR] " << error.what() << '\n';
        return 1;
    } catch (...) {
        errors << "[SIMULATION_ERROR] Unknown error\n";
        return 1;
    }
}

class Swarm {
public:
    int n;
    std::vector<std::unique_ptr<Robot>> robots;
    std::ofstream ofstream;
    json data;
    json stepData;
    json config;
    GridWorld gridWorldGroundTruth;
    json updatedGridWorldGroundTruth;
    std::string folderName;
    std::string filename;
    std::string customOutputPath;
    std::string outputDir;
    std::vector<int> all_ids;

    std::unique_ptr<CentralizedModel> centralizedModel;
    std::unique_ptr<OptimiserBase> optimizer;

    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    json opt;
    cbf2026::CommittedCertificateState committedCertificateState;
    cbf2026::ActiveDag committedActiveDag;
    std::vector<cbf2026::ResetTransaction> certificateResetHistory;
    std::optional<std::string> certificateUnavailableReason;
    std::set<std::uint64_t> attemptedCertificateResetFrames;
public:
    Swarm(json &settings)
            : config(settings),
              n(settings["num"]),
              gridWorldGroundTruth(settings["world"]){
        if (settings.contains("qualified-estimator")
            && !cbf2026::validateQualifiedMaterializedConfig(
                settings, getAvailableOptimisers()
            )) {
            throw std::invalid_argument(
                "qualified hybrid distributed CBF configuration is invalid"
            );
        }
        if (settings.contains("output_path")) {
            customOutputPath = settings["output_path"];
        } else {
            customOutputPath = "";
        }

        std::generate_n(std::back_inserter(all_ids), n, [i = 1]() mutable {
            return i++;
        });
        settings["all"] = all_ids;
        for (int i = 0; i < n; i++) {
            auto robot = std::make_unique<Robot>(i + 1, settings);
            robots.push_back(std::move(robot));
        }

        initializeCentralizedOptimization();
    }

    void output() {
        printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, robots[0]->runtime);
        for (auto &robot: robots) {
            robot->output();
        }
        printf("--------------\n");
    }

    void endLog() {
        ofstream << std::fixed << std::setprecision(6) << data;
        ofstream.close();
    }

    void logParams() {
        data["para"] = robots[0]->getParams();
        data["config"] = config;
    }

    void logOnce() {
        stepData["runtime"] = robots[0]->runtime;
        {
            json robotsJson = json::array();
            for (auto &robot: robots) {
                robotsJson.emplace_back(robot->getState());
            }
            stepData["robots"] = robotsJson;
        }
        stepData["formation"] = json::array();
        stepData["covariance_formation"] = json::array();
        for (auto &robot: robots) {
            stepData["formation"].push_back(robot->myFormation);
            stepData["covariance_formation"].push_back(robot->myCovarianceFormation);
        }
        stepData["update"] = updatedGridWorldGroundTruth;

        if (isCentralizedExecution()) {
            json centralizedData = opt;

            json cbfValues = json::object();

            for (const auto& [name, cbf] : cbfNoSlack.cbfs) {
                auto x = centralizedModel->getX();
                double value = cbf.h(x, robots[0]->runtime);
                cbfValues[name] = value;
            }

            for (const auto& [name, cbf] : cbfSlack) {
                auto x = centralizedModel->getX();
                double value = cbf.h(x, robots[0]->runtime);
                cbfValues[name] = value;
            }

            centralizedData["cbfs"] = cbfValues;
            stepData["centralized"] = centralizedData;
        }

        data["state"].push_back(stepData);
        stepData.clear();
    }

    void initLog() {
        std::string dataDir;
        std::string runSuffix;

        if (!customOutputPath.empty()) {
            dataDir = customOutputPath;
        } else {
            dataDir = PROJECT_ROOT "/data";
        }

        if (config.contains("run_suffix")) {
            runSuffix = config["run_suffix"].get<std::string>();
        }

        if (mkdir(dataDir.c_str(), 0777) == -1 && errno != EEXIST) {
            std::cerr << "[Swarm::initLog] Warning: Could not create data directory: " << strerror(errno) << std::endl;
        }

        time_t now = time(nullptr);
        tm *t = localtime(&now);
        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(4) << t->tm_year + 1900 << "-"
            << std::setw(2) << t->tm_mon + 1 << "-"
            << std::setw(2) << t->tm_mday << "_"
            << std::setw(2) << t->tm_hour << "-"
            << std::setw(2) << t->tm_min << "-"
            << std::setw(2) << t->tm_sec;
        folderName = oss.str();

        std::string outputDirName = folderName + runSuffix;
        outputDir = dataDir + "/" + outputDirName;

        if (mkdir(outputDir.c_str(), 0777) == -1) {
            std::cerr << "Error :  " << strerror(errno) << std::endl;
        }
        filename = outputDir + "/data.json";
        ofstream.open(filename, std::ios::app);
    }

    void exchangeData() {
        for (const auto &robot: robots) {
            Point pos2d = robot->model->xy();
            auto vel2d = robot->model->getVelocity();
            double yawRad = robot->model->getStateVariable("yawRad");
            double batteryLevel = robot->model->getStateVariable("battery");
            Eigen::Matrix2d positionCovariance = robot->positionCovariance;
            double uncertaintyRate = robot->uncertaintyRate;

            for (auto &otherRobot: robots) {
                otherRobot->comm->receivePosition2D(robot->id, pos2d);
                otherRobot->comm->receiveVelocity2D(robot->id, vel2d);
                otherRobot->comm->receiveYawRad(robot->id, yawRad);
                otherRobot->comm->receiveBatteryLevel(robot->id, batteryLevel);
                otherRobot->comm->receivePositionCovariance(robot->id, positionCovariance);
                otherRobot->comm->receiveUncertaintyRate(robot->id, uncertaintyRate);
            }
        };
    }

    // External data injection interface (for simulation environment)
    void injectExternalVelocities(const std::vector<std::pair<double, double>>& velocities) {
        for (size_t i = 0; i < robots.size() && i < velocities.size(); ++i) {
            robots[i]->setExternalVelocity(velocities[i].first, velocities[i].second);
        }
    }

    void checkInformationExchange() {
        for (auto &robot: robots) {
            for (auto &other: robots) {
                assert(robot->model->xy().distance_to(other->comm->_othersPos[robot->id]) < 1e-3);
            }
        }
    }

    cbf2026::ResetTransaction refreshTheoremCertificateFrame(
        std::uint64_t frameIndex,
        const std::vector<int>& changedNodes,
        const std::vector<cbf2026::ResetCause>& causes
    ) {
        if (!usesTheoremAlignedCertificates()) {
            throw std::invalid_argument(
                "theorem certificate refresh is not enabled"
            );
        }
        if (causes.empty()) {
            throw std::invalid_argument("reset cause set must not be empty");
        }
        if (certificateUnavailableReason.has_value()) {
            throw std::runtime_error(
                "theorem certificate segment is unavailable: "
                + *certificateUnavailableReason
            );
        }
        if (!attemptedCertificateResetFrames.insert(frameIndex).second) {
            certificateUnavailableReason =
                "theorem certificate reset retried within one frame";
            throw std::runtime_error(*certificateUnavailableReason);
        }
        cbf2026::ResetProposal proposal;
        proposal.simulationTime = robots.front()->runtime;
        proposal.cause = causes.front();
        proposal.predecessorVersion = committedCertificateState.valid
            ? committedCertificateState.version : 0;
        proposal.proposedVersion = proposal.predecessorVersion + 1U;
        proposal.changedNodes = changedNodes;
        proposal.frameIndex = frameIndex;
        proposal.causes.assign(causes.begin() + 1, causes.end());
        try {
        cbf2026::CommittedCertificateState predecessorState =
            committedCertificateState;
        if (!predecessorState.valid) {
            const auto bootstrap = buildTheoremCandidate(
                0, all_ids, nullptr
            );
            predecessorState.version = 0;
            predecessorState.valid = true;
            predecessorState.endpoints = bootstrap.endpoints;
            predecessorState.certificates = bootstrap.certificates;
            for (int robotId : all_ids) {
                predecessorState.nodeVersions.emplace(robotId, 0);
            }
        } else {
            std::map<int, std::vector<int>> frozenReferenceIds;
            for (int robotId : all_ids) {
                frozenReferenceIds.emplace(
                    robotId,
                    cbf2026::canonicalFrozenReferenceIds(
                        committedCertificateState.certificates.at(robotId)
                    )
                );
            }
            const auto currentPreReset = buildTheoremCandidate(
                predecessorState.version,
                all_ids,
                nullptr,
                &frozenReferenceIds
            );
            predecessorState.endpoints = currentPreReset.endpoints;
            predecessorState.certificates = currentPreReset.certificates;
            predecessorState.hardProblems.clear();
        }

        const std::uint64_t proposedVersion =
            predecessorState.version + 1U;
        const auto preview = buildTheoremCandidate(
            proposedVersion, all_ids, &predecessorState
        );
        const std::vector<int> closure = cbf2026::transitiveDescendants(
            changedNodes, preview.activeDag
        );
        const auto candidate = buildTheoremCandidate(
            proposedVersion, closure, &predecessorState
        );
        proposal.predecessorVersion = predecessorState.version;
        proposal.proposedVersion = proposedVersion;
        proposal.activeDag = candidate.activeDag;
        proposal.descendantClosure = closure;
        proposal.candidateEndpoints = candidate.endpoints;
        proposal.candidateCertificates = candidate.certificates;

        for (std::size_t index = 0; index < closure.size(); ++index) {
            const int nodeId = closure[index];
            const auto& predecessor =
                predecessorState.endpoints.at(nodeId);
            const auto& proposed = candidate.endpoints.at(nodeId);
            proposal.nodes.push_back({
                nodeId,
                static_cast<int>(index + 1),
                proposedVersion,
                proposed,
                proposed.estimate - predecessor.estimate,
                proposed.epsilon - predecessor.epsilon,
                cbf2026::canonicalFrozenReferenceIds(
                    predecessorState.certificates.at(nodeId)
                ),
                cbf2026::canonicalFrozenReferenceIds(
                    candidate.certificates.at(nodeId)
                )
            });
        }
        appendHardResetRecords(
            candidate.endpoints, predecessorState, proposal
        );

        cbf2026::CommittedCertificateState candidateState;
        candidateState.version = proposedVersion;
        candidateState.valid = true;
        candidateState.endpoints = candidate.endpoints;
        for (int robotId : all_ids) {
            candidateState.nodeVersions.emplace(robotId, proposedVersion);
        }
        for (int robotId : all_ids) {
            const auto problem = robots.at(robotId - 1)
                ->buildHardConstraintProblem(candidateState);
            proposal.requiredUavNodes.push_back(robotId);
            proposal.localHardQps.push_back({
                robotId,
                problem,
                {
                    false,
                    "unchecked",
                    -std::numeric_limits<double>::infinity(),
                    cbf2026::canonicalHardConstraintProblemHash(problem)
                }
            });
        }

        auto nextCommitted = predecessorState;
        const auto authority = theoremGuardAuthority();
        auto transaction = cbf2026::commitResetTransaction(
            proposal,
            nextCommitted,
            authority,
            [this](
                int robotId,
                const cbf2026::HardConstraintProblem& problem
            ) {
                return robots.at(robotId - 1)
                    ->checkLocalHardQpFeasibility(problem);
            }
        );
        auto prospectiveHistory = certificateResetHistory;
        prospectiveHistory.push_back(transaction);
        const auto lifecycle = cbf2026::validateResetHistory(
            prospectiveHistory
        );
        if (transaction.status != cbf2026::GuardStatus::Accepted
            || lifecycle.status != cbf2026::GuardStatus::Accepted) {
            if (transaction.status == cbf2026::GuardStatus::Accepted) {
                transaction.status = cbf2026::GuardStatus::Rejected;
                transaction.reason = lifecycle.reason;
            }
            certificateResetHistory.push_back(transaction);
            certificateUnavailableReason = transaction.reason;
            throw std::runtime_error(
                "theorem certificate reset rejected: "
                + transaction.reason
            );
        }

        installTheoremCandidate(
            candidate, nextCommitted, candidate.activeDag
        );
        certificateResetHistory.swap(prospectiveHistory);
        certificateUnavailableReason.reset();
        return transaction;
        } catch (const std::exception& error) {
            if (!certificateUnavailableReason.has_value()) {
                certificateUnavailableReason = error.what();
            }
            const bool recorded = std::any_of(
                certificateResetHistory.begin(),
                certificateResetHistory.end(),
                [frameIndex](const cbf2026::ResetTransaction& transaction) {
                    return transaction.frameIndex == frameIndex;
                }
            );
            if (!recorded) {
                cbf2026::ResetTransaction transaction;
                transaction.predecessorVersion = proposal.predecessorVersion;
                transaction.proposedVersion = proposal.proposedVersion;
                transaction.changedNodes = proposal.changedNodes;
                transaction.descendantClosure = proposal.descendantClosure;
                transaction.nodes = proposal.nodes;
                transaction.status = cbf2026::GuardStatus::Rejected;
                transaction.reason = *certificateUnavailableReason;
                transaction.simulationTime = proposal.simulationTime;
                transaction.frameIndex = proposal.frameIndex;
                transaction.causes = cbf2026::coalescedResetCauses(
                    proposal.cause, proposal.causes
                );
                transaction.hardEdges = proposal.hardEdges;
                transaction.localHardQps = proposal.localHardQps;
                certificateResetHistory.push_back(std::move(transaction));
            }
            throw;
        } catch (...) {
            if (!certificateUnavailableReason.has_value()) {
                certificateUnavailableReason =
                    "unknown theorem certificate reset failure";
            }
            const bool recorded = std::any_of(
                certificateResetHistory.begin(),
                certificateResetHistory.end(),
                [frameIndex](const cbf2026::ResetTransaction& transaction) {
                    return transaction.frameIndex == frameIndex;
                }
            );
            if (!recorded) {
                cbf2026::ResetTransaction transaction;
                transaction.predecessorVersion = proposal.predecessorVersion;
                transaction.proposedVersion = proposal.proposedVersion;
                transaction.changedNodes = proposal.changedNodes;
                transaction.descendantClosure = proposal.descendantClosure;
                transaction.nodes = proposal.nodes;
                transaction.status = cbf2026::GuardStatus::Rejected;
                transaction.reason = *certificateUnavailableReason;
                transaction.simulationTime = proposal.simulationTime;
                transaction.frameIndex = proposal.frameIndex;
                transaction.causes = cbf2026::coalescedResetCauses(
                    proposal.cause, proposal.causes
                );
                transaction.hardEdges = proposal.hardEdges;
                transaction.localHardQps = proposal.localHardQps;
                certificateResetHistory.push_back(std::move(transaction));
            }
            throw;
        }
    }

    void refreshTheoremFlowFrame(std::uint64_t frameIndex) {
        if (!usesTheoremAlignedCertificates()) {
            return;
        }
        if (certificateUnavailableReason.has_value()) {
            throw std::runtime_error(
                "theorem certificate segment is unavailable: "
                + *certificateUnavailableReason
            );
        }
        try {
        if (!committedCertificateState.valid) {
            refreshTheoremCertificateFrame(
                frameIndex,
                all_ids,
                {cbf2026::ResetCause::CertificateDiscontinuity}
            );
            return;
        }
        const auto candidate = buildTheoremCandidate(
            committedCertificateState.version, all_ids
        );
        std::vector<int> changed;
        for (int robotId : all_ids) {
            if (cbf2026::frozenReferenceSetChanged(
                committedCertificateState.certificates.at(robotId),
                candidate.certificates.at(robotId)
            )) {
                changed.push_back(robotId);
            }
        }
        if (changed.empty() && candidate.activeDag != committedActiveDag) {
            certificateUnavailableReason =
                "active DAG changed without an exact certificate reference change";
            throw std::runtime_error(*certificateUnavailableReason);
        }
        if (!changed.empty()) {
            refreshTheoremCertificateFrame(
                frameIndex,
                changed,
                {cbf2026::ResetCause::ActiveReferenceChange}
            );
            return;
        }

        cbf2026::CommittedCertificateState next;
        next.version = committedCertificateState.version;
        next.valid = true;
        next.endpoints = candidate.endpoints;
        next.certificates = candidate.certificates;
        for (int robotId : all_ids) {
            next.nodeVersions.emplace(robotId, next.version);
        }
        for (int robotId : all_ids) {
            auto problem = robots.at(robotId - 1)
                ->buildHardConstraintProblem(next);
            for (const auto& row : problem.rows) {
                if (!std::isfinite(row.postResetBarrier)
                    || row.postResetBarrier < 0.0) {
                    certificateUnavailableReason =
                        "continuous certificate flow produced a negative hard barrier";
                    throw std::runtime_error(*certificateUnavailableReason);
                }
            }
            const auto feasibility = robots.at(robotId - 1)
                ->checkLocalHardQpFeasibility(problem);
            if (!feasibility.feasible || feasibility.status != "optimal") {
                certificateUnavailableReason =
                    "continuous certificate flow produced an infeasible hard QP";
                throw std::runtime_error(*certificateUnavailableReason);
            }
            next.hardProblems.emplace(robotId, std::move(problem));
        }
        installTheoremCandidate(candidate, next, candidate.activeDag);
        certificateUnavailableReason.reset();
        } catch (const std::exception& error) {
            if (!certificateUnavailableReason.has_value()) {
                certificateUnavailableReason = error.what();
            }
            throw;
        } catch (...) {
            if (!certificateUnavailableReason.has_value()) {
                certificateUnavailableReason =
                    "unknown theorem certificate flow failure";
            }
            throw;
        }
    }

    void updateGridWorld() {
        updatedGridWorldGroundTruth = json::array();
        json searchSettings = config["searching"];
        std::string method = searchSettings["method"];
        json params = searchSettings[method];
        for (auto &robot: robots) {
            auto updatedFor1 = json::array();
            if (method == "front-sector") {
                params["centerAngleRad"] = robot->model->getStateVariable("yawRad");
                updatedFor1 = gridWorldGroundTruth.setValueInSectorRing(
                    robot->model->xy(),
                    params,
                    true, true
                );
            }
            else if (method == "front-cone") {
                params["yaw-rad"] = robot->model->getStateVariable("yawRad");
                updatedFor1 = gridWorldGroundTruth.setValueInTiltedCone(
                    robot->model->xy(),
                    params,
                    true, true
                );
            }
            else if (method == "downward") {
                updatedFor1 = gridWorldGroundTruth.setValueInCircle(
                    robot->model->xy(),
                    params, true, true
                );
            }
            updatedGridWorldGroundTruth.insert(
                updatedGridWorldGroundTruth.end(),
                updatedFor1.begin(), updatedFor1.end()
            );
        }
    }

    void checkUpdatedGridWorld() {
        for (auto &robot: robots) {
            if (updatedGridWorldGroundTruth.size() != robot->updatedGridWorld.size()) {
                throw std::runtime_error("updatedGridWorldGroundTruth size mismatch, robot id: " + std::to_string(robot->id));
            }
        }
    }

    void run() {
        auto settings = config["execute"];
        double tTotal = settings["time-total"], tStep = settings["time-step"];

        exchangeData();
        if (usesTheoremAlignedCertificates()) {
            refreshTheoremCertificateFrame(
                0,
                all_ids,
                {cbf2026::ResetCause::CertificateDiscontinuity}
            );
        } else {
            std::vector<Robot*> covarianceBootstrapOrder;
            covarianceBootstrapOrder.reserve(robots.size());
            for (auto &robot: robots) {
                covarianceBootstrapOrder.push_back(robot.get());
            }
            std::sort(
                covarianceBootstrapOrder.begin(),
                covarianceBootstrapOrder.end(),
                [](const Robot* lhs, const Robot* rhs) {
                    if (lhs->idInMyPart != rhs->idInMyPart) {
                        return lhs->idInMyPart < rhs->idInMyPart;
                    }
                    return lhs->partId < rhs->partId;
                }
            );
            for (Robot* robot: covarianceBootstrapOrder) {
                robot->updateCovarianceAndRate(tStep);
                for (auto &receiver: robots) {
                    receiver->comm->receivePositionCovariance(
                        robot->id, robot->positionCovariance
                    );
                    receiver->comm->receiveUncertaintyRate(
                        robot->id, robot->uncertaintyRate
                    );
                }
            }
        }
        checkInformationExchange();
        initLog();
        logParams();
        output();

        if (isCentralizedExecution()) {
            initializeCentralizedOptimization();
            presetCBFs();
        } else {
            for (auto &robot: robots) robot->presetCBF();
        }

        std::exception_ptr loopFailure;
        while (robots[0]->runtime < tTotal) {
            bool frameLogAttempted = false;
            try {
                if (robots[0]->runtime > 0.0) {
                    exchangeData();
                    if (usesTheoremAlignedCertificates()) {
                        const auto frameIndex = static_cast<std::uint64_t>(
                            std::llround(robots[0]->runtime / tStep)
                        );
                        refreshTheoremFlowFrame(frameIndex);
                    } else {
                        for (auto &robot: robots) {
                            robot->updateCovarianceAndRate(tStep);
                        }
                        exchangeData();
                    }
                }
                checkInformationExchange();
                for (auto &robot: robots) robot->checkRobotsInsideWorld();
                printf("\r%.2lf seconds elapsed... %.2lf%%", robots[0]->runtime, gridWorldGroundTruth.getPercentage() * 100);
                for (auto &robot: robots) robot->updateGridWorld();
                updateGridWorld();
                checkUpdatedGridWorld();
                for (auto &robot: robots) robot->postsetCBF();

                if (isCentralizedExecution()) {
                    postsetCBFs();
                    centralizedOptimise();
                } else {
                    for (auto &robot: robots) robot->optimise();
                }

                // if (checkConstraintViolation()) {
                //     logOnce();
                //     std::cout << "\n[Simulation Terminated] Constraint violation detected at t=" << robots[0]->runtime << "s" << std::endl;
                //     break;
                // }
                if (settings.value("check-constraint-violation", false)) {
                    if (checkConstraintViolation()) {
                        frameLogAttempted = true;
                        logOnce();
                        std::cout << "\n[Simulation Terminated] Constraint violation detected at t=" << robots[0]->runtime << "s" << std::endl;
                        break;
                    }
                }

                frameLogAttempted = true;
                logOnce();
                for (auto &robot: robots) robot->stepTimeForward(tStep);
            }
            catch (...) {
                loopFailure = recoverFailedIteration(
                    std::current_exception(),
                    frameLogAttempted,
                    [&]() {
                        for (auto &robot: robots) {
                            robot->updateGridWorld();
                        }
                    },
                    [&]() {
                        logOnce();
                    }
                );
                break;
            }
        }

        printf("\nAfter %.4lf seconds\n", robots[0]->runtime);
        output();
        endLog();

        if (loopFailure) {
            printf("Data so far has been saved in %s\n", filename.c_str());
        } else {
            printf("Data saved in %s\n", filename.c_str());
        }
        std::cout << "[OUTPUT_DIR] " << outputDir << std::endl;

        if (loopFailure) {
            std::rethrow_exception(loopFailure);
        }
    }

private:
    struct TheoremCandidate {
        std::map<int, cbf2026::NodeRateCertificate> certificates;
        std::map<int, cbf2026::EndpointCertificateSnapshot> endpoints;
        cbf2026::ActiveDag activeDag;
    };

    struct CommunicatorCertificateReplacement {
        std::unordered_map<int, Eigen::Matrix2d> covariances;
        std::unordered_map<int, double> uncertaintyRates;
        std::unordered_map<int, cbf2026::EndpointCertificateSnapshot>
            endpoints;
        std::unordered_map<int, double> epsilons;
        std::unordered_map<int, double> barNus;
        std::unordered_map<int, double> covarianceRates;
        std::unordered_map<int, std::uint64_t> snapshotVersions;
        std::unordered_map<int, std::uint64_t> allocationVersions;
    };

    struct RobotCertificateReplacement {
        cbf2026::NodeRateCertificate certificate;
        Eigen::Matrix2d covariance;
        double previousUncertainty;
        double currentUncertainty;
        double uncertaintyRate;
        bool hasUncertaintyHistory;
        bool certificateAvailable;
        std::uint64_t snapshotVersion;
        std::uint64_t allocationVersion;
        std::optional<cbf2026::EndpointCertificateSnapshot> localEndpoint;
        cbf2026::CommittedCertificateState committedState;
        json covarianceFormation;
    };

    bool usesTheoremAlignedCertificates() const {
        return !robots.empty()
               && robots.front()->usesAnalyticTopologicalRateCertificate();
    }

    std::vector<Robot*> theoremTopologicalOrder() const {
        std::vector<Robot*> ordered;
        ordered.reserve(robots.size());
        for (const auto& robot : robots) {
            ordered.push_back(robot.get());
        }
        std::sort(
            ordered.begin(), ordered.end(),
            [](const Robot* lhs, const Robot* rhs) {
                if (lhs->idInMyPart != rhs->idInMyPart) {
                    return lhs->idInMyPart < rhs->idInMyPart;
                }
                return lhs->id < rhs->id;
            }
        );
        return ordered;
    }

    TheoremCandidate buildTheoremCandidate(
        std::uint64_t snapshotVersion,
        const std::vector<int>& recomputeNodes,
        const cbf2026::CommittedCertificateState* predecessor = nullptr,
        const std::map<int, std::vector<int>>* frozenReferenceIds = nullptr
    ) const {
        std::set<int> recompute(
            recomputeNodes.begin(), recomputeNodes.end()
        );
        if (recompute.size() != recomputeNodes.size()) {
            throw std::invalid_argument(
                "theorem recomputation node set is duplicated"
            );
        }
        if (frozenReferenceIds != nullptr
            && frozenReferenceIds->size() != robots.size()) {
            throw std::invalid_argument(
                "frozen reference-ID state is incomplete"
            );
        }
        TheoremCandidate candidate;
        if (predecessor != nullptr && predecessor->valid) {
            if (predecessor->certificates.size()
                != predecessor->endpoints.size()) {
                throw std::invalid_argument(
                    "committed full-certificate state is incomplete"
                );
            }
            for (const auto& [robotId, previousEndpoint] :
                 predecessor->endpoints) {
                auto cloned = previousEndpoint;
                cloned.snapshotVersion = snapshotVersion;
                const Point current = robots.at(robotId - 1)->model->xy();
                cloned.estimate = Eigen::Vector2d(current.x, current.y);
                candidate.endpoints.emplace(robotId, cloned);
                auto certificate = predecessor->certificates.at(robotId);
                certificate.snapshotVersion = snapshotVersion;
                candidate.certificates.emplace(
                    robotId, std::move(certificate)
                );
            }
        }
        for (Robot* robot : theoremTopologicalOrder()) {
            if (recompute.count(robot->id) == 0U) {
                if (candidate.endpoints.count(robot->id) == 0U) {
                    throw std::invalid_argument(
                        "theorem candidate omits an unrecomputed UAV"
                    );
                }
                continue;
            }
            const auto certificate = frozenReferenceIds == nullptr
                ? robot->proposeRateCertificate(
                    snapshotVersion, 1, candidate.endpoints
                )
                : robot->proposeRateCertificate(
                    snapshotVersion,
                    1,
                    candidate.endpoints,
                    frozenReferenceIds->at(robot->id)
                );
            const auto endpoint = cbf2026::makeEndpointCertificateSnapshot(
                certificate,
                Eigen::Vector2d(
                    robot->model->xy().x,
                    robot->model->xy().y
                ),
                1
            );
            candidate.certificates.insert_or_assign(
                robot->id, certificate
            );
            candidate.endpoints.insert_or_assign(robot->id, endpoint);
        }
        if (static_cast<int>(candidate.certificates.size()) != n
            || static_cast<int>(candidate.endpoints.size()) != n) {
            throw std::invalid_argument(
                "theorem candidate is not a complete global state"
            );
        }

        candidate.activeDag.emplace(0, std::vector<int>{});
        for (int robotId : all_ids) {
            candidate.activeDag.emplace(robotId, std::vector<int>{});
        }
        for (const auto& [robotId, certificate] : candidate.certificates) {
            bool hasBaseReference = false;
            for (const auto& reference : certificate.frozenReferences) {
                if (reference.referenceId < 0) {
                    hasBaseReference = true;
                } else if (reference.referenceId > 0) {
                    candidate.activeDag.at(reference.referenceId)
                        .push_back(robotId);
                } else {
                    throw std::invalid_argument(
                        "active certificate contains UAV reference zero"
                    );
                }
            }
            if (hasBaseReference) {
                candidate.activeDag.at(0).push_back(robotId);
            }
        }
        for (auto& [nodeId, children] : candidate.activeDag) {
            (void)nodeId;
            std::sort(children.begin(), children.end());
            children.erase(
                std::unique(children.begin(), children.end()),
                children.end()
            );
        }
        cbf2026::transitiveDescendants(all_ids, candidate.activeDag);
        return candidate;
    }

    static double reconstructedBarrier(
        const cbf2026::EdgeId& edge,
        double threshold,
        const std::map<int, cbf2026::EndpointCertificateSnapshot>& endpoints,
        const Eigen::Vector2d& basePosition
    ) {
        const auto& first = endpoints.at(edge.low);
        const bool baseEdge = edge.baseId >= 0;
        const auto secondPosition = baseEdge
            ? basePosition : endpoints.at(edge.high).estimate;
        const double separation =
            (first.estimate - secondPosition).norm();
        const double epsilonJ = baseEdge
            ? 0.0 : endpoints.at(edge.high).epsilon;
        return edge.kind == cbf2026::EdgeKind::Localization
            ? threshold - separation - first.epsilon - epsilonJ
            : separation - threshold - first.epsilon - epsilonJ;
    }

    cbf2026::GuardAuthority theoremGuardAuthority() const {
        cbf2026::GuardAuthority authority;
        authority.requiredUavNodes = all_ids;
        const auto registry = robots.front()->fixedBarrierEdgeRegistry();
        const auto& hard = config.at("cbfs").at("without-slack");
        const auto& localization = hard.at("comm-fixed");
        const auto& collision = hard.at("safety");
        const ClassKParameters localizationAlpha =
            readClassKParameters(localization, 0.1, 1);
        const ClassKParameters collisionAlpha =
            readClassKParameters(collision, 0.1, 1);
        for (const auto& edge : registry.fixedLocalizationEdges()) {
            Eigen::Vector2d basePosition = Eigen::Vector2d::Zero();
            if (edge.baseId >= 0) {
                const Point base = robots.at(edge.low - 1)
                    ->bases.at(edge.baseId);
                basePosition = Eigen::Vector2d(base.x, base.y);
            }
            authority.hardEdges.push_back({
                edge,
                localization.at("max-range").get<double>(),
                basePosition,
                localizationAlpha.coefficient,
                localizationAlpha.power
            });
        }
        for (const auto& edge : registry.collisionEdges()) {
            authority.hardEdges.push_back({
                edge,
                collision.at("safe-distance").get<double>(),
                Eigen::Vector2d::Zero(),
                collisionAlpha.coefficient,
                collisionAlpha.power
            });
        }
        return authority;
    }

    void appendHardResetRecords(
        const std::map<int, cbf2026::EndpointCertificateSnapshot>& endpoints,
        const cbf2026::CommittedCertificateState& predecessorState,
        cbf2026::ResetProposal& proposal
    ) const {
        const auto registry = robots.front()->fixedBarrierEdgeRegistry();
        const auto& hard = config.at("cbfs").at("without-slack");
        const auto& localization = hard.at("comm-fixed");
        const auto& collision = hard.at("safety");
        const ClassKParameters localizationAlpha =
            readClassKParameters(localization, 0.1, 1);
        const ClassKParameters collisionAlpha =
            readClassKParameters(collision, 0.1, 1);

        const auto append = [&](const cbf2026::EdgeId& edge,
                                double threshold,
                                const ClassKParameters& alpha) {
            Eigen::Vector2d basePosition = Eigen::Vector2d::Zero();
            if (edge.baseId >= 0) {
                const Point base = robots.at(edge.low - 1)
                    ->bases.at(edge.baseId);
                basePosition = Eigen::Vector2d(base.x, base.y);
            }
            const double barrier = reconstructedBarrier(
                edge, threshold, endpoints, basePosition
            );
            const auto& first = endpoints.at(edge.low);
            const bool baseEdge = edge.baseId >= 0;
            const auto secondPosition = baseEdge
                ? basePosition : endpoints.at(edge.high).estimate;
            const double secondRate = baseEdge
                ? 0.0 : endpoints.at(edge.high).barNu;
            const double alphaValue = alpha.coefficient
                * std::pow(barrier, alpha.power);
            const auto snapshot = cbf2026::makeEdgeSnapshot(
                edge,
                first.estimate,
                secondPosition,
                first.barNu,
                secondRate,
                alphaValue,
                proposal.proposedVersion,
                first.allocationVersion
            );
            const auto rows = cbf2026::allocatedRows(
                snapshot,
                baseEdge ? 1.0 : 0.5,
                baseEdge ? 0.0 : 0.5
            );
            double predecessorBarrier = barrier;
            if (predecessorState.valid) {
                predecessorBarrier = reconstructedBarrier(
                    edge,
                    threshold,
                    predecessorState.endpoints,
                    basePosition
                );
            }
            proposal.requiredHardEdges.push_back(edge);
            proposal.hardEdges.push_back({
                edge,
                threshold,
                basePosition,
                predecessorBarrier,
                rows,
                alpha.coefficient,
                alpha.power,
                barrier
            });
        };
        for (const auto& edge : registry.fixedLocalizationEdges()) {
            append(
                edge,
                localization.at("max-range").get<double>(),
                localizationAlpha
            );
        }
        for (const auto& edge : registry.collisionEdges()) {
            append(
                edge,
                collision.at("safe-distance").get<double>(),
                collisionAlpha
            );
        }
    }

    void installTheoremCandidate(
        const TheoremCandidate& candidate,
        cbf2026::CommittedCertificateState& nextCommitted,
        const cbf2026::ActiveDag& nextDag
    ) {
        CommunicatorCertificateReplacement common;
        for (const auto& [robotId, endpoint] : candidate.endpoints) {
            common.covariances.emplace(robotId, endpoint.covariance);
            common.endpoints.emplace(robotId, endpoint);
            common.epsilons.emplace(robotId, endpoint.epsilon);
            common.barNus.emplace(robotId, endpoint.barNu);
            common.covarianceRates.emplace(
                robotId, endpoint.covarianceRateBound
            );
            common.snapshotVersions.emplace(
                robotId, endpoint.snapshotVersion
            );
            common.allocationVersions.emplace(
                robotId, endpoint.allocationVersion
            );
        }
        std::vector<RobotCertificateReplacement> robotReplacements;
        robotReplacements.reserve(robots.size());
        const double timeStep = config.at("execute")
            .at("time-step").get<double>();
        for (const auto& robot : robots) {
            const auto& endpoint = candidate.endpoints.at(robot->id);
            double nextPrevious = endpoint.epsilon;
            double nextCurrent = endpoint.epsilon;
            double nextDescriptiveRate = 0.0;
            if (robot->hasUncertaintyHistory) {
                nextPrevious = robot->currentUncertainty;
                nextDescriptiveRate = std::max(
                    0.0,
                    (nextCurrent - nextPrevious) / timeStep
                );
            }
            common.uncertaintyRates.emplace(
                robot->id, nextDescriptiveRate
            );
            json baseIds = json::array();
            json anchorIds = json::array();
            for (const auto& reference :
                 candidate.certificates.at(robot->id).frozenReferences) {
                if (reference.referenceId < 0) {
                    baseIds.push_back(-reference.referenceId - 1);
                } else {
                    anchorIds.push_back(reference.referenceId);
                }
            }
            robotReplacements.push_back({
                candidate.certificates.at(robot->id),
                endpoint.covariance,
                nextPrevious,
                nextCurrent,
                nextDescriptiveRate,
                true,
                true,
                endpoint.snapshotVersion,
                endpoint.allocationVersion,
                endpoint,
                nextCommitted,
                {
                    {"id", robot->id},
                    {"anchorIds", anchorIds},
                    {"baseIds", baseIds}
                }
            });
        }
        std::vector<CommunicatorCertificateReplacement> replacements(
            robots.size(), common
        );
        cbf2026::ActiveDag activeDagReplacement = nextDag;

        for (std::size_t index = 0; index < robots.size(); ++index) {
            auto& robot = robots[index];
            auto& replacement = replacements[index];
            auto& robotReplacement = robotReplacements[index];
            robot->comm->_othersPositionCovariance.swap(
                replacement.covariances
            );
            robot->comm->_othersUncertaintyRate.swap(
                replacement.uncertaintyRates
            );
            robot->comm->_othersEndpointCertificateSnapshots.swap(
                replacement.endpoints
            );
            robot->comm->_othersEpsilon.swap(replacement.epsilons);
            robot->comm->_othersBarNu.swap(replacement.barNus);
            robot->comm->_othersCovarianceRateBound.swap(
                replacement.covarianceRates
            );
            robot->comm->_othersCertificateSnapshotVersion.swap(
                replacement.snapshotVersions
            );
            robot->comm->_othersAllocationVersion.swap(
                replacement.allocationVersions
            );
            std::swap(
                robot->rateCertificate, robotReplacement.certificate
            );
            std::swap(
                robot->positionCovariance, robotReplacement.covariance
            );
            std::swap(
                robot->previousUncertainty,
                robotReplacement.previousUncertainty
            );
            std::swap(
                robot->currentUncertainty,
                robotReplacement.currentUncertainty
            );
            std::swap(
                robot->uncertaintyRate,
                robotReplacement.uncertaintyRate
            );
            std::swap(
                robot->hasUncertaintyHistory,
                robotReplacement.hasUncertaintyHistory
            );
            std::swap(
                robot->certificateAvailable,
                robotReplacement.certificateAvailable
            );
            std::swap(
                robot->certificateSnapshotVersion,
                robotReplacement.snapshotVersion
            );
            std::swap(
                robot->certificateAllocationVersion,
                robotReplacement.allocationVersion
            );
            std::swap(
                robot->localCertificateSnapshot,
                robotReplacement.localEndpoint
            );
            std::swap(
                robot->committedCertificateState,
                robotReplacement.committedState
            );
            robot->myCovarianceFormation.swap(
                robotReplacement.covarianceFormation
            );
        }
        std::swap(committedCertificateState, nextCommitted);
        committedActiveDag.swap(activeDagReplacement);
    }

    void presetCBFs() {
        if (config["cbfs"]["without-slack"]["comm-fixed"]["on"]) setupCommCBF();
    }

    void postsetCBFs() {
        if (config["cbfs"]["with-slack"]["cvt"]["on"]) setupCVTCBF();
    }
    
    void setupCommCBF() {
        auto commConfig = config["cbfs"]["without-slack"]["comm-fixed"];
        if (!commConfig["on"]) return;

        double maxRange = commConfig["max-range"];
        double k = commConfig["k"];
        int maxOffset = commConfig["max-neighbour-id-offset"];
        int minOffset = commConfig["min-neighbour-id-offset"];

        for (auto &robot : robots) {
            robot->postsetCBF();
            std::vector<int> anchorIds;
            if (robot->myFormation.contains("anchorIds")) {
                for (auto &id : robot->myFormation["anchorIds"]) {
                    anchorIds.push_back(id);
                }
            }

            std::vector<Point> formationPoints;
            if (robot->myFormation.contains("anchorPoints")) {
                for (auto &point : robot->myFormation["anchorPoints"]) {
                    formationPoints.emplace_back(point[0], point[1]);
                }
            }

            for (int anchorId : anchorIds) {
                if (anchorId >= robot->id) continue;

                auto &other = robots[anchorId - 1];
                CBF commCBF;
                commCBF.name = "commCBF_" + std::to_string(robot->id) + "_" + std::to_string(other->id);

                double alpha_coe = commConfig["alpha"]["coe"];
                int alpha_pow = commConfig["alpha"]["pow"];
                commCBF.setAlphaClassK(alpha_coe, alpha_pow);

                auto commFunc = [this, &robot, &other, maxRange, k](VectorXd x, double t) -> double {
                    Eigen::VectorXd robot1State = centralizedModel->getRobotStateFromX(x, robot->id);
                    Eigen::VectorXd robot2State = centralizedModel->getRobotStateFromX(x, other->id);

                    Point pos1 = robot->model->extractXYFromVector(robot1State);
                    Point pos2 = other->model->extractXYFromVector(robot2State);

                    double distance = pos1.distance_to(pos2);
                    double h = k * (maxRange - distance);

                    // Robust CBF: ĥ = h - lε, where l = k for comm CBF
                    // Use distance from origin for uncertainty calculation
                    Point origin(0, 0);
                    double robotDistFromOrigin = pos1.distance_to(origin);
                    double otherDistFromOrigin = pos2.distance_to(origin);
                    double uncertainty1 = robot->getUncertaintyAtDistance(robotDistFromOrigin);
                    double uncertainty2 = other->getUncertaintyAtDistance(otherDistFromOrigin);
                    double robust_h = h - k * (uncertainty1 + uncertainty2);

                    return robust_h;
                };
                commCBF.h = commFunc;

                cbfNoSlack.cbfs[commCBF.name] = commCBF;
            }

            for (int i = 0; i < formationPoints.size(); ++i) {
                Point anchorPoint = formationPoints[i];
                CBF anchorCBF;
                anchorCBF.name = "anchorCBF_" + std::to_string(robot->id) + "_" + std::to_string(i);

                double alpha_coe = commConfig["alpha"]["coe"];
                int alpha_pow = commConfig["alpha"]["pow"];
                anchorCBF.setAlphaClassK(alpha_coe, alpha_pow);

                auto anchorFunc = [this, &robot, anchorPoint, maxRange, k](VectorXd x, double t) -> double {
                    Eigen::VectorXd robotState = centralizedModel->getRobotStateFromX(x, robot->id);

                    Point robotPos = robot->model->extractXYFromVector(robotState);

                    double distance = robotPos.distance_to(anchorPoint);
                    double h = k * (maxRange - distance);

                    // Robust CBF: ĥ = h - lε, where l = k for anchor CBF
                    // Anchor points have no uncertainty, only robot uncertainty
                    // Use distance from origin for uncertainty calculation
                    Point origin(0, 0);
                    double robotDistFromOrigin = robotPos.distance_to(origin);
                    double uncertainty = robot->getUncertaintyAtDistance(robotDistFromOrigin);
                    double robust_h = h - k * uncertainty;

                    return robust_h;
                };
                anchorCBF.h = anchorFunc;

                cbfNoSlack.cbfs[anchorCBF.name] = anchorCBF;
            }
        }
    }

    void setupCVTCBF() {
        auto cvtConfig = config["cbfs"]["with-slack"]["cvt"];
        if (!cvtConfig["on"]) return;

        CVT tempCVT = CVT(n, robots[0]->world.boundary);
        for (auto &robot : robots) {
            tempCVT.pt[robot->id] = robot->model->xy();
        }
        tempCVT.cal_poly();
        for (int i = 1; i <= tempCVT.n; i++) {
            tempCVT.ct[i] = robots[0]->gridWorld.getCentroidInPolygon(tempCVT.pl[i]);
        }

        double alpha_coe = cvtConfig["alpha"]["coe"];
        int alpha_pow = cvtConfig["alpha"]["pow"];
        double kp = cvtConfig["kp"];

        CBF cvtCBF;
        cvtCBF.name = "cvtCBF";
        cvtCBF.setAlphaClassK(alpha_coe, alpha_pow);

        auto cvtFunc = [this, tempCVT, kp](VectorXd x, double t) -> double {
            double totalDistance = 0.0;
            for (auto &robot : robots) {
                int robotIdx = robot->id - 1;
                int stateOffset = centralizedModel->getStateOffset(robotIdx);
                int stateSize = centralizedModel->stateSizes[robotIdx];
                Eigen::VectorXd robotState = x.segment(stateOffset, stateSize);

                Point robotPos = robot->model->extractXYFromVector(robotState);

                Point cvtCenter = tempCVT.ct[robot->id];
                totalDistance += cvtCenter.distance_to(robotPos);
            }
            return -kp * totalDistance;
        };
        cvtCBF.h = cvtFunc;

        cbfSlack[cvtCBF.name] = cvtCBF;
    }


    void initializeCentralizedOptimization() {
        centralizedModel = std::make_unique<CentralizedModel>();
        for (auto &robot : robots) {
            centralizedModel->addRobot(robot->model.get());
        }

        if (config.contains("cbfs") && config["cbfs"].contains("objective-function")) {
            optimizer = createOptimiser(config["optimiser"], config["cbfs"]["objective-function"]);
        } else {
            printf("Warning: missing objective-function config\n");
        }
    }

    void centralizedOptimise() {
        if (!optimizer || !centralizedModel) {
            return;
        }

        centralizedModel->updateConcatenatedStates();

        Eigen::VectorXd uNominal(centralizedModel->getTotalControlSize());
        uNominal.setZero();

        opt = {
            {"nominal",    json::array()},
            {"result",     json::array()},
            {"cbfNoSlack", json::array()},
            {"cbfSlack",   json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();

        optimizer->clear();

        auto f = centralizedModel->f();
        auto g = centralizedModel->g();
        auto x = centralizedModel->getX();

        int uSize = centralizedModel->getTotalControlSize();
        int slackSize = cbfSlack.size();
        int totalSize = uSize + slackSize;

        optimizer->start(totalSize, uSize);
        optimizer->setObjective(uNominal);

        opt["nominal"] = convertCentralizedControlToJson(uNominal);

        std::string cbf_method = config["cbfs"]["without-slack"].value("method", "all");
        if (cbf_method == "all") {
            for (auto &[name, cbf] : cbfNoSlack.cbfs) {
                VectorXd uCoe = cbf.constraintUCoe(f, g, x, robots[0]->runtime);
                double constraintConstWithTime = cbf.constraintConstWithTime(f, g, x, robots[0]->runtime);
                optimizer->addLinearConstraint(uCoe, -constraintConstWithTime);

                jsonCBFNoSlack.emplace_back(json{
                    {"name",  cbf.name},
                    {"coe",   convertCentralizedControlToJson(uCoe)},
                    {"const", constraintConstWithTime}
                });
            }
        }

        int cnt = 0;
        for (auto &[name, cbf] : cbfSlack) {
            VectorXd uCoe = cbf.constraintUCoe(f, g, x, robots[0]->runtime);
            Eigen::VectorXd sCoe = Eigen::VectorXd::Zero(slackSize);
            sCoe(cnt) = 1.0;
            Eigen::VectorXd coe(totalSize);
            coe << uCoe, sCoe;
            double constraintConst = cbf.constraintConstWithoutTime(f, g, x, robots[0]->runtime);

            optimizer->addLinearConstraint(coe, -constraintConst);

            jsonCBFSlack.emplace_back(json{
                {"name",  cbf.name},
                {"coe",   convertCentralizedControlToJson(uCoe)},
                {"const", constraintConst}
            });
            ++cnt;
        }

        opt["cbfNoSlack"] = jsonCBFNoSlack;
        opt["cbfSlack"] = jsonCBFSlack;

        auto result = optimizer->solve();

        optimizer->write("centralized_optimization.lp");

        auto u = result.head(uSize);
        opt["result"] = convertCentralizedControlToJson(u);
        opt["slacks"] = convertCentralizedSlackToJson(result.tail(slackSize));

        for (int i = 0; i < n; ++i) {
            centralizedModel->setRobotControl(i, result);
        }
    }

    bool isCentralizedExecution() const {
        auto executeConfig = config["execute"];
        if (executeConfig.contains("execution-mode")) {
            std::string mode = executeConfig["execution-mode"];
            return mode == "centralized";
        }
        return false;
    }

    json convertCentralizedControlToJson(const VectorXd& u) {
        json result = json::array();
        for (int i = 0; i < robots.size(); ++i) {
            int controlOffset = centralizedModel->getControlOffset(i);
            int controlSize = centralizedModel->controlSizes[i];
            Eigen::VectorXd robotControl = u.segment(controlOffset, controlSize);

            json robotControlJson = json::array();
            for (int j = 0; j < robotControl.size(); ++j) {
                robotControlJson.push_back(robotControl(j));
            }
            result.push_back(robotControlJson);
        }
        return result;
    }

    json convertCentralizedSlackToJson(const VectorXd& slack) {
        json result = json::array();
        for (int i = 0; i < slack.size(); ++i) {
            result.push_back(slack(i));
        }
        return result;
    }

    bool checkConstraintViolation() {
        // Check h_loc constraints for violation
        // Constraint: distance + uncertainty <= max_range
        // Violation occurs when: distance + uncertainty1 + uncertainty2 > max_range
        // Add buffer for numerical stability

        // Get max_range from config
        auto commConfig = config["cbfs"]["without-slack"]["comm-fixed"];
        if (!commConfig.contains("max-range")) {
            return false;  // No max-range specified, cannot check
        }
        double maxRange = commConfig["max-range"];
        double buffer = maxRange * 0.1;  // 1% buffer for numerical stability
        double violationThreshold = maxRange + buffer;

        // Check all robots and their constrained pairs
        for (auto &robot : robots) {
            // Debug: Check if myFormation is properly set
            if (!robot->myFormation.contains("anchorIds") && !robot->myFormation.contains("baseIds")) {
                throw std::runtime_error("Robot " + std::to_string(robot->id) + " myFormation is empty! anchorIds and baseIds not found.");
            }
            if (robot->myFormation.contains("anchorIds") && robot->myFormation["anchorIds"].size() == 0 &&
                robot->myFormation.contains("baseIds") && robot->myFormation["baseIds"].size() == 0) {
                throw std::runtime_error("Robot " + std::to_string(robot->id) + " myFormation has empty anchorIds and baseIds!");
            }

            Point robotPos = robot->model->xy();
            double robotUncertainty = robot->uncertaintyFromCovarianceFunction(robot->positionCovariance);

            // Check robot-robot constraints (from anchorIds)
            if (robot->myFormation.contains("anchorIds")) {
                for (auto &anchorId : robot->myFormation["anchorIds"]) {
                    int otherId = anchorId.get<int>();
                    if (otherId <= 0 || otherId > n) continue;

                    auto &otherRobot = robots[otherId - 1];
                    Point otherPos = otherRobot->model->xy();
                    double otherUncertainty = otherRobot->uncertaintyFromCovarianceFunction(otherRobot->positionCovariance);

                    double distance = robotPos.distance_to(otherPos);
                    double totalDistance = distance + robotUncertainty + otherUncertainty;

                    if (totalDistance > violationThreshold) {
                        std::cout << "\n[Constraint Violation] Robot " << robot->id
                                  << " - Robot " << otherId
                                  << ": distance=" << distance
                                  << ", uncertainty=" << robotUncertainty << "+" << otherUncertainty
                                  << ", total=" << totalDistance
                                  << " > threshold=" << violationThreshold << " (maxRange=" << maxRange << " + buffer=" << buffer << ")"
                                  << " at t=" << robot->runtime << std::endl;
                        return true;
                    }
                }
            }

            // Check robot-base constraints (from baseIds)
            if (robot->myFormation.contains("baseIds")) {
                json bases = config["bases"];
                for (auto &baseIdx : robot->myFormation["baseIds"]) {
                    int baseId = baseIdx.get<int>();
                    if (baseId < 0 || baseId >= bases.size()) continue;

                    Point basePos(bases[baseId][0], bases[baseId][1]);
                    double distance = robotPos.distance_to(basePos);

                    // Base has no uncertainty, only robot uncertainty
                    double totalDistance = distance + robotUncertainty;

                    if (totalDistance > violationThreshold) {
                        std::cout << "\n[Constraint Violation] Robot " << robot->id
                                  << " - Base " << baseId
                                  << ": distance=" << distance
                                  << ", uncertainty=" << robotUncertainty
                                  << ", total=" << totalDistance
                                  << " > threshold=" << violationThreshold << " (maxRange=" << maxRange << " + buffer=" << buffer << ")"
                                  << " at t=" << robot->runtime << std::endl;
                        return true;
                    }
                }
            }
        }

        return false;
    }
};


#endif //CBF_SWARM_HPP
