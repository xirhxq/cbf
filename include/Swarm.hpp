#ifndef CBF_SWARM_HPP
#define CBF_SWARM_HPP

#include "Robot.hpp"
#include "diagnostics/EvidenceStream.hpp"

#ifdef CBF_EVIDENCE_TEST_HOOKS
#include <cstdlib>
#endif
#include <algorithm>
#include <exception>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <unordered_set>
#include <utility>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <thread>

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
    bool route1Mode = false;
    int route1SquadFirst = 0;
    bool route1JointResolve = false;
    int route1JointResolveCount = 0;
    double route1ChainLatencySeconds = 0.0;
    bool estimatorInLoop = false;
    std::string estimatorStatePath;
    std::string estimatorEstimatesPath;
    double estimatorWaitTimeoutSeconds = 5.0;
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
    std::unique_ptr<cbf2026::diagnostics::EvidenceStream> evidenceStream;
    cbf2026::diagnostics::ExactResetWitnessStore<Robot::LocalHardQpEvidence>
        exactResetQpWitnesses;
    std::set<std::uint64_t> emittedEvidenceControllerFrames;
    std::set<std::uint64_t> completedEvidenceControllerFrames;
    std::set<std::uint64_t> emittedEvidenceResetFrames;
    std::map<std::uint64_t, std::size_t>
        emittedEvidenceEndpointRowCounts;
    std::string pendingEvidenceAbortReason;
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
        validateEvidenceStreamConfig();
        if (evidenceMode()) {
            evidenceStream = std::make_unique<
                cbf2026::diagnostics::EvidenceStream
            >(std::cout);
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
        if (evidenceMode()) {
            std::cerr
                << "An Swarm with " << n << " robots @ time "
                << std::fixed << std::setprecision(4)
                << robots[0]->runtime << std::endl;
            return;
        }
        printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, robots[0]->runtime);
        for (auto &robot: robots) {
            robot->output();
        }
        printf("--------------\n");
    }

    void endLog() {
        if (evidenceMode()) {
            return;
        }
        ofstream << std::fixed << std::setprecision(6) << data;
        ofstream.close();
    }

    void logParams() {
        if (evidenceMode()) {
            return;
        }
        data["para"] = robots[0]->getParams();
        data["config"] = config;
    }

    void logOnce() {
        if (evidenceMode()) {
            const bool complete = std::all_of(
                robots.begin(), robots.end(),
                [](const std::unique_ptr<Robot>& robot) {
                    return robot->opt.is_object()
                           && robot->opt.value("status", "") == "success";
                }
            );
            if (!complete) {
                const double timeStep = config.at("execute")
                    .at("time-step").get<double>();
                const auto frameIndex = static_cast<std::uint64_t>(
                    std::llround(robots.front()->runtime / timeStep)
                );
                emitControllerAbortEvidence(
                    frameIndex,
                    pendingEvidenceAbortReason.empty()
                        ? "optimization_incomplete"
                        : pendingEvidenceAbortReason
                );
                return;
            }
            const double timeStep = config.at("execute")
                .at("time-step").get<double>();
            const auto frameIndex = static_cast<std::uint64_t>(
                std::llround(robots.front()->runtime / timeStep)
            );
            emitControllerEvidence(frameIndex);
            return;
        }
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
        if (route1Mode) {
            stepData["route1"] = {
                {"on", true},
                {"chain_latency_s", route1ChainLatencySeconds},
                {"joint_resolutions", route1JointResolveCount}
            };
        }

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
        if (evidenceMode()) {
            outputDir = "evidence-stream://stdout";
            filename.clear();
            return;
        }
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

    bool estimatorFileExists(const std::string& path) const {
        std::ifstream stream(path);
        return stream.good();
    }

    void writeEstimatorState(std::uint64_t frameIndex) {
        json state = {
            {"frame_index", frameIndex},
            {"robots", json::array()}
        };
        for (auto &robot : robots) {
            const auto velocity = robot->model->getVelocity();
            state["robots"].push_back({
                {"id", robot->id},
                {"x", robot->model->xy().x},
                {"y", robot->model->xy().y},
                {"vx", velocity[0]},
                {"vy", velocity[1]},
                {"covariance_formation", robot->myCovarianceFormation}
            });
        }
        {
            std::ofstream out(estimatorStatePath);
            out << state.dump() << '\n';
        }
        std::ofstream ready(estimatorStatePath + ".ready");
        ready << frameIndex << '\n';
    }

    void readEstimatorEstimates(std::uint64_t frameIndex) {
        const auto started = std::chrono::steady_clock::now();
        while (!estimatorFileExists(estimatorEstimatesPath)) {
            if (std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - started
                ).count() > estimatorWaitTimeoutSeconds) {
                throw std::runtime_error(
                    "estimator-in-loop timeout waiting for estimates"
                );
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        json estimates;
        {
            std::ifstream in(estimatorEstimatesPath);
            in >> estimates;
        }
        std::remove(estimatorEstimatesPath.c_str());
        if (estimates.value("frame_index", -1)
            != static_cast<std::int64_t>(frameIndex)) {
            throw std::runtime_error("estimator-in-loop frame mismatch");
        }
        for (const auto &entry : estimates.value("robots", json::array())) {
            const int id = entry.value("id", -1);
            const auto estimate = entry.value("estimate", json::array());
            if (id < 1 || id > n || estimate.size() != 2) {
                throw std::runtime_error(
                    "estimator-in-loop estimate schema invalid"
                );
            }
            const Point position(
                estimate[0].get<double>(), estimate[1].get<double>()
            );
            const double epsilon = entry.value("epsilon", 0.0);
            if (epsilon <= 0.0) {
                throw std::runtime_error(
                    "estimator-in-loop epsilon invalid"
                );
            }
            auto &robot = robots.at(id - 1);
            robot->applyEstimatorInput(position, epsilon, true);
            for (auto &other : robots) {
                other->comm->receivePosition2D(id, position);
            }
        }
    }

    void estimatorFrameExchange(std::uint64_t frameIndex) {
        writeEstimatorState(frameIndex);
        readEstimatorEstimates(frameIndex);
    }

    void restoreTruthPositions() {
        for (auto &robot : robots) {
            for (auto &other : robots) {
                other->comm->receivePosition2D(
                    robot->id, robot->model->xy()
                );
            }
        }
    }

    // Bounded neighborhood coordination: when the sequential route-1 hard QP
    // of `failedIndex` is infeasible, jointly re-solve over the failed robot
    // and the partners appearing in its rows violated at zero input.  All
    // hard rows stay hard; the joint problem is built from each robot's own
    // logged rows (two-velocity form for shared rows).  Returns false when
    // the joint problem is infeasible or cannot be formed (fail-closed).
    bool attemptJointRoute1Resolve(
        size_t failedIndex,
        std::unordered_set<size_t>& solvedIndices
    ) {
        auto& failedRobot = *robots[failedIndex];
        const double t = failedRobot.runtime;
        const json inputLimitsConfig =
            config["cbfs"].value("input-limits", json::object());
        const double planarMax =
            inputLimitsConfig.value("planar-component-max", 50.0);
        const double yawMax =
            inputLimitsConfig.value("yaw-rate-max", 0.35);

        // Partners: robots whose hard row on the failed robot is violated at
        // zero input, capped at the three most violated.
        std::vector<std::pair<double, size_t>> violated;
        {
            auto f = failedRobot.model->f();
            auto g = failedRobot.model->g();
            auto x = failedRobot.model->getX();
            for (auto &[name, cbf] : failedRobot.cbfNoSlack.cbfs) {
                const bool anchorRow =
                    name.rfind("safetyCBF(#", 0) == 0
                    || name.rfind("fixedCommCBF(#", 0) == 0;
                if (!anchorRow) continue;
                VectorXd uCoe = cbf.constraintUCoe(f, g, x, t);
                double constValue =
                    cbf.constraintConstWithTime(f, g, x, t);
                if (constValue >= -1e-9) continue;
                const size_t hash = name.find('#');
                int otherId = std::stoi(name.substr(hash + 1));
                if (otherId < 1
                    || otherId > static_cast<int>(robots.size())) {
                    continue;
                }
                const size_t otherIndex =
                    static_cast<size_t>(otherId - 1);
                if (otherIndex == failedIndex) continue;
                violated.push_back({constValue, otherIndex});
            }
        }
        if (violated.empty()) return false;
        std::sort(violated.begin(), violated.end());
        std::vector<size_t> partners;
        for (auto &[violation, otherIndex] : violated) {
            if (partners.size() >= 3) break;
            if (std::find(partners.begin(), partners.end(), otherIndex)
                == partners.end()) {
                partners.push_back(otherIndex);
            }
        }

        const int partnerCount = static_cast<int>(partners.size());
        const int jointVars = 3 + 2 * partnerCount;
        auto jointOptimiser = createOptimiser(
            config["optimiser"], config["cbfs"]["objective-function"]
        );
        jointOptimiser->start(jointVars, jointVars);

        Eigen::VectorXd target = Eigen::VectorXd::Zero(jointVars);
        {
            const auto nominal =
                failedRobot.opt.value("nominal", json::object());
            target[0] = nominal.value("vx", 0.0);
            target[1] = nominal.value("vy", 0.0);
            target[2] = nominal.value("yawRateRad", 0.0);
        }
        for (int p = 0; p < partnerCount; ++p) {
            auto velocity = robots[partners[p]]->model->getVelocity();
            target[3 + 2 * p] = velocity[0];
            target[4 + 2 * p] = velocity[1];
        }
        jointOptimiser->setObjective(target);

        const auto slotOf = [&](size_t robotIndex) -> int {
            if (robotIndex == failedIndex) return 0;
            for (int p = 0; p < partnerCount; ++p) {
                if (partners[p] == robotIndex) return 3 + 2 * p;
            }
            return -1;
        };

        const auto addJointRows = [&](size_t robotIndex) -> bool {
            auto& robot = *robots[robotIndex];
            auto f = robot.model->f();
            auto g = robot.model->g();
            auto x = robot.model->getX();
            const int selfSlot = slotOf(robotIndex);
            if (selfSlot < 0) return false;
            for (auto &[name, cbf] : robot.cbfNoSlack.cbfs) {
                VectorXd uCoe = cbf.constraintUCoe(f, g, x, t);
                double constValue =
                    cbf.constraintConstWithTime(f, g, x, t);
                Eigen::VectorXd jointCoe =
                    Eigen::VectorXd::Zero(jointVars);
                int otherIndex = -1;
                if (name.rfind("safetyCBF(#", 0) == 0
                    || name.rfind("fixedCommCBF(#", 0) == 0) {
                    const size_t hash = name.find('#');
                    int otherId = std::stoi(name.substr(hash + 1));
                    if (otherId >= 1
                        && otherId <= static_cast<int>(robots.size())) {
                        otherIndex = otherId - 1;
                    }
                }
                const int otherSlot =
                    otherIndex >= 0 ? slotOf(otherIndex) : -1;
                if (otherSlot >= 0) {
                    // Two-velocity form:
                    // coe·u_self - coe·u_other
                    //     >= -const - coe·v_other_current.
                    jointCoe[selfSlot] += uCoe[0];
                    jointCoe[selfSlot + 1] += uCoe[1];
                    if (selfSlot == 0) {
                        jointCoe[2] += uCoe[2];
                    } else if (std::abs(uCoe[2]) > 1e-12) {
                        return false;
                    }
                    jointCoe[otherSlot] -= uCoe[0];
                    jointCoe[otherSlot + 1] -= uCoe[1];
                    auto otherVel =
                        robots[otherIndex]->model->getVelocity();
                    const double rhs =
                        -constValue
                        - (uCoe[0] * otherVel[0]
                           + uCoe[1] * otherVel[1]);
                    jointOptimiser->addLinearConstraint(jointCoe, rhs);
                } else {
                    jointCoe[selfSlot] = uCoe[0];
                    jointCoe[selfSlot + 1] = uCoe[1];
                    if (selfSlot == 0) {
                        jointCoe[2] = uCoe[2];
                    } else if (std::abs(uCoe[2]) > 1e-12) {
                        return false;
                    }
                    jointOptimiser->addLinearConstraint(
                        jointCoe, -constValue
                    );
                }
            }
            return true;
        };

        if (!addJointRows(failedIndex)) return false;
        for (int p = 0; p < partnerCount; ++p) {
            if (!addJointRows(partners[p])) return false;
        }

        const auto addBound = [&](int variable, double limit) {
            Eigen::VectorXd upper = Eigen::VectorXd::Zero(jointVars);
            upper[variable] = 1.0;
            jointOptimiser->addLinearConstraint(upper, -limit);
            Eigen::VectorXd lower = Eigen::VectorXd::Zero(jointVars);
            lower[variable] = -1.0;
            jointOptimiser->addLinearConstraint(lower, -limit);
        };
        for (int v = 0; v < jointVars; ++v) {
            addBound(v, v == 2 ? yawMax : planarMax);
        }

        auto result = jointOptimiser->solve();
        const json solverStatus = jointOptimiser->getStatus();
        if (solverStatus.value("status", "") != "optimal") {
            return false;
        }
        const Eigen::VectorXd solution = result.head(jointVars);

        std::vector<size_t> resolved = {failedIndex};
        resolved.insert(resolved.end(), partners.begin(), partners.end());
        std::map<size_t, Eigen::VectorXd> preJointVelocities;
        for (size_t index : resolved) {
            preJointVelocities[index] =
                robots[index]->model->getVelocity();
        }

        Eigen::VectorXd failedInput(3);
        failedInput << solution[0], solution[1], solution[2];
        failedRobot.model->setControlInput(failedInput);
        failedRobot.opt["status"] = "success";
        failedRobot.opt["result"] = {
            {"vx", solution[0]},
            {"vy", solution[1]},
            {"yawRateRad", solution[2]}
        };
        json partnerIds = json::array();
        json partnerPreviousVelocities = json::array();
        for (int p = 0; p < partnerCount; ++p) {
            partnerIds.push_back(robots[partners[p]]->id);
            const auto& previous = preJointVelocities[partners[p]];
            partnerPreviousVelocities.push_back(
                {previous[0], previous[1]}
            );
        }
        failedRobot.opt["joint_resolution"] = {
            {"phase", "joint"},
            {"partners", partnerIds},
            {"partner_previous_velocities", partnerPreviousVelocities}
        };

        for (int p = 0; p < partnerCount; ++p) {
            auto& partner = *robots[partners[p]];
            const double yawRate =
                partner.opt.value("result", json::object())
                    .value("yawRateRad", 0.0);
            Eigen::VectorXd partnerInput(3);
            partnerInput << solution[3 + 2 * p],
                            solution[4 + 2 * p],
                            yawRate;
            partner.model->setControlInput(partnerInput);
            partner.opt["result"] = {
                {"vx", solution[3 + 2 * p]},
                {"vy", solution[4 + 2 * p]},
                {"yawRateRad", yawRate}
            };
            partner.opt["joint_resolution"] = {
                {"phase", "joint"},
                {"requested_by", failedRobot.id}
            };
        }

        // Log residuals for every hard row with the applied joint commands
        // (two-velocity correction for rows shared with a joint partner).
        for (size_t index : resolved) {
            auto& robot = *robots[index];
            auto f = robot.model->f();
            auto g = robot.model->g();
            auto x = robot.model->getX();
            const int selfSlot = slotOf(index);
            for (auto &[name, cbf] : robot.cbfNoSlack.cbfs) {
                VectorXd uCoe = cbf.constraintUCoe(f, g, x, t);
                double constValue =
                    cbf.constraintConstWithTime(f, g, x, t);
                double residual = constValue
                    + uCoe[0] * solution[selfSlot]
                    + uCoe[1] * solution[selfSlot + 1];
                if (selfSlot == 0) {
                    residual += uCoe[2] * solution[2];
                }
                int otherIndex = -1;
                if (name.rfind("safetyCBF(#", 0) == 0
                    || name.rfind("fixedCommCBF(#", 0) == 0) {
                    const size_t hash = name.find('#');
                    int otherId = std::stoi(name.substr(hash + 1));
                    if (otherId >= 1
                        && otherId <= static_cast<int>(robots.size())) {
                        otherIndex = otherId - 1;
                    }
                }
                if (otherIndex >= 0 && slotOf(otherIndex) >= 0) {
                    const auto& otherPre = preJointVelocities[otherIndex];
                    residual += uCoe[0] * (
                        otherPre[0] - solution[slotOf(otherIndex)]
                    ) + uCoe[1] * (
                        otherPre[1] - solution[slotOf(otherIndex) + 1]
                    );
                }
                for (auto &row : robot.opt["cbfNoSlack"]) {
                    if (row.value("name", "") == name) {
                        row["residual"] = residual;
                        break;
                    }
                }
            }
        }

        for (size_t index : resolved) {
            for (auto &otherRobot : robots) {
                otherRobot->comm->receiveVelocity2D(
                    robots[index]->id,
                    robots[index]->model->getVelocity()
                );
            }
            solvedIndices.insert(index);
        }
        ++route1JointResolveCount;
        return true;
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
        cbf2026::CommittedCertificateState predecessorState;
        try {
#ifdef CBF_EVIDENCE_TEST_HOOKS
        if (evidenceTestFixtureMode()) {
            const char* failBootstrap = std::getenv(
                "CBF_EVIDENCE_TEST_FAIL_BOOTSTRAP"
            );
            if (failBootstrap != nullptr
                && std::string(failBootstrap) == "1") {
                throw std::runtime_error(
                    "injected evidence bootstrap failure"
                );
            }
        }
#endif
        predecessorState = committedCertificateState;
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
            [this, frameIndex](
                int robotId,
                const cbf2026::HardConstraintProblem& problem
            ) {
                auto evidence = robots.at(robotId - 1)
                    ->solveLocalHardQpEvidence(problem);
                exactResetQpWitnesses.record(
                    frameIndex, robotId, evidence
                );
                return evidence.feasibility;
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
        emitResetEvidence(
            transaction,
            predecessorState.endpoints,
            candidate.endpoints,
            predecessorState.certificates,
            candidate.certificates
        );
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
            const auto recordedTransaction = std::find_if(
                certificateResetHistory.rbegin(),
                certificateResetHistory.rend(),
                [frameIndex](const cbf2026::ResetTransaction& transaction) {
                    return transaction.frameIndex == frameIndex;
                }
            );
            if (recordedTransaction != certificateResetHistory.rend()) {
                emitResetEvidence(
                    *recordedTransaction,
                    predecessorState.endpoints,
                    proposal.candidateEndpoints,
                    predecessorState.certificates,
                    proposal.candidateCertificates
                );
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
            const auto recordedTransaction = std::find_if(
                certificateResetHistory.rbegin(),
                certificateResetHistory.rend(),
                [frameIndex](const cbf2026::ResetTransaction& transaction) {
                    return transaction.frameIndex == frameIndex;
                }
            );
            if (recordedTransaction != certificateResetHistory.rend()) {
                emitResetEvidence(
                    *recordedTransaction,
                    predecessorState.endpoints,
                    proposal.candidateEndpoints,
                    predecessorState.certificates,
                    proposal.candidateCertificates
                );
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
                if (evidenceMode()) {
                    json rowsJson = json::array();
                    for (const auto& row : problem.rows) {
                        rowsJson.push_back({
                            {"owner", row.owner},
                            {"name", row.name},
                            {"constant", row.constant},
                            {"coefficients", vectorJson(row.coefficients)},
                            {"post_reset_barrier", row.postResetBarrier}
                        });
                    }
                    evidenceStream->write({
                        {"record_type", "flow_failure"},
                        {"robot_id", robotId},
                        {"qp_status", feasibility.status},
                        {"row_count", static_cast<std::uint64_t>(problem.rows.size())},
                        {"rows", std::move(rowsJson)}
                    });
                }
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
        if (evidenceMode()) {
            emitInitializationEvidence();
        }
        try {
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
        } catch (...) {
            if (!evidenceMode()) {
                throw;
            }
            const std::exception_ptr bootstrapFailure =
                std::current_exception();
            const std::string reason = exceptionMessage(bootstrapFailure);
            try {
                emitControllerAbortEvidence(0, reason);
            } catch (...) {
            }
            try {
                emitMissionTerminal(false, "bootstrap_failure");
            } catch (...) {
            }
            std::rethrow_exception(bootstrapFailure);
        }
        checkInformationExchange();
        initLog();
        logParams();
        output();

        if (isCentralizedExecution()) {
            initializeCentralizedOptimization();
            presetCBFs();
        } else {
            route1Mode =
                config["cbfs"].value("route1", json::object()).value("on", false);
            route1SquadFirst =
                config["cbfs"].value("route1", json::object())
                    .value("squad-first", 0);
            route1JointResolve =
                config["cbfs"].value("route1", json::object())
                    .value("joint-resolve", false);
            const auto estimatorConfig =
                config.value("estimator-in-loop", json::object());
            estimatorInLoop = estimatorConfig.value("on", false);
            if (estimatorInLoop) {
                estimatorStatePath = estimatorConfig.value("state-path", "");
                estimatorEstimatesPath =
                    estimatorConfig.value("estimates-path", "");
                estimatorWaitTimeoutSeconds =
                    estimatorConfig.value("wait-timeout-s", 5.0);
                if (estimatorStatePath.empty()
                    || estimatorEstimatesPath.empty()) {
                    throw std::invalid_argument(
                        "estimator-in-loop paths are required"
                    );
                }
            }
            for (auto &robot: robots) robot->presetCBF();
        }

        std::exception_ptr loopFailure;
        while (robots[0]->runtime < tTotal) {
            bool frameLogAttempted = false;
            try {
                if (evidenceMode()) {
                    pendingEvidenceAbortReason.clear();
                    for (auto& robot : robots) {
                        robot->clearEvidenceOptimisationState();
                    }
                }
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
                if (evidenceMode()) {
                    std::cerr
                        << std::fixed << std::setprecision(2)
                        << robots[0]->runtime
                        << " seconds elapsed... "
                        << gridWorldGroundTruth.getPercentage() * 100
                        << "%\n";
                } else {
                    printf("\r%.2lf seconds elapsed... %.2lf%%", robots[0]->runtime, gridWorldGroundTruth.getPercentage() * 100);
                }
                for (auto &robot: robots) robot->updateGridWorld();
                updateGridWorld();
                checkUpdatedGridWorld();
                if (estimatorInLoop) {
                    estimatorFrameExchange(
                        static_cast<std::uint64_t>(
                            std::llround(robots[0]->runtime / tStep)
                        )
                    );
                }
                for (auto &robot: robots) robot->postsetCBF();

                if (isCentralizedExecution()) {
                    postsetCBFs();
                    centralizedOptimise();
                } else if (route1Mode) {
                    const auto route1Start = std::chrono::steady_clock::now();
                    std::vector<size_t> solveOrder(robots.size());
                    std::iota(solveOrder.begin(), solveOrder.end(), 0);
                    if (route1SquadFirst == 2 && robots.size() == 14) {
                        // Squad 2 (robots 8-14, indices 7-13) first, then
                        // squad 1 (robots 1-7, indices 0-6); intra-squad
                        // order stays low -> high.
                        solveOrder = {7, 8, 9, 10, 11, 12, 13,
                                      0, 1, 2, 3, 4, 5, 6};
                    }
                    std::unordered_set<size_t> jointSolved;
                    for (const size_t robotIndex : solveOrder) {
                        if (jointSolved.count(robotIndex) > 0) {
                            continue;
                        }
                        auto &robot = robots[robotIndex];
                        try {
                            robot->optimise();
                        } catch (...) {
                            if (!route1JointResolve
                                || !attemptJointRoute1Resolve(
                                    robotIndex, jointSolved
                                )) {
                                throw;
                            }
                            continue;
                        }
                        for (auto &otherRobot: robots) {
                            otherRobot->comm->receiveVelocity2D(
                                robot->id,
                                robot->model->getVelocity()
                            );
                        }
                    }
                    route1ChainLatencySeconds =
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - route1Start
                        ).count();
                } else {
                    for (auto &robot: robots) robot->optimise();
                }
                if (estimatorInLoop) {
                    restoreTruthPositions();
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
                        std::ostream& termination = evidenceMode()
                            ? std::cerr : std::cout;
                        termination << "\n[Simulation Terminated] Constraint violation detected at t=" << robots[0]->runtime << "s" << std::endl;
                        break;
                    }
                }

                frameLogAttempted = true;
                logOnce();
                for (auto &robot: robots) robot->stepTimeForward(tStep);
            }
            catch (...) {
                const std::exception_ptr currentFailure =
                    std::current_exception();
                if (evidenceMode()) {
                    pendingEvidenceAbortReason = exceptionMessage(
                        currentFailure
                    );
                    const auto frameIndex = static_cast<std::uint64_t>(
                        std::llround(robots.front()->runtime / tStep)
                    );
                    try {
                        emitControllerAbortEvidence(
                            frameIndex, pendingEvidenceAbortReason
                        );
                    } catch (...) {
                    }
                }
                loopFailure = recoverFailedIteration(
                    currentFailure,
                    evidenceMode() ? true : frameLogAttempted,
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

        if (evidenceMode()) {
            std::cerr << "After " << std::fixed << std::setprecision(4)
                      << robots[0]->runtime << " seconds\n";
        } else {
            printf("\nAfter %.4lf seconds\n", robots[0]->runtime);
        }
        output();
        if (evidenceMode()) {
            const bool missionCompleted = !loopFailure
                && robots.front()->runtime >= tTotal;
            emitMissionTerminal(
                missionCompleted,
                missionCompleted ? "completed" : (
                    loopFailure ? "loop_failure" : "terminated_early"
                )
            );
        }
        endLog();

        if (evidenceMode()) {
            std::cerr << (loopFailure
                ? "Evidence prefix retained after failure\n"
                : "Evidence stream completed\n");
            std::cerr << "[OUTPUT_DIR] " << outputDir << std::endl;
        } else {
            if (loopFailure) {
                printf("Data so far has been saved in %s\n", filename.c_str());
            } else {
                printf("Data saved in %s\n", filename.c_str());
            }
            std::cout << "[OUTPUT_DIR] " << outputDir << std::endl;
        }

        if (loopFailure) {
            std::rethrow_exception(loopFailure);
        }
    }

private:
    bool evidenceMode() const {
        return config.contains("evidence-stream")
               && config.at("evidence-stream").is_object()
               && config.at("evidence-stream").value("enabled", false);
    }

#ifdef CBF_EVIDENCE_TEST_HOOKS
    bool evidenceTestFixtureMode() const {
        return evidenceMode()
               && config.at("evidence-stream").value(
                    "campaign-id", ""
                  ) == "two-frame-fixture";
    }
#endif

    void validateEvidenceStreamConfig() const {
        if (!config.contains("evidence-stream")) {
            return;
        }
        const auto& evidence = config.at("evidence-stream");
        const std::set<std::string> expectedKeys = {
            "enabled",
            "schema-version",
            "campaign-id",
            "trajectory-seed",
            "range-noise-seed",
            "condition"
        };
        std::set<std::string> actualKeys;
        if (evidence.is_object()) {
            for (const auto& [key, value] : evidence.items()) {
                (void)value;
                actualKeys.insert(key);
            }
        }
        if (!evidence.is_object()
            || actualKeys != expectedKeys
            || !evidence.at("enabled").is_boolean()
            || !evidence.at("enabled").get<bool>()
            || evidence.at("schema-version")
                   != "cbf2026-qualified-evidence-v1"
            || !evidence.at("campaign-id").is_string()
            || evidence.at("campaign-id").get<std::string>().empty()
            || !evidence.at("trajectory-seed").is_number_integer()
            || !evidence.at("range-noise-seed").is_number_integer()
            || evidence.at("condition") != "dynamic_primary"
            || !config.contains("qualified-estimator")) {
            throw std::invalid_argument(
                "evidence-stream identity is invalid"
            );
        }
    }

    json evidenceBaseRecord(
        const std::string& recordType,
        std::uint64_t frameIndex
    ) const {
        const auto& identity = config.at("evidence-stream");
        return {
            {"record_type", recordType},
            {"schema_version", identity.at("schema-version")},
            {"campaign_id", identity.at("campaign-id")},
            {"condition", identity.at("condition")},
            {"trajectory_seed", identity.at("trajectory-seed")},
            {"range_noise_seed", identity.at("range-noise-seed")},
            {"frame_index", frameIndex}
        };
    }

    static json vectorJson(const Eigen::VectorXd& value) {
        json result = json::array();
        for (Eigen::Index index = 0; index < value.size(); ++index) {
            result.push_back(value[index]);
        }
        return result;
    }

    static json vectorJson(const Eigen::Vector2d& value) {
        return json::array({value.x(), value.y()});
    }

    static json matrixJson(const Eigen::Matrix2d& value) {
        return json::array({
            json::array({value(0, 0), value(0, 1)}),
            json::array({value(1, 0), value(1, 1)})
        });
    }

    static json finiteOrNull(double value) {
        return std::isfinite(value) ? json(value) : json(nullptr);
    }

    static std::string exceptionMessage(
        const std::exception_ptr& failure
    ) {
        try {
            if (failure) {
                std::rethrow_exception(failure);
            }
        } catch (const std::exception& error) {
            return error.what();
        } catch (...) {
            return "unknown_error";
        }
        return "no_exception";
    }

    static json edgeIdentityJson(const cbf2026::EdgeId& edge) {
        return {
            {"kind", edge.kind == cbf2026::EdgeKind::Localization
                ? "localization" : "collision"},
            {"low", edge.low},
            {"high", edge.high},
            {"base_id", edge.baseId}
        };
    }

    static json endpointStateJson(
        const cbf2026::EndpointCertificateSnapshot& endpoint
    ) {
        return {
            {"robot_id", endpoint.robotId},
            {"estimate", vectorJson(endpoint.estimate)},
            {"covariance", matrixJson(endpoint.covariance)},
            {"covariance_rate_bound", endpoint.covarianceRateBound},
            {"epsilon", endpoint.epsilon},
            {"bar_nu", endpoint.barNu},
            {"snapshot_version", endpoint.snapshotVersion},
            {"allocation_version", endpoint.allocationVersion}
        };
    }

    static json endpointStateMapJson(
        const std::map<int, cbf2026::EndpointCertificateSnapshot>& endpoints
    ) {
        json result = json::array();
        for (const auto& [robotId, endpoint] : endpoints) {
            (void)robotId;
            result.push_back(endpointStateJson(endpoint));
        }
        return result;
    }

    static json hardProblemJson(
        const cbf2026::HardConstraintProblem& problem
    ) {
        json bounds = json::array();
        for (const auto& bound : problem.bounds) {
            bounds.push_back({
                {"control_index", bound.controlIndex},
                {"coefficient", bound.coefficient},
                {"limit", bound.limit}
            });
        }
        json rows = json::array();
        for (const auto& row : problem.rows) {
            rows.push_back({
                {"edge", edgeIdentityJson(row.edge)},
                {"owner", row.owner},
                {"name", row.name},
                {"coefficients", vectorJson(row.coefficients)},
                {"constant", row.constant},
                {"post_reset_barrier", row.postResetBarrier},
                {"snapshot_version", row.snapshotVersion},
                {"allocation_version", row.allocationVersion}
            });
        }
        return {
            {"owner", problem.owner},
            {"control_size", problem.controlSize},
            {"planar_component_max", problem.planarComponentMax},
            {"yaw_rate_max", problem.yawRateMax},
            {"snapshot_version", problem.snapshotVersion},
            {"allocation_version", problem.allocationVersion},
            {"bounds", bounds},
            {"rows", rows},
            {"hard_problem_id",
             cbf2026::canonicalHardConstraintProblemHash(problem)}
        };
    }

    void emitInitializationEvidence() {
        if (!evidenceMode()) {
            return;
        }
        for (const auto& robot : robots) {
            json row = evidenceBaseRecord("initialization", 0);
            const Point position = robot->model->xy();
            row["robot_id"] = robot->id;
            row["runtime"] = {
                {"local_index", robot->idInMyPart}
            };
            row["analyzer_only"] = {
                {"truth_position", {position.x, position.y}}
            };
            evidenceStream->write(row);
        }
    }

    void emitResetEvidence(
        const cbf2026::ResetTransaction& transaction,
        const std::map<int, cbf2026::EndpointCertificateSnapshot>&
            predecessorEndpoints,
        const std::map<int, cbf2026::EndpointCertificateSnapshot>&
            proposedEndpoints,
        const std::map<int, cbf2026::NodeRateCertificate>&
            predecessorCertificates,
        const std::map<int, cbf2026::NodeRateCertificate>&
            proposedCertificates
    ) {
        if (!evidenceMode()
            || !emittedEvidenceResetFrames.insert(
                transaction.frameIndex
            ).second) {
            return;
        }
        json row = evidenceBaseRecord("reset", transaction.frameIndex);
        json causes = json::array();
        for (const auto cause : transaction.causes) {
            causes.push_back(cbf2026::resetCauseName(cause));
        }
        json nodes = json::array();
        for (const auto& node : transaction.nodes) {
            json item = {
                {"node_id", node.nodeId},
                {"topological_index", node.topologicalIndex},
                {"snapshot_version", node.snapshotVersion},
                {"delta_p", vectorJson(node.deltaEstimate)},
                {"delta_epsilon", node.deltaEpsilon},
                {"pre_active_references", node.preActiveReferences},
                {"post_active_references", node.postActiveReferences}
            };
            item["proposed_snapshot"] = node.proposedSnapshot.has_value()
                ? endpointStateJson(*node.proposedSnapshot) : json(nullptr);
            nodes.push_back(std::move(item));
        }
        json hardEdges = json::array();
        for (const auto& edge : transaction.hardEdges) {
            json endpointRows = json::array();
            for (const auto& endpoint : edge.endpointRows) {
                endpointRows.push_back({
                    {"owner", endpoint.owner},
                    {"coefficient", vectorJson(endpoint.coefficient)},
                    {"constant", endpoint.constant},
                    {"allocation", endpoint.allocation},
                    {"snapshot_version", endpoint.snapshotVersion},
                    {"allocation_version", endpoint.allocationVersion}
                });
            }
            hardEdges.push_back({
                {"edge", edgeIdentityJson(edge.edge)},
                {"threshold", edge.threshold},
                {"base_position", vectorJson(edge.basePosition)},
                {"b_minus", edge.preBarrier},
                {"b_plus", edge.postBarrier},
                {"class_k_coefficient", edge.classKCoefficient},
                {"class_k_power", edge.classKPower},
                {"endpoint_rows", endpointRows}
            });
        }
        json hardProblems = json::array();
        for (const auto& [robotId, problem] :
             transaction.checkedHardProblems) {
            (void)robotId;
            hardProblems.push_back(hardProblemJson(problem));
        }
        json localQps = json::array();
        for (const auto& local : transaction.localHardQps) {
            json solution = nullptr;
            const Robot::LocalHardQpEvidence* exactEvidence = nullptr;
            if (local.feasibility.status != "unchecked") {
                try {
                    exactEvidence = &exactResetQpWitnesses.at(
                        transaction.frameIndex, local.nodeId
                    );
                } catch (const std::out_of_range&) {
                    throw std::runtime_error(
                        "verified reset QP is missing its exact solve witness"
                    );
                }
                const auto& exactFeasibility = exactEvidence->feasibility;
                if (exactFeasibility.feasible != local.feasibility.feasible
                    || exactFeasibility.status != local.feasibility.status
                    || exactFeasibility.minimumResidual
                        != local.feasibility.minimumResidual
                    || exactFeasibility.hardProblemHash
                        != local.feasibility.hardProblemHash) {
                    throw std::runtime_error(
                        "reset QP witness disagrees with the guard decision"
                    );
                }
                if (exactEvidence->solution.has_value()) {
                    solution = vectorJson(*exactEvidence->solution);
                }
            }
            if (transaction.status == cbf2026::GuardStatus::Accepted
                && (exactEvidence == nullptr
                    || !exactEvidence->solution.has_value())) {
                throw std::runtime_error(
                    "accepted reset QP is missing its exact solution witness"
                );
            }
            const auto& feasibility = exactEvidence == nullptr
                ? local.feasibility : exactEvidence->feasibility;
            localQps.push_back({
                {"node_id", local.nodeId},
                {"problem", hardProblemJson(local.problem)},
                {"feasible", feasibility.feasible},
                {"status", feasibility.status},
                {"minimum_residual",
                 finiteOrNull(feasibility.minimumResidual)},
                {"hard_problem_id", feasibility.hardProblemHash},
                {"solution", solution}
            });
        }
        std::string resetStage = "proposal_started";
        if (transaction.status == cbf2026::GuardStatus::Accepted) {
            resetStage = "committed";
        } else if (!transaction.checkedHardProblems.empty()) {
            resetStage = "lifecycle_rejected";
        } else if (std::any_of(
            transaction.localHardQps.begin(),
            transaction.localHardQps.end(),
            [](const cbf2026::LocalHardQpResetRecord& local) {
                return local.feasibility.status != "unchecked";
            }
        )) {
            resetStage = "hard_qp_verified";
        } else if (!transaction.localHardQps.empty()) {
            resetStage = "hard_qp_preflight";
        } else if (!transaction.hardEdges.empty()) {
            resetStage = "hard_edges_built";
        } else if (!transaction.nodes.empty()) {
            resetStage = "candidate_built";
        }
        json activeSets = json::array();
        std::set<int> activeNodeIds;
        for (const auto& [robotId, certificate] :
             predecessorCertificates) {
            (void)certificate;
            activeNodeIds.insert(robotId);
        }
        for (const auto& [robotId, certificate] : proposedCertificates) {
            (void)certificate;
            activeNodeIds.insert(robotId);
        }
        for (int robotId : activeNodeIds) {
            const auto predecessor = predecessorCertificates.find(robotId);
            const auto proposed = proposedCertificates.find(robotId);
            activeSets.push_back({
                {"node_id", robotId},
                {"pre_active_references",
                 predecessor == predecessorCertificates.end()
                    ? std::vector<int>{}
                    : cbf2026::canonicalFrozenReferenceIds(
                        predecessor->second
                      )},
                {"post_active_references",
                 proposed == proposedCertificates.end()
                    ? std::vector<int>{}
                    : cbf2026::canonicalFrozenReferenceIds(
                        proposed->second
                      )}
            });
        }
        row["runtime"] = {
            {"stage", resetStage},
            {"trigger", causes},
            {"predecessor_version", transaction.predecessorVersion},
            {"proposed_version", transaction.proposedVersion},
            {"changed_nodes", transaction.changedNodes},
            {"descendant_closure", transaction.descendantClosure},
            {"nodes", nodes},
            {"active_sets", activeSets},
            {"pre_endpoint_states",
             endpointStateMapJson(predecessorEndpoints)},
            {"post_endpoint_states",
             endpointStateMapJson(proposedEndpoints)},
            {"hard_edges", hardEdges},
            {"hard_problems", hardProblems},
            {"local_hard_qps", localQps},
            {"guard_decision", transaction.status
                == cbf2026::GuardStatus::Accepted
                ? "accepted" : "rejected"},
            {"outcome", transaction.status
                == cbf2026::GuardStatus::Accepted
                ? "commit" : "abort"},
            {"reason", transaction.reason}
        };
        evidenceStream->write(row);
        exactResetQpWitnesses.eraseFrame(transaction.frameIndex);
    }

    struct RateEvidenceState {
        std::map<int, cbf2026::NodeRateInput> inputs;
        std::map<int, Eigen::Matrix2d> covarianceDerivatives;
        std::map<int, double> upperDiniEpsilonRates;
        std::map<int, double> realizedEpsilonRates;
    };

    RateEvidenceState reconstructRateEvidenceState() const {
        RateEvidenceState evidence;
        for (Robot* robot : theoremTopologicalOrder()) {
            auto input = robot->committedRateEvidenceInput(
                committedCertificateState
            );
            const auto& certificate =
                committedCertificateState.certificates.at(robot->id);
            const Eigen::Vector2d nodeVelocity =
                robot->model->getControlInput().head<2>();
            Eigen::Matrix2d dotInformation = Eigen::Matrix2d::Zero();
            std::vector<cbf2026::ReferenceRealizedDerivative> derivatives;
            for (const auto& reference : input.references) {
                Eigen::Vector2d predecessorVelocity =
                    Eigen::Vector2d::Zero();
                if (!reference.predecessor.hoveringBase) {
                    predecessorVelocity = robots.at(
                        reference.predecessor.referenceId - 1
                    )->model->getControlInput().head<2>();
                }
                const Eigen::Vector2d relativeVelocity =
                    nodeVelocity - predecessorVelocity;
                const Eigen::Vector2d dotDirection = (
                    Eigen::Matrix2d::Identity()
                    - reference.direction
                      * reference.direction.transpose()
                ) * relativeVelocity / reference.distance;
                const Eigen::Matrix2d predecessorPdot =
                    reference.predecessor.hoveringBase
                    ? Eigen::Matrix2d::Zero()
                    : evidence.covarianceDerivatives.at(
                        reference.predecessor.referenceId
                    );
                const double dotEffectiveVariance =
                    2.0 * dotDirection.dot(
                        reference.predecessor.covariance
                        * reference.direction
                    )
                    + reference.direction.dot(
                        predecessorPdot * reference.direction
                    );
                const double effectiveVariance =
                    reference.direction.dot(
                        reference.predecessor.covariance
                        * reference.direction
                    ) + reference.rangingVariance;
                dotInformation.noalias() +=
                    -dotEffectiveVariance
                     / std::pow(effectiveVariance, 2)
                     * (reference.direction
                        * reference.direction.transpose())
                    + (
                        dotDirection * reference.direction.transpose()
                        + reference.direction * dotDirection.transpose()
                    ) / effectiveVariance;
                derivatives.push_back({
                    reference.predecessor.referenceId,
                    dotEffectiveVariance,
                    dotDirection
                });
            }
            Eigen::Matrix2d covarianceDerivative =
                -certificate.covariance
                 * dotInformation
                 * certificate.covariance;
            covarianceDerivative = 0.5 * (
                covarianceDerivative + covarianceDerivative.transpose()
            );
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(
                certificate.covariance
            );
            if (solver.info() != Eigen::Success) {
                throw std::runtime_error(
                    "evidence covariance eigendecomposition failed"
                );
            }
            const double lambdaMax = solver.eigenvalues().maxCoeff();
            const double gap = solver.eigenvalues()[1]
                               - solver.eigenvalues()[0];
            const double scale = std::max(1.0, std::abs(lambdaMax));
            const double lambdaDini = gap <= 1e-12 * scale
                ? Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    covarianceDerivative
                  ).eigenvalues().maxCoeff()
                : solver.eigenvectors().col(1).dot(
                    covarianceDerivative
                    * solver.eigenvectors().col(1)
                  );
            evidence.inputs.emplace(robot->id, input);
            evidence.covarianceDerivatives.emplace(
                robot->id, covarianceDerivative
            );
            evidence.upperDiniEpsilonRates.emplace(
                robot->id,
                3.0 * lambdaDini / (2.0 * std::sqrt(lambdaMax))
            );
            evidence.realizedEpsilonRates.emplace(
                robot->id,
                cbf2026::realizedEpsilonRate(certificate, derivatives)
            );
        }
        return evidence;
    }

    void emitControllerAbortEvidence(
        std::uint64_t frameIndex,
        const std::string& reason
    ) {
        if (!evidenceMode()
            || emittedEvidenceControllerFrames.count(frameIndex) != 0U) {
            return;
        }
        json nodes = json::array();
        for (const auto& robot : robots) {
            const bool solved = robot->opt.is_object()
                && robot->opt.value("status", "") == "success";
            json command = nullptr;
            if (solved) {
                command = vectorJson(robot->model->getControlInput());
            }
            nodes.push_back({
                {"robot_id", robot->id},
                {"snapshot_version", robot->certificateSnapshotVersion},
                {"optimization_status", solved
                    ? robot->opt.at("solver_info")
                        .value("status", "unknown")
                    : (robot->opt.is_object()
                        ? robot->opt.value("status", "not-attempted")
                        : "not-attempted")},
                {"applied_command", command},
                {"committed_hard_problem_id",
                 robot->committedCertificateState.valid
                    && robot->committedCertificateState.hardProblems.count(
                        robot->id
                    ) == 1U
                    ? cbf2026::canonicalHardConstraintProblemHash(
                        robot->committedCertificateState.hardProblems.at(
                            robot->id
                        )
                      )
                    : "unavailable"},
                {"consumed_hard_problem_id",
                 robot->lastConsumedHardProblemHash.empty()
                    ? "unavailable"
                    : robot->lastConsumedHardProblemHash}
            });
        }
        json row = evidenceBaseRecord("controller_interval", frameIndex);
        row["runtime"] = {
            {"snapshot_version", committedCertificateState.version},
            {"allocation_version", 1},
            {"nodes", nodes},
            {"expected_node_count", 14},
            {"expected_endpoint_row_count", 232},
            {"expected_reconstructed_row_count", 119},
            {"observed_endpoint_row_count",
             emittedEvidenceEndpointRowCounts[frameIndex]},
            {"abort_reason", reason},
            {"complete", false}
        };
        row["analyzer_only"] = json::object();
        evidenceStream->write(row);
        emittedEvidenceControllerFrames.insert(frameIndex);
    }

    void emitControllerEvidence(std::uint64_t frameIndex) {
        if (!evidenceMode()
            || emittedEvidenceControllerFrames.count(frameIndex) != 0U) {
            return;
        }
        if (!committedCertificateState.valid
            || committedCertificateState.endpoints.size() != 14U
            || committedCertificateState.hardProblems.size() != 14U) {
            throw std::runtime_error(
                "controller evidence requires a complete committed state"
            );
        }
        const auto authority = theoremGuardAuthority();
        std::map<cbf2026::EdgeId, cbf2026::HardEdgeAuthority>
            authorityByEdge;
        for (const auto& edge : authority.hardEdges) {
            authorityByEdge.emplace(edge.edge, edge);
        }
        struct OwnedRow {
            const cbf2026::HardConstraintRow* row;
            const cbf2026::HardConstraintProblem* problem;
        };
        std::vector<OwnedRow> ownedRows;
        for (const auto& [robotId, problem] :
             committedCertificateState.hardProblems) {
            (void)robotId;
            for (const auto& row : problem.rows) {
                ownedRows.push_back({&row, &problem});
            }
        }
        std::sort(
            ownedRows.begin(), ownedRows.end(),
            [](const OwnedRow& lhs, const OwnedRow& rhs) {
                if (lhs.row->edge != rhs.row->edge) {
                    return lhs.row->edge < rhs.row->edge;
                }
                return lhs.row->owner < rhs.row->owner;
            }
        );
        if (ownedRows.size() != 232U || authorityByEdge.size() != 119U) {
            throw std::runtime_error(
                "controller evidence hard-edge universe is incomplete"
            );
        }

        double localResidualMinimum =
            std::numeric_limits<double>::infinity();
        std::map<cbf2026::EdgeId, std::vector<cbf2026::EndpointRow>>
            rowsByEdge;
        for (const auto& owned : ownedRows) {
            const auto& row = *owned.row;
            const auto& edgeAuthority = authorityByEdge.at(row.edge);
            const auto& first =
                committedCertificateState.endpoints.at(row.edge.low);
            const Eigen::Vector2d secondPosition = row.edge.baseId >= 0
                ? edgeAuthority.basePosition
                : committedCertificateState.endpoints.at(
                    row.edge.high
                  ).estimate;
            const Eigen::Vector2d displacement =
                first.estimate - secondPosition;
            const double separation = displacement.norm();
            const Eigen::Vector2d normal = displacement / separation;
            const double alphaValue = edgeAuthority.classKCoefficient
                * std::pow(
                    row.postResetBarrier,
                    edgeAuthority.classKPower
                );
            const double allocation = row.edge.baseId >= 0 ? 1.0 : 0.5;
            const Eigen::VectorXd ownerCommand =
                robots.at(row.owner - 1)->model->getControlInput();
            const double residual =
                row.constant + row.coefficients.dot(ownerCommand);
            localResidualMinimum = std::min(
                localResidualMinimum, residual
            );
            rowsByEdge[row.edge].push_back({
                row.edge,
                row.owner,
                row.coefficients.head<2>(),
                row.constant,
                allocation,
                row.snapshotVersion,
                row.allocationVersion
            });

            json edge = edgeIdentityJson(row.edge);
            edge["threshold"] = edgeAuthority.threshold;
            edge["base_position"] = vectorJson(
                edgeAuthority.basePosition
            );
            edge["normal"] = vectorJson(normal);
            edge["separation"] = separation;
            edge["tightened_barrier"] = row.postResetBarrier;
            edge["class_k_coefficient"] =
                edgeAuthority.classKCoefficient;
            edge["class_k_power"] = edgeAuthority.classKPower;
            edge["alpha_value"] = alphaValue;

            json evidenceRow = evidenceBaseRecord(
                "endpoint_row", frameIndex
            );
            evidenceRow["edge"] = edge;
            evidenceRow["owner"] = row.owner;
            evidenceRow["allocation"] = allocation;
            evidenceRow["coefficient"] = vectorJson(Eigen::Vector2d(
                row.coefficients.head<2>()
            ));
            evidenceRow["constant"] = row.constant;
            evidenceRow["snapshot_version"] = row.snapshotVersion;
            evidenceRow["allocation_version"] = row.allocationVersion;
            evidenceRow["owner_applied_command"] = vectorJson(
                ownerCommand
            );
            evidenceRow["qp_status"] = robots.at(row.owner - 1)
                ->opt.at("solver_info").value("status", "unknown");
            evidenceRow["hard_problem_id"] =
                cbf2026::canonicalHardConstraintProblemHash(
                    *owned.problem
                );
            evidenceStream->write(evidenceRow);
            ++emittedEvidenceEndpointRowCounts[frameIndex];
#ifdef CBF_EVIDENCE_TEST_HOOKS
            const char* failAfterRows = std::getenv(
                "CBF_EVIDENCE_TEST_FAIL_AFTER_ENDPOINT_ROWS"
            );
            if (evidenceTestFixtureMode() && failAfterRows != nullptr) {
                char* end = nullptr;
                const unsigned long long requested = std::strtoull(
                    failAfterRows, &end, 10
                );
                if (end != failAfterRows && *end == '\0'
                    && requested > 0U
                    && emittedEvidenceEndpointRowCounts[frameIndex]
                        >= requested) {
                    throw std::runtime_error(
                        "injected evidence endpoint emission failure"
                    );
                }
            }
#endif
        }

        if (rowsByEdge.size() != 119U) {
            throw std::runtime_error(
                "controller evidence reconstructed-row universe is incomplete"
            );
        }
        double reconstructedResidualMinimum =
            std::numeric_limits<double>::infinity();
        for (const auto& [edge, endpointRows] : rowsByEdge) {
            const auto full = cbf2026::reconstructFullRow(endpointRows);
            const Eigen::Vector2d lowCommand = robots.at(edge.low - 1)
                ->model->getControlInput().head<2>();
            Eigen::Vector2d highCommand = Eigen::Vector2d::Zero();
            if (edge.baseId < 0) {
                highCommand = robots.at(edge.high - 1)
                    ->model->getControlInput().head<2>();
            }
            reconstructedResidualMinimum = std::min(
                reconstructedResidualMinimum,
                full.constant
                + full.coefficientI.dot(lowCommand)
                + full.coefficientJ.dot(highCommand)
            );
        }

        const auto rateEvidence = reconstructRateEvidenceState();
        std::optional<std::string> controllerSchemaVersion = std::nullopt;
        if (config.contains("qualified-controller")
            && config["qualified-controller"].is_object()
            && config["qualified-controller"].contains("schema-version")
            && config["qualified-controller"]["schema-version"].is_string()) {
            controllerSchemaVersion =
                config["qualified-controller"]["schema-version"]
                    .get<std::string>();
        }
        json nodes = json::array();
        double maximumVx = 0.0;
        double maximumVy = 0.0;
        double maximumYaw = 0.0;
        for (Robot* robot : theoremTopologicalOrder()) {
            const auto& input = rateEvidence.inputs.at(robot->id);
            const auto& certificate =
                committedCertificateState.certificates.at(robot->id);
            const auto& endpoint =
                committedCertificateState.endpoints.at(robot->id);
            const auto& problem =
                committedCertificateState.hardProblems.at(robot->id);
            const Eigen::VectorXd command =
                robot->model->getControlInput();
            maximumVx = std::max(maximumVx, std::abs(command[0]));
            maximumVy = std::max(maximumVy, std::abs(command[1]));
            maximumYaw = std::max(maximumYaw, std::abs(command[2]));
            json references = json::array();
            for (const auto& reference : input.references) {
                references.push_back({
                    {"canonical_reference_id",
                     reference.predecessor.referenceId},
                    {"reference_kind",
                     reference.predecessor.hoveringBase ? "base" : "uav"},
                    {"direction", vectorJson(reference.direction)},
                    {"distance", reference.distance},
                    {"ranging_variance", reference.rangingVariance},
                    {"predecessor_local_index",
                     reference.predecessor.localIndex},
                    {"predecessor_snapshot_version",
                     reference.predecessor.snapshotVersion},
                    {"predecessor_covariance",
                     matrixJson(reference.predecessor.covariance)},
                    {"predecessor_covariance_rate_bound",
                     reference.predecessor.covarianceRateBound},
                    {"predecessor_speed_bound",
                     reference.predecessor.speedBound}
                });
            }
            const std::string committedProblemId =
                cbf2026::canonicalHardConstraintProblemHash(problem);
            const auto hardOnly =
                robot->solveLocalHardQpEvidence(problem);
            json hardInteriorSelection = robot->opt.value(
                "hard_interior_selection", json()
            );
            cbf2026::diagnostics::validateHardInteriorSelectionEvidence(
                controllerSchemaVersion, hardInteriorSelection
            );
            if (!hardInteriorSelection.is_null()) {
                double minimumOriginalHardResidual =
                    std::numeric_limits<double>::infinity();
                for (const auto& hardRow : problem.rows) {
                    minimumOriginalHardResidual = std::min(
                        minimumOriginalHardResidual,
                        hardRow.constant + hardRow.coefficients.dot(command)
                    );
                }
                if (!std::isfinite(minimumOriginalHardResidual)) {
                    throw std::runtime_error(
                        "controller evidence original hard residual is unavailable"
                    );
                }
                hardInteriorSelection["minimum_original_hard_residual_mps"] =
                    minimumOriginalHardResidual;
            }
            double normalMinimum =
                std::numeric_limits<double>::infinity();
            for (const auto& bound : problem.bounds) {
                normalMinimum = std::min(
                    normalMinimum,
                    bound.coefficient * command[bound.controlIndex]
                    + bound.limit
                );
            }
            for (const auto& hardRow : problem.rows) {
                normalMinimum = std::min(
                    normalMinimum,
                    hardRow.constant
                    + hardRow.coefficients.dot(command)
                );
            }
            json evidenceNode = {
                {"robot_id", robot->id},
                {"local_index", robot->idInMyPart},
                {"snapshot_version", endpoint.snapshotVersion},
                {"allocation_version", endpoint.allocationVersion},
                {"interface_estimate", vectorJson(endpoint.estimate)},
                {"node_component_bound", input.planarComponentMax},
                {"references", references},
                {"information", matrixJson(certificate.information)},
                {"covariance", matrixJson(certificate.covariance)},
                {"covariance_derivative", matrixJson(
                    rateEvidence.covarianceDerivatives.at(robot->id)
                )},
                {"covariance_rate_bound",
                 certificate.covarianceRateBound},
                {"epsilon", certificate.epsilon},
                {"dplus_epsilon",
                 rateEvidence.upperDiniEpsilonRates.at(robot->id)},
                {"nu_inst",
                 rateEvidence.realizedEpsilonRates.at(robot->id)},
                {"bar_nu", certificate.epsilonRateBound},
                {"applied_command", vectorJson(command)},
                {"normal_qp", {
                    {"status", robot->opt.at("solver_info")
                        .value("status", "unknown")},
                    {"minimum_residual", normalMinimum},
                    {"hard_problem_id", robot->lastConsumedHardProblemHash},
                    {"solution", vectorJson(command)}
                }},
                {"hard_only_qp", {
                    {"status", hardOnly.feasibility.status},
                    {"minimum_residual", hardOnly.feasibility.minimumResidual},
                    {"hard_problem_id", hardOnly.feasibility.hardProblemHash},
                    {"solution", hardOnly.solution.has_value()
                        ? vectorJson(*hardOnly.solution) : json(nullptr)}
                }},
                {"normal_problem", hardProblemJson(problem)},
                {"hard_only_problem", hardProblemJson(problem)},
                {"committed_hard_problem_id", committedProblemId},
                {"consumed_hard_problem_id",
                 robot->lastConsumedHardProblemHash}
            };
            if (!hardInteriorSelection.is_null()) {
                evidenceNode["hard_interior_selection"] =
                    hardInteriorSelection;
            }
            nodes.push_back(evidenceNode);
        }

        bool resetAttempted = false;
        bool resetCommitted = false;
        std::string resetStatus = "not-attempted";
        for (const auto& transaction : certificateResetHistory) {
            if (transaction.frameIndex == frameIndex) {
                resetAttempted = true;
                resetCommitted = transaction.status
                    == cbf2026::GuardStatus::Accepted;
                resetStatus = resetCommitted ? "accepted" : "rejected";
                break;
            }
        }
        json truth = json::array();
        for (const auto& robot : robots) {
            const Point position = robot->model->xy();
            truth.push_back({
                {"robot_id", robot->id},
                {"position", {position.x, position.y}}
            });
        }
        json controller = evidenceBaseRecord(
            "controller_interval", frameIndex
        );
        controller["runtime"] = {
            {"snapshot_version", committedCertificateState.version},
            {"allocation_version", 1},
            {"nodes", nodes},
            {"expected_node_count", 14},
            {"expected_endpoint_row_count", 232},
            {"expected_reconstructed_row_count", 119},
            {"observed_endpoint_row_count", ownedRows.size()},
            {"complete_finite_snapshot", true},
            {"reset", {
                {"attempted", resetAttempted},
                {"guard_status", resetStatus},
                {"committed", resetCommitted}
            }},
            {"component_maxima", {
                {"vx", maximumVx},
                {"vy", maximumVy},
                {"yaw_rate", maximumYaw}
            }},
            {"local_residual_minimum", localResidualMinimum},
            {"reconstructed_residual_minimum",
             reconstructedResidualMinimum},
            {"abort_reason", ""},
            {"complete", true}
        };
        controller["analyzer_only"] = {{"truth", truth}};
        evidenceStream->write(controller);
        emittedEvidenceControllerFrames.insert(frameIndex);
        completedEvidenceControllerFrames.insert(frameIndex);
    }

    void emitMissionTerminal(bool success, const std::string& reason) {
        if (!evidenceMode()) {
            return;
        }
        const double timeStep = config.at("execute")
            .at("time-step").get<double>();
        const std::uint64_t declaredFrames = static_cast<std::uint64_t>(
            std::llround(
                config.at("execute").at("time-total").get<double>()
                / timeStep
            )
        );
        json row = evidenceBaseRecord("mission_terminal", declaredFrames);
        row["runtime"] = {
            {"success", success},
            {"reason", reason},
            {"process_outcome", reason},
            {"declared_frames", declaredFrames},
            {"completed_intervals",
             completedEvidenceControllerFrames.size()}
        };
        evidenceStream->write(row);
    }

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
        std::ostream& diagnostics = evidenceMode()
            ? std::cerr : std::cout;
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
                        diagnostics << "\n[Constraint Violation] Robot " << robot->id
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
                        diagnostics << "\n[Constraint Violation] Robot " << robot->id
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
