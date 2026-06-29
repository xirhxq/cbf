#ifndef CBF_SWARM_HPP
#define CBF_SWARM_HPP

#include "Robot.hpp"
#include "bridge/BridgeExperiment.hpp"
#include "bridge/BridgeSearchTracker.hpp"
#include "bridge/BridgeTopology.hpp"


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
    std::unordered_map<std::string, SecondOrderCBF> secondOrderCbfNoSlack;
    json opt;
    BridgeExperimentConfig bridgeConfig;
    BridgeSearchTracker bridgeSearch;
    BridgeTopologyConfig bridgeTopologyConfig;
    BridgeTopologyDecision bridgeTopologyDecision;
    bool bridgeHasPreviousSupportChainMargin = false;
    double bridgePreviousSupportChainMinMargin = std::numeric_limits<double>::infinity();
    double bridgeCurrentEffectiveReserveMargin = 0.0;
public:
    Swarm(json &settings)
            : config(settings),
              n(settings["num"]),
              gridWorldGroundTruth(settings["world"]){
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
        bridgeConfig = loadBridgeExperimentConfig(config);
        if (bridgeConfig.enabled) {
            bridgeSearch = BridgeSearchTracker(config, bridgeConfig);
            bridgeTopologyConfig = loadBridgeTopologyConfig(config);
        }
    }

    void output() {
        printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, robots[0]->runtime);
        for (auto &robot: robots) {
            robot->output();
        }
        printf("--------------\n");
    }

    void endLog() {
        if (bridgeConfig.enabled) {
            data["bridge"]["final_search_map"] = bridgeSearch.finalMapJson();
        }
        ofstream << std::fixed << std::setprecision(6) << data;
        ofstream.close();
    }

    void logParams() {
        data["para"] = robots[0]->getParams();
        data["config"] = config;
        if (bridgeConfig.enabled) {
            data["bridge"]["metadata"] = makeBridgeMetadata(config, bridgeConfig);
        }
    }

    void logOnce() {
        stepData["runtime"] = robots[0]->runtime;
        if (bridgeConfig.enabled) {
            stepData["bridge"]["row"] = bridgeConfig.row;
        }
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
        updateBridgeSearchLog();
        if (bridgeConfig.enabled) {
            stepData["bridge"]["topology"] = bridgeTopologyDecision.log;
        }

        if (isCentralizedExecution()) {
            json centralizedData = opt;

            json cbfValues = json::object();
            auto x = centralizedModel->getX();

            for (const auto& [name, cbf] : cbfNoSlack.cbfs) {
                double value = cbf.h(x, robots[0]->runtime);
                cbfValues[name] = value;
            }

            for (const auto& [name, cbf] : cbfSlack) {
                double value = cbf.h(x, robots[0]->runtime);
                cbfValues[name] = value;
            }

            for (const auto& [name, cbf] : secondOrderCbfNoSlack) {
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
            auto acc2d = robot->model->getAcceleration();
            double yawRad = robot->model->getStateVariable("yawRad");
            double batteryLevel = robot->model->getStateVariable("battery");
            Eigen::Matrix2d positionCovariance = robot->positionCovariance;

            for (auto &otherRobot: robots) {
                otherRobot->comm->receivePosition2D(robot->id, pos2d);
                otherRobot->comm->receiveVelocity2D(robot->id, vel2d);
                otherRobot->comm->receiveAcceleration2D(robot->id, acc2d);
                otherRobot->comm->receiveYawRad(robot->id, yawRad);
                otherRobot->comm->receiveBatteryLevel(robot->id, batteryLevel);
                otherRobot->comm->receivePositionCovariance(robot->id, positionCovariance);
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
        exchangeData();
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

        auto settings = config["execute"];

        double tTotal = settings["time-total"], tStep = settings["time-step"];
        while (robots[0]->runtime < tTotal) {
            try {
                exchangeData();
                checkInformationExchange();
                for (auto &robot: robots) robot->checkRobotsInsideWorld();
                printf("\r%.2lf seconds elapsed... %.2lf%%", robots[0]->runtime, gridWorldGroundTruth.getPercentage() * 100);
                for (auto &robot: robots) robot->updateGridWorld();
                updateGridWorld();
                checkUpdatedGridWorld();
                applyBridgeTopology();
                applyBridgeNominalControls();
                for (auto &robot: robots) robot->postsetCBF();
                applyBridgeNominalFeasibilityGuard();

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
                        logOnce();
                        std::cout << "\n[Simulation Terminated] Constraint violation detected at t=" << robots[0]->runtime << "s" << std::endl;
                        break;
                    }
                }

                logOnce();
                for (auto &robot: robots) robot->stepTimeForward(tStep);
            }
            catch (...) {
                for (auto &robot: robots) robot->updateGridWorld();
                logOnce();
                endLog();
                printf("Data so far has been saved in %s\n", filename.c_str());
                try {
                    throw;
                }
                catch (std::exception &e) {
                    std::cerr << e.what() << std::endl;
                }
                catch (...) {
                    std::cerr << "Unknown error" << std::endl;
                }
                break;
            }
        }

        printf("\nAfter %.4lf seconds\n", robots[0]->runtime);
        output();
        endLog();

        printf("Data saved in %s\n", filename.c_str());
        std::cout << "[OUTPUT_DIR] " << outputDir << std::endl;

    }

private:
    void updateBridgeSearchLog() {
        if (!bridgeConfig.enabled) {
            return;
        }
        bridgeSearch.observeCells(updatedGridWorldGroundTruth, robots[0]->runtime);
        std::vector<Point> positions;
        for (auto &robot : robots) {
            positions.push_back(robot->model->xy());
        }
        bridgeSearch.observeRobots(positions, robots[0]->runtime);
        stepData["bridge"]["search"] = bridgeSearch.snapshot();
    }

    void applyBridgeTopology() {
        if (!bridgeConfig.enabled) {
            return;
        }
        std::map<int, Point> positions;
        for (auto &robot : robots) {
            positions[robot->id] = robot->model->xy();
        }
        bridgeTopologyDecision = chooseBridgeTopology(positions, bridgeTopologyConfig, bridgeConfig.topologyPolicy);
        for (auto &robot : robots) {
            auto it = bridgeTopologyDecision.anchorIds.find(robot->id);
            if (it == bridgeTopologyDecision.anchorIds.end()) {
                robot->clearBridgeFormationOverride();
                continue;
            }
            std::vector<int> baseIds;
            if (robot->id == 1 && !robot->bases.empty()) {
                baseIds.push_back(0);
            }
            robot->setBridgeFormationOverride(it->second, baseIds);
        }
    }

    void applyBridgeNominalControls() {
        if (!bridgeConfig.enabled) {
            return;
        }
        double dt = config.at("execute").at("time-step").get<double>();
        double maxSpeed = config.at("bridge").at("nominal").value("max-speed", 8.0);
        double maxAcceleration = config.at("bridge").at("nominal").value("max-acceleration", 2.0);
        double maxYawRate = config.at("bridge").at("nominal").value("max-yaw-rate", 0.35);

        if (bridgeTopologyConfig.failSafeHold) {
            stepData["bridge"]["nominal"]["fail_safe"] = {
                {"enabled", true},
                {"active", bridgeTopologyDecision.failSafe},
                {"mode", "hold"}
            };
        }
        if (bridgeTopologyConfig.failSafeHold && bridgeTopologyDecision.failSafe) {
            for (auto &robot : robots) {
                robot->setNominalControlOverride(Eigen::VectorXd::Zero(robot->model->uSize()));
            }
            return;
        }

        std::map<int, Point> positions;
        for (auto &robot : robots) {
            positions[robot->id] = robot->model->xy();
        }
        std::map<int, Eigen::VectorXd> goalDiversionVelocities;
        if (bridgeConfig.goalDiversionEnabled) {
            for (auto &robot : robots) {
                goalDiversionVelocities[robot->id] = robot->model->getVelocity();
            }
        }
        std::map<int, Point> goalDiversionOffsets;
        json goalDiversionLinks = json::array();
        int goalDiversionActiveCount = 0;
        if (bridgeConfig.goalDiversionEnabled) {
            std::vector<int> orderedIds;
            orderedIds.reserve(positions.size());
            for (const auto &entry : positions) {
                orderedIds.push_back(entry.first);
            }
            for (size_t i = 0; i < orderedIds.size(); ++i) {
                for (size_t j = i + 1; j < orderedIds.size(); ++j) {
                    int idA = orderedIds[i];
                    int idB = orderedIds[j];
                    if (bridgeConfig.goalDiversionPairScope == "named-pair"
                        && !(idA == bridgeConfig.goalDiversionPairIdA
                             && idB == bridgeConfig.goalDiversionPairIdB)
                        && !(idA == bridgeConfig.goalDiversionPairIdB
                             && idB == bridgeConfig.goalDiversionPairIdA)) {
                        continue;
                    }
                    Point posA = positions[idA];
                    Point posB = positions[idB];
                    Point rel = posB - posA;
                    double dist = rel.len();
                    if (dist < 1.0e-9 || dist >= bridgeConfig.goalDiversionDistance) {
                        continue;
                    }
                    Point unit = rel / dist;
                    const Eigen::VectorXd &velA = goalDiversionVelocities[idA];
                    const Eigen::VectorXd &velB = goalDiversionVelocities[idB];
                    double radialA = unit.x * velA(0) + unit.y * velA(1);
                    double radialB = -unit.x * velB(0) - unit.y * velB(1);
                    double closingRate = -(radialA + radialB) / 2.0;
                    if (closingRate > -bridgeConfig.goalDiversionRadial) {
                        continue;
                    }
                    int divergeId = radialA < radialB ? idA : idB;
                    Point divergePos = positions[divergeId];
                    Point awayDir = (divergePos - (divergeId == idA ? posB : posA));
                    double awayLen = awayDir.len();
                    if (awayLen < 1.0e-9) {
                        continue;
                    }
                    Point awayUnit = awayDir / awayLen;
                    double excess = bridgeConfig.goalDiversionDistance - dist;
                    double magnitude = bridgeConfig.goalDiversionSeparationScale
                                       * (excess + std::max(0.0, -closingRate) * 2.0);
                    if (magnitude > bridgeConfig.goalDiversionMaxOffset) {
                        magnitude = bridgeConfig.goalDiversionMaxOffset;
                    }
                    Point offset = awayUnit * magnitude;
                    auto existing = goalDiversionOffsets.find(divergeId);
                    if (existing == goalDiversionOffsets.end()
                        || offset.len() > existing->second.len()) {
                        goalDiversionOffsets[divergeId] = offset;
                    }
                    ++goalDiversionActiveCount;
                    goalDiversionLinks.push_back({
                        {"pair_id_a", idA},
                        {"pair_id_b", idB},
                        {"diverged_id", divergeId},
                        {"distance", dist},
                        {"radial_a", radialA},
                        {"radial_b", radialB},
                        {"closing_rate", closingRate},
                        {"offset", {{"x", offset.x}, {"y", offset.y}}}
                    });
                }
            }
            stepData["bridge"]["nominal"]["goal_diversion"] = {
                {"enabled", true},
                {"active_count", goalDiversionActiveCount},
                {"distance_threshold", bridgeConfig.goalDiversionDistance},
                {"radial_threshold", bridgeConfig.goalDiversionRadial},
                {"separation_scale", bridgeConfig.goalDiversionSeparationScale},
                {"pair_scope", bridgeConfig.goalDiversionPairScope},
                {"links", goalDiversionLinks}
            };
        }
        int relayId = positions.size() >= 2 ? std::next(positions.begin())->first : -1;
        double relaySupportMargin = bridgeRelaySupportMargin(positions, bridgeTopologyConfig);
        bool relaySupportGuardActive = bridgeConfig.relaySupportGuardEnabled
                                       && relayId >= 0
                                       && std::isfinite(relaySupportMargin)
                                       && relaySupportMargin < bridgeConfig.relaySupportRobustMargin;
        Point relaySupportGoal = relaySupportGuardActive
                                 ? bridgeRelaySupportGoal(
                                     positions,
                                     bridgeTopologyConfig,
                                     bridgeConfig.relaySupportRobustMargin)
                                 : Point(0.0, 0.0);
        if (bridgeConfig.relaySupportGuardEnabled) {
            stepData["bridge"]["nominal"]["relay_support_guard"] = {
                {"enabled", true},
                {"active", relaySupportGuardActive},
                {"robust_margin", relaySupportMargin},
                {"required_margin", bridgeConfig.relaySupportRobustMargin},
                {"goal", {{"x", relaySupportGoal.x}, {"y", relaySupportGoal.y}}}
            };
        }

        int supportChainActiveCount = 0;
        double supportChainMinMarginBefore = std::numeric_limits<double>::infinity();
        double supportChainMinMarginAfter = std::numeric_limits<double>::infinity();
        json supportChainLinks = json::array();
        bool predictiveGateEnabled = bridgeConfig.searchPolicy == "active-predictive"
                                     || bridgeConfig.searchPolicy == "active-predictive-exposure";
        bool allActiveSupportEdges = bridgeConfig.supportChainGuardScope == "all-active-edges";
        int predictiveGateActiveCount = 0;
        int predictiveGateEvaluatedCandidates = 0;
        int predictiveGatePenalizedCandidates = 0;
        double predictiveGateMaxPenalty = 0.0;
        double predictiveGateMinSelectedRobustMargin = std::numeric_limits<double>::infinity();
        json predictiveGateLinks = json::array();
        auto predictBridgeNominalStep = [&](Robot *robot, const Point &position, const Point &goal) {
            Point direction = goal - position;
            double distance = direction.len();
            Point unit = distance > 1e-9 ? direction / distance : Point(0.0, 0.0);
            if (config.value("model", "") == "DoubleIntegrate2D") {
                Eigen::VectorXd velocity = robot->model->getVelocity();
                Eigen::VectorXd desiredVelocity(2);
                desiredVelocity << maxSpeed * unit.x, maxSpeed * unit.y;
                Eigen::VectorXd accel = (desiredVelocity - velocity) / std::max(dt, 1e-6);
                double accelNorm = accel.norm();
                if (accelNorm > maxAcceleration && accelNorm > 1e-9) {
                    accel *= maxAcceleration / accelNorm;
                }
                return Point(
                    position.x + velocity(0) * dt + 0.5 * accel(0) * dt * dt,
                    position.y + velocity(1) * dt + 0.5 * accel(1) * dt * dt);
            }
            return position + unit * (maxSpeed * dt);
        };
        auto chooseGoalForPrediction = [&](Robot *candidate, const Point &position) {
            Point predictionGoal = bridgeSearch.chooseGoal(position, bridgeConfig.searchPolicy);
            if (predictiveGateEnabled) {
                auto anchorIt = bridgeTopologyDecision.anchorIds.find(candidate->id);
                if (anchorIt != bridgeTopologyDecision.anchorIds.end() && !anchorIt->second.empty()) {
                    int anchorId = anchorIt->second.front();
                    auto anchorPositionIt = positions.find(anchorId);
                    if (anchorPositionIt != positions.end()) {
                        predictionGoal = bridgeSearch.chooseGoal(
                            position,
                            bridgeConfig.searchPolicy,
                            anchorPositionIt->second,
                            bridgeTopologyConfig);
                    }
                }
            }
            if (relaySupportGuardActive && candidate->id == relayId) {
                predictionGoal = relaySupportGoal;
            }
            return predictionGoal;
        };

        double effectiveSupportChainMargin = bridgeConfig.supportChainRobustMargin;
        double stepSupportChainMinMarginBefore = std::numeric_limits<double>::infinity();
        double stepSupportChainClosingRate = 0.0;
        if (bridgeConfig.supportChainGuardEnabled
            && bridgeConfig.supportChainStateReserveEnabled) {
            bool sdAllActive = bridgeConfig.supportChainGuardScope == "all-active-edges";
            for (auto &robot : robots) {
                auto anchorIt = bridgeTopologyDecision.anchorIds.find(robot->id);
                if (anchorIt == bridgeTopologyDecision.anchorIds.end()) {
                    continue;
                }
                if (!sdAllActive && anchorIt->second.empty()) {
                    continue;
                }
                Point childPosition = robot->model->xy();
                if (sdAllActive && robot->id == 1 && !robot->bases.empty()) {
                    double margin = bridgeRobustMargin(robot->bases.front(), childPosition, bridgeTopologyConfig);
                    stepSupportChainMinMarginBefore =
                        std::min(stepSupportChainMinMarginBefore, margin);
                }
                for (int anchorId : anchorIt->second) {
                    if (anchorId <= 0 || anchorId > static_cast<int>(robots.size())) {
                        continue;
                    }
                    auto anchorPositionIt = positions.find(anchorId);
                    if (anchorPositionIt == positions.end()) {
                        continue;
                    }
                    double margin = bridgeRobustMargin(anchorPositionIt->second, childPosition, bridgeTopologyConfig);
                    stepSupportChainMinMarginBefore =
                        std::min(stepSupportChainMinMarginBefore, margin);
                }
            }
            if (!std::isfinite(stepSupportChainMinMarginBefore)) {
                stepSupportChainMinMarginBefore = bridgeConfig.supportChainStateReserveTightenMargin;
            }
            if (bridgeHasPreviousSupportChainMargin) {
                stepSupportChainClosingRate =
                    bridgePreviousSupportChainMinMargin - stepSupportChainMinMarginBefore;
            }
            double base = bridgeConfig.supportChainStateReserveBaseMargin;
            double headroom = bridgeConfig.supportChainStateReserveHeadroom;
            double tighten = bridgeConfig.supportChainStateReserveTightenMargin;
            double gap = std::max(0.0, tighten - stepSupportChainMinMarginBefore);
            double tightenScale = tighten > 0.0 ? std::min(1.0, gap / tighten) : 0.0;
            double closingTerm = std::max(0.0, stepSupportChainClosingRate)
                                 * bridgeConfig.supportChainStateReserveClosingRateGain;
            double effective = base + headroom * tightenScale + closingTerm;
            effective = std::min(effective, bridgeConfig.supportChainStateReserveMaxMargin);
            effective = std::max(effective, base);
            effectiveSupportChainMargin = effective;
            bridgeCurrentEffectiveReserveMargin = effective;
        }

        for (auto &robot : robots) {
            Point position = robot->model->xy();
            Point goal = bridgeSearch.chooseGoal(position, bridgeConfig.searchPolicy);
            if (predictiveGateEnabled) {
                auto anchorIt = bridgeTopologyDecision.anchorIds.find(robot->id);
                if (anchorIt != bridgeTopologyDecision.anchorIds.end() && !anchorIt->second.empty()) {
                    int anchorId = anchorIt->second.front();
                    auto anchorPositionIt = positions.find(anchorId);
                    if (anchorPositionIt != positions.end()) {
                        BridgeSearchGoalDecision decision = bridgeSearch.chooseGoalWithDiagnostics(
                            position,
                            bridgeConfig.searchPolicy,
                            anchorPositionIt->second,
                            bridgeTopologyConfig);
                        goal = decision.goal;
                        predictiveGateEvaluatedCandidates += decision.evaluatedCandidates;
                        predictiveGatePenalizedCandidates += decision.penalizedCandidates;
                        predictiveGateMaxPenalty = std::max(predictiveGateMaxPenalty, decision.maxPenalty);
                        if (std::isfinite(decision.selectedMinRobustMargin)) {
                            predictiveGateMinSelectedRobustMargin = std::min(
                                predictiveGateMinSelectedRobustMargin,
                                decision.selectedMinRobustMargin);
                        }
                        if (decision.predictiveChangedGoal || decision.exposureChangedGoal
                            || decision.beliefConcentrationChangedGoal) {
                            ++predictiveGateActiveCount;
                        }
                        json link = {
                            {"robot", robot->id},
                            {"anchor", anchorId},
                            {"active", decision.predictiveChangedGoal || decision.exposureChangedGoal
                                || decision.beliefConcentrationChangedGoal},
                            {"predictive_active", decision.predictiveChangedGoal},
                            {"exposure_active", decision.exposureChangedGoal},
                            {"belief_concentration_active", decision.beliefConcentrationChangedGoal},
                            {"selected_penalty", decision.selectedPenalty},
                            {"max_penalty", decision.maxPenalty},
                            {"selected_exposure_utility", decision.selectedExposureUtility},
                            {"max_exposure_utility", decision.maxExposureUtility},
                            {"belief_concentration_enabled", decision.beliefConcentrationEnabled},
                            {"selected_belief_concentration_utility", decision.selectedBeliefConcentrationUtility},
                            {"max_belief_concentration_utility", decision.maxBeliefConcentrationUtility},
                            {"belief_centroid_x", decision.beliefCentroidX},
                            {"belief_centroid_y", decision.beliefCentroidY},
                            {"top_belief_mass", decision.topBeliefMass},
                            {"peak_searched_fraction", decision.peakSearchedFraction},
                            {"exposure_service_gate_enabled", decision.exposureServiceGateEnabled},
                            {"exposure_service_schedule_enabled", decision.exposureServiceScheduleEnabled},
                            {"service_schedule_due", decision.serviceScheduleDue},
                            {"selected_exposure_service_eligible", decision.selectedExposureServiceEligible},
                            {"selected_service_utility", decision.selectedServiceUtility},
                            {"max_service_utility", decision.maxServiceUtility},
                            {"required_service_utility", decision.requiredServiceUtility},
                            {"searched_cells", decision.searchedCells},
                            {"required_searched_cells", decision.requiredSearchedCells},
                            {"service_schedule_deficit", decision.serviceScheduleDeficit},
                            {"service_rejected_candidates", decision.serviceRejectedCandidates},
                            {"selected_boundary_deficit", decision.selectedBoundaryDeficit},
                            {"required_robust_margin", decision.requiredRobustMargin},
                            {"evaluated_candidates", decision.evaluatedCandidates},
                            {"penalized_candidates", decision.penalizedCandidates},
                            {"baseline_goal", {{"x", decision.baselineGoal.x}, {"y", decision.baselineGoal.y}}},
                            {"selected_goal", {{"x", decision.goal.x}, {"y", decision.goal.y}}}
                        };
                        link["selected_min_robust_margin"] = std::isfinite(decision.selectedMinRobustMargin)
                                                             ? json(decision.selectedMinRobustMargin)
                                                             : json(nullptr);
                        predictiveGateLinks.push_back(link);
                    }
                }
            }
            if (relaySupportGuardActive && robot->id == relayId) {
                goal = relaySupportGoal;
            }
            if (bridgeConfig.supportChainGuardEnabled) {
                auto anchorIt = bridgeTopologyDecision.anchorIds.find(robot->id);
                if (anchorIt != bridgeTopologyDecision.anchorIds.end()
                    && (allActiveSupportEdges || !anchorIt->second.empty())) {
                    struct SupportAnchorPrediction {
                        int id;
                        Point current;
                        Point predicted;
                    };
                    std::vector<SupportAnchorPrediction> supportAnchors;
                    if (allActiveSupportEdges && robot->id == 1 && !robot->bases.empty()) {
                        supportAnchors.push_back({0, robot->bases.front(), robot->bases.front()});
                    }
                    if (allActiveSupportEdges) {
                        for (int anchorId : anchorIt->second) {
                            auto anchorPositionIt = positions.find(anchorId);
                            if (anchorPositionIt == positions.end()
                                || anchorId <= 0
                                || anchorId > static_cast<int>(robots.size())) {
                                continue;
                            }
                            Point anchorGoal = chooseGoalForPrediction(robots[anchorId - 1].get(), anchorPositionIt->second);
                            Point anchorPredicted = predictBridgeNominalStep(
                                robots[anchorId - 1].get(),
                                anchorPositionIt->second,
                                anchorGoal);
                            supportAnchors.push_back({anchorId, anchorPositionIt->second, anchorPredicted});
                        }
                    } else if (!anchorIt->second.empty()) {
                        int anchorId = anchorIt->second.front();
                        auto anchorPositionIt = positions.find(anchorId);
                        if (anchorPositionIt != positions.end()) {
                            supportAnchors.push_back({anchorId, anchorPositionIt->second, anchorPositionIt->second});
                        }
                    }
                    for (const auto &supportAnchor : supportAnchors) {
                        Point predictedBefore = predictBridgeNominalStep(robot.get(), position, goal);
                        Point gatedGoal = allActiveSupportEdges
                                          ? bridgePredictiveMovingAnchorSupportGoal(
                                              supportAnchor.current,
                                              supportAnchor.predicted,
                                              position,
                                              predictedBefore,
                                              goal,
                                              bridgeTopologyConfig,
                                              effectiveSupportChainMargin)
                                          : bridgePredictiveSupportChainGoal(
                                              supportAnchor.current,
                                              position,
                                              predictedBefore,
                                              goal,
                                              bridgeTopologyConfig,
                                              effectiveSupportChainMargin);
                        Point predictedAfter = predictBridgeNominalStep(robot.get(), position, gatedGoal);
                        Point marginAnchor = allActiveSupportEdges ? supportAnchor.predicted : supportAnchor.current;
                        double marginBefore = bridgeRobustMargin(marginAnchor, predictedBefore, bridgeTopologyConfig);
                        double marginAfter = bridgeRobustMargin(marginAnchor, predictedAfter, bridgeTopologyConfig);
                        if (allActiveSupportEdges && marginAfter < effectiveSupportChainMargin) {
                            Point fallbackGoal = supportAnchor.predicted;
                            Point fallbackPredicted = predictBridgeNominalStep(robot.get(), position, fallbackGoal);
                            BridgeOneStepSupportGoal selectedGoal = bridgeChooseBetterOneStepSupportGoal(
                                marginAnchor,
                                gatedGoal,
                                predictedAfter,
                                fallbackGoal,
                                fallbackPredicted,
                                bridgeTopologyConfig);
                            if (selectedGoal.improved) {
                                gatedGoal = selectedGoal.goal;
                                predictedAfter = selectedGoal.predicted;
                                marginAfter = selectedGoal.margin;
                            }
                        }
                        supportChainMinMarginBefore = std::min(supportChainMinMarginBefore, marginBefore);
                        supportChainMinMarginAfter = std::min(supportChainMinMarginAfter, marginAfter);
                        bool active = gatedGoal.distance_to(goal) > 1.0e-6;
                        if (active) {
                            ++supportChainActiveCount;
                        }
                        if (active || allActiveSupportEdges) {
                            supportChainLinks.push_back({
                                {"robot", robot->id},
                                {"anchor", supportAnchor.id},
                                {"active", active},
                                {"margin_before", marginBefore},
                                {"margin_after", marginAfter},
                                {"anchor_predicted", {{"x", supportAnchor.predicted.x}, {"y", supportAnchor.predicted.y}}},
                                {"predicted_before", {{"x", predictedBefore.x}, {"y", predictedBefore.y}}},
                                {"predicted_after", {{"x", predictedAfter.x}, {"y", predictedAfter.y}}},
                                {"goal_before", {{"x", goal.x}, {"y", goal.y}}},
                                {"goal_after", {{"x", gatedGoal.x}, {"y", gatedGoal.y}}}
                            });
                        }
                        if (active) {
                            goal = gatedGoal;
                        }
                    }
                }
            }
            if (bridgeConfig.goalDiversionEnabled) {
                auto divertIt = goalDiversionOffsets.find(robot->id);
                if (divertIt != goalDiversionOffsets.end()) {
                    goal = goal + divertIt->second;
                }
            }
            Point direction = goal - position;
            double distance = direction.len();
            Point unit = distance > 1e-9 ? direction / distance : Point(0.0, 0.0);
            double desiredYaw = std::atan2(unit.y, unit.x);
            double yaw = robot->model->getStateVariable("yawRad");
            double yawError = std::atan2(std::sin(desiredYaw - yaw), std::cos(desiredYaw - yaw));
            double yawRate = std::max(-maxYawRate, std::min(maxYawRate, yawError / std::max(dt, 1e-6)));

            Eigen::VectorXd nominal = Eigen::VectorXd::Zero(robot->model->uSize());
            if (config.value("model", "") == "DoubleIntegrate2D") {
                Eigen::VectorXd velocity = robot->model->getVelocity();
                Eigen::VectorXd desiredVelocity(2);
                desiredVelocity << maxSpeed * unit.x, maxSpeed * unit.y;
                Eigen::VectorXd accel = (desiredVelocity - velocity) / std::max(dt, 1e-6);
                double accelNorm = accel.norm();
                if (accelNorm > maxAcceleration && accelNorm > 1e-9) {
                    accel *= maxAcceleration / accelNorm;
                }
                nominal(0) = accel(0);
                nominal(1) = accel(1);
                if (nominal.size() > 2) {
                    nominal(2) = yawRate;
                }
            } else {
                nominal(0) = maxSpeed * unit.x;
                nominal(1) = maxSpeed * unit.y;
                if (nominal.size() > 2) {
                    nominal(2) = yawRate;
                }
            }
            robot->setNominalControlOverride(nominal);
        }
        if (predictiveGateEnabled) {
            stepData["bridge"]["nominal"]["predictive_active_gate"] = {
                {"enabled", true},
                {"active_count", predictiveGateActiveCount},
                {"max_penalty", predictiveGateMaxPenalty},
                {"evaluated_candidates", predictiveGateEvaluatedCandidates},
                {"penalized_candidates", predictiveGatePenalizedCandidates},
                {"links", predictiveGateLinks}
            };
            stepData["bridge"]["nominal"]["predictive_active_gate"]["min_selected_robust_margin"] =
                std::isfinite(predictiveGateMinSelectedRobustMargin)
                ? json(predictiveGateMinSelectedRobustMargin)
                : json(nullptr);
        }
        if (bridgeConfig.supportChainGuardEnabled) {
            stepData["bridge"]["nominal"]["support_chain_guard"] = {
                {"enabled", true},
                {"scope", bridgeConfig.supportChainGuardScope},
                {"active_count", supportChainActiveCount},
                {"evaluated_count", supportChainLinks.size()},
                {"required_margin", bridgeConfig.supportChainRobustMargin},
                {"effective_margin", bridgeConfig.supportChainStateReserveEnabled
                                         ? json(bridgeCurrentEffectiveReserveMargin)
                                         : json(bridgeConfig.supportChainRobustMargin)},
                {"min_margin_before", std::isfinite(supportChainMinMarginBefore) ? json(supportChainMinMarginBefore) : json(nullptr)},
                {"min_margin_after", std::isfinite(supportChainMinMarginAfter) ? json(supportChainMinMarginAfter) : json(nullptr)},
                {"links", supportChainLinks}
            };
            if (bridgeConfig.supportChainStateReserveEnabled) {
                stepData["bridge"]["nominal"]["support_chain_guard"]["state_reserve"] = {
                    {"step_min_margin_before", std::isfinite(stepSupportChainMinMarginBefore)
                                                    ? json(stepSupportChainMinMarginBefore)
                                                    : json(nullptr)},
                    {"closing_rate", stepSupportChainClosingRate},
                    {"previous_min_margin", bridgeHasPreviousSupportChainMargin
                                                ? json(bridgePreviousSupportChainMinMargin)
                                                : json(nullptr)}
                };
            }
        }
        if (bridgeConfig.supportChainGuardEnabled) {
            bridgePreviousSupportChainMinMargin = supportChainMinMarginBefore;
            bridgeHasPreviousSupportChainMargin = std::isfinite(bridgePreviousSupportChainMinMargin);
        }
    }

    void applyBridgeNominalFeasibilityGuard() {
        if (!bridgeConfig.enabled || !bridgeConfig.nominalGuardEnabled) {
            return;
        }
        if (isCentralizedExecution()) {
            return;
        }
        if (config.value("model", "") != "DoubleIntegrate2D") {
            return;
        }
        if (!isSecondOrderCbfEnabled()) {
            return;
        }
        for (auto &robot : robots) {
            robot->applySecondOrderNominalFeasibilityGuard(bridgeConfig.nominalGuardTolerance);
        }
    }

    void presetCBFs() {
        if (config["cbfs"]["without-slack"]["comm-fixed"]["on"]) setupCommCBF();
    }

    void postsetCBFs() {
        secondOrderCbfNoSlack.clear();
        if (isSecondOrderCbfEnabled() && config["cbfs"]["without-slack"]["safety"]["on"]) {
            setupSecondOrderSafetyCBF();
        }
        if (config["cbfs"]["with-slack"]["cvt"]["on"]) setupCVTCBF();
    }

    bool isSecondOrderCbfEnabled() const {
        if (config.value("model", "") != "DoubleIntegrate2D") {
            return false;
        }
        if (!config.contains("cbfs") || !config["cbfs"].contains("high-order")) {
            return false;
        }
        return config["cbfs"]["high-order"].value("enabled", false);
    }

    double secondOrderLambda1() const {
        return config["cbfs"]["high-order"].value("lambda1", 1.0);
    }

    double secondOrderLambda2() const {
        return config["cbfs"]["high-order"].value("lambda2", 1.0);
    }

    double secondOrderSampledDataReserve() const {
        double reserve = config["cbfs"]["high-order"].value("sampled-data-reserve", 0.0);
        if (!std::isfinite(reserve) || reserve < 0.0) {
            throw std::invalid_argument("cbfs.high-order.sampled-data-reserve must be a non-negative finite number");
        }
        return reserve;
    }

    bool hasSecondOrderAccelerationBound() const {
        return isSecondOrderCbfEnabled()
               && config["cbfs"]["high-order"].contains("acceleration-bound");
    }

    double secondOrderAccelerationBound() const {
        double bound = config["cbfs"]["high-order"].value("acceleration-bound", 0.0);
        if (!std::isfinite(bound) || bound < 0.0) {
            throw std::invalid_argument("cbfs.high-order.acceleration-bound must be a non-negative finite number");
        }
        return bound;
    }

    void addSecondOrderAccelerationBounds(json &jsonControlBounds) {
        if (!hasSecondOrderAccelerationBound()) {
            return;
        }

        double bound = secondOrderAccelerationBound();
        const std::array<std::string, 2> names = {"ax", "ay"};
        int uSize = centralizedModel->getTotalControlSize();
        for (int i = 0; i < n; ++i) {
            int offset = centralizedModel->getControlOffset(i);
            for (int axis = 0; axis < 2 && axis < centralizedModel->controlSizes[i]; ++axis) {
                VectorXd lower = VectorXd::Zero(uSize);
                lower(offset + axis) = 1.0;
                optimizer->addLinearConstraint(lower, -bound);
                jsonControlBounds.emplace_back(json{
                    {"name", "robot" + std::to_string(i + 1) + "-" + names[axis] + "-lower"},
                    {"coe", convertCentralizedControlToJson(lower)},
                    {"rhs", -bound}
                });

                VectorXd upper = VectorXd::Zero(uSize);
                upper(offset + axis) = -1.0;
                optimizer->addLinearConstraint(upper, -bound);
                jsonControlBounds.emplace_back(json{
                    {"name", "robot" + std::to_string(i + 1) + "-" + names[axis] + "-upper"},
                    {"coe", convertCentralizedControlToJson(upper)},
                    {"rhs", -bound}
                });
            }
        }
    }

    VectorXd extractPlanarVelocityFromRobotState(Robot *robot, const VectorXd &robotState) const {
        VectorXd velocity(2);
        velocity << robot->model->extractFromVector(robotState, "vx"),
                    robot->model->extractFromVector(robotState, "vy");
        return velocity;
    }

    void setupSecondOrderSafetyCBF() {
        auto safetyConfig = config["cbfs"]["without-slack"]["safety"];
        double safeDistance = safetyConfig["safe-distance"];
        double k = safetyConfig.contains("k") ? static_cast<double>(safetyConfig["k"]) : 1.0;
        bool considerUncertainty = safetyConfig.value("consider-uncertainty", true);
        double lambda1 = secondOrderLambda1();
        double lambda2 = secondOrderLambda2();
        double sampledDataReserve = secondOrderSampledDataReserve();

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                Robot *robotI = robots[i].get();
                Robot *robotJ = robots[j].get();
                double uncertainty = 0.0;
                if (considerUncertainty) {
                    uncertainty += robotI->uncertaintyFromCovarianceFunction(robotI->positionCovariance);
                    uncertainty += robotJ->uncertaintyFromCovarianceFunction(robotJ->positionCovariance);
                }

                SecondOrderCBF safetyCBF;
                safetyCBF.name = "centralSecondOrderSafetyCBF(#" + std::to_string(robotI->id) + ",#" + std::to_string(robotJ->id) + ")";
                safetyCBF.lambda1 = lambda1;
                safetyCBF.k1 = lambda1 + lambda2;
                safetyCBF.k0 = lambda1 * lambda2;
                safetyCBF.sampledDataReserve = sampledDataReserve;
                safetyCBF.h = [this, robotI, robotJ, safeDistance, k, uncertainty](const VectorXd &x, double) {
                    VectorXd stateI = centralizedModel->getRobotStateFromX(x, robotI->id);
                    VectorXd stateJ = centralizedModel->getRobotStateFromX(x, robotJ->id);
                    Point pi = robotI->model->extractXYFromVector(stateI);
                    Point pj = robotJ->model->extractXYFromVector(stateJ);
                    VectorXd vi = extractPlanarVelocityFromRobotState(robotI, stateI);
                    VectorXd vj = extractPlanarVelocityFromRobotState(robotJ, stateJ);
                    auto terms = computePairwiseDistanceKinematics(pi, pj, vi, vj);
                    return k * (terms.distance - safeDistance - uncertainty);
                };
                safetyCBF.hdot = [this, robotI, robotJ, k](const VectorXd &x, double) {
                    VectorXd stateI = centralizedModel->getRobotStateFromX(x, robotI->id);
                    VectorXd stateJ = centralizedModel->getRobotStateFromX(x, robotJ->id);
                    Point pi = robotI->model->extractXYFromVector(stateI);
                    Point pj = robotJ->model->extractXYFromVector(stateJ);
                    VectorXd vi = extractPlanarVelocityFromRobotState(robotI, stateI);
                    VectorXd vj = extractPlanarVelocityFromRobotState(robotJ, stateJ);
                    auto terms = computePairwiseDistanceKinematics(pi, pj, vi, vj);
                    return k * terms.radialVelocity;
                };
                safetyCBF.hddotConst = [this, robotI, robotJ, k](const VectorXd &x, double) {
                    VectorXd stateI = centralizedModel->getRobotStateFromX(x, robotI->id);
                    VectorXd stateJ = centralizedModel->getRobotStateFromX(x, robotJ->id);
                    Point pi = robotI->model->extractXYFromVector(stateI);
                    Point pj = robotJ->model->extractXYFromVector(stateJ);
                    VectorXd vi = extractPlanarVelocityFromRobotState(robotI, stateI);
                    VectorXd vj = extractPlanarVelocityFromRobotState(robotJ, stateJ);
                    auto terms = computePairwiseDistanceKinematics(pi, pj, vi, vj);
                    return k * terms.curvature;
                };
                safetyCBF.uCoe = [this, robotI, robotJ, k](const VectorXd &x, double) {
                    VectorXd stateI = centralizedModel->getRobotStateFromX(x, robotI->id);
                    VectorXd stateJ = centralizedModel->getRobotStateFromX(x, robotJ->id);
                    Point pi = robotI->model->extractXYFromVector(stateI);
                    Point pj = robotJ->model->extractXYFromVector(stateJ);
                    VectorXd vi = extractPlanarVelocityFromRobotState(robotI, stateI);
                    VectorXd vj = extractPlanarVelocityFromRobotState(robotJ, stateJ);
                    auto terms = computePairwiseDistanceKinematics(pi, pj, vi, vj);

                    VectorXd coe = VectorXd::Zero(centralizedModel->getTotalControlSize());
                    int offsetI = centralizedModel->getControlOffset(robotI->id - 1);
                    int offsetJ = centralizedModel->getControlOffset(robotJ->id - 1);
                    coe(offsetI) = k * terms.normal.x();
                    coe(offsetI + 1) = k * terms.normal.y();
                    coe(offsetJ) = -k * terms.normal.x();
                    coe(offsetJ + 1) = -k * terms.normal.y();
                    return coe;
                };

                secondOrderCbfNoSlack[safetyCBF.name] = safetyCBF;
            }
        }
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
            {"cbfSlack",   json::array()},
            {"controlBounds", json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array(), jsonSecondOrderCBFNoSlack = json::array();
        json jsonControlBounds = json::array();

        optimizer->clear();

        auto f = centralizedModel->f();
        auto g = centralizedModel->g();
        auto x = centralizedModel->getX();

        int uSize = centralizedModel->getTotalControlSize();
        int slackSize = cbfSlack.size();
        int totalSize = uSize + slackSize;

        optimizer->start(totalSize, uSize);
        optimizer->setObjective(uNominal);
        addSecondOrderAccelerationBounds(jsonControlBounds);
        opt["controlBounds"] = jsonControlBounds;

        opt["nominal"] = convertCentralizedControlToJson(uNominal);

        std::string cbf_method = config["cbfs"]["without-slack"].value("method", "all");
        if (cbf_method == "all") {
            for (auto &[name, cbf] : cbfNoSlack.cbfs) {
                auto evaluation = cbf.evaluateConstraint(f, g, x, robots[0]->runtime);
                optimizer->addLinearConstraint(evaluation.uCoe, -evaluation.constWithTime);

                jsonCBFNoSlack.emplace_back(json{
                    {"name",  cbf.name},
                    {"coe",   convertCentralizedControlToJson(evaluation.uCoe)},
                    {"const", evaluation.constWithTime}
                });
            }
        }

        std::unordered_map<std::string, SecondOrderCBFConstraintEvaluation> secondOrderEvaluations;
        for (auto &[name, cbf] : secondOrderCbfNoSlack) {
            auto evaluation = cbf.evaluateConstraint(x, robots[0]->runtime);
            secondOrderEvaluations[name] = evaluation;
            optimizer->addLinearConstraint(evaluation.uCoe, -evaluation.constTerm);

            jsonSecondOrderCBFNoSlack.emplace_back(json{
                {"name",  cbf.name},
                {"coe",   convertCentralizedControlToJson(evaluation.uCoe)},
                {"const", evaluation.constTerm},
                {"h", evaluation.h},
                {"hdot", evaluation.hdot},
                {"psi1", evaluation.psi1},
                {"sampledDataReserve", evaluation.sampledDataReserve},
                {"hocbf", evaluation.constTerm}
            });
        }

        int cnt = 0;
        for (auto &[name, cbf] : cbfSlack) {
            auto evaluation = cbf.evaluateConstraint(f, g, x, robots[0]->runtime);
            Eigen::VectorXd coe = makeSlackConstraintCoefficients(evaluation.uCoe, slackSize, cnt);

            optimizer->addLinearConstraint(coe, -evaluation.constWithoutTime);

            jsonCBFSlack.emplace_back(json{
                {"name",  cbf.name},
                {"coe",   convertCentralizedControlToJson(evaluation.uCoe)},
                {"const", evaluation.constWithoutTime}
            });
            ++cnt;
        }

        opt["cbfNoSlack"] = jsonCBFNoSlack;
        opt["hocbfNoSlack"] = jsonSecondOrderCBFNoSlack;
        opt["cbfSlack"] = jsonCBFSlack;

        auto result = optimizer->solve();

        optimizer->write("centralized_optimization.lp");

        auto u = result.head(uSize);
        opt["result"] = convertCentralizedControlToJson(u);
        opt["slacks"] = convertCentralizedSlackToJson(result.tail(slackSize));
        for (auto &item : opt["hocbfNoSlack"]) {
            std::string name = item.at("name").get<std::string>();
            if (secondOrderEvaluations.find(name) != secondOrderEvaluations.end()) {
                item["hocbf"] = secondOrderEvaluations.at(name).value(u);
            }
        }

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
