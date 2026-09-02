#ifndef CBF_SWARM_HPP
#define CBF_SWARM_HPP

#include "Robot.hpp"
#include "bridge/BridgeExperiment.hpp"
#include "bridge/BridgeSearchTracker.hpp"
#include "bridge/BridgeTopology.hpp"
#include "bridge/ExactGammaStar2D.hpp"
#include "bridge/FullRowPredictiveFeedback.hpp"
#include "bridge/FullRowPredictionAudit.hpp"
#include "bridge/LookaheadCollisionGate.hpp"
#include "bridge/PostDetectionResponse.hpp"
#include "bridge/ReserveTaskHomotopy.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"

#include <functional>
#include <optional>


class Swarm {
public:
    struct CertifiedStepResult {
        bool advanced = false;
        std::string reason;
        std::size_t updated_truth_cells = 0;
    };

    using CertifiedControlBatch = std::map<int, Eigen::Vector2d>;
    using CertifiedYawRateBatch = std::map<int, double>;
    using CertifiedControlHook = std::function<
        std::optional<CertifiedControlBatch>(Swarm&)>;

    void prepareCertifiedControlStep(bool enforce_world_membership=true) {
        exchangeData();
        checkInformationExchange();
        if (enforce_world_membership)
            for (auto &robot : robots) robot->checkRobotsInsideWorld();
    }

    void observeGridWorldAtCurrentState() {
        exchangeData();
        checkInformationExchange();
        for (auto &robot : robots) robot->updateGridWorld();
        updateGridWorld();
        checkUpdatedGridWorld();
    }

    CertifiedStepResult applyCertifiedControlsAndAdvance(
            double dt,
            const std::optional<CertifiedControlBatch>& controls) {
        return applyCertifiedControlsAndAdvance(dt,controls,std::nullopt);
    }

    CertifiedStepResult applyCertifiedControlsAndAdvance(
            double dt,
            const std::optional<CertifiedControlBatch>& controls,
            const std::optional<CertifiedYawRateBatch>& yaw_rates) {
        return applyCertifiedControlsAndAdvance(
            dt,controls,yaw_rates,std::nullopt);
    }

    CertifiedStepResult applyCertifiedControlsAndAdvance(
            double dt,
            const std::optional<CertifiedControlBatch>& controls,
            const std::optional<CertifiedYawRateBatch>& yaw_rates,
            const std::optional<double>& plant_speed_limit_mps,
            const std::optional<double>& plant_speed_fuse_mps = std::nullopt) {
        CertifiedStepResult result;
        if (!std::isfinite(dt) || dt <= 0.0) {
            result.reason = "invalid_step_request";
            return result;
        }
        if (!controls.has_value()) {
            result.reason = "certified_control_unavailable";
            return result;
        }
        if (controls->size() != robots.size()) {
            result.reason = "incomplete_certified_control_batch";
            return result;
        }
        if (yaw_rates.has_value() && yaw_rates->size()!=robots.size()) {
            result.reason = "incomplete_certified_yaw_rate_batch";
            return result;
        }
        if (plant_speed_limit_mps.has_value() &&
            (!std::isfinite(*plant_speed_limit_mps) ||
             *plant_speed_limit_mps<=0.0)) {
            result.reason="invalid_plant_speed_contract";
            return result;
        }
        for (const auto &robot : robots) {
            const auto it = controls->find(robot->id);
            if (it == controls->end() || !it->second.allFinite()) {
                result.reason = "invalid_certified_control_batch";
                return result;
            }
            if (yaw_rates.has_value()) {
                const auto yaw=yaw_rates->find(robot->id);
                if (yaw==yaw_rates->end() || !std::isfinite(yaw->second)) {
                    result.reason = "invalid_certified_yaw_rate_batch";
                    return result;
                }
            }
        }
        // Preflight the complete batch before the first model mutation.  This
        // is an independent plant-state invariant audit; it never modifies
        // the certified control and never feeds truth into the controller.
        if (plant_speed_limit_mps.has_value()) {
            for (const auto& robot:robots) {
                const Eigen::VectorXd velocity=robot->model->getVelocity();
                if (velocity.size()!=2 || !velocity.allFinite()) {
                    result.reason="plant_speed_preflight_invalid_state";
                    return result;
                }
                const auto audit=gf::auditPlantSpeedExactZoh(
                    velocity.head<2>(),controls->at(robot->id),dt,
                    *plant_speed_limit_mps,1.0e-9);
                // Task 11b S1-v3: the preflight is demoted to telemetry with
                // a coarse fuse when one is configured (researcher-frozen
                // 31 m/s); the audit numbers remain the realism record.
                if (plant_speed_fuse_mps.has_value()) {
                    if (!(audit.maximum_interval_speed_mps<=
                            *plant_speed_fuse_mps)) {
                        result.reason="plant_speed_fuse_tripped";
                        return result;
                    }
                } else if (!audit.valid) {
                    result.reason="plant_speed_preflight_rejected";
                    return result;
                }
            }
        }
        for (auto &robot : robots) {
            Eigen::VectorXd control = Eigen::VectorXd::Zero(
                robot->model->uSize());
            control.head<2>() = controls->at(robot->id);
            if (yaw_rates.has_value() && control.size()>2)
                control(2)=yaw_rates->at(robot->id);
            robot->model->setControlInput(control);
        }
        advanceDynamics(dt);
        // GridWorld observations belong to the post-ZOH state.  Refresh the
        // shared state before each local map observes the swarm so the local
        // union and centralized truth map use the same positions.
        observeGridWorldAtCurrentState();
        result.advanced = true;
        result.reason = "advanced";
        result.updated_truth_cells = updatedGridWorldGroundTruth.size();
        return result;
    }

    void advanceDynamics(double dt) {
        for (auto &robot : robots) robot->stepTimeForward(dt);
    }

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
    std::uint64_t bridgeFeedbackStepIndex = 0;
    std::vector<BridgePredictionAuditEntry> bridgePendingPredictionAudits;
    std::map<int, Point> bridgePreviousTaskGoals;
    BridgeR13DwellTracker bridgeR13Dwell;
    BridgeR13StaticGeometry bridgeR13Geometry;
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
            if (!bridgeConfig.postDetectionResponseEnabled) {
                bridgeSearch = BridgeSearchTracker(config, bridgeConfig);
            } else {
                bridgeR13Dwell = BridgeR13DwellTracker(
                        bridgeConfig.responseTargetRadiusM,
                        bridgeConfig.responseDwellTimeS);
                bridgeR13Geometry = bridgeR13StaticGeometry(
                        bridgeConfig.responseDistanceM);
            }
            bridgeTopologyConfig = loadBridgeTopologyConfig(config);
            if (!bridgeTopologyConfig.fixedReferences.empty()) {
                const int baseCount = robots.empty()
                        ? 0
                        : static_cast<int>(robots.front()->bases.size());
                validateBridgeFixedReferences(
                        bridgeTopologyConfig.fixedReferences, n, baseCount);
            }
            if (bridgeConfig.gammaStarFeedbackEnabled) {
                validateBridgeSingleTriangularLadder(
                        bridgeTopologyConfig.fixedReferences);
            }
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
        json terminalRobots = json::array();
        for (auto &robot : robots) {
            terminalRobots.push_back(robot->getState());
        }
        data["terminal"] = {
                {"runtime", robots.empty() ? 0.0 : robots.front()->runtime},
                {"robots", terminalRobots},
        };
        if (bridgeConfig.enabled) {
            if (!bridgeConfig.postDetectionResponseEnabled) {
                data["bridge"]["final_search_map"] = bridgeSearch.finalMapJson();
            }
            if (bridgeConfig.gammaStarFeedbackEnabled) {
                data["bridge"]["prediction_audit_unresolved_at_horizon"] =
                        bridgePendingPredictionAudits.size();
            }
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
        if (!bridgeConfig.postDetectionResponseEnabled) {
            stepData["update"] = updatedGridWorldGroundTruth;
        }
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

    CertifiedStepResult stepWithCertifiedControls(
            double dt,
            const CertifiedControlHook& controller) {
        CertifiedStepResult result;
        if (!std::isfinite(dt) || dt <= 0.0 || !controller) {
            result.reason = "invalid_step_request";
            return result;
        }

        prepareCertifiedControlStep();
        const std::optional<CertifiedControlBatch> controls = controller(*this);
        return applyCertifiedControlsAndAdvance(dt, controls);
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
                if (!bridgeConfig.postDetectionResponseEnabled) {
                    for (auto &robot: robots) robot->updateGridWorld();
                    updateGridWorld();
                    checkUpdatedGridWorld();
                }
                applyBridgeTopology();
                for (auto &robot: robots) robot->postsetCBF();
                applyBridgeNominalControls();
                applyBridgeNominalFeasibilityGuard();

                if (isCentralizedExecution()) {
                    postsetCBFs();
                    centralizedOptimise();
                } else {
                    for (auto &robot: robots) robot->optimise();
                    finalizeBridgeFeedbackExecutionDiagnostics();
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

                if (bridgeConfig.postDetectionResponseEnabled) {
                    Robot *leader = robots.at(static_cast<std::size_t>(
                            bridgeConfig.jointSingleLadderLeaderId - 1)).get();
                    const auto dwell = bridgeR13Dwell.observeZohInterval(
                            leader->model->xy(),
                            Point(leader->model->getVelocity()),
                            Point(leader->model->getAcceleration()),
                            bridgeR13Geometry.target, tStep, true);
                    stepData["bridge"]["response"] = {
                            {"schema", "r13-reach-dwell-step-v1"},
                            {"static_tuple", true},
                            {"leader_id", 4},
                            {"target", {{"x", bridgeR13Geometry.target.x},
                                        {"y", bridgeR13Geometry.target.y}}},
                            {"target_radius_m", dwell.targetRadiusM},
                            {"required_dwell_s", dwell.requiredDwellS},
                            {"continuous_dwell_s", dwell.continuousDwellS},
                            {"interval_valid", dwell.intervalValid},
                            {"max_horizontal_distance_m",
                                    dwell.maxHorizontalDistanceM},
                            {"geometric_interval_inside",
                                    dwell.intervalValid},
                            {"offline_full_certificate_required", true},
                            {"complete_candidate", dwell.complete},
                    };
                }
                logOnce();
                if (bridgeConfig.enabled
                    && bridgeConfig.stopOnDetection
                    && bridgeSearch.detected()) {
                    data["termination"] = {
                            {"status", "task-complete"},
                            {"runtime", robots.empty()
                                    ? 0.0 : robots.front()->runtime},
                            {"message", nullptr},
                    };
                    break;
                }
                advanceDynamics(tStep);
            }
            catch (...) {
                std::string terminationMessage = "unknown exception";
                try {
                    throw;
                }
                catch (const std::exception &error) {
                    terminationMessage = error.what();
                }
                catch (...) {
                }
                data["termination"] = {
                        {"status", "exception"},
                        {"runtime", robots.empty()
                                ? 0.0 : robots.front()->runtime},
                        {"message", terminationMessage},
                };
                if (!bridgeConfig.postDetectionResponseEnabled) {
                    for (auto &robot: robots) robot->updateGridWorld();
                }
                logOnce();
                endLog();
                printf("Data so far has been saved in %s\n", filename.c_str());
                std::cerr << terminationMessage << std::endl;
                break;
            }
        }

        if (!data.contains("termination")) {
            data["termination"] = {
                    {"status", robots.empty()
                            || robots.front()->runtime >= tTotal
                                    ? "horizon" : "stopped"},
                    {"runtime", robots.empty()
                            ? 0.0 : robots.front()->runtime},
                    {"message", nullptr},
            };
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
        if (bridgeConfig.postDetectionResponseEnabled) {
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
        bridgeTopologyDecision = chooseBridgeTopology(
                positions,
                bridgeTopologyConfig,
                bridgeConfig.topologyPolicy,
                bridgeFixedBases());
        for (auto &robot : robots) {
            auto referenceIt = bridgeTopologyDecision.references.find(robot->id);
            if (referenceIt != bridgeTopologyDecision.references.end()) {
                robot->setBridgeFormationOverride(
                    referenceIt->second.anchorIds,
                    referenceIt->second.baseIds);
                continue;
            }
            if (bridgeConfig.topologyPolicy == "fixed"
                && !bridgeTopologyConfig.fixedReferences.empty()) {
                throw std::runtime_error(
                    "fixed bridge topology omitted robot " + std::to_string(robot->id));
            }
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

    std::map<int, BridgePredictionState2D> bridgePredictionStates() const {
        std::map<int, BridgePredictionState2D> states;
        for (const auto &robot : robots) {
            states[robot->id] = {
                    robot->model->xy(),
                    robot->model->getVelocity(),
                    robot->model->getAcceleration(),
            };
        }
        return states;
    }

    std::map<int, Point> bridgeFixedBases() const {
        std::map<int, Point> bases;
        if (robots.empty()) {
            return bases;
        }
        for (size_t index = 0; index < robots.front()->bases.size(); ++index) {
            bases[static_cast<int>(index)] = robots.front()->bases[index];
        }
        return bases;
    }

    std::vector<BridgeFullRowDescriptor> bridgeFullRowDescriptors() const {
        const json &safety = config.at("cbfs").at("without-slack").at("safety");
        const json &communication =
                config.at("cbfs").at("without-slack").at("comm-fixed");
        const double lambda1 = secondOrderLambda1();
        const double lambda2 = secondOrderLambda2();
        const double reserve = secondOrderSampledDataReserve();
        const double effectiveSafeDistance =
                safety.at("safe-distance").get<double>()
                + safety.value("safe-distance-tightening-margin", 0.0);
        const double effectiveCommunicationRange =
                communication.at("max-range").get<double>()
                - communication.value("range-tightening-margin", 0.0);
        const PairwiseSecondOrderRowSpec safetySpec{
                PairwiseSecondOrderBarrierKind::CollisionLower,
                effectiveSafeDistance,
                0.0,
                safety.value("k", 1.0),
                lambda1,
                lambda2,
                reserve,
        };
        const PairwiseSecondOrderRowSpec communicationSpec{
                PairwiseSecondOrderBarrierKind::CommunicationUpper,
                effectiveCommunicationRange,
                0.0,
                communication.value("k", 1.0),
                lambda1,
                lambda2,
                reserve,
        };

        std::vector<BridgeFullRowDescriptor> rows;
        for (const auto &owner : robots) {
            for (const auto &reference : robots) {
                if (owner->id == reference->id) {
                    continue;
                }
                rows.push_back({
                        "secondOrderSafetyCBF(#"
                                + std::to_string(reference->id) + ")",
                        owner->id,
                        BridgeReferenceKind::Mobile,
                        reference->id,
                        safetySpec,
                });
            }

            const auto fixedIt = bridgeTopologyDecision.references.find(owner->id);
            if (fixedIt == bridgeTopologyDecision.references.end()) {
                throw std::runtime_error(
                        "full-row feedback is missing fixed references for robot "
                        + std::to_string(owner->id));
            }
            for (int baseId : fixedIt->second.baseIds) {
                rows.push_back({
                        "secondOrderFixedCommCBF(base-"
                                + std::to_string(baseId) + ")",
                        owner->id,
                        BridgeReferenceKind::FixedBase,
                        baseId,
                        communicationSpec,
                });
            }
            for (int anchorId : fixedIt->second.anchorIds) {
                rows.push_back({
                        "secondOrderFixedCommCBF(#"
                                + std::to_string(anchorId) + ")",
                        owner->id,
                        BridgeReferenceKind::Mobile,
                        anchorId,
                        communicationSpec,
                });
            }
        }
        return rows;
    }

    struct FullRowFeedbackResult {
        bool evaluated = false;
        bool selected = false;
        double selectedAccelX = 0.0;
        double selectedAccelY = 0.0;
        json diagnostic = json::object();
        std::vector<std::map<int, BridgePredictionState2D>> selectedForecast;
        std::vector<double> selectedStepBudgets;
    };

    json bridgeFullRowBudgetAudit(
            const std::map<int, BridgePredictionState2D> &states,
            const std::map<int, Point> &fixedBases,
            const std::vector<BridgeFullRowDescriptor> &descriptors) const {
        json robotAudits = json::array();
        double minimumGamma = std::numeric_limits<double>::infinity();
        bool allValid = true;
        for (const auto &robot : robots) {
            const auto evaluatedRows = evaluateBridgeFullRows(
                    robot->id, states, fixedBases, descriptors);
            std::vector<BridgeGammaStarResidual2D> residuals;
            std::vector<std::string> expectedIdentities;
            residuals.reserve(evaluatedRows.size());
            expectedIdentities.reserve(evaluatedRows.size());
            for (const auto &evaluated : evaluatedRows) {
                residuals.push_back(
                        bridgeGammaStarResidualFromAffineMargin(
                                evaluated.row.uCoe(0),
                                evaluated.row.uCoe(1),
                                evaluated.row.constTerm));
                expectedIdentities.push_back(evaluated.identity);
            }

            const bool installedHard =
                    !robot->secondOrderCbfNoSlack.empty()
                    && robot->secondOrderCbfSlack.empty();
            const bool installedSoft =
                    robot->secondOrderCbfNoSlack.empty()
                    && !robot->secondOrderCbfSlack.empty();
            std::vector<std::string> installedIdentities;
            const auto &installedRows = installedSoft
                    ? robot->secondOrderCbfSlack
                    : robot->secondOrderCbfNoSlack;
            installedIdentities.reserve(installedRows.size());
            for (const auto &[identity, cbf] : installedRows) {
                (void) cbf;
                installedIdentities.push_back(identity);
            }
            std::sort(expectedIdentities.begin(), expectedIdentities.end());
            std::sort(installedIdentities.begin(), installedIdentities.end());
            const bool rowIdentityConsistent =
                    expectedIdentities == installedIdentities;
            const bool rowClassConsistent = installedHard || installedSoft;
            const auto solution = solveExactBridgeGammaStar2D(
                    residuals, secondOrderAccelerationBound());
            const bool valid = solution.valid
                    && rowIdentityConsistent && rowClassConsistent;
            if (solution.valid) {
                minimumGamma = std::min(minimumGamma, solution.gamma);
            }
            allValid = allValid && valid;
            robotAudits.push_back({
                    {"robot", robot->id},
                    {"valid", valid},
                    {"execution_class", installedSoft ? "soft"
                            : installedHard ? "hard" : "invalid"},
                    {"row_identity_consistent", rowIdentityConsistent},
                    {"row_class_consistent", rowClassConsistent},
                    {"row_identities", expectedIdentities},
                    {"installed_row_identities", installedIdentities},
                    {"current_gamma_star", solution.valid
                            ? json(solution.gamma) : json(nullptr)},
                    {"current_hard_set_feasible",
                            solution.valid && solution.gamma >= 0.0},
                    {"gamma_witness", solution.valid
                            ? json{{"ax", solution.accelX},
                                   {"ay", solution.accelY}}
                            : json(nullptr)},
            });
        }
        return {
                {"valid", allValid},
                {"minimum_gamma_star", std::isfinite(minimumGamma)
                        ? json(minimumGamma) : json(nullptr)},
                {"all_current_hard_sets_feasible",
                        allValid && minimumGamma >= 0.0},
                {"robots", robotAudits},
        };
    }

    FullRowFeedbackResult fullRowFeedbackNominal(
            int robotId,
            const std::map<int, BridgePredictionState2D> &currentStates,
            const std::map<int, BridgePredictionState2D> &forecastStates,
            const std::map<int, Point> &fixedBases,
            const std::vector<BridgeFullRowDescriptor> &descriptors,
            double nominalAccelX,
            double nominalAccelY) const {
        FullRowFeedbackResult result;
        result.evaluated = true;
        const double accelerationBound = secondOrderAccelerationBound();
        const double dt = config.at("execute").at("time-step").get<double>();
        const auto evaluatedRows = evaluateBridgeFullRows(
                robotId, currentStates, fixedBases, descriptors);

        std::vector<BridgeGammaStarResidual2D> residuals;
        std::vector<BridgeHocbfHalfspace2D> constraints;
        json rowLog = json::array();
        json rowIdentities = json::array();
        for (const auto &evaluated : evaluatedRows) {
            residuals.push_back(bridgeGammaStarResidualFromAffineMargin(
                    evaluated.row.uCoe(0),
                    evaluated.row.uCoe(1),
                    evaluated.row.constTerm));
            constraints.push_back({
                    evaluated.row.uCoe(0),
                    evaluated.row.uCoe(1),
                    -evaluated.row.constTerm,
            });
            rowIdentities.push_back(evaluated.identity);
            rowLog.push_back({
                    {"identity", evaluated.identity},
                    {"u_coe", {evaluated.row.uCoe(0), evaluated.row.uCoe(1)}},
                    {"const_term", evaluated.row.constTerm},
                    {"h", evaluated.row.h},
                    {"hdot", evaluated.row.hdot},
                    {"psi1", evaluated.row.psi1},
                    {"total_reserve", evaluated.row.totalReserve},
            });
        }

        if (robotId < 1 || robotId > static_cast<int>(robots.size())) {
            result.diagnostic = {
                    {"robot", robotId},
                    {"evaluated", true},
                    {"selected", false},
                    {"scoring_valid", false},
                    {"scoring_error", "invalid robot id for installed-row audit"},
                    {"full_row_identities", rowIdentities},
                    {"current_rows", rowLog},
            };
            return result;
        }
        const bool softExecution =
                bridgeConfig.gammaStarFeedbackConstraintExecution == "soft";
        const Robot *robot = robots[robotId - 1].get();
        std::vector<std::string> expectedIdentities =
                rowIdentities.get<std::vector<std::string>>();
        std::vector<std::string> installedIdentities;
        std::vector<std::string> oppositeClassIdentities;
        const auto appendIdentities = [](
                const auto &rows,
                std::vector<std::string> *identities) {
            identities->reserve(rows.size());
            for (const auto &[identity, cbf] : rows) {
                (void) cbf;
                identities->push_back(identity);
            }
        };
        if (softExecution) {
            appendIdentities(robot->secondOrderCbfSlack, &installedIdentities);
            appendIdentities(
                    robot->secondOrderCbfNoSlack, &oppositeClassIdentities);
        } else {
            appendIdentities(robot->secondOrderCbfNoSlack, &installedIdentities);
            appendIdentities(
                    robot->secondOrderCbfSlack, &oppositeClassIdentities);
        }
        std::sort(expectedIdentities.begin(), expectedIdentities.end());
        std::sort(installedIdentities.begin(), installedIdentities.end());
        std::sort(
                oppositeClassIdentities.begin(),
                oppositeClassIdentities.end());
        const bool preScoreIdentityConsistent =
                expectedIdentities == installedIdentities;
        const bool preScoreClassConsistent = oppositeClassIdentities.empty();
        if (!preScoreIdentityConsistent || !preScoreClassConsistent) {
            result.diagnostic = {
                    {"robot", robotId},
                    {"evaluated", true},
                    {"selected", false},
                    {"scoring_valid", false},
                    {"scoring_error",
                            "predictor rows do not match the installed execution ledger"},
                    {"current_rows", rowLog},
                    {"full_row_identities", rowIdentities},
                    {"pre_score_row_execution_class",
                            softExecution ? "soft" : "hard"},
                    {"pre_score_installed_row_identities",
                            installedIdentities},
                    {"pre_score_opposite_class_row_identities",
                            oppositeClassIdentities},
                    {"pre_score_row_identity_consistent",
                            preScoreIdentityConsistent},
                    {"pre_score_row_class_consistent",
                            preScoreClassConsistent},
                    {"pre_score_constraint_ledger_consistent", false},
                    {"candidates", json::array()},
            };
            return result;
        }

        const auto current = solveExactBridgeGammaStar2D(
                residuals, accelerationBound);
        const auto legacy = projectBridgeHocbfNominalAcceleration(
                nominalAccelX,
                nominalAccelY,
                accelerationBound,
                constraints,
                0.0);
        const auto certifiedLegacy = certifyBridgeHocbfEndpointTowardWitness(
                legacy.projected_ax,
                legacy.projected_ay,
                current.accelX,
                current.accelY,
                accelerationBound,
                constraints,
                BRIDGE_FULL_ROW_NUMERICAL_REPAIR_TOLERANCE);
        const double currentWitnessMargin = current.valid
                ? bridgeHocbfMinMargin(
                        constraints, current.accelX, current.accelY)
                : -std::numeric_limits<double>::infinity();
        const bool currentInfeasible = !current.valid || !legacy.feasible
                || current.gamma < 0.0 || currentWitnessMargin < 0.0
                || !certifiedLegacy.valid;
        const bool certifiedNegativeCurrentHardSet =
                current.valid && std::isfinite(current.gamma)
                && current.gamma < 0.0;
        result.diagnostic = {
                {"robot", robotId},
                {"evaluated", true},
                {"current_gamma_star", current.valid ? json(current.gamma) : json(nullptr)},
                {"current_gamma_witness", current.valid
                        ? json{{"ax", current.accelX}, {"ay", current.accelY}}
                        : json(nullptr)},
                {"current_gamma_witness_margin", current.valid
                        ? json(currentWitnessMargin) : json(nullptr)},
                {"legacy_projection_raw", legacy.feasible
                        ? json{{"ax", legacy.projected_ax},
                               {"ay", legacy.projected_ay},
                               {"minimum_margin", legacy.margin_after}}
                        : json(nullptr)},
                {"legacy_execution_certified_numeric", certifiedLegacy.valid
                        ? json{{"ax", certifiedLegacy.accelX},
                               {"ay", certifiedLegacy.accelY}}
                        : json(nullptr)},
                {"legacy_numerical_repair", {
                        {"valid", certifiedLegacy.valid},
                        {"repaired", certifiedLegacy.repaired},
                        {"mix_toward_gamma_witness", certifiedLegacy.repairMix},
                        {"norm", certifiedLegacy.repairNorm},
                        {"margin_before", certifiedLegacy.marginBefore},
                        {"margin_after", certifiedLegacy.marginAfter},
                }},
                {"current_rows", rowLog},
                {"full_row_identities", rowIdentities},
                {"pre_score_row_execution_class",
                        softExecution ? "soft" : "hard"},
                {"pre_score_installed_row_identities",
                        installedIdentities},
                {"pre_score_opposite_class_row_identities",
                        oppositeClassIdentities},
                {"pre_score_row_identity_consistent", true},
                {"pre_score_row_class_consistent", true},
                {"pre_score_constraint_ledger_consistent", true},
                {"current_infeasible", currentInfeasible},
                {"certified_negative_current_hard_set",
                        certifiedNegativeCurrentHardSet},
                {"selected", false},
                {"candidates", json::array()},
        };
        if (currentInfeasible) {
            return result;
        }

        auto candidates = buildReserveTaskHomotopy(
                certifiedLegacy.accelX,
                certifiedLegacy.accelY,
                current.accelX,
                current.accelY,
                static_cast<size_t>(bridgeConfig.gammaStarHomotopyIntervals));
        std::vector<BridgeFullRowScore> scores(candidates.size());
        for (size_t candidateIndex = 0;
             candidateIndex < candidates.size(); ++candidateIndex) {
            auto &candidate = candidates[candidateIndex];
            const auto candidateCertification =
                    certifyBridgeHocbfEndpointTowardWitness(
                            candidate.accelX,
                            candidate.accelY,
                            current.accelX,
                            current.accelY,
                            accelerationBound,
                            constraints,
                            BRIDGE_FULL_ROW_NUMERICAL_REPAIR_TOLERANCE);
            if (!candidateCertification.valid) {
                result.diagnostic["scoring_valid"] = false;
                result.diagnostic["scoring_error"] =
                        "homotopy candidate failed exact current-feasibility certification";
                return result;
            }
            candidate.accelX = candidateCertification.accelX;
            candidate.accelY = candidateCertification.accelY;
            const auto score = scoreFullRowCandidate(
                    robotId,
                    Eigen::Vector2d(candidate.accelX, candidate.accelY),
                    forecastStates,
                    fixedBases,
                    descriptors,
                    accelerationBound,
                    dt,
                    bridgeConfig.gammaStarLookaheadSteps);
            if (!score.valid) {
                result.diagnostic["scoring_valid"] = false;
                return result;
            }
            candidate.predictedBudget = score.minimumBudget;
            scores[candidateIndex] = score;
            result.diagnostic["candidates"].push_back({
                    {"alpha", candidate.alpha},
                    {"accel_x", candidate.accelX},
                    {"accel_y", candidate.accelY},
                    {"predicted_minimum_budget", score.minimumBudget},
                    {"step_budgets", score.stepBudgets},
                    {"step_dominant_rows", score.stepDominantRows},
                    {"worst_step", score.worstStep},
                    {"dominant_row", score.dominantRow},
                    {"current_minimum_margin",
                            candidateCertification.marginAfter},
                    {"numerical_repair", {
                            {"repaired", candidateCertification.repaired},
                            {"mix_toward_gamma_witness",
                                    candidateCertification.repairMix},
                            {"norm", candidateCertification.repairNorm},
                    }},
            });
        }
        result.diagnostic["scoring_valid"] = true;

        const auto selection =
                bridgeConfig.gammaStarFeedbackSelectionRule == "maximum-reserve"
                ? selectMaximumReserveHomotopy(
                        candidates,
                        certifiedLegacy.accelX,
                        certifiedLegacy.accelY,
                        bridgeConfig.gammaStarPredictiveGate,
                        bridgeConfig.nominalGuardTolerance)
                : selectReserveTaskHomotopy(
                        candidates,
                        certifiedLegacy.accelX,
                        certifiedLegacy.accelY,
                        bridgeConfig.gammaStarPredictiveGate,
                        bridgeConfig.nominalGuardTolerance);
        if (!selection.selected) {
            return result;
        }

        const size_t selectedIndex = selection.candidateIndex;
        if (selectedIndex >= scores.size()) {
            return result;
        }

        const double currentMargin = bridgeHocbfMinMargin(
                constraints, selection.accelX, selection.accelY);
        if (currentMargin < 0.0) {
            result.diagnostic["selection_error"] =
                    "selected candidate is outside the exact current hard set";
            result.diagnostic["candidate_current_feasible"] = false;
            result.diagnostic["current_candidate_minimum_margin"] =
                    currentMargin;
            return result;
        }
        result.selected = true;
        result.selectedAccelX = selection.accelX;
        result.selectedAccelY = selection.accelY;
        result.diagnostic["selected"] = true;
        result.diagnostic["selection_rule"] =
                bridgeConfig.gammaStarFeedbackSelectionRule;
        result.diagnostic["selected_alpha"] = selection.alpha;
        result.diagnostic["selected_accel_x"] = selection.accelX;
        result.diagnostic["selected_accel_y"] = selection.accelY;
        result.diagnostic["selected_predictive_minimum_budget"] =
                selection.predictedBudget;
        result.diagnostic["selected_candidate_index"] =
                selection.candidateIndex;
        result.diagnostic["maximum_homotopy_candidate_index"] =
                selection.maximumCandidateIndex;
        result.diagnostic["maximum_homotopy_predicted_budget"] =
                selection.maximumPredictedBudget;
        result.diagnostic["selected_is_homotopy_argmax"] =
                selection.selectedMaximumPredictedBudget;
        result.diagnostic["gate_satisfied"] = selection.gateSatisfied;
        result.diagnostic["fallback_used"] = !selection.gateSatisfied;
        result.diagnostic["fallback_reason"] = selection.gateSatisfied
                ? json(nullptr)
                : json("predictive-gate-unattainable");
        result.diagnostic["fallback_positive_recoverability"] =
                selection.gateSatisfied
                ? json(nullptr)
                : json(selection.predictedBudget > 0.0);
        result.diagnostic["regulation_threshold"] =
                bridgeConfig.gammaStarPredictiveGate;
        result.diagnostic["hard_feasibility_boundary"] = 0.0;
        result.diagnostic["candidate_mechanism"] =
                bridgeR13CandidateMechanism(
                        selection.maximumPredictedBudget,
                        bridgeConfig.gammaStarPredictiveGate,
                        true,
                        bridgeConfig.gammaStarFeedbackSelectionRule);
        result.diagnostic["nominal_retained"] = selection.nominalRetained;
        result.diagnostic["task_deviation"] = selection.taskDeviation;
        result.diagnostic["dominant_row"] = scores[selectedIndex].dominantRow;
        result.diagnostic["worst_step"] = scores[selectedIndex].worstStep;
        result.diagnostic["current_candidate_minimum_margin"] = currentMargin;
        result.diagnostic["candidate_current_feasible"] =
                true;
        result.selectedForecast = rolloutBridgePredictionStates(
                robotId,
                Eigen::Vector2d(selection.accelX, selection.accelY),
                forecastStates,
                dt,
                bridgeConfig.gammaStarLookaheadSteps);
        result.selectedStepBudgets = scores[selectedIndex].stepBudgets;
        if (result.selectedForecast.size()
                != result.selectedStepBudgets.size()) {
            result.selected = false;
            result.diagnostic["selected"] = false;
            result.diagnostic["prediction_audit_ready"] = false;
            return result;
        }
        result.diagnostic["prediction_audit_ready"] = true;
        return result;
    }

    static json bridgePredictionStateJson(
            const BridgePredictionState2D &state) {
        return {
                {"position", {state.position.x, state.position.y}},
                {"velocity", {state.velocity(0), state.velocity(1)}},
                {"held_acceleration", {
                        state.heldAcceleration(0),
                        state.heldAcceleration(1)}},
        };
    }

    static json bridgePredictionAuditJson(
            const BridgePredictionAuditResult &audit) {
        json stateErrors = json::array();
        for (const auto &state : audit.stateErrors) {
            stateErrors.push_back({
                    {"robot", state.robotId},
                    {"predicted", bridgePredictionStateJson(state.predicted)},
                    {"observed", bridgePredictionStateJson(state.observed)},
                    {"position_error_m", state.positionError},
                    {"velocity_error_mps", state.velocityError},
                    {"held_acceleration_error_mps2", state.accelerationError},
            });
        }
        return {
                {"valid", audit.valid},
                {"error", audit.error.empty() ? json(nullptr) : json(audit.error)},
                {"origin_step", audit.originStep},
                {"due_step", audit.dueStep},
                {"observed_step", audit.observedStep},
                {"origin_time_s", audit.originTime},
                {"predicted_time_s", audit.predictedTime},
                {"observed_time_s", audit.observedTime},
                {"robot", audit.robotId},
                {"horizon_step", audit.horizonStep},
                {"predicted_budget", audit.predictedBudget},
                {"actual_budget", audit.valid
                        ? json(audit.actualBudget) : json(nullptr)},
                {"budget_error_actual_minus_predicted", audit.valid
                        ? json(audit.budgetError) : json(nullptr)},
                {"max_position_error_m", audit.maxPositionError},
                {"max_velocity_error_mps", audit.maxVelocityError},
                {"max_held_acceleration_error_mps2",
                        audit.maxAccelerationError},
                {"states", stateErrors},
        };
    }

    void attachBridgePredictionAuditLog(
            const std::vector<BridgePredictionAuditResult> &resolvedAudits) {
        json auditLog = json::array();
        double maxPositionError = 0.0;
        double maxVelocityError = 0.0;
        double maxAccelerationError = 0.0;
        double maxAbsoluteBudgetError = 0.0;
        int invalidAudits = 0;
        for (const auto &audit : resolvedAudits) {
            auditLog.push_back(bridgePredictionAuditJson(audit));
            if (!audit.valid) {
                ++invalidAudits;
                continue;
            }
            maxPositionError = std::max(
                    maxPositionError, audit.maxPositionError);
            maxVelocityError = std::max(
                    maxVelocityError, audit.maxVelocityError);
            maxAccelerationError = std::max(
                    maxAccelerationError, audit.maxAccelerationError);
            maxAbsoluteBudgetError = std::max(
                    maxAbsoluteBudgetError, std::abs(audit.budgetError));
        }
        stepData["bridge"]["nominal"]["gamma_star_feedback"]
                ["prediction_audit"] = {
                        {"step", bridgeFeedbackStepIndex},
                        {"resolved_count", resolvedAudits.size()},
                        {"invalid_count", invalidAudits},
                        {"pending_count", bridgePendingPredictionAudits.size()},
                        {"max_position_error_m", maxPositionError},
                        {"max_velocity_error_mps", maxVelocityError},
                        {"max_held_acceleration_error_mps2",
                                maxAccelerationError},
                        {"max_absolute_budget_error", maxAbsoluteBudgetError},
                        {"resolved", auditLog},
                };
    }

    void applyBridgeNominalControls() {
        if (!bridgeConfig.enabled) {
            return;
        }
        double dt = config.at("execute").at("time-step").get<double>();
        double maxSpeed = config.at("bridge").at("nominal").value("max-speed", 8.0);
        double maxAcceleration = config.at("bridge").at("nominal").value("max-acceleration", 2.0);
        double maxYawRate = config.at("bridge").at("nominal").value("max-yaw-rate", 0.35);

        std::map<int, BridgePredictionState2D> feedbackStates =
                bridgePredictionStates();
        std::map<int, Point> feedbackBases = bridgeFixedBases();
        std::vector<BridgeFullRowDescriptor> feedbackRows =
                bridgeFullRowDescriptors();
        stepData["bridge"]["nominal"]["full_row_budget_audit"] =
                bridgeFullRowBudgetAudit(
                        feedbackStates, feedbackBases, feedbackRows);
        std::vector<BridgePredictionAuditResult> resolvedPredictionAudits;
        if (bridgeConfig.gammaStarFeedbackEnabled) {
            resolvedPredictionAudits = resolveBridgePredictionAudits(
                    bridgeFeedbackStepIndex,
                    robots.front()->runtime,
                    feedbackStates,
                    feedbackBases,
                    feedbackRows,
                    secondOrderAccelerationBound(),
                    bridgePendingPredictionAudits);
        }

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
            if (bridgeConfig.gammaStarFeedbackEnabled) {
                stepData["bridge"]["nominal"]["gamma_star_feedback"] = {
                        {"enabled", true},
                        {"mode", bridgeConfig.gammaStarFeedbackMode},
                        {"analysis_role",
                                bridgeConfig.gammaStarFeedbackAnalysisRole},
                        {"selection_rule",
                                bridgeConfig.gammaStarFeedbackSelectionRule},
                        {"constraint_execution",
                                bridgeConfig.gammaStarFeedbackConstraintExecution},
                        {"homotopy_intervals",
                                bridgeConfig.gammaStarHomotopyIntervals},
                        {"lookahead_steps",
                                bridgeConfig.gammaStarLookaheadSteps},
                        {"predictive_gate",
                                bridgeConfig.gammaStarPredictiveGate},
                        {"physical_acceleration_bound",
                                secondOrderAccelerationBound()},
                        {"communication_distance_m", 850.0},
                        {"safety_distance_m", 10.0},
                        {"control_skipped_due_to_fail_safe", true},
                        {"active_count", 0},
                        {"evaluated_candidates", 0},
                        {"min_current_gamma_star", nullptr},
                        {"min_selected_predictive_budget", nullptr},
                        {"links", json::array()},
                };
                attachBridgePredictionAuditLog(resolvedPredictionAudits);
                ++bridgeFeedbackStepIndex;
            }
            return;
        }

        std::map<int, Point> positions;
        for (auto &robot : robots) {
            positions[robot->id] = robot->model->xy();
        }
        std::map<int, Eigen::VectorXd> velocities;
        if (bridgeConfig.goalDiversionEnabled || bridgeConfig.gammaStarFeedbackEnabled) {
            for (auto &robot : robots) {
                velocities[robot->id] = robot->model->getVelocity();
            }
        }
        std::map<int, Eigen::VectorXd> goalDiversionVelocities = velocities;
        int gammaStarFeedbackActiveCount = 0;
        int gammaStarFeedbackEvaluatedCandidates = 0;
        double gammaStarFeedbackMinCurrent = std::numeric_limits<double>::infinity();
        double gammaStarFeedbackMinPredicted = std::numeric_limits<double>::infinity();
        json gammaStarFeedbackLinks = json::array();
        std::map<int, Point> goalDiversionOffsets;
        std::map<int, std::vector<Point>> goalDiversionPerPairOffsets;
        json goalDiversionLinks = json::array();
        json taskGoalLinks = json::array();
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
                    if (dist < 1.0e-9) {
                        continue;
                    }
                    const Eigen::VectorXd &velA = goalDiversionVelocities[idA];
                    const Eigen::VectorXd &velB = goalDiversionVelocities[idB];
                    int lookaheadSteps = bridgeConfig.goalDiversionLookaheadSteps;
                    double horizon = static_cast<double>(lookaheadSteps) * dt;
                    Point predPosA = posA;
                    Point predPosB = posB;
                    if (lookaheadSteps > 0) {
                        predPosA = Point(posA.x + velA(0) * horizon, posA.y + velA(1) * horizon);
                        predPosB = Point(posB.x + velB(0) * horizon, posB.y + velB(1) * horizon);
                    }
                    Point predRel = predPosB - predPosA;
                    double predDist = predRel.len();
                    Point unit = rel / dist;
                    double radialA = unit.x * velA(0) + unit.y * velA(1);
                    double radialB = -unit.x * velB(0) - unit.y * velB(1);
                    double closingRate = -(radialA + radialB) / 2.0;
                    bool currentDangerous = dist < bridgeConfig.goalDiversionDistance
                                            && closingRate < -bridgeConfig.goalDiversionRadial;
                    bool predictedDangerous = false;
                    double predClosingRate = closingRate;
                    if (lookaheadSteps > 0 && predDist > 1.0e-9) {
                        Point predUnit = predRel / predDist;
                        double predRadialA = predUnit.x * velA(0) + predUnit.y * velA(1);
                        double predRadialB = -predUnit.x * velB(0) - predUnit.y * velB(1);
                        predClosingRate = -(predRadialA + predRadialB) / 2.0;
                        predictedDangerous = predDist < bridgeConfig.goalDiversionLookaheadDistance
                                             && predClosingRate < -bridgeConfig.goalDiversionLookaheadRadial;
                    }
                    double scanCeil = lookaheadSteps > 0
                                      ? std::max(bridgeConfig.goalDiversionDistance,
                                                 bridgeConfig.goalDiversionLookaheadDistance)
                                      : bridgeConfig.goalDiversionDistance;
                    if (dist >= scanCeil && !predictedDangerous) {
                        continue;
                    }
                    if (!currentDangerous && !predictedDangerous) {
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
                    double triggerDist = predictedDangerous ? std::min(dist, predDist) : dist;
                    double triggerClosing = predictedDangerous
                                            ? std::min(closingRate, predClosingRate)
                                            : closingRate;
                    double excess = bridgeConfig.goalDiversionDistance - triggerDist;
                    double magnitude = bridgeConfig.goalDiversionSeparationScale
                                       * (excess + std::max(0.0, -triggerClosing) * 2.0);
                    if (magnitude > bridgeConfig.goalDiversionMaxOffset) {
                        magnitude = bridgeConfig.goalDiversionMaxOffset;
                    }
                    Point offset = awayUnit * magnitude;
                    goalDiversionPerPairOffsets[divergeId].push_back(offset);
                    if (!bridgeConfig.goalDiversionMultiPair) {
                        auto existing = goalDiversionOffsets.find(divergeId);
                        if (existing == goalDiversionOffsets.end()
                            || offset.len() > existing->second.len()) {
                            goalDiversionOffsets[divergeId] = offset;
                        }
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
                        {"predicted_distance", predDist},
                        {"predicted_closing_rate", predClosingRate},
                        {"trigger_source", predictedDangerous ? "lookahead" : "current"},
                        {"offset", {{"x", offset.x}, {"y", offset.y}}}
                    });
                }
            }
            if (bridgeConfig.goalDiversionMultiPair) {
                for (auto &entry : goalDiversionPerPairOffsets) {
                    int robotId = entry.first;
                    const std::vector<Point> &pairOffsets = entry.second;
                    double sumX = 0.0;
                    double sumY = 0.0;
                    for (const auto &off : pairOffsets) {
                        sumX += off.x;
                        sumY += off.y;
                    }
                    Point consensus(sumX, sumY);
                    double consensusLen = consensus.len();
                    if (consensusLen > bridgeConfig.goalDiversionMaxOffset) {
                        consensus = consensus * (bridgeConfig.goalDiversionMaxOffset / consensusLen);
                    }
                    goalDiversionOffsets[robotId] = consensus;
                }
            }
            stepData["bridge"]["nominal"]["goal_diversion"] = {
                {"enabled", true},
                {"active_count", goalDiversionActiveCount},
                {"distance_threshold", bridgeConfig.goalDiversionDistance},
                {"radial_threshold", bridgeConfig.goalDiversionRadial},
                {"separation_scale", bridgeConfig.goalDiversionSeparationScale},
                {"pair_scope", bridgeConfig.goalDiversionPairScope},
                {"lookahead_steps", bridgeConfig.goalDiversionLookaheadSteps},
                {"lookahead_distance_threshold", bridgeConfig.goalDiversionLookaheadDistance},
                {"lookahead_radial_threshold", bridgeConfig.goalDiversionLookaheadRadial},
                {"multi_pair", bridgeConfig.goalDiversionMultiPair},
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
        BridgeSingleLadderGoalDecision jointSingleLadderGoal;
        std::array<Point, 2> jointSingleLadderBases;
        Point jointSingleLadderLeaderPosition;
        if (bridgeConfig.jointSingleLadderGoalsEnabled) {
            if (robots.size() != 4 || robots.front()->bases.size() != 2) {
                throw std::runtime_error(
                        "r10 joint goal runtime requires four mobiles and two bases");
            }
            jointSingleLadderBases = {
                    robots.front()->bases.at(0),
                    robots.front()->bases.at(1),
            };
            jointSingleLadderLeaderPosition = positions.at(
                    bridgeConfig.jointSingleLadderLeaderId);
            if (bridgeConfig.postDetectionResponseEnabled) {
                jointSingleLadderGoal.tuple = bridgeR13Geometry.tuple;
                jointSingleLadderGoal.certificate =
                        bridgeR13Geometry.exactEightEdgeCertificate;
                jointSingleLadderGoal.evaluatedCandidates = 1;
                jointSingleLadderGoal.reusedPrevious = true;
            } else {
                jointSingleLadderGoal =
                        bridgeSearch.choosePersistentSingleLadderGoals(
                                jointSingleLadderLeaderPosition,
                                jointSingleLadderBases,
                                bridgeTopologyConfig.fixedReferences);
            }
            stepData["bridge"]["nominal"]["single_ladder_goal"] =
                    bridgeSingleLadderGoalDecisionJson(
                            jointSingleLadderGoal,
                            jointSingleLadderLeaderPosition,
                            jointSingleLadderBases,
                            bridgeTopologyConfig.fixedReferences,
                            Point(0.0, 0.0),
                            Point(bridgeWorldExtent(config, 0),
                                  bridgeWorldExtent(config, 1)));
        }
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
            if (bridgeConfig.jointSingleLadderGoalsEnabled) {
                return jointSingleLadderGoal.tuple.goals.at(
                        static_cast<std::size_t>(candidate->id - 1));
            }
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

        std::map<int, Eigen::VectorXd> taskNominals;
        for (auto &robot : robots) {
            Point position = robot->model->xy();
            Point goal = bridgeConfig.jointSingleLadderGoalsEnabled
                    ? jointSingleLadderGoal.tuple.goals.at(
                            static_cast<std::size_t>(robot->id - 1))
                    : bridgeSearch.chooseGoal(
                            position, bridgeConfig.searchPolicy);
            const auto previousTaskGoal = bridgePreviousTaskGoals.find(robot->id);
            const bool taskGoalChanged = previousTaskGoal
                            != bridgePreviousTaskGoals.end()
                    && previousTaskGoal->second.distance_to(goal) > 1.0e-9;
            taskGoalLinks.push_back({
                    {"robot", robot->id},
                    {"x", goal.x},
                    {"y", goal.y},
                    {"changed", taskGoalChanged},
            });
            bridgePreviousTaskGoals[robot->id] = goal;
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
            taskNominals.emplace(robot->id, nominal);
        }

        if (bridgeConfig.gammaStarFeedbackEnabled) {
            std::map<int, Eigen::Vector2d> taskAccelerations;
            for (const auto &[robotId, nominal] : taskNominals) {
                if (nominal.size() < 2 || !nominal.allFinite()) {
                    throw std::runtime_error(
                        "full-row feedback received an invalid task acceleration");
                }
                taskAccelerations.emplace(
                    robotId,
                    Eigen::Vector2d(nominal(0), nominal(1)));
            }
            const auto forecastStates =
                bridgePredictionStatesWithAccelerations(
                    feedbackStates,
                    taskAccelerations);

            std::map<int, FullRowFeedbackResult> feedbackResults;
            for (auto &robot : robots) {
                const Eigen::VectorXd &nominal = taskNominals.at(robot->id);
                auto fb = fullRowFeedbackNominal(
                        robot->id,
                        feedbackStates,
                        forecastStates,
                        feedbackBases,
                        feedbackRows,
                        nominal(0),
                        nominal(1));
                fb.diagnostic["task_reference"] = {
                        {"ax", nominal(0)}, {"ay", nominal(1)}};
                fb.diagnostic["neighbor_prediction_mode"] =
                        "synchronous-task-reference";
                feedbackResults.emplace(robot->id, std::move(fb));
            }

            for (auto &robot : robots) {
                Eigen::VectorXd nominal = taskNominals.at(robot->id);
                auto &fb = feedbackResults.at(robot->id);
                gammaStarFeedbackEvaluatedCandidates += static_cast<int>(
                        fb.diagnostic.value("candidates", json::array()).size());
                if (fb.diagnostic.contains("current_gamma_star")
                    && fb.diagnostic["current_gamma_star"].is_number()) {
                    gammaStarFeedbackMinCurrent = std::min(
                            gammaStarFeedbackMinCurrent,
                            fb.diagnostic["current_gamma_star"].get<double>());
                }
                if (fb.selected) {
                    const double alpha = fb.diagnostic.value("selected_alpha", 0.0);
                    if (alpha > bridgeConfig.nominalGuardTolerance) {
                        ++gammaStarFeedbackActiveCount;
                    }
                    gammaStarFeedbackMinPredicted = std::min(
                            gammaStarFeedbackMinPredicted,
                            fb.diagnostic.value(
                                    "selected_predictive_minimum_budget",
                                    std::numeric_limits<double>::infinity()));
                    nominal(0) = fb.selectedAccelX;
                    nominal(1) = fb.selectedAccelY;
                    auto entries = buildBridgePredictionAuditEntries(
                            bridgeFeedbackStepIndex,
                            robot->runtime,
                            robot->id,
                            dt,
                            fb.selectedForecast,
                            fb.selectedStepBudgets);
                    if (entries.size()
                            != static_cast<size_t>(
                                    bridgeConfig.gammaStarLookaheadSteps)) {
                        throw std::runtime_error(
                                "selected full-row forecast could not be registered for audit");
                    }
                    bridgePendingPredictionAudits.insert(
                            bridgePendingPredictionAudits.end(),
                            entries.begin(),
                            entries.end());
                }
                gammaStarFeedbackLinks.push_back(fb.diagnostic);
                robot->setNominalControlOverride(nominal);
            }
        } else {
            for (auto &robot : robots) {
                robot->setNominalControlOverride(taskNominals.at(robot->id));
            }
        }
        stepData["bridge"]["nominal"]["task_goals"] = taskGoalLinks;
        if (bridgeConfig.gammaStarFeedbackEnabled) {
            json fixedReferenceMap = json::object();
            for (const auto &[robotId, references] :
                 bridgeTopologyDecision.references) {
                fixedReferenceMap[std::to_string(robotId)] = {
                        {"anchor_ids", references.anchorIds},
                        {"base_ids", references.baseIds},
                };
            }
            stepData["bridge"]["nominal"]["gamma_star_feedback"] = {
                {"enabled", true},
                {"mode", bridgeConfig.gammaStarFeedbackMode},
                {"analysis_role", bridgeConfig.gammaStarFeedbackAnalysisRole},
                {"selection_rule", bridgeConfig.gammaStarFeedbackSelectionRule},
                {"constraint_execution",
                        bridgeConfig.gammaStarFeedbackConstraintExecution},
                {"homotopy_intervals", bridgeConfig.gammaStarHomotopyIntervals},
                {"lookahead_steps", bridgeConfig.gammaStarLookaheadSteps},
                {"predictive_gate", bridgeConfig.gammaStarPredictiveGate},
                {"physical_acceleration_bound", secondOrderAccelerationBound()},
                {"communication_distance_m", 850.0},
                {"safety_distance_m", 10.0},
                {"fixed_references", fixedReferenceMap},
                {"active_count", gammaStarFeedbackActiveCount},
                {"evaluated_candidates", gammaStarFeedbackEvaluatedCandidates},
                {"min_current_gamma_star", std::isfinite(gammaStarFeedbackMinCurrent)
                                              ? json(gammaStarFeedbackMinCurrent)
                                              : json(nullptr)},
                {"min_selected_predictive_budget",
                        std::isfinite(gammaStarFeedbackMinPredicted)
                                ? json(gammaStarFeedbackMinPredicted)
                                : json(nullptr)},
                {"links", gammaStarFeedbackLinks}
            };
            attachBridgePredictionAuditLog(resolvedPredictionAudits);
            ++bridgeFeedbackStepIndex;
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

    void finalizeBridgeFeedbackExecutionDiagnostics() {
        if (!bridgeConfig.enabled || !bridgeConfig.gammaStarFeedbackEnabled
            || !stepData.contains("bridge")
            || !stepData["bridge"].contains("nominal")
            || !stepData["bridge"]["nominal"].contains("gamma_star_feedback")) {
            return;
        }

        json &feedback =
                stepData["bridge"]["nominal"]["gamma_star_feedback"];
        if (!feedback.contains("links") || !feedback["links"].is_array()) {
            return;
        }
        for (auto &link : feedback["links"]) {
            const int robotId = link.value("robot", 0);
            if (robotId < 1 || robotId > static_cast<int>(robots.size())) {
                link["execution_consistent"] = false;
                link["diagnostic_error"] = "invalid robot id";
                continue;
            }
            Robot *robot = robots[robotId - 1].get();

            std::vector<std::string> expectedRows;
            if (link.contains("full_row_identities")) {
                expectedRows = link["full_row_identities"]
                        .get<std::vector<std::string>>();
            }
            const bool softExecution =
                    bridgeConfig.gammaStarFeedbackConstraintExecution == "soft";
            std::vector<std::string> installedRows;
            if (softExecution) {
                installedRows.reserve(robot->secondOrderCbfSlack.size());
                for (const auto &[identity, cbf] : robot->secondOrderCbfSlack) {
                    (void) cbf;
                    installedRows.push_back(identity);
                }
            } else {
                installedRows.reserve(robot->secondOrderCbfNoSlack.size());
                for (const auto &[identity, cbf] : robot->secondOrderCbfNoSlack) {
                    (void) cbf;
                    installedRows.push_back(identity);
                }
            }
            std::sort(expectedRows.begin(), expectedRows.end());
            std::sort(installedRows.begin(), installedRows.end());
            link["row_execution_class"] = softExecution ? "soft" : "hard";
            link["installed_row_identities"] = installedRows;
            link[softExecution ? "installed_soft_row_identities"
                               : "installed_hard_row_identities"] = installedRows;
            link["row_identity_consistent"] = expectedRows == installedRows;
            link["guard"] = robot->nominalGuardDiagnostic;
            const bool selected = link.value("selected", false);
            const bool softCurrentInfeasibleFallback =
                    softExecution
                    && !selected
                    && link.value("current_infeasible", false)
                    && link.value(
                            "certified_negative_current_hard_set", false)
                    && link.value(
                            "pre_score_constraint_ledger_consistent", false);
            link["soft_current_infeasible_fallback"] =
                    softCurrentInfeasibleFallback;

            const Eigen::Vector2d guarded(
                    robot->nominalControlOverride(0),
                    robot->nominalControlOverride(1));
            const Eigen::Vector2d executed = robot->model->getAcceleration();
            double guardDifference = std::numeric_limits<double>::infinity();
            double qpDifference = std::numeric_limits<double>::infinity();
            double softFallbackTaskGuardDifference =
                    std::numeric_limits<double>::infinity();
            if (selected) {
                const Eigen::Vector2d candidate(
                        link.at("selected_accel_x").get<double>(),
                        link.at("selected_accel_y").get<double>());
                guardDifference = (guarded - candidate).norm();
                qpDifference = (executed - candidate).norm();
            } else if (softCurrentInfeasibleFallback
                       && link.contains("task_reference")
                       && link["task_reference"].is_object()
                       && link["task_reference"].contains("ax")
                       && link["task_reference"].contains("ay")
                       && link["task_reference"]["ax"].is_number()
                       && link["task_reference"]["ay"].is_number()) {
                const Eigen::Vector2d taskReference(
                        link["task_reference"]["ax"].get<double>(),
                        link["task_reference"]["ay"].get<double>());
                softFallbackTaskGuardDifference =
                        (guarded - taskReference).norm();
            }
            double qpMinimumPhysicalMargin =
                    std::numeric_limits<double>::infinity();
            double qpMinimumRelaxedMargin =
                    std::numeric_limits<double>::infinity();
            double qpMaximumSlack = 0.0;
            double qpMinimumSlack = std::numeric_limits<double>::infinity();
            double qpSlackLowerBoundViolation = 0.0;
            size_t qpSlackCount = 0;
            const char *rowLogName = softExecution
                                     ? "hocbfSlack" : "hocbfNoSlack";
            if (robot->opt.contains(rowLogName)
                && robot->opt[rowLogName].is_array()) {
                for (const auto &row : robot->opt[rowLogName]) {
                    if (row.contains("hocbf") && row["hocbf"].is_number()) {
                        const double physicalMargin =
                                row["hocbf"].get<double>();
                        qpMinimumPhysicalMargin = std::min(
                                qpMinimumPhysicalMargin, physicalMargin);
                        const bool hasSlack = row.contains("slack")
                                              && row["slack"].is_number();
                        const double slack = hasSlack
                                ? row["slack"].get<double>() : 0.0;
                        if (softExecution && hasSlack) {
                            ++qpSlackCount;
                            qpMaximumSlack = std::max(qpMaximumSlack, slack);
                            qpMinimumSlack = std::min(qpMinimumSlack, slack);
                            qpSlackLowerBoundViolation = std::max(
                                    qpSlackLowerBoundViolation,
                                    std::max(0.0, -slack));
                        }
                        qpMinimumRelaxedMargin = std::min(
                                qpMinimumRelaxedMargin,
                                row.value(
                                        "hocbf_with_slack",
                                        physicalMargin + slack));
                    }
                }
            }
            const bool solverOptimal =
                    robot->opt.value("status", "failed") == "success";
            const double accelerationBound = secondOrderAccelerationBound();
            const double accelerationBoxViolation = std::max({
                    0.0,
                    std::abs(executed(0)) - accelerationBound,
                    std::abs(executed(1)) - accelerationBound});
            link["guard_output"] = {
                    {"ax", guarded(0)}, {"ay", guarded(1)}};
            link["qp_output"] = {
                    {"ax", executed(0)}, {"ay", executed(1)}};
            link["guard_candidate_difference"] = selected
                    ? json(guardDifference) : json(nullptr);
            link["qp_candidate_difference"] = selected
                    ? json(qpDifference) : json(nullptr);
            link["soft_fallback_task_guard_difference"] =
                    softCurrentInfeasibleFallback
                    && std::isfinite(softFallbackTaskGuardDifference)
                            ? json(softFallbackTaskGuardDifference)
                            : json(nullptr);
            link["guard_reproduction_tolerance"] =
                    BRIDGE_FULL_ROW_GUARD_REPRODUCTION_TOLERANCE;
            link["qp_reproduction_tolerance"] =
                    BRIDGE_FULL_ROW_QP_REPRODUCTION_TOLERANCE;
            link["qp_hard_margin_tolerance"] =
                    BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE;
            link["qp_minimum_physical_row_margin"] =
                    std::isfinite(qpMinimumPhysicalMargin)
                            ? json(qpMinimumPhysicalMargin) : json(nullptr);
            link["qp_minimum_hard_margin"] =
                    !softExecution && std::isfinite(qpMinimumPhysicalMargin)
                            ? json(qpMinimumPhysicalMargin) : json(nullptr);
            link["qp_minimum_relaxed_row_margin"] =
                    softExecution && std::isfinite(qpMinimumRelaxedMargin)
                            ? json(qpMinimumRelaxedMargin) : json(nullptr);
            link["qp_maximum_slack"] = softExecution
                    && qpSlackCount > 0
                            ? json(qpMaximumSlack) : json(nullptr);
            link["qp_minimum_slack"] = softExecution
                    && qpSlackCount > 0
                            ? json(qpMinimumSlack) : json(nullptr);
            link["qp_slack_lower_bound_violation"] = softExecution
                    ? json(qpSlackLowerBoundViolation) : json(nullptr);
            link["qp_slack_count"] = softExecution
                    ? json(qpSlackCount) : json(nullptr);
            link["acceleration_bound"] = accelerationBound;
            link["qp_acceleration_box_violation"] =
                    accelerationBoxViolation;
            link["solver_optimal"] = solverOptimal;
            const double relevantRowMargin = softExecution
                    ? qpMinimumRelaxedMargin : qpMinimumPhysicalMargin;
            const bool slackConsistent = !softExecution
                    || (qpSlackCount == installedRows.size()
                        && std::isfinite(qpMinimumSlack)
                        && qpSlackLowerBoundViolation
                                <= BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
            const bool selectionExecutionConsistent = selected
                    ? (link.value("candidate_current_feasible", false)
                       && guardDifference
                            <= BRIDGE_FULL_ROW_GUARD_REPRODUCTION_TOLERANCE
                       && qpDifference
                            <= BRIDGE_FULL_ROW_QP_REPRODUCTION_TOLERANCE)
                    : (softCurrentInfeasibleFallback
                       && softFallbackTaskGuardDifference
                            <= BRIDGE_FULL_ROW_GUARD_REPRODUCTION_TOLERANCE);
            link["execution_consistent"] =
                    link.value("pre_score_constraint_ledger_consistent", false)
                    && link["row_identity_consistent"].get<bool>()
                    && selectionExecutionConsistent
                    && solverOptimal
                    && slackConsistent
                    && std::isfinite(relevantRowMargin)
                    && relevantRowMargin
                            >= -BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE
                    && accelerationBoxViolation
                            <= BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE;
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

        if (config.value("debug", json::object()).value(
                    "write-centralized-optimization-model", false)) {
            optimizer->write("centralized_optimization.lp");
        }

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
