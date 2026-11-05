#include "LocalViewpoint.hpp"
#include "LeaderReuse.hpp"
#include "Triangle750.hpp"
#include "ShrunkSector.hpp"
#include <cstring>
#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "cbf/cbf"
#include "cbf/AllocatedPairwiseCBF.hpp"
#include "cbf/CBFConfig.hpp"
#include "cbf/FimRateCertificate.hpp"
#include "cbf/HardInteriorSelection.hpp"
#include "cbf/HybridCertificateGuard.hpp"
#include "cbf/LiteratureBaselines.hpp"
#include "world/world"
#include "cbf/HierarchicalTarget.hpp"
#include "cbf/M2ExactFiniteTarget.hpp"
#include "diagnostics/CanonicalCaptureAudit.hpp"
#include "cbf/CommandBall.hpp"
#include "models/models"
#include "optimisers/optimisers"
#include "communicators/communicators"

#include <optional>

typedef std::pair<int, Point> intPoint;

class Robot {
public:
    int id = 0;
    int n;
    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    std::unique_ptr<BaseModel> model;
    json opt;
    json tvtRowsSnapshot = json::object();
    std::unique_ptr<OptimiserBase> optimiser;
    std::unique_ptr<CommunicatorBase> comm;
    World world;
    GridWorld gridWorld;
    json settings;
    json myFormation = json::object();
    json myCovarianceFormation = json::object();
    json updatedGridWorld;
    std::string folderName;
    std::string filename;
    CVT cvt;
    double runtime;
    std::function<double(double)> uncertaintyFunction;

    Eigen::Matrix2d positionCovariance;
    bool enablePositionCovariance;
    double rangingSigma;
    double basePositionSigma;
    std::function<double(double)> rangingUncertaintyFunction;
    std::function<double(Eigen::Matrix2d)> uncertaintyFromCovarianceFunction;
    double previousUncertainty = 0.0;
    double currentUncertainty = 0.0;
    double uncertaintyRate = 0.0;
    bool hasUncertaintyHistory = false;
    cbf2026::NodeRateCertificate rateCertificate =
        cbf2026::baseRateCertificate(0, 0);
    bool certificateAvailable = false;
    bool estimatorInLoop = false;
    Point estimatorPosition;
    std::optional<std::uint64_t> estimatorSnapshotFrame;
    double estimatorEpsilon = 0.0;
    // Shared same-frame EKF covariance-derived scales; heuristics, not bounds.
    std::map<int, double> tvtEstimatorEpsilons;
    std::uint64_t certificateSnapshotVersion = 0;
    std::uint64_t certificateAllocationVersion = 0;
    std::optional<cbf2026::EndpointCertificateSnapshot>
        localCertificateSnapshot;
    cbf2026::CommittedCertificateState committedCertificateState;
    std::optional<cbf2026::HardConstraintProblem>
        lastConsumedHardConstraintProblem;
    std::string lastConsumedHardProblemHash;

    std::string cvtExplorationMode;
    double cvtFrontFocusDistance;

    int numParts, numSquad;
    std::function<int(int)> getPartId;
    std::function<int(int)> getIdInPart;
    std::function<bool(int)> isSamePartAsMe;

    std::vector<int> idsInMyPart;
    int idInMyPart;
    int endInMyPart;
    int partId;
    std::vector<int> endIds;
    std::map<int, int> endId2CvtId;
    std::map<int, int> cvtId2EndId;

    std::vector<Point> bases;
    std::vector<int> myBasesId;

    std::set<int> myNeighboursId;

    // External velocity data (for simulation environment)
    double externalVx = 0.0;
    double externalVy = 0.0;
    bool hasExternalVelocity = false;

public:

    Robot() = default;

    Robot(int id, json &settings)
            : id(id),
              n(settings["num"]),
              world(settings["world"]),
              gridWorld(settings["world"]),
              settings(settings),
              runtime(0.0) {
        if(canonical::active)++canonical::robotConstructorAttempts;
        settings["id"] = id;
        if (settings["model"] == "SingleIntegrate2D") {
            model = std::make_unique<SingleIntegrate2D>(settings);
        } else if (settings["model"] == "DoubleIntegrate2D") {
            model = std::make_unique<DoubleIntegrate2D>(settings);
        } else {
            throw std::invalid_argument("Invalid model type");
        }
        if(canonical::active)++canonical::environmentFactoryAttempts;
        optimiser = createOptimiser(settings["optimiser"], settings["cbfs"]["objective-function"]);
        if (settings.contains("uncertainty")) {
            auto unc_config = settings["uncertainty"];
            std::string method = unc_config.value("method", "const");

            if (method == "const" && unc_config.contains("const")) {
                double const_uncertainty = unc_config["const"]["epsilon"];
                // Set constant function
                uncertaintyFunction = [const_uncertainty](double distance) { return const_uncertainty; };
            } else if (method == "dist" && unc_config.contains("dist")) {
                // Distance-based uncertainty: linear function y = kx + b
                auto dist_config = unc_config["dist"];
                double k = dist_config.value("k", 0.05);    // Slope: uncertainty increase per meter
                double b = dist_config.value("b", 1.0);     // Intercept: uncertainty at origin

                uncertaintyFunction = [k, b](double distance) {
                    return k * distance + b;
                };
            } else {
                // Default constant function (zero uncertainty)
                uncertaintyFunction = [](double distance) { return 0.0; };
            }
        } else {
            // Default constant function (zero uncertainty)
            uncertaintyFunction = [](double distance) { return 0.0; };
        }

        // Initialize Scissor uncertainty propagation
        enablePositionCovariance = false;
        rangingSigma = 5.0;
        basePositionSigma = 1.0;
        positionCovariance = Eigen::Matrix2d::Zero();

        rangingUncertaintyFunction = [this](double distance) { return rangingSigma; };

        if (settings.contains("position_covariance")) {
            auto config = settings["position_covariance"];
            enablePositionCovariance = config.value("enable", false);

            if (config.contains("ranging_sigma")) {
                rangingSigma = config["ranging_sigma"];
                rangingUncertaintyFunction = [this](double distance) { return rangingSigma; };
            }

            // Initialize uncertainty from covariance function
            std::string uncertainty_type = config.value("uncertainty-type", "std_avg");
            if (uncertainty_type == "max_eigenvalue") {
                // Use 3-sigma ellipse semi-major axis: 3 * sqrt(lambda_max)
                uncertaintyFromCovarianceFunction = [](Eigen::Matrix2d cov) {
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
                    double lambda_max = solver.eigenvalues().maxCoeff();
                    return 3.0 * std::sqrt(lambda_max);
                };
            } else {
                // Default method: average of standard deviations (std_avg)
                uncertaintyFromCovarianceFunction = [](Eigen::Matrix2d cov) {
                    return (std::sqrt(cov(0, 0)) + std::sqrt(cov(1, 1))) / 2;
                };
            }
        } else {
            // Default to std_avg if config not found
            uncertaintyFromCovarianceFunction = [](Eigen::Matrix2d cov) {
                return (std::sqrt(cov(0, 0)) + std::sqrt(cov(1, 1))) / 2;
            };
        }

        if (settings["cbfs"]["with-slack"]["cvt"]["on"]) {
            auto config = settings["cbfs"]["with-slack"]["cvt"];
            if (config.contains("exploration-mode")) {
                cvtExplorationMode = config["exploration-mode"];
            }
            else {
                throw std::invalid_argument("CVT: exploration-mode not specified");
            }

            if (config.contains("front-focus-distance")) {
                cvtFrontFocusDistance = config["front-focus-distance"];
            }
            else {
                throw std::invalid_argument("CVT: front-focus-distance not specified");
            }
        }

        numParts = settings["formation"]["parts"];
        numSquad = (n - 1) / numParts + 1;
        getPartId = [this](int id) { return (id - 1) / numSquad + 1; };
        getIdInPart = [this](int id) { return (id - 1) % numSquad + 1; };
        partId = getPartId(id);
        idInMyPart = getIdInPart(id);
        isSamePartAsMe = [this](int id) { return getPartId(id) == partId; };

        idsInMyPart.clear();
        endId2CvtId.clear();
        cvtId2EndId.clear();
        endIds.clear();

        // Always set myNeighboursId for constraint violation detection
        // regardless of whether comm-fixed CBF is enabled
        auto commConfig = settings["cbfs"]["without-slack"]["comm-fixed"];
        int minOffset = commConfig.value("min-neighbour-id-offset", -2);
        int maxOffset = commConfig.value("max-neighbour-id-offset", 0);

        for (int i = 1; i <= n; i++) {
            if (isSamePartAsMe(i)) {
                idsInMyPart.push_back(i);
            }
            if (i == n || getIdInPart(i) == numSquad) {
                endIds.push_back(i);
            }
            // Always calculate neighbours for constraint violation detection
            if (i >= id + minOffset && i <= id + maxOffset && isSamePartAsMe(i) && i != id) {
                myNeighboursId.insert(i);
            }
        }
        for (int i = 0; i < endIds.size(); i++) {
            int endId = endIds[i];
            int cvtId = i + 1;
            endId2CvtId[endId] = cvtId;
            cvtId2EndId[cvtId] = endId;
        }
        endInMyPart = idsInMyPart.back();

        bases = getPointsFromJson(settings["bases"]);
        for (auto &id: settings["formation"]["bases-id"][partId - 1]) {
            myBasesId.push_back(int(id));
        }

        setup();
        cbfSlack.clear();
        cbfNoSlack.cbfs.clear();
        comm = std::make_unique<CommunicatorCentral>(settings);
        if(canonical::active)++canonical::robotConstructorReturns;
    }

    void setup() {
        std::string method = settings["initial"]["position"]["method"];
        if (method == "random-in-world") {
             model->setPosition2D(world.getRandomPoint());
        } else if (method == "random-in-polygon") {
            Polygon poly = Polygon(getPointsFromJson(settings["initial"]["position"]["polygon"]));
            model->setPosition2D(poly.get_random_point());
        } else if (method == "specified") {
            std::vector<Point> initialPositions = getPointsFromJson(settings["initial"]["position"]["positions"]);
            model->setPosition2D(initialPositions[id - 1]);
        } else {
            throw std::invalid_argument("Invalid method for setting initial position");
        }

        double bMin = settings["initial"]["battery"]["min"], bMax = settings["initial"]["battery"]["max"];
        if(settings.contains("canonical-replay-state")) {
            const auto& saved=settings.at("canonical-replay-state").at(id-1);
            if(saved.at("id")!=id || saved.at("runtime_s")!=0.)throw std::runtime_error("shared replay ID/phase mismatch");
            const auto& x=saved.at("state");Eigen::VectorXd state(4),u(3);
            state<<x.at("x").get<double>(),x.at("y").get<double>(),x.at("battery").get<double>(),x.at("yawRad").get<double>();
            const auto& input=saved.at("u0");u<<input.at("vx").get<double>(),input.at("vy").get<double>(),input.at("yawRateRad").get<double>();
            if(!state.allFinite() || !u.allFinite())throw std::runtime_error("shared replay nonfinite state");
            model->setStateVector(state);model->setControlInput(u);runtime=saved.at("runtime_s");
        }else {
            if(canonical::active)++canonical::repoRandAttempts;
            model->setStateVariable("battery", bMin + (bMax - bMin) * rand() / RAND_MAX);
            if(canonical::active)++canonical::repoRandReturns;
            model->setYawDeg(settings["initial"]["yawDeg"]);
        }
    }

    void checkRobotsInsideWorld() {
        auto xLim = world.boundary.get_x_limit(1.5), yLim = world.boundary.get_y_limit(1.5);
        auto robotPosition = model->xy();
        if (robotPosition.x < xLim.first || robotPosition.x > xLim.second ||
            robotPosition.y < yLim.first || robotPosition.y > yLim.second) {
            throw std::runtime_error("Robot is outside the world");
        }
    }

    void setEnergyCBF() {
        auto config = settings["cbfs"]["without-slack"]["energy"];
        auto batteryH = [&, config](VectorXd x, double t) {
            std::function<double(Point)> minDistanceToChargingStations = [&](Point myPosition) {
                return (
                        world.distanceToChargingStations(myPosition)
                        / world.chargingStations[world.nearestChargingStation(myPosition)].second
                );
            };

            std::function rho = [&, config](Point p) {
                double k = config["k"];
                return k * log(minDistanceToChargingStations(p));
            };

            std::string energy_value_type = config.value("energy-value", "neighbour-min");
            double energy_value;

            if (energy_value_type == "self") {
                energy_value = model->extractFromVector(x, "battery");
            } else if (energy_value_type == "neighbour-min"){
                energy_value = model->extractFromVector(x, "battery");
                if (myFormation.contains("anchorIds")) {
                    for (int neighborId : myFormation["anchorIds"]) {
                        if (comm->_othersBatteryLevel.find(neighborId) != comm->_othersBatteryLevel.end()) {
                            energy_value = std::min(energy_value, comm->_othersBatteryLevel[neighborId]);
                        }
                    }
                }
            }
            else {
                throw std::invalid_argument("unknown energy-value type");
            }

            bool normalize_on = config["normalize"].value("on", false);
            double normalized_energy;

            if (normalize_on) {
                double normalize_max = config["normalize"].value("max", 100);
                double battery_min = model->BATTERY_MIN;
                double battery_max = model->BATTERY_MAX;
                normalized_energy = (energy_value - battery_min) / (battery_max - battery_min) * normalize_max;
            } else {
                double min_value = model->BATTERY_MIN;
                normalized_energy = energy_value - min_value;
            }

            return normalized_energy - rho(model->extractXYFromVector(x));
        };

        CBF energyCBF;
        energyCBF.name = config["name"];
        energyCBF.h = batteryH;
        ClassKParameters alphaParameters = readClassKParameters(config, 0.1, 1);
        energyCBF.setAlphaClassK(alphaParameters.coefficient, alphaParameters.power);

        energyCBF.dhdx_analytical = [&, config](VectorXd x, double t) -> VectorXd {
            bool normalize_on = config["normalize"]["on"];

            Point currentPos = model->extractXYFromVector(x);
            int nearestIdx = world.nearestChargingStation(currentPos);
            Point nearestStation = world.chargingStations[nearestIdx].first;

            double d_min = currentPos.distance_to(nearestStation);

            VectorXd grad = VectorXd::Zero(4);

            double normalize_max = config["normalize"].value("max", 100);
            double battery_min = model->BATTERY_MIN;
            double battery_max = model->BATTERY_MAX;

            if (normalize_on) {
                // For normalized case: h = (E - min) / (max - min) * max - k*log(d)
                grad[2] = normalize_max / (battery_max - battery_min);
            } else {
                // For non-normalized case: h = (E - min) - k*log(d)
                grad[2] = 1.0;
            }

            Point direction = (currentPos - nearestStation) / d_min;

            double k = config["k"];

            // The spatial derivative (∂h/∂x, ∂h/∂y) is the same for both cases
            grad[0] = -k * direction.x / d_min;  // ∂h/∂x
            grad[1] = -k * direction.y / d_min;  // ∂h/∂y

            grad[3] = 0; // ∂h/∂θ

            return grad;
        };

        energyCBF.dhdt_analytical = [&, config](VectorXd x, double t) -> double {
            return 0.0;
        };

        cbfNoSlack.cbfs[energyCBF.name] = energyCBF;
    }

    void setTargetYawCBF() {
        auto config = settings["cbfs"]["with-slack"]["target-yaw"];
        std::function<double(Point, double)> headingToNearestTarget = [&](Point myPosition, double t) {
            double res = -1, headingRad = 0.0;
            for (auto target: world.targets) {
                if (target.visibleAtTime(t)) {
                    if (res == -1 || myPosition.distance_to(target.pos(t)) < res) {
                        res = myPosition.distance_to(target.pos(t));
                        headingRad = myPosition.angle_to(target.pos(t));
                    }
                }
            }
            return headingRad;
        };
        CBF targetYawCBF;
        targetYawCBF.name = config["name"];
        targetYawCBF.h = [&, config](VectorXd x, double t) {
            Point myPosition = model->extractXYFromVector(x);
            double headingRad = headingToNearestTarget(myPosition, t);
            double deltaHeadingRad = headingRad - model->extractFromVector(x, "yawRad");
            deltaHeadingRad = atan2(sin(deltaHeadingRad), cos(deltaHeadingRad));
            double kp = config["kp"];
            return kp * (1.0 - cos(deltaHeadingRad)) / (-2.0);
        };
        cbfSlack[targetYawCBF.name] = targetYawCBF;
    }

    void presetCBF() {
        setupFormation();
        if (settings["cbfs"]["without-slack"]["energy"]["on"]) setEnergyCBF();
        if (settings["cbfs"]["with-slack"]["target-yaw"]["on"]) setTargetYawCBF();
    }

    void setCommunicationAutoCBF(const json& config) {
        double maxRange = config["max-range"];
        double maxConsiderRange = config["max-consider-range"];
        Point origin(0, 0);

        std::vector<intPoint> neighbours;
        neighbours.clear();

        for (auto &[id, pos2d]: comm->_othersPos) {
            if (id == this->id) continue;
            if (model->xy().distance_to(pos2d) > maxConsiderRange) continue;
            if (pos2d.distance_to(origin) < model->xy().distance_to(origin)) continue;
            neighbours.push_back({id, pos2d});
        }

        std::sort(
                neighbours.begin(), neighbours.end(),
                [&](intPoint a, intPoint b) {
                    return a.second.distance_to(origin) < b.second.distance_to(origin);
                }
        );

        auto closeNeighbours = std::vector<intPoint> (
                neighbours.begin(),
                neighbours.begin() + std::min(2, (int) neighbours.size())
        );

        std::vector<Point> formationPoints;
        myFormation = {
            {"id",           id},
            {"anchorPoints", json::array()},
            {"anchorIds",    json::array()}
        };

        for (auto &[id, pos]: closeNeighbours) {
            myFormation["anchorIds"].push_back(id);
            formationPoints.push_back(pos);
        }

        if (formationPoints.size() == 0) return;

        auto autoFormationCommH = [this, formationPoints, maxRange, config](VectorXd x, double t) {
            Point myPosition = model->extractXYFromVector(x);

            double k = config["k"];

            double h = inf;
            for (auto &point: formationPoints) {
                h = std::min(
                        h,
                        k * (
                                maxRange -
                                myPosition.distance_to(point)
                        )
                );
            }
            return h == inf ? 0 : h;
        };

        CBF commCBF;
        commCBF.name = "commCBF";
        commCBF.h = autoFormationCommH;
        cbfNoSlack.cbfs[commCBF.name] = commCBF;
    }

    json fixedLocalizationReferencesFor(int ownerId) const {
        if (ownerId <= 0 || ownerId > n) {
            throw std::invalid_argument(
                "fixed localization owner is outside the mission"
            );
        }
        json references = {
            {"anchorIds", json::array()},
            {"baseIds", json::array()}
        };

        const auto commConfig =
            settings["cbfs"]["without-slack"]["comm-fixed"];
        const int minOffset =
            commConfig.value("min-neighbour-id-offset", -2);
        const int maxOffset =
            commConfig.value("max-neighbour-id-offset", 0);
        const int ownerLocalIndex = getIdInPart(ownerId);
        const int maximumBaseIndex =
            -ownerLocalIndex - minOffset;
        const int ownerPartId = getPartId(ownerId);
        const auto& ownerBaseIds =
            settings["formation"]["bases-id"].at(ownerPartId - 1);

        for (std::size_t index = 0; index < ownerBaseIds.size(); ++index) {
            if (static_cast<int>(index) > maximumBaseIndex) {
                continue;
            }
            references["baseIds"].push_back(ownerBaseIds[index]);
        }

        for (int otherId = 1; otherId <= n; ++otherId) {
            if (getPartId(otherId) != ownerPartId) {
                continue;
            }
            if (getIdInPart(otherId) >= ownerLocalIndex) {
                continue;
            }
            if (otherId < ownerId + minOffset
                || otherId > ownerId + maxOffset) {
                continue;
            }
            references["anchorIds"].push_back(otherId);
        }
        return references;
    }

    json fixedLocalizationReferences() const {
        return fixedLocalizationReferencesFor(id);
    }

    cbf2026::BarrierEdgeRegistry fixedBarrierEdgeRegistry() const {
        std::vector<cbf2026::FixedLocalizationReference> references;
        for (int ownerId = 1; ownerId <= n; ++ownerId) {
            const json ownerReferences =
                fixedLocalizationReferencesFor(ownerId);
            for (const json& baseId : ownerReferences.at("baseIds")) {
                references.push_back({
                    ownerId, baseId.get<int>(), true
                });
            }
            for (const json& referenceId :
                 ownerReferences.at("anchorIds")) {
                references.push_back({
                    ownerId, referenceId.get<int>(), false
                });
            }
        }
        return cbf2026::BarrierEdgeRegistry(n, references);
    }

    json incidentFixedLocalizationReferences() const {
        json references = {
            {"anchorIds", json::array()},
            {"baseIds", json::array()}
        };
        const auto registry = fixedBarrierEdgeRegistry();
        for (const auto& edge : registry.incidentEdges(id)) {
            if (edge.kind != cbf2026::EdgeKind::Localization) {
                continue;
            }
            if (edge.baseId >= 0) {
                references["baseIds"].push_back(edge.baseId);
                continue;
            }
            references["anchorIds"].push_back(
                edge.low == id ? edge.high : edge.low
            );
        }
        return references;
    }

    json activeLocalizationReferences(const json& config) const {
        json references = fixedLocalizationReferences();
        const double maxRange = config.at("max-range").get<double>();
        const Point myPosition = model->xy();

        auto appendUnique = [](json& ids, int candidate) {
            for (const json& existing : ids) {
                if (existing.get<int>() == candidate) return;
            }
            ids.push_back(candidate);
        };

        auto deduplicate = [&appendUnique](json& ids) {
            json uniqueIds = json::array();
            for (const json& existing : ids) {
                appendUnique(uniqueIds, existing.get<int>());
            }
            ids = uniqueIds;
        };

        deduplicate(references["anchorIds"]);
        deduplicate(references["baseIds"]);

        if (usesAnalyticTopologicalRateCertificate()) {
            const auto& covarianceConfig = settings.at("position_covariance");
            const std::string referenceSelection =
                covarianceConfig.value(
                    "reference-selection", "dynamic-lower-index"
                );
            if (referenceSelection == "fixed-cbf-only") {
                return references;
            }
            if (referenceSelection != "dynamic-lower-index") {
                throw std::invalid_argument(
                    "position covariance reference selection is unknown"
                );
            }
        }

        for (std::size_t baseId = 0; baseId < bases.size(); ++baseId) {
            if (bases.at(baseId).distance_to(myPosition) <= maxRange) {
                appendUnique(references["baseIds"], static_cast<int>(baseId));
            }
        }

        if (usesAnalyticTopologicalRateCertificate()) {
            for (const auto& [otherId, snapshot] :
                 comm->_othersEndpointCertificateSnapshots) {
                if (!isSamePartAsMe(otherId)) continue;
                if (getIdInPart(otherId) >= idInMyPart) continue;
                cbf2026::validateEndpointCertificateSnapshot(
                    otherId, snapshot
                );
                if (snapshot.snapshotVersion
                        != certificateSnapshotVersion
                    || snapshot.allocationVersion
                       != certificateAllocationVersion) {
                    throw std::invalid_argument(
                        "active localization certificate version mismatch"
                    );
                }
                const Point otherPosition(
                    snapshot.estimate.x(), snapshot.estimate.y()
                );
                if (otherPosition.distance_to(myPosition) > maxRange) {
                    continue;
                }
                appendUnique(references["anchorIds"], otherId);
            }
        } else {
            for (const auto& [otherId, otherPosition] : comm->_othersPos) {
                if (!isSamePartAsMe(otherId)) continue;
                if (getIdInPart(otherId) >= idInMyPart) continue;
                if (otherPosition.distance_to(myPosition) > maxRange) continue;
                appendUnique(references["anchorIds"], otherId);
            }
        }
        return references;
    }

    bool usesAnalyticTopologicalRateCertificate() const {
        return settings.contains("cbfs")
               && settings["cbfs"].contains("uncertainty-rate")
               && settings["cbfs"]["uncertainty-rate"].value("mode", "off")
                  == "analytic-topological";
    }

    bool usesEstimatedTaskContract() const {
        return settings.value("tvt-experiment", json::object())
            .value("information-contract", "legacy-task-truth") == "estimated-task-v1";
    }

    Point taskPosition() const {
        if (!usesEstimatedTaskContract()) return model->xy();
        if (!estimatorInLoop) throw std::runtime_error("estimated task contract requires an active position snapshot");
        if ((settings.value("tvt-experiment",json::object()).value("controller-identity", "") == "symmetric-l1-main-v1" || usesCommandBallRows())
            && (!estimatorSnapshotFrame || *estimatorSnapshotFrame != static_cast<std::uint64_t>(std::llround(runtime / settings.at("execute").at("time-step").get<double>()))))
            throw std::runtime_error("main estimated task requires the current validated frame");
        return estimatorPosition;
    }

    VectorXd taskEvaluationState() const {
        VectorXd state = model->getX();
        if (usesEstimatedTaskContract()) {
            const Point point = taskPosition();
            state[0] = point.x; state[1] = point.y;
        }
        return state;
    }

    bool usesSymmetricL1Rows() const {
        return settings.value("tvt-experiment", json::object())
            .value("row-mode", "legacy") == "symmetric-l1";
    }

    std::optional<Eigen::Vector3d> canoPcgNominal;
    std::optional<std::uint64_t> canoPcgNominalFrame;
    json canoPcgNominalRecord;
    json tvtSymmetricRows = json::array();
    json tvtLiteratureRows = json::array();
    json tvtBaselineTask = json::object();
    std::optional<std::uint64_t> tvtCommandBallCommandFrame;
    bool usesCommandBallRows() const {
        return settings.value("tvt-experiment",json::object()).value("row-mode","legacy")=="oruganti-command-ball";
    }
    bool usesLiteratureRows() const {
        const std::string mode=settings.value("tvt-experiment",json::object()).value("row-mode","legacy");
        return mode=="gdms-min" || mode=="sd-drcbf-socp" || mode=="oruganti-command-ball";
    }
    bool usesNormConeRows() const {
        return settings.value("tvt-experiment",json::object()).value("row-mode","legacy")=="sd-drcbf-socp" || usesCommandBallRows();
    }
    void setTvtLiteratureRows(const std::string& mode) {
        if (!usesEstimatedTaskContract() || !estimatorInLoop || settings.at("model")!="SingleIntegrate2D")
            throw std::invalid_argument("literature adaptation requires common estimated first-order contract");
        cbfNoSlack.cbfs.clear();
        std::vector<Point> positions;
        std::map<int,Eigen::VectorXd> commands;
        for(int peer=1;peer<=n;++peer) {positions.push_back(comm->_othersPos.at(peer));commands[peer]=comm->_othersVel.at(peer);}
        const auto e=settings.at("tvt-experiment");
        tvtLiteratureRows=tvt_literature::pairRows(id,positions,bases,myFormation,commands,usesCommandBallRows()?"sd-drcbf-socp":mode,
            e.at("node-error-bound-m"),settings.at("execute").at("time-step"),configuredPlanarComponentMax(),
            e.at("squared-alpha"),settings.at("cbfs").at("without-slack").at("safety").at("safe-distance"),
            settings.at("cbfs").at("without-slack").at("comm-fixed").at("max-range"));
        if(usesCommandBallRows()) {
            for(const auto& peer:myFormation.at("anchorIds"))if(peer.get<int>()>=id)throw std::invalid_argument("command ball requires predecessor-only links");
            tvtLiteratureRows=cbf2026::commandBallRows(tvtLiteratureRows,e.at("squared-alpha"),settings.at("execute").at("time-step"));
            for(auto& row:tvtLiteratureRows)row["command_snapshot_frame_index"]=estimatorSnapshotFrame?json(*estimatorSnapshotFrame):json(nullptr);
        }
    }

    void applyEstimatorInput(
        const Point& position, double epsilon, bool active,
        std::optional<std::uint64_t> frame = std::nullopt
    ) {
        if (active && (!std::isfinite(position.x) || !std::isfinite(position.y)
            || !std::isfinite(epsilon) || epsilon<=0))
            throw std::invalid_argument("active estimator input must be finite with positive epsilon");
        estimatorSnapshotFrame = active ? frame : std::nullopt;
        estimatorInLoop = active;
        estimatorPosition = position;
        estimatorEpsilon = epsilon;
    }

    double configuredPlanarComponentMax() const {
        const json inputLimitsConfig =
            settings["cbfs"].value("input-limits", json::object());
        const double value =
            inputLimitsConfig.value("planar-component-max", 25.0);
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(
                "input-limits.planar-component-max must be finite and positive"
            );
        }
        return value;
    }

    void getCovariance(const json& config) {
        const bool theoremAligned = usesAnalyticTopologicalRateCertificate();
        certificateAvailable = false;
        localCertificateSnapshot.reset();
        if (!enablePositionCovariance) {
            return;
        }
        const json inputLimitsConfig =
            settings["cbfs"].value("input-limits", json::object());
        if (theoremAligned
            && (!inputLimitsConfig.value("on", false)
                || !inputLimitsConfig.contains("planar-component-max")
                || !inputLimitsConfig.at("planar-component-max").is_number()
                || !std::isfinite(
                    inputLimitsConfig.at("planar-component-max").get<double>()
                )
                || inputLimitsConfig.at("planar-component-max").get<double>()
                   <= 0.0
                || settings.value("model", "") != "SingleIntegrate2D"
                || model->uSize() != 3)) {
            throw std::invalid_argument(
                "analytic topological certificate requires enabled +/-25 planar QP bounds"
            );
        }

        const json references = activeLocalizationReferences(config);
        json certificateReferences = references;
        auto sortIds = [](json& ids) {
            std::vector<int> ordered = ids.get<std::vector<int>>();
            std::sort(ordered.begin(), ordered.end());
            ids = ordered;
        };
        sortIds(certificateReferences["baseIds"]);
        sortIds(certificateReferences["anchorIds"]);
        const json& diagnosticReferences = theoremAligned
            ? certificateReferences
            : references;
        myCovarianceFormation = {
            {"id", id},
            {"anchorIds", diagnosticReferences.at("anchorIds")},
            {"baseIds", diagnosticReferences.at("baseIds")}
        };

        const Point currentPosition = model->xy();
        const double planarComponentMax = theoremAligned
            ? inputLimitsConfig.at("planar-component-max").get<double>()
            : 25.0;
        const double uavSpeedBound =
            std::sqrt(2.0) * planarComponentMax;
        std::vector<cbf2026::ReferenceRateInput> rateInputs;

        auto makeGeometry = [&](
            int referenceId,
            int localIndex,
            bool hoveringBase,
            const Point& referencePosition,
            const Eigen::Matrix2d& covariance,
            double covarianceRateBound,
            double speedBound,
            std::uint64_t predecessorSnapshotVersion
        ) {
            const double distance =
                currentPosition.distance_to(referencePosition);
            const double rangingSigmaAtDistance =
                rangingUncertaintyFunction(distance);
            const double rangingVariance =
                rangingSigmaAtDistance * rangingSigmaAtDistance;
            if (!std::isfinite(rangingVariance)
                || rangingVariance <= 0.0) {
                throw std::invalid_argument(
                    "#" + std::to_string(id)
                    + " has invalid covariance validity: total variance for active reference must be finite and positive"
                );
            }
            const Eigen::Vector2d displacement(
                currentPosition.x - referencePosition.x,
                currentPosition.y - referencePosition.y
            );
            rateInputs.push_back({
                {
                    referenceId,
                    localIndex,
                    predecessorSnapshotVersion,
                    hoveringBase,
                    covariance,
                    covarianceRateBound,
                    speedBound
                },
                distance,
                displacement / distance,
                rangingVariance
            });
        };

        for (const json& baseIdJson :
             certificateReferences.at("baseIds")) {
            const int baseId = baseIdJson.get<int>();
            if (baseId < 0 || baseId >= static_cast<int>(bases.size())) {
                throw std::invalid_argument(
                    "#" + std::to_string(id)
                    + " has an invalid active base reference "
                    + std::to_string(baseId)
                );
            }
            makeGeometry(
                cbf2026::canonicalBaseReferenceId(baseId),
                0,
                true,
                bases.at(baseId),
                Eigen::Matrix2d::Zero(),
                0.0,
                0.0,
                certificateSnapshotVersion
            );
        }

        for (const json& anchorIdJson :
             certificateReferences.at("anchorIds")) {
            const int otherId = anchorIdJson.get<int>();
            if (theoremAligned) {
                const auto transportedIt =
                    comm->_othersEndpointCertificateSnapshots.find(otherId);
                if (transportedIt
                    == comm->_othersEndpointCertificateSnapshots.end()) {
                    throw std::invalid_argument(
                        "#" + std::to_string(id)
                        + " is missing atomic predecessor certificate for #"
                        + std::to_string(otherId)
                    );
                }
                cbf2026::validateEndpointCertificateSnapshot(
                    otherId, transportedIt->second
                );
                const auto& transported = transportedIt->second;
                if (transported.snapshotVersion
                        != certificateSnapshotVersion
                    || transported.allocationVersion
                       != certificateAllocationVersion) {
                    throw std::invalid_argument(
                        "transported predecessor certificate version mismatch"
                    );
                }
                makeGeometry(
                    cbf2026::canonicalUavReferenceId(otherId),
                    getIdInPart(otherId),
                    false,
                    Point(
                        transported.estimate.x(),
                        transported.estimate.y()
                    ),
                    transported.covariance,
                    transported.covarianceRateBound,
                    uavSpeedBound,
                    transported.snapshotVersion
                );
            } else {
                const auto positionIt = comm->_othersPos.find(otherId);
                const auto covarianceIt =
                    comm->_othersPositionCovariance.find(otherId);
                if (positionIt == comm->_othersPos.end()
                    || covarianceIt
                    == comm->_othersPositionCovariance.end()) {
                    throw std::invalid_argument(
                        "#" + std::to_string(id)
                        + " is missing active localization data for #"
                        + std::to_string(otherId)
                    );
                }
                makeGeometry(
                    cbf2026::canonicalUavReferenceId(otherId),
                    getIdInPart(otherId),
                    false,
                    positionIt->second,
                    covarianceIt->second,
                    0.0,
                    uavSpeedBound,
                    certificateSnapshotVersion
                );
            }
        }

        cbf2026::NodeRateCertificate candidate;
        try {
            candidate = cbf2026::computeNodeRateCertificate({
                id,
                idInMyPart,
                certificateSnapshotVersion,
                planarComponentMax,
                std::move(rateInputs)
            });
        } catch (const std::invalid_argument& error) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance FIM certificate: "
                + error.what()
            );
        }

        rateCertificate = std::move(candidate);
        positionCovariance = rateCertificate.covariance;
        if (theoremAligned) {
            try {
                localCertificateSnapshot =
                    cbf2026::makeEndpointCertificateSnapshot(
                        rateCertificate,
                        Eigen::Vector2d(
                            currentPosition.x,
                            currentPosition.y
                        ),
                        certificateAllocationVersion
                    );
                cbf2026::validateEndpointCertificateSnapshot(
                    id, *localCertificateSnapshot
                );
                currentUncertainty = rateCertificate.epsilon;
                certificateAvailable = true;
            } catch (...) {
                localCertificateSnapshot.reset();
                throw;
            }
        }
    }

    double positiveBackwardUncertaintyRate(double nextUncertainty, double dt) const {
        if (dt <= 0.0) {
            throw std::invalid_argument("Uncertainty rate requires a positive time step");
        }
        return std::max(0.0, (nextUncertainty - previousUncertainty) / dt);
    }

    void updateUncertaintyHistory(double nextUncertainty, double dt) {
        if (dt <= 0.0) {
            throw std::invalid_argument("Uncertainty history requires a positive time step");
        }
        if (!hasUncertaintyHistory) {
            previousUncertainty = nextUncertainty;
            currentUncertainty = nextUncertainty;
            uncertaintyRate = 0.0;
            hasUncertaintyHistory = true;
            return;
        }

        previousUncertainty = currentUncertainty;
        uncertaintyRate = positiveBackwardUncertaintyRate(nextUncertainty, dt);
        currentUncertainty = nextUncertainty;
    }

    void updateCovarianceAndRate(double dt) {
        const auto& commConfig =
            settings["cbfs"]["without-slack"]["comm-fixed"];
        const bool theoremAligned =
            usesAnalyticTopologicalRateCertificate();
        const double descriptiveCurrentBeforeRefresh = currentUncertainty;
        getCovariance(commConfig);
        const double nextUncertainty =
            theoremAligned
            ? rateCertificate.epsilon
            : uncertaintyFromCovarianceFunction(positionCovariance);
        if (theoremAligned) {
            currentUncertainty = descriptiveCurrentBeforeRefresh;
        }
        updateUncertaintyHistory(nextUncertainty, dt);
    }

    void setupFormation() {
        // Setup formation information for constraint violation detection
        // This should always be called, regardless of whether comm-fixed CBF is enabled
        // Uses the already-initialized myNeighboursId and myBasesId

        const json references = usesAnalyticTopologicalRateCertificate()
            ? incidentFixedLocalizationReferences()
            : fixedLocalizationReferences();
        myFormation = {
            {"id", id},
            {"anchorPoints", json::array()},
            {"anchorIds", references.at("anchorIds")},
            {"baseIds", references.at("baseIds")}
        };

        for (const json& baseIdJson : references.at("baseIds")) {
            const int baseId = baseIdJson.get<int>();
            const Point& base = bases.at(baseId);
            myFormation["anchorPoints"].push_back({base.x, base.y});
        }
    }

    const cbf2026::EndpointCertificateSnapshot&
    localEndpointCertificateSnapshot() {
        if (!certificateAvailable
            || !localCertificateSnapshot.has_value()
            || rateCertificate.robotId != id
            || rateCertificate.snapshotVersion
               != certificateSnapshotVersion
            || localCertificateSnapshot->robotId != id
            || localCertificateSnapshot->snapshotVersion
               != certificateSnapshotVersion
            || localCertificateSnapshot->allocationVersion
               != certificateAllocationVersion) {
            certificateAvailable = false;
            localCertificateSnapshot.reset();
            throw std::invalid_argument(
                "local analytic endpoint certificate is unavailable"
            );
        }
        try {
            cbf2026::validateEndpointCertificateSnapshot(
                id, *localCertificateSnapshot
            );
        } catch (...) {
            certificateAvailable = false;
            localCertificateSnapshot.reset();
            throw;
        }
        return *localCertificateSnapshot;
    }

    const cbf2026::EndpointCertificateSnapshot&
    transportedEndpointCertificateSnapshot(int otherId) {
        const auto snapshotIt =
            comm->_othersEndpointCertificateSnapshots.find(otherId);
        if (snapshotIt
            == comm->_othersEndpointCertificateSnapshots.end()) {
            certificateAvailable = false;
            throw std::invalid_argument(
                "required transported endpoint certificate is unavailable"
            );
        }
        try {
            cbf2026::validateEndpointCertificateSnapshot(
                otherId, snapshotIt->second
            );
        } catch (...) {
            certificateAvailable = false;
            throw;
        }
        if (snapshotIt->second.snapshotVersion
                != certificateSnapshotVersion
            || snapshotIt->second.allocationVersion
               != certificateAllocationVersion) {
            certificateAvailable = false;
            throw std::invalid_argument(
                "transported endpoint certificate version mismatch"
            );
        }
        return snapshotIt->second;
    }

    double allocatedAlphaValue(
        const ClassKParameters& parameters,
        double barrier
    ) const {
        if (!std::isfinite(parameters.coefficient)
            || parameters.coefficient < 0.0
            || parameters.power <= 0
            || parameters.power % 2 == 0
            || !std::isfinite(barrier)) {
            throw std::invalid_argument(
                "allocated row class-K data is invalid"
            );
        }
        const double value =
            parameters.coefficient * std::pow(barrier, parameters.power);
        if (!std::isfinite(value)) {
            throw std::invalid_argument(
                "allocated row class-K value is nonfinite"
            );
        }
        return value;
    }

    void installAllocatedEndpointCBF(
        const std::string& name,
        const cbf2026::EndpointRow& row,
        double barrier,
        double alphaValue,
        const ClassKParameters& parameters
    ) {
        cbf2026::endpointRowToModelControl(row, model->uSize());
        CBF endpointCBF;
        endpointCBF.name = name;
        endpointCBF.h = [barrier](VectorXd, double) {
            return barrier;
        };
        endpointCBF.dhdx_analytical = [row](
            VectorXd x,
            double
        ) {
            VectorXd gradient = VectorXd::Zero(x.size());
            if (x.size() < 2) {
                throw std::invalid_argument(
                    "allocated endpoint state has fewer than two axes"
                );
            }
            gradient.head<2>() = row.coefficient;
            return gradient;
        };
        endpointCBF.dhdt_analytical = [
            constant = row.constant,
            alphaValue
        ](VectorXd, double) {
            return constant - alphaValue;
        };
        endpointCBF.setAlphaClassK(
            parameters.coefficient, parameters.power
        );
        cbfNoSlack.cbfs[name] = endpointCBF;
    }

    void setAllocatedFixedCommCBF(const json& config) {
        if (!config.value("consider-uncertainty", true)) {
            throw std::invalid_argument(
                "theorem-aligned localization requires uncertainty tightening"
            );
        }
        const double maximumRange = config.at("max-range").get<double>();
        if (!std::isfinite(maximumRange) || maximumRange <= 0.0) {
            throw std::invalid_argument(
                "localization range must be finite and positive"
            );
        }
        const ClassKParameters parameters =
            readClassKParameters(config, 0.1, 1);
        const auto local = localEndpointCertificateSnapshot();
        const json references = incidentFixedLocalizationReferences();

        for (const json& baseIdJson : references.at("baseIds")) {
            const int baseId = baseIdJson.get<int>();
            if (baseId < 0 || baseId >= static_cast<int>(bases.size())) {
                throw std::invalid_argument(
                    "fixed base localization edge has an invalid base ID"
                );
            }
            const Point basePosition = bases.at(baseId);
            const auto edge =
                cbf2026::canonicalBaseLocalizationEdge(id, baseId);
            const auto geometry = cbf2026::makeEdgeSnapshot(
                edge,
                local.estimate,
                Eigen::Vector2d(basePosition.x, basePosition.y),
                local.barNu,
                0.0,
                0.0,
                certificateSnapshotVersion,
                certificateAllocationVersion
            );
            const double barrier =
                maximumRange - geometry.separation - local.epsilon;
            const double alphaValue =
                allocatedAlphaValue(parameters, barrier);
            auto snapshot = geometry;
            snapshot.alpha = alphaValue;
            const auto rows = cbf2026::allocatedRows(
                snapshot, 1.0, 0.0
            );
            installAllocatedEndpointCBF(
                "fixedCommCBF(base-" + std::to_string(baseId) + ")",
                rows.front(),
                barrier,
                alphaValue,
                parameters
            );
        }

        for (const json& otherIdJson : references.at("anchorIds")) {
            const int otherId = otherIdJson.get<int>();
            const auto& other =
                transportedEndpointCertificateSnapshot(otherId);
            const auto edge = cbf2026::canonicalUavEdge(
                cbf2026::EdgeKind::Localization, id, otherId
            );
            const auto& first = id == edge.low ? local : other;
            const auto& second = id == edge.low ? other : local;
            auto snapshot = cbf2026::makeEdgeSnapshot(
                edge,
                first.estimate,
                second.estimate,
                first.barNu,
                second.barNu,
                0.0,
                certificateSnapshotVersion,
                certificateAllocationVersion
            );
            const double barrier =
                maximumRange - snapshot.separation
                - first.epsilon - second.epsilon;
            const double alphaValue =
                allocatedAlphaValue(parameters, barrier);
            snapshot.alpha = alphaValue;
            const auto rows = cbf2026::allocatedRows(
                snapshot, 0.5, 0.5
            );
            const auto rowIt = std::find_if(
                rows.begin(), rows.end(),
                [this](const cbf2026::EndpointRow& row) {
                    return row.owner == id;
                }
            );
            if (rowIt == rows.end()) {
                throw std::invalid_argument(
                    "allocated localization row is missing its owner"
                );
            }
            installAllocatedEndpointCBF(
                "fixedCommCBF(#" + std::to_string(otherId) + ")",
                *rowIt,
                barrier,
                alphaValue,
                parameters
            );
        }
    }

    void setAllocatedSafetyCBF(const json& config) {
        if (!config.value("consider-uncertainty", true)) {
            throw std::invalid_argument(
                "theorem-aligned collision rows require uncertainty tightening"
            );
        }
        const double safeDistance = config.at("safe-distance").get<double>();
        if (!std::isfinite(safeDistance) || safeDistance < 0.0) {
            throw std::invalid_argument(
                "safe distance must be finite and nonnegative"
            );
        }
        const ClassKParameters parameters =
            readClassKParameters(config, 0.1, 1);
        const auto local = localEndpointCertificateSnapshot();
        for (const json& otherIdJson : settings.at("all")) {
            const int otherId = otherIdJson.get<int>();
            if (otherId == id) {
                continue;
            }
            const auto& other =
                transportedEndpointCertificateSnapshot(otherId);
            const auto edge = cbf2026::canonicalUavEdge(
                cbf2026::EdgeKind::Collision, id, otherId
            );
            const auto& first = id == edge.low ? local : other;
            const auto& second = id == edge.low ? other : local;
            auto snapshot = cbf2026::makeEdgeSnapshot(
                edge,
                first.estimate,
                second.estimate,
                first.barNu,
                second.barNu,
                0.0,
                certificateSnapshotVersion,
                certificateAllocationVersion
            );
            const double barrier =
                snapshot.separation - safeDistance
                - first.epsilon - second.epsilon;
            const double alphaValue =
                allocatedAlphaValue(parameters, barrier);
            snapshot.alpha = alphaValue;
            const auto rows = cbf2026::allocatedRows(
                snapshot, 0.5, 0.5
            );
            const auto rowIt = std::find_if(
                rows.begin(), rows.end(),
                [this](const cbf2026::EndpointRow& row) {
                    return row.owner == id;
                }
            );
            if (rowIt == rows.end()) {
                throw std::invalid_argument(
                    "allocated collision row is missing its owner"
                );
            }
            installAllocatedEndpointCBF(
                "safetyCBF(#" + std::to_string(otherId) + ")",
                *rowIt,
                barrier,
                alphaValue,
                parameters
            );
        }
    }

    void setFixedCommCBF(json& config) {
        // This function only creates the CBF constraints
        // Formation information is already set by setupFormation()
        double maxRange = config["max-range"];

        std::vector<Point> formationPoints;
        std::vector<Point> formationVels;
        std::vector<std::string> anchorCBFNames;
        std::vector<double> formationUncertainties;
        std::vector<double> formationUncertaintyRates;
        const bool useAnalyticRate =
            usesAnalyticTopologicalRateCertificate();
        if (useAnalyticRate && !certificateAvailable) {
            throw std::invalid_argument(
                "analytic topological rate certificate is unavailable"
            );
        }
        if (useAnalyticRate) {
            setAllocatedFixedCommCBF(config);
            return;
        }

        for (auto &baseId : myFormation["baseIds"]) {
            int id = baseId.get<int>();
            auto base = bases[id];
            formationPoints.push_back(base);
            formationVels.emplace_back(0, 0);
            anchorCBFNames.emplace_back("base-" + std::to_string(id));
            formationUncertainties.push_back(0);
            formationUncertaintyRates.push_back(0);
        }

        for (auto &anchorId : myFormation["anchorIds"]) {
            int otherId = anchorId.get<int>();
            formationPoints.push_back(comm->_othersPos[otherId]);
            formationVels.push_back(comm->_othersVel[otherId]);
            anchorCBFNames.push_back("#" + std::to_string(otherId));
            formationUncertainties.push_back(
                uncertaintyFromCovarianceFunction(
                    comm->_othersPositionCovariance[otherId]
                )
            );
            auto rateIt = comm->_othersUncertaintyRate.find(otherId);
            formationUncertaintyRates.push_back(
                rateIt == comm->_othersUncertaintyRate.end()
                ? 0.0
                : rateIt->second
            );
        }

        const double myUncertainty = estimatorInLoop
            ? estimatorEpsilon
            : (useAnalyticRate ? rateCertificate.epsilon : currentUncertainty);
        std::string uncertaintyRateMode = "off";
        if (settings["cbfs"].contains("uncertainty-rate")) {
            uncertaintyRateMode =
                settings["cbfs"]["uncertainty-rate"].value("mode", "off");
        }
        const bool useBackwardRate =
            uncertaintyRateMode == "backward-difference-positive";
        const bool useUncertaintyRate =
            useBackwardRate || useAnalyticRate;
        const double myUncertaintyRate = useAnalyticRate
            ? rateCertificate.epsilonRateBound
            : uncertaintyRate;

        for (int i = 0; i < formationPoints.size(); i++) {
            auto otherPoint = formationPoints[i];
            auto otherVel = config["compensate-velocity"] ? formationVels[i] : Point(0, 0);
            double k = config["k"];
            bool isAnchor = anchorCBFNames[i].find("base-") != std::string::npos;
            auto otherUncertainty = formationUncertainties[i];
            auto otherUncertaintyRate = formationUncertaintyRates[i];
            bool considerUncertainty = config.value("consider-uncertainty", true);
            double uncertaintyRateSum =
                considerUncertainty && useUncertaintyRate
                ? myUncertaintyRate + otherUncertaintyRate
                : 0.0;

            auto fixedFormationCommH = [this, otherPoint, otherVel, maxRange, k, isAnchor, myUncertainty, otherUncertainty, considerUncertainty](VectorXd x, double t) {
                Point myPosition = estimatorInLoop
                    ? estimatorPosition
                    : model->extractXYFromVector(x);
                double distance = myPosition.distance_to(otherPoint);
                double h = k * (maxRange - distance);

                double robust_h;
                if (isAnchor) {
                    robust_h = h - (considerUncertainty ? k * myUncertainty : 0);
                } else {
                    robust_h = h - (considerUncertainty ? k * (myUncertainty + otherUncertainty) : 0);
                }

                return robust_h;
            };

            // Analytical spatial gradient: dhdx = -k * (p - p_anchor_rel) / ||p - p_anchor_rel||
            auto dhdx = [this, otherPoint, otherVel, k](VectorXd x, double t) -> VectorXd {
                Point myPosition = estimatorInLoop
                    ? estimatorPosition
                    : model->extractXYFromVector(x);
                Point diff = myPosition - otherPoint;
                double distance = diff.len();
                if (distance < 1e-8) {
                    return VectorXd::Zero(x.size());
                }
                VectorXd dhdx = VectorXd::Zero(x.size());
                dhdx(0) = -k * diff.x / distance;
                dhdx(1) = -k * diff.y / distance;
                return dhdx;
            };

            // Analytical temporal derivative: dhdt = k * (p - p_anchor_rel) · v_anchor / ||p - p_anchor_rel||
            auto dhdt = [this, otherPoint, otherVel, k, uncertaintyRateSum](VectorXd x, double t) -> double {
                Point myPosition = estimatorInLoop
                    ? estimatorPosition
                    : model->extractXYFromVector(x);
                Point diff = myPosition - otherPoint;
                double distance = diff.len();
                if (distance < 1e-8) {
                    return -k * uncertaintyRateSum;
                }
                double dotProduct = diff * otherVel;
                return k * dotProduct / distance - k * uncertaintyRateSum;
            };

            CBF commCBF;
            commCBF.name = "fixedCommCBF(" + anchorCBFNames[i] + ")";
            commCBF.h = fixedFormationCommH;
            commCBF.dhdx_analytical = dhdx;
            commCBF.dhdt_analytical = dhdt;
            ClassKParameters alphaParameters = readClassKParameters(config, 0.1, 1);
            commCBF.setAlphaClassK(alphaParameters.coefficient, alphaParameters.power);
            cbfNoSlack.cbfs[commCBF.name] = commCBF;
        }
    }

    void setSafetyCBF(const json& config) {
        if (settings["num"] == 1) return;
        const bool route1Mode =
            settings["cbfs"].value("route1", json::object()).value("on", false);

        for (auto it = cbfNoSlack.cbfs.begin(); it != cbfNoSlack.cbfs.end();) {
            if (it->first == "safetyCBF" ||
                it->first.rfind("safetyCBF(#", 0) == 0) {
                it = cbfNoSlack.cbfs.erase(it);
            } else {
                ++it;
            }
        }

        bool considerUncertainty = config.value("consider-uncertainty", true);
        double safeDistance = config["safe-distance"];
        double k = config.value("k", 1.0);
        std::string safetyMode = config.value("mode", "minimum");
        std::string uncertaintyRateMode = "off";
        if (settings["cbfs"].contains("uncertainty-rate")) {
            uncertaintyRateMode =
                settings["cbfs"]["uncertainty-rate"].value("mode", "off");
        }
        const bool useAnalyticRate =
            uncertaintyRateMode == "analytic-topological";
        if (useAnalyticRate && !certificateAvailable) {
            throw std::invalid_argument(
                "analytic topological rate certificate is unavailable"
            );
        }
        if (useAnalyticRate) {
            setAllocatedSafetyCBF(config);
            return;
        }
        const double myUncertainty = estimatorInLoop
            ? estimatorEpsilon
            : (useAnalyticRate ? rateCertificate.epsilon : currentUncertainty);
        const double myUncertaintyRate = useAnalyticRate
            ? rateCertificate.epsilonRateBound
            : uncertaintyRate;
        const bool useUncertaintyRate =
            considerUncertainty &&
            (uncertaintyRateMode == "backward-difference-positive"
             || useAnalyticRate);

        struct SafetyPair {
            int id;
            Point position;
            Point velocity;
            double uncertainty;
            double uncertaintyRate;
            double currentRobustH;
        };
        std::vector<SafetyPair> safetyPairs;
        Point myPosition = estimatorInLoop
            ? estimatorPosition
            : model->xy();
        for (auto &[otherId, otherPosition]: comm->_othersPos) {
            if (id == otherId) continue;
            if (route1Mode && otherId > id) continue;

            double otherUncertainty = 0.0;
            double otherUncertaintyRate = 0.0;
            auto covarianceIt =
                comm->_othersPositionCovariance.find(otherId);
            if (enablePositionCovariance &&
                covarianceIt
                != comm->_othersPositionCovariance.end()) {
                otherUncertainty =
                    uncertaintyFromCovarianceFunction(
                        covarianceIt->second
                    );
            }
            auto rateIt = comm->_othersUncertaintyRate.find(otherId);
            if (rateIt != comm->_othersUncertaintyRate.end()) {
                otherUncertaintyRate = rateIt->second;
            }

            Point otherVelocity(0.0, 0.0);
            auto velocityIt = comm->_othersVel.find(otherId);
            if (velocityIt != comm->_othersVel.end()) {
                otherVelocity = Point(velocityIt->second);
            }

            double uncertaintySum =
                considerUncertainty ? myUncertainty + otherUncertainty : 0.0;
            safetyPairs.push_back({
                otherId,
                otherPosition,
                otherVelocity,
                otherUncertainty,
                otherUncertaintyRate,
                k * (
                    myPosition.distance_to(otherPosition) -
                    safeDistance -
                    uncertaintySum
                )
            });
        }

        if (safetyMode == "minimum" && !safetyPairs.empty()) {
            auto minimumIt = std::min_element(
                safetyPairs.begin(),
                safetyPairs.end(),
                [](const SafetyPair& lhs, const SafetyPair& rhs) {
                    if (lhs.currentRobustH == rhs.currentRobustH) {
                        return lhs.id < rhs.id;
                    }
                    return lhs.currentRobustH < rhs.currentRobustH;
                }
            );
            SafetyPair minimumPair = *minimumIt;
            safetyPairs.assign(1, minimumPair);
        }

        for (const SafetyPair& pair : safetyPairs) {
            double uncertaintySum =
                considerUncertainty ? myUncertainty + pair.uncertainty : 0.0;
            double uncertaintyRateSum =
                useUncertaintyRate
                ? myUncertaintyRate + pair.uncertaintyRate
                : 0.0;

            CBF safetyCBF;
            safetyCBF.name =
                "safetyCBF(#" + std::to_string(pair.id) + ")";
            safetyCBF.h = [
                this,
                otherPosition = pair.position,
                safeDistance,
                uncertaintySum,
                k
            ](VectorXd x, double t) {
                Point currentPosition = estimatorInLoop
                    ? estimatorPosition
                    : model->extractXYFromVector(x);
                return k * (
                    currentPosition.distance_to(otherPosition) -
                    safeDistance -
                    uncertaintySum
                );
            };
            safetyCBF.dhdx_analytical = [
                this,
                otherPosition = pair.position,
                k
            ](VectorXd x, double t) -> VectorXd {
                Point currentPosition = estimatorInLoop
                    ? estimatorPosition
                    : model->extractXYFromVector(x);
                Point diff = currentPosition - otherPosition;
                double distance = diff.len();
                if (distance < 1e-8) {
                    return VectorXd::Zero(x.size());
                }
                VectorXd dhdx = VectorXd::Zero(x.size());
                dhdx(0) = k * diff.x / distance;
                dhdx(1) = k * diff.y / distance;
                return dhdx;
            };
            safetyCBF.dhdt_analytical = [
                this,
                otherPosition = pair.position,
                otherVelocity = pair.velocity,
                uncertaintyRateSum,
                k
            ](VectorXd x, double t) -> double {
                Point currentPosition = estimatorInLoop
                    ? estimatorPosition
                    : model->extractXYFromVector(x);
                Point diff = currentPosition - otherPosition;
                double distance = diff.len();
                if (distance < 1e-8) {
                    return -k * uncertaintyRateSum;
                }
                return -k * (diff * otherVelocity) / distance
                       - k * uncertaintyRateSum;
            };
            ClassKParameters alphaParameters =
                readClassKParameters(config, 0.1, 1);
            safetyCBF.setAlphaClassK(
                alphaParameters.coefficient,
                alphaParameters.power
            );
            cbfNoSlack.cbfs[safetyCBF.name] = safetyCBF;
        }
    }

    // Periodic target interface PROPOSAL: installed only by new bound consumer.
    std::optional<std::uint64_t> gdmsCentroidFrame;
    Point gdmsCentroid;
    double gdmsCentroidMass = 0.0;
    int gdmsOwnedCells = 0;
    json periodicFollowerTarget = json::object();
    json periodicTaskTargetRecord = json::object();
    json tvtTargetSelection = json::object();

    Point getEndTargetPoint() {
        cvt = CVT(endIds.size(), world.boundary);
        for (auto &[endId, cvtId]: endId2CvtId) {
            cvt.pt[cvtId] = comm->_othersPos[endId];
        }
        const std::string targetProtocol=settings.value("tvt-experiment", json::object()).value("target-protocol", "legacy-clipped-raster-v1");
        if (targetProtocol=="continuous-halfplane-grid-v2" || targetProtocol=="continuous-halfplane-grid-v3-exact-dyadic") {
            std::vector<std::pair<int,Point>> sites;
            for (const auto& item:endId2CvtId) sites.push_back({item.first,comm->_othersPos.at(item.first)});
            const bool reuse=settings.value("tvt-experiment",json::object()).value("same-frame-leader-reuse",false);
            if(reuse && !estimatorSnapshotFrame) throw std::runtime_error("leader reuse requires current estimator frame");
            const auto selected=reuse ? leader_reuse::select(*estimatorSnapshotFrame,gridWorld,world.boundary,endInMyPart,sites,comm->_othersYawRad.at(endInMyPart),cvtFrontFocusDistance,targetProtocol=="continuous-halfplane-grid-v3-exact-dyadic") : cbf2026::selectLeaderTarget(gridWorld,world.boundary,endInMyPart,sites,comm->_othersYawRad.at(endInMyPart),cvtFrontFocusDistance,targetProtocol=="continuous-halfplane-grid-v3-exact-dyadic");
            tvtTargetSelection=selected.record;
            tvtTargetSelection["snapshot_frame_index"]=estimatorSnapshotFrame ? json(*estimatorSnapshotFrame) : json(nullptr);
            return selected.point;
        }
        cvt.cal_poly();
        Point densityCentroid = gridWorld.getCentroidInPolygon(cvt.pl[partId]);

        double heading = comm->_othersYawRad[endInMyPart];
        Point focus = comm->_othersPos[endInMyPart] + cvtFrontFocusDistance * Point(std::cos(heading), std::sin(heading));

        return getExplorationPoint(densityCentroid, focus, cvt.pl[partId]);
    }

    Point getExplorationPoint(const Point& densityCentroid, const Point& focus, const Polygon& poly) {
        if (settings.value("tvt-experiment", json::object()).value("gdms-centroid-target", false)) {
            return densityCentroid;
        }
        if (cvtExplorationMode == "centroid") {
            return densityCentroid;
        }
        if (cvtExplorationMode == "intelligent") {
            if (gridWorld.isExplored(densityCentroid)) {
                return gridWorld.getNearestUnexploredPointInPolygon(
                    poly, focus, densityCentroid
                );
            }
            return densityCentroid;
        }
        if (cvtExplorationMode == "nearest-unexplored") {
            return gridWorld.getNearestUnexploredPointInPolygon(
                poly, focus, densityCentroid
            );
        }
        if (cvtExplorationMode == "cvt") {
            return gridWorld.getNearestUnexploredPointInPolygon(
                poly, focus, densityCentroid
            );
        }
        throw std::invalid_argument("Invalid cvtExplorationMode");
    }

    Point getMyExplorationPoint(const Point& endExplorationPoint) {
        int numTotalSection = (idsInMyPart.size() + 1) / 2;
        const auto experiment = settings.value("tvt-experiment", json::object());
        const std::string originRule = experiment.value("target-origin", "first-base");
        Point origin = bases[myBasesId[0]];
        if (originRule == "base-midpoint") {
            origin = (bases[myBasesId[0]] + bases[myBasesId[1]]) / 2.0;
        } else if (originRule != "first-base") {
            throw std::invalid_argument("unknown TVT target origin");
        }
        Point sectionVector = (endExplorationPoint - origin) / numTotalSection;
        int numSection = (idsInMyPart.size() - idInMyPart + 1) / 2;
        Point explorationPoint = endExplorationPoint - sectionVector * numSection;
        double rotateAngleRad = experiment.value("ladder-angle-deg", 60.0)
            * pi / 180.0 * ((partId % 2 == 1)? -1: 1);
        if ((idsInMyPart.size() - idInMyPart) % 2 == 1) {
            explorationPoint = explorationPoint + sectionVector.rotate(rotateAngleRad);
        }
        return explorationPoint;
    }

    void setCVTCBF(const json& config) {
        const auto targetStarted=std::chrono::steady_clock::now();
        const auto periodic=settings.value("periodic-target-interface",json::object());
        const bool instrumentPeriodic=!periodic.empty();
        const bool externalFollower=instrumentPeriodic
            && periodic.at("arm")=="CANO-FOLLOWER-REPLAN30/shared-QP"
            && std::find(endIds.begin(),endIds.end(),id)==endIds.end();
        if(instrumentPeriodic && (!settings.at("tvt-experiment").at("hierarchical-target-direct").get<bool>()
           || settings.contains("cano-pcg") || !usesEstimatedTaskContract()))
            throw std::runtime_error("periodic target proposal requires original direct M1/zero nominal estimated contract");
        const bool directHierarchicalTarget = settings.value("tvt-experiment", json::object())
            .value("hierarchical-target-direct", false) && cvtExplorationMode != "cvt";
        const bool gdmsCore=settings.value("tvt-experiment",json::object()).value("gdms-planar-core",false);
        const bool m2Exact=settings.value("tvt-experiment",json::object()).value("m2-exact-finite-centroid",false);
        if(externalFollower) {
            if(!estimatorSnapshotFrame || periodicFollowerTarget.empty()
               || periodicFollowerTarget.at("frame")!=*estimatorSnapshotFrame)
                throw std::runtime_error("current follower terminal target unavailable; no fallback");
            const auto& raw=periodicFollowerTarget.at("raw_m");
            const Point projected=world.boundary.clip(Point(raw.at(0).get<double>(),raw.at(1).get<double>()));
            const auto& expected=periodicFollowerTarget.at("projected_m");
            const double ex=expected.at(0).get<double>(),ey=expected.at(1).get<double>();
            if(!std::isfinite(projected.x)||!std::isfinite(projected.y)
               || std::memcmp(&projected.x,&ex,sizeof(double))!=0 || std::memcmp(&projected.y,&ey,sizeof(double))!=0)
                throw std::runtime_error("current native projection mismatch");
            cvt.ct[id]=projected;
            periodicTaskTargetRecord=periodicFollowerTarget;
            periodicTaskTargetRecord["rule"]="CANO terminal follower waypoint/native boundary.clip";
            tvtTargetSelection={{"continuous_cell_vertices",json::array()},
                {"kind","external-follower-terminal-target"},{"snapshot_frame_index",*estimatorSnapshotFrame}};
        } else if(m2Exact) {
            const auto experiment=settings.at("tvt-experiment");
            if(n!=14 || !usesEstimatedTaskContract() || !estimatorSnapshotFrame
               || settings.at("execute").at("time-step").get<double>()!=.5
               || *estimatorSnapshotFrame!=static_cast<std::uint64_t>(std::llround(runtime/.5))
               || experiment.at("row-mode")!="symmetric-l1" || gdmsCore || directHierarchicalTarget
               || settings.contains("cano-pcg"))
                throw std::invalid_argument("M2 prototype requires the unchanged common estimated local-QP interface");
            std::vector<std::pair<int,Point>> sites;
            for(int peer=1;peer<=14;++peer)sites.emplace_back(peer,comm->_othersPos.at(peer));
            const auto selected=cbf2026::selectM2ExactFiniteTarget(gridWorld,id,sites);
            cvt.ct[id]=selected.point;tvtTargetSelection=selected.record;
            tvtTargetSelection["snapshot_frame_index"]=*estimatorSnapshotFrame;
            tvtTargetSelection["map_information_contract"]="same-frame common estimated sensor map; truth offline only";
            tvtBaselineTask={{"kind","M2-exact-finite-own-unvisited-centroid-prototype"},
                {"nominal_policy","unchanged main zero objective reference"},
                {"source_qualification","pending; compile is not mathematical/control qualification"}};
        } else if (gdmsCore) {
            if (!estimatorSnapshotFrame || !gdmsCentroidFrame || *gdmsCentroidFrame != *estimatorSnapshotFrame)
                throw std::runtime_error("same-frame common GdMS centroid unavailable");
            cvt=CVT(n,world.boundary);
            for(int peer=1;peer<=n;++peer) cvt.pt[peer]=comm->_othersPos.at(peer);
            cvt.cal_poly();
            cvt.ct[id]=gdmsCentroid;
            tvtBaselineTask={{"kind","gdms-own-weighted-voronoi"},{"mass",gdmsCentroidMass},{"owned_grid_cells",gdmsOwnedCells},
                {"zero_mass_rule","hold estimated self; finite zero task"},{"zero_mass_applied",gdmsCentroidMass<=0},
                {"tie_rule","smallest node ID"}};
        } else if (directHierarchicalTarget && settings.at("tvt-experiment").value("interior-triangle-750",false)) {
            if(!usesEstimatedTaskContract() || !estimatorSnapshotFrame || n!=14 || myBasesId.size()!=2 || idsInMyPart.size()!=7)
                throw std::invalid_argument("triangle750 requires two 7-member squads and current common estimate");
            const Point goal=getEndTargetPoint();
            const Point outer=bases.at(myBasesId.at(1)),shared=bases.at(myBasesId.at(0));
            const auto triangle=triangle750(outer,shared,goal);
            cvt.ct[id]=triangle.points.at(idInMyPart-1);
            json points=json::array();for(const auto& p:triangle.points)points.push_back({p.x,p.y});
            tvtTargetSelection["triangle750"]={{"group_goal",{goal.x,goal.y}},{"outer_base",{outer.x,outer.y}},
              {"shared_base",{shared.x,shared.y}},{"step_m",triangle.step},{"first_distance_m",triangle.first_distance},
              {"side",triangle.side},{"angle_degrees",triangle.angle_degrees},{"offset_m",triangle.offset},{"geometric_interval_feasible",triangle.feasible},{"member_targets",points},{"member_index",idInMyPart},
              {"target_rule","fixed-beacon-narrow-retract-v1; infeasible interval uses page saturation; no clipping or smoothing"}};
        } else if (directHierarchicalTarget && settings.at("tvt-experiment").value("near-observation-local",false)) {
            const auto marginMode=settings.at("tvt-experiment").value("error-margin-mode","");
            if(!usesEstimatedTaskContract() || !estimatorSnapshotFrame || n!=14 || myBasesId.size()!=2 ||
               (marginMode!="ekf-covariance-heuristic" && marginMode!="fixed"))
                throw std::invalid_argument("local viewpoint requires common 14-node estimated contract and known margin");
            auto toLocal=[](const Point& p){return local_viewpoint::P{p.x,p.y};};
            auto fromLocal=[](local_viewpoint::P p){return Point(p.x,p.y);};
            auto boundary=settings.at("world").at("boundary");
            if(boundary.size()!=4)throw std::invalid_argument("local viewpoint requires the original rectangular domain");
            local_viewpoint::Box box{1e300,-1e300,1e300,-1e300};
            for(const auto& a:boundary){double x=a.at(0),y=a.at(1);box.xmin=std::min(box.xmin,x);box.xmax=std::max(box.xmax,x);box.ymin=std::min(box.ymin,y);box.ymax=std::max(box.ymax,y);}
            for(const auto& a:boundary){double x=a.at(0),y=a.at(1);if((x!=box.xmin&&x!=box.xmax)||(y!=box.ymin&&y!=box.ymax))throw std::invalid_argument("local viewpoint unsupported nonrectangle");}
            Point q=getEndTargetPoint();const auto v=local_viewpoint::view(toLocal(q),toLocal(bases.at(myBasesId[0])),toLocal(bases.at(myBasesId[1])),box);
            const bool viewValid=v.in_domain&&v.in_sensor_range;const Point S=fromLocal(v.location);
            const double range=settings.at("cbfs").at("without-slack").at("comm-fixed").at("max-range");
            const double safe=settings.at("cbfs").at("without-slack").at("safety").at("safe-distance");
            const double es=settings.at("tvt-experiment").at("ekf-epsilon-scale");
            const double fixedError=settings.at("tvt-experiment").at("node-error-bound-m");
            const double ownMargin=marginMode=="fixed" ? fixedError : es*estimatorEpsilon;
            auto peerMargin=[&](int a){return marginMode=="fixed" ? fixedError : es*tvtEstimatorEpsilons.at(a);};
            std::vector<local_viewpoint::Ref> refs,others;
            for(const auto& j:myFormation.at("baseIds")){int b=j.get<int>();refs.push_back({toLocal(bases.at(b)),ownMargin});}
            for(const auto& j:myFormation.at("anchorIds")){int a=j.get<int>();refs.push_back({toLocal(comm->_othersPos.at(a)),ownMargin+peerMargin(a)});}
            for(const auto& item:comm->_othersPos){int a=item.first;if(a!=id)others.push_back({toLocal(item.second),ownMargin+peerMargin(a)});}
            const bool leader=(id==endInMyPart);const bool unexplored=!gridWorld.isExplored(q);
            const bool inViewRadius=taskPosition().distance_to(q)<=400.;
            Point guide=leader && inViewRadius && unexplored ? taskPosition() : world.boundary.clip(getMyExplorationPoint(S));
            local_viewpoint::Pick pick={toLocal(taskPosition()),0,0,0.,"INVALID_VIEWPOINT_HOLD"};
            if(viewValid)pick=local_viewpoint::choose(toLocal(taskPosition()),toLocal(guide),refs,others,box,range,safe,configuredPlanarComponentMax()*settings.at("execute").at("time-step").get<double>());
            cvt.ct[id]=fromLocal(pick.point);
            const double bearing=std::atan2(q.y-taskPosition().y,q.x-taskPosition().x),yaw=model->getStateVariable("yawRad");
            const double yawGap=std::abs(std::atan2(std::sin(bearing-yaw),std::cos(bearing-yaw)));
            tvtTargetSelection["near_observation"]={{"grid_target",{q.x,q.y}},{"grid_target_unexplored_on_estimated_map",unexplored},
                {"viewpoint",{S.x,S.y}},{"viewpoint_shifted",v.shifted},{"viewpoint_in_domain",v.in_domain},
                {"viewpoint_in_sensor_range",v.in_sensor_range},{"position_guide",{guide.x,guide.y}},
                {"local_waypoint",{pick.point.x,pick.point.y}},{"candidate_status",pick.status},{"feasible_count",pick.feasible},
                {"angle_preferred_count",pick.preferred},{"assigned_ref_abs_sin",pick.angle_sin},
                {"is_leader",leader},{"estimated_current_in_sensor_radius",inViewRadius},
                {"estimated_current_in_fov",inViewRadius&&yawGap<=std::acos(-1.)/3.},
                {"minimum_yaw_turn_s",yawGap/settings.at("cbfs").at("input-limits").at("yaw-rate-max").get<double>()}};
        } else if (directHierarchicalTarget) {
            // The hierarchical target uses the leaders' Voronoi cells. Avoid an
            // unused follower-cell centroid, which can be empty outside the map.
            const Point endTarget = getEndTargetPoint();
            const Point rawHierarchical=getMyExplorationPoint(endTarget);
            cvt.ct[this->id] = world.boundary.clip(rawHierarchical);
            if(instrumentPeriodic) periodicTaskTargetRecord={{"frame",*estimatorSnapshotFrame},
                {"raw_m",{rawHierarchical.x,rawHierarchical.y}},
                {"projected_m",{cvt.ct[id].x,cvt.ct[id].y}},
                {"rule","original M1 getEndTargetPoint/getMyExplorationPoint/native boundary.clip"}};
        } else {
            cvt = CVT(n, world.boundary);
            for (auto &[id, pos2d]: comm->_othersPos) {
                if (id == this->id) continue;
                cvt.pt[id] = pos2d;
            }
            cvt.pt[this->id] = taskPosition();
            cvt.cal_poly();
            Point densityCentroid = gridWorld.getCentroidInPolygon(cvt.pl[this->id]);

            double heading = this->model->getStateVariable("yawRad");
            Point focus = taskPosition() + cvtFrontFocusDistance * Point(std::cos(heading), std::sin(heading));

            cvt.ct[this->id] = getExplorationPoint(densityCentroid, focus, cvt.pl[this->id]);

            if (cvtExplorationMode == "cvt") {
                cvt.ct[this->id] = world.boundary.clip(cvt.ct[this->id]);
            } else {
                cvt.ct[this->id] = world.boundary.clip(getMyExplorationPoint(getEndTargetPoint()));
            }
        }

        if(instrumentPeriodic){
            if(periodicTaskTargetRecord.empty())throw std::runtime_error("missing current native target record");
            periodicTaskTargetRecord["selection_and_projection_wall_s"]=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-targetStarted).count();
            if(periodicTaskTargetRecord.dump().size()>2048)
                throw std::runtime_error("whole native target record cap exceeded; no truncation");
        }
        Point cvtCenter = cvt.ct[this->id];
        Point cvtYawTarget=cvtCenter;
        if(settings.at("tvt-experiment").value("near-observation-local",false) && id==endInMyPart && tvtTargetSelection.contains("near_observation")) {
            const auto& q=tvtTargetSelection.at("near_observation").at("grid_target");cvtYawTarget=Point(q.at(0).get<double>(),q.at(1).get<double>());
        }

        if (config.contains("cvt") && config["cvt"]["on"]) {
            CBF cvtDistanceCBF;
            cvtDistanceCBF.name = config["cvt"]["name"];
            cvtDistanceCBF.h = [cvtCenter, config, this](VectorXd x, double t) {
                Point myPosition = this->model->extractXYFromVector(x);
                double kp = config["cvt"]["kp"];
                double distance = cvtCenter.distance_to(myPosition);
                return -kp * distance;
            };
            ClassKParameters alphaParameters = readClassKParameters(config["cvt"], 1.0, 1);
            cvtDistanceCBF.setAlphaClassK(alphaParameters.coefficient, alphaParameters.power);
            if (usesEstimatedTaskContract()) {
                const double kp = config["cvt"]["kp"];
                cvtDistanceCBF.dhdx_analytical = [cvtCenter, kp, this](VectorXd x, double) {
                    const Point d = model->extractXYFromVector(x) - cvtCenter;
                    VectorXd gradient = VectorXd::Zero(x.size());
                    if (d.len() > 1e-12) {
                        gradient[0] = -kp * d.x / d.len();
                        gradient[1] = -kp * d.y / d.len();
                    } // At the norm cusp the declared soft task subgradient is zero.
                    return gradient;
                };
            }
            cbfSlack[cvtDistanceCBF.name] = cvtDistanceCBF;
        }

        if (config.contains("cvt-yaw") && config["cvt-yaw"]["on"]) {
            CBF cvtYawCBF;
            cvtYawCBF.name = config["cvt-yaw"]["name"];
            cvtYawCBF.h = [cvtYawTarget, config, this](VectorXd x, double t) {
                Point myPosition = this->model->extractXYFromVector(x);
                double k_yaw = config["cvt-yaw"]["k_yaw"];
                double desired_yaw = atan2(cvtYawTarget.y - myPosition.y, cvtYawTarget.x - myPosition.x);
                double current_yaw = this->model->extractFromVector(x, "yawRad");
                if (usesEstimatedTaskContract() && cvtYawTarget.distance_to(myPosition) <= 1e-12)
                    desired_yaw = current_yaw;
                double yaw_error = desired_yaw - current_yaw;
                yaw_error = atan2(sin(yaw_error), cos(yaw_error));
                return k_yaw * (1.0 - cos(yaw_error)) / (-2.0);
            };
            ClassKParameters alphaParameters = readClassKParameters(config["cvt-yaw"], 1.0, 1);
            cvtYawCBF.setAlphaClassK(alphaParameters.coefficient, alphaParameters.power);
            cbfSlack[cvtYawCBF.name] = cvtYawCBF;
        }
    }

    void postsetCBF() {
        canonical::forbid("void postsetCBF");
        auto cbfConfig = settings["cbfs"];

        setupFormation();

        if (usesAnalyticTopologicalRateCertificate()) {
            if (cbfConfig["with-slack"]["cvt"]["on"]
                || cbfConfig["with-slack"]["cvt-yaw"]["on"]) {
                setCVTCBF(cbfConfig["with-slack"]);
            }
            return;
        }

        if (cbfConfig["without-slack"]["comm-fixed"]["on"]) setFixedCommCBF(cbfConfig["without-slack"]["comm-fixed"]);
        if (cbfConfig["without-slack"]["comm-auto"]["on"]) setCommunicationAutoCBF(cbfConfig["without-slack"]["comm-auto"]);
        if (cbfConfig["with-slack"]["cvt"]["on"] || cbfConfig["with-slack"]["cvt-yaw"]["on"]) setCVTCBF(cbfConfig["with-slack"]);
        if (cbfConfig["without-slack"]["safety"]["on"]) setSafetyCBF(cbfConfig["without-slack"]["safety"]);
        const auto experiment = settings.value("tvt-experiment", json::object());
        const std::string rowMode = experiment.value("row-mode", "legacy");
        if (rowMode == "symmetric-l1") setTvtSymmetricRows();
        else if (usesLiteratureRows()) setTvtLiteratureRows(rowMode);
        else if (rowMode != "legacy") setTvtFixedErrorRows(rowMode);
        if (experiment.value("record-protocol", false)) {
            json positions = json::object();
            for (const auto& [peer, point] : comm->_othersPos) positions[std::to_string(peer)] = {point.x, point.y};
            json peers = json::object();
            for (const auto& [peer, velocity] : comm->_othersVel) {
                peers[std::to_string(peer)] = {velocity[0], velocity[1]};
            }
            const Point used = estimatorInLoop ? estimatorPosition : model->xy();
            tvtRowsSnapshot = {{"constructed_at_s", runtime},
                {"position_used", {used.x, used.y}}, {"positions_used", positions}, {"peer_commands_used", peers},
                {"information_contract", experiment.value("information-contract", "legacy-task-truth")},
                {"task_position_used", {taskPosition().x, taskPosition().y}},
                {"task_map_coverage", gridWorld.getPercentage()},
                {"row_mode", rowMode},
                {"error_margin_mode", experiment.value("error-margin-mode", "fixed")},
                {"estimator_epsilons", tvtEstimatorEpsilons},
                {"controller_identity", experiment.value("controller-identity", "historical-development")},
                {"snapshot_frame_index", estimatorSnapshotFrame ? json(*estimatorSnapshotFrame) : json(nullptr)}};
        }
    }


    // Experimental first-order ZOH rows. These encode sufficient one-interval
    // inequalities, not a recursive local-feasibility theorem or sensor bound.
    void setTvtFixedErrorRows(const std::string& mode) {
        if (mode != "squared-margin" && mode != "squared-zoh") {
            throw std::invalid_argument("unknown TVT fixed-error row mode");
        }
        const auto experiment = settings.at("tvt-experiment");
        const double error = experiment.at("node-error-bound-m").get<double>();
        const double T = settings.at("execute").at("time-step").get<double>();
        const double component = settings.at("cbfs").at("input-limits")
            .at("planar-component-max").get<double>();
        const double alpha = experiment.value("squared-alpha", 1.0 / T);
        if (!settings.at("cbfs").at("input-limits").at("on").get<bool>()
            || settings.at("model") != "SingleIntegrate2D" || error < 0.0
            || T <= 0.0 || component <= 0.0 || alpha <= 0.0 || alpha * T > 1.0) {
            throw std::invalid_argument("invalid TVT first-order row conditions");
        }
        const double D = settings.at("cbfs").at("without-slack")
            .at("safety").at("safe-distance").get<double>();
        const double R = settings.at("cbfs").at("without-slack")
            .at("comm-fixed").at("max-range").get<double>();
        for (auto it = cbfNoSlack.cbfs.begin(); it != cbfNoSlack.cbfs.end();) {
            if (it->first.rfind("safetyCBF", 0) == 0
                || it->first.rfind("fixedCommCBF", 0) == 0) {
                it = cbfNoSlack.cbfs.erase(it);
            } else ++it;
        }
        const Point position = estimatorInLoop ? estimatorPosition : model->xy();
        const VectorXd xSnapshot = taskEvaluationState();
        auto install = [&](const std::string& name, const Point& other,
                           const Point& otherVelocity, bool isBase, bool collision) {
            const Point relative = position - other;
            const double distance = relative.len();
            const double E = isBase ? error : 2.0 * error;
            const double V = std::sqrt(2.0) * component * (isBase ? 1.0 : 2.0);
            // Lower and upper bounds on the TRUE squared-distance barrier.
            const double h0 = collision
                ? std::pow(std::max(0.0, distance - E), 2) - D * D
                : R * R - std::pow(distance + E, 2);
            const double sign = collision ? 1.0 : -1.0;
            const double reserve = mode == "squared-zoh"
                ? 2.0 * E * V + (collision ? 0.0 : T * V * V) : 0.0;
            VectorXd gradient = VectorXd::Zero(xSnapshot.size());
            gradient[0] = sign * 2.0 * relative.x;
            gradient[1] = sign * 2.0 * relative.y;
            CBF row;
            row.name = name;
            row.h = [h0, gradient, xSnapshot](VectorXd x, double) {
                return h0 + gradient.dot(x - xSnapshot);
            };
            row.dhdx_analytical = [gradient](VectorXd, double) { return gradient; };
            const double offset = -sign * 2.0 * (relative * otherVelocity) - reserve;
            row.dhdt_analytical = [offset](VectorXd, double) { return offset; };
            row.setAlphaClassK(alpha, 1);
            cbfNoSlack.cbfs[name] = row;
        };
        for (const auto& [peer, other] : comm->_othersPos) {
            if (peer >= id) continue; // Higher ID owns each collision pair.
            install("safetyCBF(#" + std::to_string(peer) + ")", other,
                    Point(comm->_othersVel.at(peer)), false, true);
        }
        for (const auto& entry : myFormation.at("baseIds")) {
            const int base = entry.get<int>();
            install("fixedCommCBF(base-" + std::to_string(base) + ")",
                    bases.at(base), Point(0, 0), true, false);
        }
        for (const auto& entry : myFormation.at("anchorIds")) {
            const int peer = entry.get<int>();
            if (peer >= id) throw std::invalid_argument("TVT protocol needs a lower-ID reference DAG");
            install("fixedCommCBF(#" + std::to_string(peer) + ")",
                    comm->_othersPos.at(peer), Point(comm->_othersVel.at(peer)), false, false);
        }
    }

    void setTvtSymmetricRows() {
        const auto experiment = settings.at("tvt-experiment");
        const double error = experiment.at("node-error-bound-m");
        const std::string marginMode = experiment.value("error-margin-mode", "fixed");
        const double epsilonScale = experiment.value("ekf-epsilon-scale", 1.0);
        if ((marginMode != "fixed" && marginMode != "ekf-covariance-heuristic") ||
            !std::isfinite(epsilonScale) || epsilonScale <= 0.0)
            throw std::invalid_argument("invalid symmetric L1 margin mode or scale");
        if (marginMode == "ekf-covariance-heuristic" &&
            (tvtEstimatorEpsilons.size() != static_cast<std::size_t>(n) ||
             tvtEstimatorEpsilons.at(id) != estimatorEpsilon))
            throw std::invalid_argument("missing common EKF uncertainty snapshot");
        const double T = settings.at("execute").at("time-step");
        const double U = configuredPlanarComponentMax();
        const double alpha = experiment.value("squared-alpha", 1.0 / T);
        const double linkAlpha=experiment.value("work2-link-alpha",alpha);
        if(!std::isfinite(linkAlpha)||linkAlpha<=0||linkAlpha*T>1.)throw std::invalid_argument("work2 link alpha outside hold condition");
        if (!usesEstimatedTaskContract() || !estimatorInLoop || settings.at("model") != "SingleIntegrate2D"
            || !settings.at("cbfs").at("input-limits").at("on").get<bool>()
            || !std::isfinite(error) || error < 0 || !std::isfinite(T) || T <= 0
            || !std::isfinite(alpha) || alpha <= 0 || alpha * T > 1.0)
            throw std::invalid_argument("invalid symmetric L1 controller conditions");
        const double D = settings.at("cbfs").at("without-slack").at("safety").at("safe-distance");
        const double R = settings.at("cbfs").at("without-slack").at("comm-fixed").at("max-range");
        if (!std::isfinite(D) || !std::isfinite(R) || D<=0 || R<=D)
            throw std::invalid_argument("symmetric L1 requires finite positive distance limits");
        for (auto it = cbfNoSlack.cbfs.begin(); it != cbfNoSlack.cbfs.end();) {
            if (it->first.rfind("safetyCBF", 0) == 0 || it->first.rfind("fixedCommCBF", 0) == 0)
                it = cbfNoSlack.cbfs.erase(it);
            else ++it;
        }
        tvtSymmetricRows = json::array();
        auto install = [&](const std::string& name, const Point& other, bool isBase, bool collision, int peer) {
            const Point relative = estimatorPosition - other;
            const double E = marginMode == "fixed"
                ? error * (isBase ? 1.0 : 2.0)
                : epsilonScale * (estimatorEpsilon +
                    (isBase ? 0.0 : tvtEstimatorEpsilons.at(peer)));
            if (!std::isfinite(E) || E < 0.0)
                throw std::invalid_argument("invalid per-edge uncertainty scale");
            const double B = U * (isBase ? 1.0 : 2.0);
            const double hLower = collision ? std::pow(std::max(0.0, relative.len() - E), 2) - D * D
                : R * R - std::pow(relative.len() + E, 2);
            const double q = hLower;
            const double beta = isBase ? 1.0 : 0.5;
            const double rowAlpha=collision?alpha:linkAlpha;
            const double sign = collision ? 1.0 : -1.0;
            const double reserve = 2.0 * E + (collision ? 0.0 : T * B);
            tvtSymmetricRows.push_back({{"name", name}, {"coe", {{"vx", sign * 2 * relative.x},
                {"vy", sign * 2 * relative.y}, {"yawRateRad", 0.0}}}, {"const", rowAlpha * beta * q},
                {"aux_coe", {-reserve, -reserve}}, {"row_mode", "symmetric-l1"},
                {"edge", {{"kind", collision ? "collision" : "link"}, {"peer", peer}, {"is_base", isBase}}},
                {"relative_estimate", {relative.x, relative.y}}, {"E", E}, {"B", B},
                {"alpha", rowAlpha}, {"T", T}, {"beta", beta}, {"h_lower", hLower}, {"q", q}});
        };
        for (int peer = 1; peer <= n; ++peer) {
            if (peer != id) install("safetyCBF(#" + std::to_string(peer) + ")", comm->_othersPos.at(peer), false, true, peer);
        }
        const json incident = incidentFixedLocalizationReferences();
        for (const auto& base : incident.at("baseIds")) {
            const int peer = base.get<int>();
            install("fixedCommCBF(base-" + std::to_string(peer) + ")", bases.at(peer), true, false, peer);
        }
        for (const auto& anchor : incident.at("anchorIds")) {
            const int peer = anchor.get<int>();
            install("fixedCommCBF(#" + std::to_string(peer) + ")", comm->_othersPos.at(peer), false, false, peer);
        }
    }

    cbf2026::NodeRateCertificate proposeRateCertificate(
        std::uint64_t snapshotVersion,
        std::uint64_t allocationVersion,
        const std::map<int, cbf2026::EndpointCertificateSnapshot>&
            candidatePredecessors,
        const std::optional<std::vector<int>>& frozenReferenceIds =
            std::nullopt
    ) const {
        if (!usesAnalyticTopologicalRateCertificate()
            || !enablePositionCovariance) {
            throw std::invalid_argument(
                "pure theorem certificate proposal is not enabled"
            );
        }
        const auto& covarianceConfig = settings.at("position_covariance");
        if (!covarianceConfig.contains("reference-selection")
            || !covarianceConfig.at("reference-selection").is_string()) {
            throw std::invalid_argument(
                "position covariance reference selection is absent"
            );
        }
        const std::string referenceSelection =
            covarianceConfig.at("reference-selection").get<std::string>();
        if (referenceSelection != "dynamic-lower-index"
            && referenceSelection != "fixed-cbf-only") {
            throw std::invalid_argument(
                "position covariance reference selection is unknown"
            );
        }
        const auto& inputLimits = settings.at("cbfs").at("input-limits");
        if (!inputLimits.at("on").get<bool>()
            || inputLimits.at("planar-component-max").get<double>()
               <= 0.0) {
            throw std::invalid_argument(
                "pure theorem proposal requires an enabled positive planar bound"
            );
        }

        json references = {
            {"anchorIds", json::array()},
            {"baseIds", json::array()}
        };
        auto appendUnique = [](json& ids, int candidate) {
            if (std::find(ids.begin(), ids.end(), json(candidate))
                == ids.end()) {
                ids.push_back(candidate);
            }
        };
        if (frozenReferenceIds.has_value()) {
            for (const int referenceId : *frozenReferenceIds) {
                if (referenceId < 0) {
                    const std::int64_t baseId =
                        -static_cast<std::int64_t>(referenceId) - 1;
                    if (baseId < 0
                        || baseId >= static_cast<std::int64_t>(bases.size())) {
                        throw std::invalid_argument(
                            "frozen base reference is outside the mission"
                        );
                    }
                    appendUnique(
                        references["baseIds"], static_cast<int>(baseId)
                    );
                } else if (referenceId > 0) {
                    appendUnique(references["anchorIds"], referenceId);
                } else {
                    throw std::invalid_argument(
                        "frozen UAV reference zero is invalid"
                    );
                }
            }
        } else {
            references = fixedLocalizationReferences();
        }
        if (!frozenReferenceIds.has_value()
            && referenceSelection == "dynamic-lower-index") {
            const double maximumRange = settings.at("cbfs")
                .at("without-slack").at("comm-fixed")
                .at("max-range").get<double>();
            const Point position = model->xy();
            for (std::size_t baseId = 0; baseId < bases.size(); ++baseId) {
                if (position.distance_to(bases.at(baseId)) <= maximumRange) {
                    appendUnique(
                        references["baseIds"],
                        static_cast<int>(baseId)
                    );
                }
            }
            for (const auto& [otherId, predecessor] :
                 candidatePredecessors) {
                if (otherId == id
                    || !isSamePartAsMe(otherId)
                    || getIdInPart(otherId) >= idInMyPart) {
                    continue;
                }
                cbf2026::validateEndpointCertificateSnapshot(
                    otherId, predecessor
                );
                if (predecessor.snapshotVersion != snapshotVersion
                    || predecessor.allocationVersion != allocationVersion) {
                    throw std::invalid_argument(
                        "candidate predecessor mixes certificate versions"
                    );
                }
                const Point predecessorPosition(
                    predecessor.estimate.x(), predecessor.estimate.y()
                );
                if (position.distance_to(predecessorPosition)
                    <= maximumRange) {
                    appendUnique(references["anchorIds"], otherId);
                }
            }
        }
        std::vector<int> baseIds =
            references.at("baseIds").get<std::vector<int>>();
        std::vector<int> anchorIds =
            references.at("anchorIds").get<std::vector<int>>();
        std::sort(baseIds.begin(), baseIds.end());
        std::sort(
            anchorIds.begin(), anchorIds.end(),
            [this](int lhs, int rhs) {
                if (getIdInPart(lhs) != getIdInPart(rhs)) {
                    return getIdInPart(lhs) < getIdInPart(rhs);
                }
                return lhs < rhs;
            }
        );
        if (std::adjacent_find(baseIds.begin(), baseIds.end())
                != baseIds.end()
            || std::adjacent_find(anchorIds.begin(), anchorIds.end())
               != anchorIds.end()) {
            throw std::invalid_argument(
                "candidate localization references are duplicated"
            );
        }

        const Point position = model->xy();
        const double planarComponentMax = configuredPlanarComponentMax();
        const double speedBound = std::sqrt(2.0) * planarComponentMax;
        std::vector<cbf2026::ReferenceRateInput> rateInputs;
        const auto appendReference = [&rateInputs, &position, this](
            const cbf2026::PredecessorRateSnapshot& predecessor,
            const Point& referencePosition
        ) {
            const double distance = position.distance_to(referencePosition);
            const double sigma = rangingUncertaintyFunction(distance);
            const Eigen::Vector2d displacement(
                position.x - referencePosition.x,
                position.y - referencePosition.y
            );
            rateInputs.push_back({
                predecessor,
                distance,
                displacement / distance,
                sigma * sigma
            });
        };
        for (int baseId : baseIds) {
            if (baseId < 0 || baseId >= static_cast<int>(bases.size())) {
                throw std::invalid_argument(
                    "candidate base reference is outside the mission"
                );
            }
            appendReference(
                {
                    cbf2026::canonicalBaseReferenceId(baseId),
                    0,
                    snapshotVersion,
                    true,
                    Eigen::Matrix2d::Zero(),
                    0.0,
                    0.0
                },
                bases.at(baseId)
            );
        }
        for (int otherId : anchorIds) {
            const auto predecessor = candidatePredecessors.find(otherId);
            if (predecessor == candidatePredecessors.end()) {
                throw std::invalid_argument(
                    "candidate UAV predecessor is missing"
                );
            }
            cbf2026::validateEndpointCertificateSnapshot(
                otherId, predecessor->second
            );
            if (predecessor->second.snapshotVersion != snapshotVersion
                || predecessor->second.allocationVersion
                   != allocationVersion) {
                throw std::invalid_argument(
                    "candidate UAV predecessor mixes versions"
                );
            }
            appendReference(
                {
                    cbf2026::canonicalUavReferenceId(otherId),
                    getIdInPart(otherId),
                    snapshotVersion,
                    false,
                    predecessor->second.covariance,
                    predecessor->second.covarianceRateBound,
                    speedBound
                },
                Point(
                    predecessor->second.estimate.x(),
                    predecessor->second.estimate.y()
                )
            );
        }
        return cbf2026::computeNodeRateCertificate({
            id,
            idInMyPart,
            snapshotVersion,
            planarComponentMax,
            std::move(rateInputs)
        });
    }

    cbf2026::NodeRateInput committedRateEvidenceInput(
        const cbf2026::CommittedCertificateState& state
    ) const {
        if (!usesAnalyticTopologicalRateCertificate()
            || !state.valid
            || state.version == 0
            || state.certificates.count(id) != 1U
            || state.endpoints.count(id) != 1U) {
            throw std::invalid_argument(
                "committed rate evidence input is unavailable"
            );
        }
        const auto& certificate = state.certificates.at(id);
        const auto& local = state.endpoints.at(id);
        if (certificate.snapshotVersion != state.version
            || local.snapshotVersion != state.version) {
            throw std::invalid_argument(
                "committed rate evidence input mixes versions"
            );
        }

        const double planarComponentMax = configuredPlanarComponentMax();
        const double predecessorSpeedBound =
            std::sqrt(2.0) * planarComponentMax;
        std::vector<cbf2026::ReferenceRateInput> references;
        references.reserve(certificate.frozenReferences.size());
        for (const auto& frozen : certificate.frozenReferences) {
            cbf2026::PredecessorRateSnapshot predecessor;
            predecessor.referenceId = frozen.referenceId;
            predecessor.snapshotVersion = state.version;
            Eigen::Vector2d referencePosition;
            if (frozen.referenceId < 0) {
                const std::int64_t baseId =
                    -static_cast<std::int64_t>(frozen.referenceId) - 1;
                if (baseId < 0
                    || baseId >= static_cast<std::int64_t>(bases.size())) {
                    throw std::invalid_argument(
                        "committed evidence base reference is invalid"
                    );
                }
                predecessor.localIndex = 0;
                predecessor.hoveringBase = true;
                predecessor.covariance = Eigen::Matrix2d::Zero();
                predecessor.covarianceRateBound = 0.0;
                predecessor.speedBound = 0.0;
                const Point base = bases.at(static_cast<std::size_t>(baseId));
                referencePosition = Eigen::Vector2d(base.x, base.y);
            } else {
                const auto endpoint = state.endpoints.find(
                    frozen.referenceId
                );
                if (endpoint == state.endpoints.end()) {
                    throw std::invalid_argument(
                        "committed evidence UAV predecessor is missing"
                    );
                }
                predecessor.localIndex = getIdInPart(frozen.referenceId);
                predecessor.hoveringBase = false;
                predecessor.covariance = endpoint->second.covariance;
                predecessor.covarianceRateBound =
                    endpoint->second.covarianceRateBound;
                predecessor.speedBound = predecessorSpeedBound;
                referencePosition = endpoint->second.estimate;
            }
            const Eigen::Vector2d displacement =
                local.estimate - referencePosition;
            const double distance = displacement.norm();
            const double sigma = rangingUncertaintyFunction(distance);
            references.push_back({
                predecessor,
                distance,
                displacement / distance,
                sigma * sigma
            });
        }
        return {
            id,
            idInMyPart,
            state.version,
            planarComponentMax,
            std::move(references)
        };
    }

    cbf2026::HardConstraintProblem buildHardConstraintProblem(
        const cbf2026::CommittedCertificateState& state
    ) const {
        if (!usesAnalyticTopologicalRateCertificate()
            || !state.valid
            || state.version == 0
            || static_cast<int>(state.endpoints.size()) != n
            || static_cast<int>(state.nodeVersions.size()) != n) {
            throw std::invalid_argument(
                "complete committed theorem certificate state is unavailable"
            );
        }
        for (int robotId = 1; robotId <= n; ++robotId) {
            const auto endpoint = state.endpoints.find(robotId);
            const auto version = state.nodeVersions.find(robotId);
            if (endpoint == state.endpoints.end()
                || version == state.nodeVersions.end()
                || version->second != state.version
                || endpoint->second.snapshotVersion != state.version) {
                throw std::invalid_argument(
                    "committed theorem endpoint state mixes versions"
                );
            }
            cbf2026::validateEndpointCertificateSnapshot(
                robotId, endpoint->second
            );
        }

        const auto& local = state.endpoints.at(id);
        cbf2026::HardConstraintProblem problem;
        problem.owner = id;
        problem.controlSize = model->uSize();
        problem.planarComponentMax = configuredPlanarComponentMax();
        problem.yawRateMax = 0.35;
        problem.snapshotVersion = state.version;
        problem.allocationVersion = local.allocationVersion;
        problem.bounds = cbf2026::theoremInputBounds(
            problem.planarComponentMax, problem.yawRateMax
        );
        if (problem.controlSize != 3) {
            throw std::invalid_argument(
                "theorem hard problem requires three controls"
            );
        }

        const auto appendOwnedRow = [&problem, this](
            const cbf2026::EndpointRow& endpointRow,
            double barrier,
            const std::string& name
        ) {
            cbf2026::HardConstraintRow row;
            row.edge = endpointRow.edge;
            row.owner = endpointRow.owner;
            row.name = name;
            row.coefficients = cbf2026::endpointRowToModelControl(
                endpointRow, problem.controlSize
            );
            row.constant = endpointRow.constant;
            row.postResetBarrier = barrier;
            row.snapshotVersion = endpointRow.snapshotVersion;
            row.allocationVersion = endpointRow.allocationVersion;
            problem.rows.push_back(std::move(row));
        };
        const auto selectOwnedRow = [this](
            const std::vector<cbf2026::EndpointRow>& rows
        ) -> const cbf2026::EndpointRow& {
            const auto row = std::find_if(
                rows.begin(), rows.end(),
                [this](const cbf2026::EndpointRow& candidate) {
                    return candidate.owner == id;
                }
            );
            if (row == rows.end()) {
                throw std::invalid_argument(
                    "hard edge is missing the local endpoint row"
                );
            }
            return *row;
        };

        const auto registry = fixedBarrierEdgeRegistry();
        const auto& commConfig =
            settings["cbfs"]["without-slack"]["comm-fixed"];
        if (commConfig.value("on", false)) {
            if (commConfig.value("mode", "") != "allocated-pairwise"
                || !commConfig.value("consider-uncertainty", true)) {
                throw std::invalid_argument(
                    "theorem localization hard rows require allocated pairwise mode"
                );
            }
            const double maximumRange =
                commConfig.at("max-range").get<double>();
            const ClassKParameters parameters =
                readClassKParameters(commConfig, 0.1, 1);
            for (const auto& edge : registry.fixedLocalizationEdges()) {
                if (edge.low != id && edge.high != id) {
                    continue;
                }
                const auto& first = state.endpoints.at(edge.low);
                Eigen::Vector2d secondPosition;
                double epsilonJ = 0.0;
                double nuJ = 0.0;
                if (edge.baseId >= 0) {
                    if (edge.baseId >= static_cast<int>(bases.size())) {
                        throw std::invalid_argument(
                            "fixed base edge has an invalid base ID"
                        );
                    }
                    const Point base = bases.at(edge.baseId);
                    secondPosition = Eigen::Vector2d(base.x, base.y);
                } else {
                    const auto& second = state.endpoints.at(edge.high);
                    if (second.allocationVersion
                        != first.allocationVersion) {
                        throw std::invalid_argument(
                            "fixed localization endpoints mix allocation versions"
                        );
                    }
                    secondPosition = second.estimate;
                    epsilonJ = second.epsilon;
                    nuJ = second.barNu;
                }
                auto snapshot = cbf2026::makeEdgeSnapshot(
                    edge,
                    first.estimate,
                    secondPosition,
                    first.barNu,
                    nuJ,
                    0.0,
                    state.version,
                    problem.allocationVersion
                );
                const double barrier = maximumRange - snapshot.separation
                    - first.epsilon - epsilonJ;
                snapshot.alpha = allocatedAlphaValue(parameters, barrier);
                const auto rows = cbf2026::allocatedRows(
                    snapshot,
                    edge.baseId >= 0 ? 1.0 : 0.5,
                    edge.baseId >= 0 ? 0.0 : 0.5
                );
                const int otherId = edge.low == id ? edge.high : edge.low;
                appendOwnedRow(
                    selectOwnedRow(rows),
                    barrier,
                    edge.baseId >= 0
                    ? "fixedCommCBF(base-"
                      + std::to_string(edge.baseId) + ")"
                    : "fixedCommCBF(#" + std::to_string(otherId) + ")"
                );
            }
        }

        const auto& safetyConfig =
            settings["cbfs"]["without-slack"]["safety"];
        if (safetyConfig.value("on", false)) {
            if (safetyConfig.value("mode", "") != "allocated-pairwise"
                || !safetyConfig.value("consider-uncertainty", true)) {
                throw std::invalid_argument(
                    "theorem collision hard rows require allocated pairwise mode"
                );
            }
            const double safeDistance =
                safetyConfig.at("safe-distance").get<double>();
            const ClassKParameters parameters =
                readClassKParameters(safetyConfig, 0.1, 1);
            for (const auto& edge : registry.collisionEdges()) {
                if (edge.low != id && edge.high != id) {
                    continue;
                }
                const auto& first = state.endpoints.at(edge.low);
                const auto& second = state.endpoints.at(edge.high);
                if (first.allocationVersion != second.allocationVersion) {
                    throw std::invalid_argument(
                        "collision endpoints mix allocation versions"
                    );
                }
                auto snapshot = cbf2026::makeEdgeSnapshot(
                    edge,
                    first.estimate,
                    second.estimate,
                    first.barNu,
                    second.barNu,
                    0.0,
                    state.version,
                    problem.allocationVersion
                );
                const double barrier = snapshot.separation - safeDistance
                    - first.epsilon - second.epsilon;
                snapshot.alpha = allocatedAlphaValue(parameters, barrier);
                const auto rows = cbf2026::allocatedRows(
                    snapshot, 0.5, 0.5
                );
                const int otherId = edge.low == id ? edge.high : edge.low;
                appendOwnedRow(
                    selectOwnedRow(rows),
                    barrier,
                    "safetyCBF(#" + std::to_string(otherId) + ")"
                );
            }
        }
        std::sort(
            problem.rows.begin(), problem.rows.end(),
            [](const cbf2026::HardConstraintRow& lhs,
               const cbf2026::HardConstraintRow& rhs) {
                if (lhs.edge != rhs.edge) {
                    return lhs.edge < rhs.edge;
                }
                if (lhs.owner != rhs.owner) {
                    return lhs.owner < rhs.owner;
                }
                return lhs.name < rhs.name;
            }
        );
        return problem;
    }

    struct LocalHardQpEvidence {
        cbf2026::FeasibilityResult feasibility;
        std::optional<Eigen::VectorXd> solution;
    };

    LocalHardQpEvidence solveLocalHardQpEvidence(
        const cbf2026::HardConstraintProblem& problem
    ) const {
        const std::string digest =
            cbf2026::canonicalHardConstraintProblemHash(problem);
        try {
            if (problem.owner != id
                || problem.controlSize != 3
                || !std::isfinite(problem.planarComponentMax)
                || problem.planarComponentMax <= 0.0
                || problem.yawRateMax != 0.35
                || problem.bounds.size() != 6) {
                return {{false, "invalid-problem", -inf, digest}, std::nullopt};
            }
            const auto requiredBounds = cbf2026::theoremInputBounds(
                problem.planarComponentMax, problem.yawRateMax
            );
            for (std::size_t index = 0;
                 index < requiredBounds.size(); ++index) {
                const auto& bound = problem.bounds[index];
                const auto& required = requiredBounds[index];
                if (bound.controlIndex < 0
                    || bound.controlIndex >= problem.controlSize
                    || !std::isfinite(bound.coefficient)
                    || !std::isfinite(bound.limit)
                    || bound.limit <= 0.0
                    || bound.controlIndex != required.controlIndex
                    || bound.coefficient != required.coefficient
                    || bound.limit != required.limit) {
                    return {{false, "invalid-bound", -inf, digest}, std::nullopt};
                }
            }
            for (const auto& row : problem.rows) {
                if (row.coefficients.size() != problem.controlSize
                    || !row.coefficients.allFinite()
                    || !std::isfinite(row.constant)) {
                    return {{false, "invalid-row", -inf, digest}, std::nullopt};
                }
            }
            json objectiveSettings =
                settings["cbfs"]["objective-function"];
            auto freshOptimiser = createOptimiser(
                settings.at("optimiser").get<std::string>(),
                objectiveSettings
            );
            freshOptimiser->start(problem.controlSize, problem.controlSize);
            Eigen::VectorXd nominal = Eigen::VectorXd::Zero(
                problem.controlSize
            );
            freshOptimiser->setObjective(nominal);
            for (const auto& bound : problem.bounds) {
                Eigen::VectorXd coefficients = Eigen::VectorXd::Zero(
                    problem.controlSize
                );
                coefficients[bound.controlIndex] = bound.coefficient;
                freshOptimiser->addLinearConstraint(
                    coefficients, -bound.limit
                );
            }
            for (const auto& row : problem.rows) {
                freshOptimiser->addLinearConstraint(
                    row.coefficients, -row.constant
                );
            }
            const Eigen::VectorXd solution = freshOptimiser->solve();
            const json solverStatus = freshOptimiser->getStatus();
            const std::string status =
                solverStatus.value("status", "unknown");
            if (status != "optimal"
                || solution.size() != problem.controlSize
                || !solution.allFinite()) {
                return {{false, status, -inf, digest}, std::nullopt};
            }
            double minimumResidual = inf;
            for (const auto& bound : problem.bounds) {
                minimumResidual = std::min(
                    minimumResidual,
                    bound.coefficient * solution[bound.controlIndex]
                    + bound.limit
                );
            }
            for (const auto& row : problem.rows) {
                minimumResidual = std::min(
                    minimumResidual,
                    row.constant + row.coefficients.dot(solution)
                );
            }
            return {{
                std::isfinite(minimumResidual)
                    && minimumResidual >= -1e-7,
                status,
                minimumResidual,
                digest
            }, solution};
        } catch (const std::exception& error) {
            return {{false, error.what(), -inf, digest}, std::nullopt};
        } catch (...) {
            return {{false, "unknown-error", -inf, digest}, std::nullopt};
        }
    }

    cbf2026::FeasibilityResult checkLocalHardQpFeasibility(
        const cbf2026::HardConstraintProblem& problem
    ) const {
        return solveLocalHardQpEvidence(problem).feasibility;
    }

    void clearEvidenceOptimisationState() {
        opt = json::object();
        lastConsumedHardConstraintProblem.reset();
        lastConsumedHardProblemHash.clear();
    }

    void optimise() {
        canonical::forbid("void optimise");
        const bool evidenceMode = settings.contains("evidence-stream")
            && settings.at("evidence-stream").is_object()
            && settings.at("evidence-stream").value("enabled", false);
        if (evidenceMode) {
            clearEvidenceOptimisationState();
        } else {
            lastConsumedHardConstraintProblem.reset();
            lastConsumedHardProblemHash.clear();
        }
        VectorXd uNominal = VectorXd::Zero(model->uSize());
        const bool theoremAligned =
            usesAnalyticTopologicalRateCertificate();
        const cbf2026::HardConstraintProblem* committedHardProblem = nullptr;
        std::optional<double> hardInteriorFloor;
        json hardInteriorSelection = json();
        bool hardInteriorEnabled = false;
        if (theoremAligned) {
            if (!committedCertificateState.valid
                || committedCertificateState.version == 0) {
                throw std::runtime_error(
                    "committed theorem hard certificate is unavailable"
                );
            }
            const auto problem =
                committedCertificateState.hardProblems.find(id);
            if (problem == committedCertificateState.hardProblems.end()) {
                throw std::runtime_error(
                    "committed theorem hard problem is unavailable"
                );
            }
            committedHardProblem = &problem->second;
            if (committedHardProblem->snapshotVersion
                    != committedCertificateState.version
                || committedHardProblem->owner != id) {
                throw std::runtime_error(
                    "committed theorem hard problem mixes versions"
                );
            }
            lastConsumedHardConstraintProblem = *committedHardProblem;
            lastConsumedHardProblemHash =
                cbf2026::canonicalHardConstraintProblemHash(
                    *committedHardProblem
                );
            const auto& cbfs = settings.at("cbfs");
            hardInteriorEnabled = settings.contains("qualified-controller");
            std::string controllerSchema;
            if (hardInteriorEnabled) {
                const auto& marker = settings.at("qualified-controller");
                if (!marker.is_object() || marker.size() != 1
                    || !marker.contains("schema-version")
                    || !marker.at("schema-version").is_string()) {
                    throw std::invalid_argument(
                        "theorem hard-interior marker is invalid"
                    );
                }
                controllerSchema = marker.at("schema-version").get<std::string>();
                if (controllerSchema != "hard-interior-v2"
                    && controllerSchema != "hard-interior-v3") {
                    throw std::invalid_argument(
                        "theorem hard-interior marker is invalid"
                    );
                }
                if (!cbfs.contains("hard-interior-selection")) {
                    throw std::invalid_argument(
                        "theorem hard-interior policy is unavailable"
                    );
                }
            } else if (cbfs.contains("hard-interior-selection")) {
                throw std::invalid_argument(
                    "theorem hard-interior policy requires a marker"
                );
            }
            if (hardInteriorEnabled) {
                const auto& policy = cbfs.at("hard-interior-selection");
                const auto exactPolicyNumber = [](const json& value,
                                                  const double expected) {
                    return value.is_number_float()
                        && std::isfinite(value.get<double>())
                        && value.get<double>() == expected;
                };
                const std::string policyMode = controllerSchema
                    == "hard-interior-v2"
                    ? "planar-chebyshev-fraction-cap-v1"
                    : "planar-chebyshev-fraction-cap-v2";
                const double fraction = controllerSchema
                    == "hard-interior-v2" ? 0.1 : 0.131;
                if (!policy.is_object() || policy.size() != 4
                    || !policy.contains("mode")
                    || !policy.at("mode").is_string()
                    || policy.at("mode").get<std::string>() != policyMode
                    || !policy.contains("fraction")
                    || !exactPolicyNumber(policy.at("fraction"), fraction)
                    || !policy.contains("cap-mps")
                    || !exactPolicyNumber(policy.at("cap-mps"), 0.1)
                    || !policy.contains("feasibility-tolerance-mps")
                    || !exactPolicyNumber(
                        policy.at("feasibility-tolerance-mps"), 1e-9
                    )) {
                    throw std::invalid_argument(
                        "theorem hard-interior selection policy is invalid"
                    );
                }
                constexpr double capMps = 0.1;
                constexpr double feasibilityTolerance = 1e-9;
                const auto chebyshev = cbf2026::solvePlanarHardRowChebyshev(
                    *committedHardProblem, feasibilityTolerance
                );
                if (!std::isfinite(chebyshev.radius)
                    || chebyshev.radius < -feasibilityTolerance) {
                    throw std::runtime_error(
                        "theorem planar hard-row Chebyshev radius is infeasible"
                    );
                }
                const double floor = cbf2026::frozenInteriorFloor(
                    chebyshev.radius,
                    fraction,
                    capMps,
                    feasibilityTolerance
                );
                hardInteriorFloor = floor;
                hardInteriorSelection = {
                    {"mode", policyMode},
                    {"fraction", fraction},
                    {"cap_mps", capMps},
                    {"feasibility_tolerance_mps", feasibilityTolerance},
                    {"planar_chebyshev_radius_mps", chebyshev.radius},
                    {"enforced_floor_mps", floor}
                };
                if (controllerSchema == "hard-interior-v3") {
                    hardInteriorSelection["schema_version"] = controllerSchema;
                }
            }
        }
        const json inputLimitsConfig =
            settings["cbfs"].value("input-limits", json::object());
        const bool inputLimitsEnabled =
            inputLimitsConfig.value("on", false);
        const double planarComponentMax =
            inputLimitsConfig.value("planar-component-max", 25.0);
        const double yawRateMax =
            inputLimitsConfig.value("yaw-rate-max", 0.35);
        constexpr double saturationTolerance = 1e-7;
        opt = {
                {"nominal",    model->control2Json(uNominal)},
                {"result",     model->control2Json(uNominal)},
                {"cbfNoSlack", json::array()},
                {"cbfSlack",   json::array()},
                {"input_limits", {
                    {"enabled", inputLimitsEnabled},
                    {"planar_component_max", planarComponentMax},
                    {"yaw_rate_max", yawRateMax},
                    {"bound_row_count", 0},
                    {"saturation_tolerance", saturationTolerance},
                    {"saturated", {
                        {"vx", false},
                        {"vy", false},
                        {"yawRateRad", false},
                        {"any", false}
                    }}
                }}
        };
        if (!hardInteriorSelection.is_null()) {
            opt["hard_interior_selection"] = hardInteriorSelection;
        }
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
        std::vector<Eigen::VectorXd> hardConstraintCoefficients;
        double chargeRate = 1.0;
        if (world.isCharging(model->xy(), chargeRate) && model->getStateVariable("battery") < model->BATTERY_MAX) {
            model->startCharge();
            model->setChargeRate(chargeRate);
            uNominal.setZero();
        }
            optimiser->clear();

            auto f = model->f();
            auto g = model->g();
            auto x = taskEvaluationState();
            opt["evaluation_position"] = {x[0], x[1]};

            int uSize = model->uSize();
            const int originalSlackSize = cbfSlack.size();
            const bool work2Soft = settings.at("work2").at("arm")=="softT";
            const Point work2Target = cvt.ct[id];
            const Eigen::Vector2d work2Delta(work2Target.x-x[0],work2Target.y-x[1]);
            const double work2Distance=work2Delta.norm(),work2Speed=std::min(25.0,work2Distance);
            Eigen::Vector2d work2Task=Eigen::Vector2d::Zero(),work2Tangent=Eigen::Vector2d::Zero();
            if(work2Distance>1e-12)work2Task=work2Speed*work2Delta/work2Distance;
            int work2Peer=-1;double work2Best=-1.;
            for(const auto& row:tvtSymmetricRows) {
                if(row.at("edge").at("kind")!="collision")continue;
                const Eigen::Vector2d a(row.at("coe").at("vx"),row.at("coe").at("vy"));
                const double norm=a.norm(),F=a.dot(work2Task)+row.at("aux_coe")[0].get<double>()*std::abs(work2Task[0])
                    +row.at("aux_coe")[1].get<double>()*std::abs(work2Task[1])+row.at("const").get<double>();
                if(F>=-1e-6||norm<=1e-12)continue;
                if(-F/norm>work2Best){work2Best=-F/norm;work2Peer=row.at("edge").at("peer");work2Tangent<<-a[1]/norm,a[0]/norm;}
            }
            const bool work2Active=work2Soft&&work2Peer>=0;
            opt["work2_side_task"]={{"enabled",work2Soft},{"active",work2Active},{"peer",work2Peer},
                {"tangent",{work2Tangent[0],work2Tangent[1]}},{"target_speed",work2Speed},{"target",{work2Target.x,work2Target.y}},
                {"slack_index",work2Soft?originalSlackSize:-1},{"original_slack_count",originalSlackSize}};
            if(!uNominal.isZero(0.))throw std::runtime_error("work2 expects original zero nominal");
            int slackSize = originalSlackSize + (work2Soft?1:0);
            const int auxiliarySize = usesSymmetricL1Rows() ? 2 : usesNormConeRows() ? tvtLiteratureRows.size() : 0;
            int totalSize = uSize + slackSize + auxiliarySize;
            optimiser->setAuxiliaryVariableCount(auxiliarySize);
            optimiser->start(totalSize, uSize);
            if (usesSymmetricL1Rows()) {
                for (int axis = 0; axis < 2; ++axis) {
                    const int z = uSize + slackSize + axis;
                    for (double sign : {-1.0, 1.0}) {
                        VectorXd coefficient = VectorXd::Zero(totalSize);
                        coefficient[z] = 1; coefficient[axis] = sign;
                        optimiser->addLinearConstraint(coefficient, 0);
                    }
                    VectorXd cap = VectorXd::Zero(totalSize); cap[z] = -1;
                    optimiser->addLinearConstraint(cap, -planarComponentMax);
                }
                opt["auxiliary_variable_layout"] = {"z_x", "z_y"};
            } else if (usesNormConeRows()) {
                opt["auxiliary_variable_layout"]=json::array();opt["norm_cones"]=json::array();
                for(size_t index=0;index<tvtLiteratureRows.size();++index) {
                    const auto& row=tvtLiteratureRows[index];
                    Eigen::Vector2d offset(row["other_command"][0].get<double>(),row["other_command"][1].get<double>());
                    optimiser->addPlanarNormCone(uSize+slackSize+index,offset);
                    // q may equal the exact norm, which is <= V on the input
                    // box. This cap bounds only an auxiliary representation;
                    // decreasing q improves its barrier row.
                    VectorXd cap=VectorXd::Zero(totalSize);cap[uSize+slackSize+index]=-1;
                    optimiser->addLinearConstraint(cap,-row["V"].get<double>());
                    opt["auxiliary_variable_layout"].push_back("relative_norm_"+std::to_string(index));
                    opt["norm_cones"].push_back({{"aux_index",index},{"offset",row["other_command"]},{"upper_bound",row["V"]},{"name",row["name"]}});
                }
            }

            if (theoremAligned) {
                if (!inputLimitsEnabled
                    || committedHardProblem == nullptr
                    || committedHardProblem->controlSize != uSize
                    || committedHardProblem->bounds.size() != 6
                    || committedHardProblem->planarComponentMax
                       != planarComponentMax
                    || committedHardProblem->yawRateMax != yawRateMax) {
                    throw std::invalid_argument(
                        "theorem optimiser input bounds disagree with the committed problem"
                    );
                }
                for (const auto& bound : committedHardProblem->bounds) {
                    Eigen::VectorXd coefficients =
                        Eigen::VectorXd::Zero(totalSize);
                    coefficients[bound.controlIndex] = bound.coefficient;
                    optimiser->addLinearConstraint(
                        coefficients, -bound.limit
                    );
                }
                opt["input_limits"]["bound_row_count"] =
                    committedHardProblem->bounds.size();
            } else if (inputLimitsEnabled) {
                if (settings.value("model", "") != "SingleIntegrate2D"
                    || uSize != 3) {
                    throw std::invalid_argument(
                        "Input limits require the SingleIntegrate2D "
                        "three-control velocity-command model"
                    );
                }
                if (!std::isfinite(planarComponentMax)
                    || planarComponentMax <= 0.0
                    || !std::isfinite(yawRateMax)
                    || yawRateMax <= 0.0) {
                    throw std::invalid_argument(
                        "Input limits must be finite and positive"
                    );
                }

                auto addInputBound = [&](int index, double sign, double limit) {
                    Eigen::VectorXd coefficients =
                        Eigen::VectorXd::Zero(totalSize);
                    coefficients[index] = sign;
                    optimiser->addLinearConstraint(coefficients, -limit);
                };
                addInputBound(0, 1.0, planarComponentMax);
                addInputBound(0, -1.0, planarComponentMax);
                addInputBound(1, 1.0, planarComponentMax);
                addInputBound(1, -1.0, planarComponentMax);
                addInputBound(2, 1.0, yawRateMax);
                addInputBound(2, -1.0, yawRateMax);
                opt["input_limits"]["bound_row_count"] = 6;
            }

            if(settings.contains("cano-pcg")) {
                if(!canoPcgNominal || !canoPcgNominalFrame || !estimatorSnapshotFrame || *canoPcgNominalFrame!=*estimatorSnapshotFrame)throw std::runtime_error("Cano current collective nominal unavailable");
                uNominal=*canoPcgNominal;opt["nominal"]=model->control2Json(uNominal);opt["cano_nominal"]=canoPcgNominalRecord;
            }
            optimiser->setObjective(uNominal);

            std::string cbf_method = settings["cbfs"]["without-slack"].value("method", "all");

            if (theoremAligned) {
                double enforcedFloor = 0.0;
                if (hardInteriorEnabled) {
                    if (!hardInteriorFloor.has_value()) {
                        throw std::logic_error(
                            "theorem hard-interior floor is unavailable"
                        );
                    }
                    enforcedFloor = *hardInteriorFloor;
                }
                for (const auto& row : committedHardProblem->rows) {
                    optimiser->addLinearConstraint(
                        row.coefficients, -row.constant + enforcedFloor
                    );
                    jsonCBFNoSlack.emplace_back(json{
                        {"name", row.name},
                        {"coe", model->control2Json(row.coefficients)},
                        {"const", row.constant},
                        {"edge", {
                            {"kind", row.edge.kind
                                == cbf2026::EdgeKind::Localization
                                ? "localization" : "collision"},
                            {"low", row.edge.low},
                            {"high", row.edge.high},
                            {"base_id", row.edge.baseId}
                        }},
                        {"owner", row.owner},
                        {"snapshot_version", row.snapshotVersion},
                        {"allocation_version", row.allocationVersion},
                        {"post_reset_barrier", row.postResetBarrier}
                    });
                    hardConstraintCoefficients.push_back(row.coefficients);
                }
            } else if (usesSymmetricL1Rows()) {
                if (!cbfNoSlack.cbfs.empty()) throw std::invalid_argument("symmetric pilot permits only declared hard pair rows and input bounds");
                for (auto row : tvtSymmetricRows) {
                    VectorXd coefficient = VectorXd::Zero(totalSize);
                    VectorXd control = VectorXd::Zero(uSize);
                    control[0] = row["coe"]["vx"]; control[1] = row["coe"]["vy"];
                    coefficient.head(uSize) = control;
                    coefficient[uSize + slackSize] = row["aux_coe"][0];
                    coefficient[uSize + slackSize + 1] = row["aux_coe"][1];
                    optimiser->addLinearConstraint(coefficient, -row["const"].get<double>());
                    jsonCBFNoSlack.push_back(row);
                    hardConstraintCoefficients.push_back(control);
                }
            } else if (usesLiteratureRows()) {
                for(auto row:tvtLiteratureRows) {
                    VectorXd coefficient=VectorXd::Zero(totalSize),control=VectorXd::Zero(uSize);
                    control[0]=row["coe"]["vx"];control[1]=row["coe"]["vy"];coefficient.head(uSize)=control;
                    if(usesNormConeRows())coefficient[uSize+slackSize+row["aux_index"].get<int>()]=row["norm_coefficient"];
                    if(usesCommandBallRows())optimiser->addConcaveAuxiliaryRow(coefficient,-row["const"].get<double>(),uSize+slackSize+row["aux_index"].get<int>(),row["q_squared_coefficient"]);
                    else optimiser->addLinearConstraint(coefficient,-row["const"].get<double>());
                    jsonCBFNoSlack.push_back(row);hardConstraintCoefficients.push_back(control);
                }
                opt["literature_task"]=tvtBaselineTask;
            } else if (cbf_method == "all") {
                for (auto &[name, cbf]: cbfNoSlack.cbfs) {
                    VectorXd uCoe = cbf.constraintUCoe(f, g, x, runtime);
                    double constraintConstWithTime = cbf.constraintConstWithTime(f, g, x, runtime);
                    optimiser->addLinearConstraint(uCoe, -constraintConstWithTime);
                    jsonCBFNoSlack.emplace_back(json{
                            {"name",  cbf.name},
                            {"coe",   model->control2Json(uCoe)},
                            {"const", constraintConstWithTime},
                            {"alpha_coe", cbf.getAlphaCoefficient()},
                            {"alpha_pow", cbf.getAlphaPower()}
                    });
                    hardConstraintCoefficients.push_back(uCoe);
                }
            } else if (cbf_method == "min") {
                if (!cbfNoSlack.cbfs.empty()) {
                    VectorXd uCoe = cbfNoSlack.constraintUCoe(f, g, x, runtime);
                    double constraintConstWithTime = cbfNoSlack.constraintConstWithTime(f, g, x, runtime);

                    optimiser->addLinearConstraint(uCoe, -constraintConstWithTime);

                    jsonCBFNoSlack.emplace_back(json{
                            {"name",  cbfNoSlack.getName()},
                            {"coe",   model->control2Json(uCoe)},
                            {"const", constraintConstWithTime},
                            // MultiCBF applies its built-in identity alpha(h)=h.
                            {"alpha_coe", 1.0},
                            {"alpha_pow", 1}
                    });
                    hardConstraintCoefficients.push_back(uCoe);
                }
            }
            else {
                throw std::runtime_error("unknown cbfs.without-slack.method");
            }
            opt["cbfNoSlack"] = jsonCBFNoSlack;

            int cnt = 0;
            for (auto &[name, cbf]: cbfSlack) {
                VectorXd uCoe = cbf.constraintUCoe(f, g, x, runtime);
                Eigen::VectorXd sCoe = Eigen::VectorXd::Zero(slackSize);
                sCoe(cnt) = 1.0;
                Eigen::VectorXd coe = Eigen::VectorXd::Zero(totalSize);
                coe.head(uSize) = uCoe; coe.segment(uSize, slackSize) = sCoe;
                double constraintConst = cbf.constraintConstWithoutTime(f, g, x, runtime);

                optimiser->addLinearConstraint(coe, -constraintConst);

                jsonCBFSlack.emplace_back(json{
                        {"name",  cbf.name},
                        {"coe",   model->control2Json(uCoe)},
                        {"const", constraintConst}
                });
                ++cnt;
            }
            if(work2Active) {
                VectorXd coefficient=VectorXd::Zero(totalSize);coefficient[0]=5*work2Tangent[0];coefficient[1]=5*work2Tangent[1];
                coefficient[uSize+originalSlackSize]=1.;optimiser->addLinearConstraint(coefficient,5*work2Speed);
                jsonCBFSlack.push_back({{"name","work2TangentVelocity"},{"coe",{{"vx",5*work2Tangent[0]},{"vy",5*work2Tangent[1]},{"yawRateRad",0.}}},
                    {"const",-5*work2Speed},{"slack_index",originalSlackSize}});
            }
            opt["cbfSlack"] = jsonCBFSlack;

            auto result = optimiser->solve();

            json solverStatus;
            try {
                solverStatus = optimiser->getStatus();
            } catch (...) {
                opt["status"] = "failed";
                opt["error"] = "Status check failed";
                opt["solver_info"] = {{"status", "status-check-error"}};
                throw std::runtime_error("QP status check failed");
            }
            opt["solver_info"] = solverStatus;
            const std::string status = solverStatus.value("status", "unknown");
            if (status != "optimal") {
                opt["status"] = "failed";
                opt["error"] = solverStatus.value("error", status);
                throw std::runtime_error("QP solve failed: " + status);
            }

            if(usesCommandBallRows()) {
                const std::string folder=settings.at("tvt-experiment").at("model-export-directory");
                if(runtime==0 || !solverStatus.value("reported_ObjVal_matches_primal_at_1e_6",false)) {
                    const std::string prefix=folder+"/frame"+std::to_string(*estimatorSnapshotFrame)+"_node"+std::to_string(id);
                    optimiser->write(prefix+".lp");optimiser->write(prefix+".sol");opt["model_export_prefix"]=prefix;
                }
            }
            auto u = result.head(uSize);
            if (inputLimitsEnabled) {
                const bool vxSaturated =
                    std::abs(u[0]) >= planarComponentMax - saturationTolerance;
                const bool vySaturated =
                    std::abs(u[1]) >= planarComponentMax - saturationTolerance;
                const bool yawSaturated =
                    std::abs(u[2]) >= yawRateMax - saturationTolerance;
                opt["input_limits"]["saturated"] = {
                    {"vx", vxSaturated},
                    {"vy", vySaturated},
                    {"yawRateRad", yawSaturated},
                    {"any", vxSaturated || vySaturated || yawSaturated}
                };
            }
            if(settings.contains("cano-pcg")) {
                try {
                if(!usesSymmetricL1Rows() || uSize!=3 || slackSize!=2 || auxiliarySize!=2 || !result.allFinite())throw std::runtime_error("Cano common-filter dimensions/nonfinite rejected");
                const double zx=result[uSize+slackSize],zy=result[uSize+slackSize+1];
                if(zx < std::abs(u[0])-1e-7||zy < std::abs(u[1])-1e-7||std::min(zx,zy)<-1e-7||std::max(zx,zy)>50+1e-7||std::max(std::abs(u[0]),std::abs(u[1]))>50+1e-7||std::abs(u[2])>.35+1e-7)throw std::runtime_error("Cano common-filter input/L1 witness rejected");
                for(auto& row:opt["cbfNoSlack"]){const double residual=row["coe"]["vx"].get<double>()*u[0]+row["coe"]["vy"].get<double>()*u[1]+row["const"].get<double>()+row["aux_coe"][0].get<double>()*zx+row["aux_coe"][1].get<double>()*zy;row["residual"]=residual;if(!std::isfinite(residual)||residual < -1e-6)throw std::runtime_error("Cano physical hard row rejected");}
                for(int k=0;k<slackSize;++k){auto& row=opt["cbfSlack"][k];double residual=row["const"].get<double>()+result[uSize+k];for(int a=0;a<3;++a)residual+=row["coe"][std::vector<std::string>{"vx","vy","yawRateRad"}[a]].get<double>()*u[a];row["residual"]=residual;if(result[uSize+k]<-1e-7||residual < -1e-6)throw std::runtime_error("Cano soft/slack row rejected");}
#ifdef ENABLE_GUROBI
                auto* solver=dynamic_cast<Gurobi*>(optimiser.get());if(!solver)throw std::runtime_error("Cano declared Gurobi required");
                const double cost=(u-uNominal).squaredNorm()+10*result.segment(uSize,slackSize).sum();const double expr=solver->canoExpressionAtPrimal(),modelExpr=solver->canoModelExpressionAtPrimal();
                opt["cost_from_final_primal"]=cost;opt["objective_expression_at_primal"]=expr;opt["model_expression_at_primal"]=modelExpr;opt["ObjVal_vs_primal_diagnostic_gap"]=std::abs(cost-solver->getObjectiveValue());
                if(!std::isfinite(cost)||!std::isfinite(expr)||!std::isfinite(modelExpr)||std::abs(cost-expr)>1e-6||std::abs(cost-modelExpr)>1e-6)throw std::runtime_error("Cano primal/model expression identity rejected");
#else
                throw std::runtime_error("Cano Gurobi-only arm excluded from Gazebo adapter");
#endif
                opt["cano_certificate"]={{"status","accepted"},{"reason",nullptr},{"command_installed",false}};
                }catch(const std::exception&e){opt["status"]="failed";opt["error"]=e.what();opt["cano_certificate"]={{"status","rejected"},{"reason",e.what()},{"command_installed",false}};throw;}
            }
            if(settings.contains("cano-pcg")){opt["M3_certificate_nominal"]=model->control2Json(uNominal);opt["M3_certificate_nominal_frame"]=*canoPcgNominalFrame;}
            model->setControlInput(u);
            if(settings.contains("cano-pcg"))opt["cano_certificate"]["command_installed"]=true;
            opt["result"] = model->control2Json(u);
            opt["slacks"] = result.segment(uSize, slackSize);
            if (auxiliarySize) {
                opt["auxiliary_values"]=json::array();
                for(int index=0;index<auxiliarySize;++index)opt["auxiliary_values"].push_back(result[uSize+slackSize+index]);
            }
            for (size_t index = 0; index < hardConstraintCoefficients.size(); ++index) {
                opt["cbfNoSlack"][index]["residual"] =
                    opt["cbfNoSlack"][index]["const"].get<double>() +
                    hardConstraintCoefficients[index].dot(u);
                if (auxiliarySize) {
                    double residual=opt["cbfNoSlack"][index]["residual"].get<double>();
                    for(int column=0;column<auxiliarySize;++column)
                        residual+=opt["cbfNoSlack"][index]["aux_coe"][column].get<double>()*result[uSize+slackSize+column];
                    if(usesCommandBallRows()) {
                        const auto& row=opt["cbfNoSlack"][index];const int qIndex=row["aux_index"];const double q=result[uSize+slackSize+qIndex];
                        residual+=row["q_squared_coefficient"].get<double>()*q*q;
                        opt["cbfNoSlack"][index]["q_value"]=q;opt["cbfNoSlack"][index]["radius"]=row["E"].get<double>()+row["T"].get<double>()*q;
                        const double norm=q-std::hypot(u[0]-row["other_command"][0].get<double>(),u[1]-row["other_command"][1].get<double>());
                        opt["cbfNoSlack"][index]["norm_residual"]=norm;
                        if(!std::isfinite(residual)||residual< -1e-6||norm< -1e-7) {opt["status"]="failed";opt["error"]="command-ball numeric row certificate rejected";throw std::runtime_error("command-ball numeric row certificate rejected");}
                    }
                    opt["cbfNoSlack"][index]["residual"]=residual;
                }
            }
            if(usesCommandBallRows()) {
                if(std::abs(u[0])>planarComponentMax+1e-7||std::abs(u[1])>planarComponentMax+1e-7||std::abs(u[2])>yawRateMax+1e-7)throw std::runtime_error("command-ball input certificate rejected");
                tvtCommandBallCommandFrame=estimatorSnapshotFrame;
                opt["command_batch_frame_index"]=*estimatorSnapshotFrame;
                opt["cost_from_final_primal"]=u.squaredNorm()+settings.at("cbfs").at("objective-function").at("k_delta").get<double>()*result.segment(uSize,slackSize).sum();
                opt["objective_expression_matches_final_primal_at_1e_6"]=std::abs(opt["cost_from_final_primal"].get<double>()-solverStatus.at("objective_expression_value").get<double>())<1e-6;
            }
            opt["status"] = "success";

            if (settings["debug"]["opt-cbc"]) {
                for (auto &[name, cbf]: cbfNoSlack.cbfs) {
                    for (auto &item : opt["cbfNoSlack"]) {
                        if (item["name"] == cbf.name) {
                            item["lhs"] = cbf.hdot(f, g, x, u, runtime);
                            item["rhs"] = -cbf.alpha(cbf.h(x, runtime));
                            item["lfh"] = cbf.dhdx(x, runtime).dot(VectorXd(f)) + cbf.dhdt(x, runtime);
                            item["dhdt"] = cbf.dhdt(x, runtime);
                            item["lgh"] = cbf.constraintUCoe(f, g, x, runtime);
                            double dt = settings["execute"]["time-step"];
                            item["expected-position"] = model->xy().vec() + u * dt;
                            item["expected-h"] = cbf.hdot(f, g, x, u, runtime) * dt + cbf.h(x, runtime);
                            break;
                        }
                    }
                }
            }

            // cbfNoSlack.checkInequality(f, g, x, u, runtime);
    }

    void stepTimeForward(double dt) {
        canonical::forbid("void stepTimeForward");
        model->stepTimeForward(dt);
        runtime += dt;
    }

    // External velocity injection interface (for simulation environment)
    void setExternalVelocity(double vx, double vy) {
        externalVx = vx;
        externalVy = vy;
        hasExternalVelocity = true;
    }

    void output() const {
        std::cout << "Robot " << id << ": ";
        model->output();
    }

    void updateGridWorld() {
        canonical::forbid("void updateGridWorld");
        updatedGridWorld = json::array();
        json searchSettings = settings["searching"];
        std::string method = searchSettings["method"];
        json params = searchSettings[method];
        if (method == "front-sector") {
            for (auto &[id, position2D]: comm->_othersPos) {
                params["centerAngleRad"] = comm->_othersYawRad[id];
                if(!estimatorInLoop || !estimatorSnapshotFrame || tvtEstimatorEpsilons.size()!=static_cast<std::size_t>(n)) throw std::runtime_error("shrunk coverage requires same-frame EKF snapshot");
                const double scale=settings.at("tvt-experiment").at("ekf-epsilon-scale").get<double>();
                const double margin=scale*tvtEstimatorEpsilons.at(id);
                auto updatedFor1 = updateShrunkPointSector(gridWorld,position2D,comm->_othersYawRad.at(id),margin,params);
                updatedGridWorld.insert(updatedGridWorld.end(), updatedFor1.begin(), updatedFor1.end());
            }
        }
        else if (method == "front-cone") {
            for (auto &[id, position2D]: comm->_othersPos) {
                params["yaw-rad"] = comm->_othersYawRad[id];
                auto updatedFor1 = gridWorld.setValueInTiltedCone(
                    position2D,
                    params,
                    true, true
                );
                updatedGridWorld.insert(updatedGridWorld.end(), updatedFor1.begin(), updatedFor1.end());
            }
        }
        else if (method == "downward") {
            for (auto &[id, position2D]: comm->_othersPos) {
                auto updatedFor1 = gridWorld.setValueInCircle(position2D, params, true, true);
                updatedGridWorld.insert(updatedGridWorld.end(), updatedFor1.begin(), updatedFor1.end());
            }
        }
    }

    void getXLimit(double _x[], double inflation = 1.2) {
        world.boundary.get_x_limit(_x, inflation);
    }

    void getYLimit(double *_y, double inflation = 1.2) {
        world.boundary.get_y_limit(_y, inflation);
    }

    json getParams() {
        json paraJson;
        {
            json worldJson;
            double xLim[2], yLim[2];
            getXLimit(xLim), getYLimit(yLim);
            worldJson["lim"] = {{xLim[0], xLim[1]},
                                {yLim[0], yLim[1]}};
            getXLimit(xLim, 1.0), getYLimit(yLim, 1.0);
            for (int i = 1; i <= world.boundary.n; i++) {
                worldJson["boundary"].push_back({world.boundary.p[i].x, world.boundary.p[i].y});
            }
            worldJson["boundary"].push_back({world.boundary.p[1].x, world.boundary.p[1].y});
            worldJson["charge"]["num"] = world.chargingStations.size();
            for (auto &i: world.chargingStations) {
                worldJson["charge"]["pos"].push_back({i.first.x, i.first.y});
                worldJson["charge"]["dist"].push_back(i.second);
            }
            paraJson["world"] = worldJson;
        }
        {
            json swarmJson;
            swarmJson["num"] = settings["num"];
            paraJson["swarm"] = swarmJson;
        }
        {
            json targetsJson = json::array();
            for (auto &i: world.targets) {
                targetsJson.push_back({
                                              {"k", i.densityParams["k"]},
                                              {"r", i.densityParams["r"]}
                                      });
            }
            paraJson["targets"] = targetsJson;
        }
        {
            json gridWorldJson;
            gridWorldJson["xNum"] = gridWorld.xNum;
            gridWorldJson["yNum"] = gridWorld.yNum;
            gridWorldJson["xLim"] = {gridWorld.xLim.first, gridWorld.xLim.second};
            gridWorldJson["yLim"] = {gridWorld.yLim.first, gridWorld.yLim.second};
            std::vector<std::vector<bool>> validArray(gridWorld.yNum, std::vector<bool>(gridWorld.xNum));
            for (int y = 0; y < gridWorld.yNum; y++) {
                for (int x = 0; x < gridWorld.xNum; x++) {
                    int index = gridWorld.getIndex(x, y);
                    validArray[y][x] = gridWorld.valid[index];
                }
            }
            gridWorldJson["valid"] = validArray;
            paraJson["gridWorld"] = gridWorldJson;
        }
        return paraJson;
    }

    json getState() {
        json robotJson = {{"state", model->state2Json()},
                          {"id",    id}};
        if (!cbfNoSlack.cbfs.empty()) {
            json cbfNoSlackJson;
            for (auto &[name, cbf]: cbfNoSlack.cbfs) {
                cbfNoSlackJson[cbf.name] = cbf.h(model->getX(), runtime);
            }
            // cbfNoSlackJson[cbfNoSlack.getName()] = cbfNoSlack.h(model->getX(), runtime);
            robotJson["cbfNoSlack"] = cbfNoSlackJson;
        }
        {
            json cbfSlackJson;
            for (auto &[name, cbf]: cbfSlack) {
                cbfSlackJson[cbf.name] = cbf.h(model->getX(), runtime);
            }
            robotJson["cbfSlack"] = cbfSlackJson;
        }

        if (settings["cbfs"]["with-slack"]["cvt"]["on"] || settings["cbfs"]["with-slack"]["cvt-yaw"]["on"]) {
            json cvtJson;
            if (!tvtTargetSelection.empty()) {
                cvtJson["target_selection"] = tvtTargetSelection;
                cvtJson["pos"] = tvtTargetSelection["continuous_cell_vertices"];
                if (!cvtJson["pos"].empty()) cvtJson["pos"].push_back(cvtJson["pos"][0]);
                cvtJson["num"] = cvtJson["pos"].size();
            } else {
                cvtJson["num"] = cvt.pl[id].n + 1;
                for (int i = 1; i <= cvt.pl[id].n; i++) cvtJson["pos"].push_back({cvt.pl[id].p[i].x, cvt.pl[id].p[i].y});
                cvtJson["pos"].push_back({cvt.pl[id].p[1].x, cvt.pl[id].p[1].y});
            }
            cvtJson["center"] = {cvt.ct[id].x, cvt.ct[id].y};
            robotJson["cvt"] = cvtJson;
        }

        if (!periodicTaskTargetRecord.empty()) robotJson["periodic_task_target"] = periodicTaskTargetRecord;
        if (!tvtRowsSnapshot.empty()) robotJson["tvt_rows"] = tvtRowsSnapshot;
        robotJson["uncertainty"] = currentUncertainty;
        robotJson["uncertainty_rate"] = uncertaintyRate;
        if (enablePositionCovariance) {
            Eigen::Matrix2d cov = positionCovariance;
            json covarianceJson = {
                {"cov_xx", cov(0, 0)},
                {"cov_xy", cov(0, 1)},
                {"cov_yx", cov(1, 0)},
                {"cov_yy", cov(1, 1)}
            };
            robotJson["position_covariance"] = covarianceJson;
        }

        {
            robotJson["opt"] = opt;
        }

        // Add external velocity data if available
        if (hasExternalVelocity) {
            robotJson["external_velocity"] = {externalVx, externalVy};
        }

        return robotJson;
    }

    double getUncertaintyAtDistance(double distance) {
        return uncertaintyFunction(distance);
    }

    };


#endif //CBF_ROBOT_HPP
