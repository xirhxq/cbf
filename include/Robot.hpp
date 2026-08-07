#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "cbf/cbf"
#include "cbf/AllocatedPairwiseCBF.hpp"
#include "cbf/CBFConfig.hpp"
#include "cbf/FimRateCertificate.hpp"
#include "cbf/HardInteriorSelection.hpp"
#include "cbf/HybridCertificateGuard.hpp"
#include "world/world"
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
    double estimatorEpsilon = 0.0;
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
        settings["id"] = id;
        if (settings["model"] == "SingleIntegrate2D") {
            model = std::make_unique<SingleIntegrate2D>(settings);
        } else if (settings["model"] == "DoubleIntegrate2D") {
            model = std::make_unique<DoubleIntegrate2D>(settings);
        } else {
            throw std::invalid_argument("Invalid model type");
        }
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
        model->setStateVariable("battery", bMin + (bMax - bMin) * rand() / RAND_MAX);
        model->setYawDeg(settings["initial"]["yawDeg"]);
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

    void applyEstimatorInput(
        const Point& position, double epsilon, bool active
    ) {
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
            if (alphaParameters.power == -1) {
                // Custom tanh-saturated recovery: h>=0 linear (safe_coe), h<0
                // recovers at up to `sat` (< 5 m/s physical cap) to break the
                // steady-state comm violation equilibrium without QP infeasibility.
                const double sat = config.value("alpha", json::object())
                    .value("sat", 4.0);
                commCBF.setAlphaTanhRecovery(alphaParameters.coefficient, sat);
            } else {
                commCBF.setAlphaClassK(alphaParameters.coefficient, alphaParameters.power);
            }
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

    Point getEndTargetPoint() {
        cvt = CVT(endIds.size(), world.boundary);
        for (auto &[endId, cvtId]: endId2CvtId) {
            cvt.pt[cvtId] = comm->_othersPos[endId];
        }
        cvt.cal_poly();
        Point densityCentroid = gridWorld.getCentroidInPolygon(cvt.pl[partId]);

        double heading = comm->_othersYawRad[endInMyPart];
        Point focus = comm->_othersPos[endInMyPart] + cvtFrontFocusDistance * Point(std::cos(heading), std::sin(heading));

        return getExplorationPoint(densityCentroid, focus, cvt.pl[partId]);
    }

    Point getExplorationPoint(const Point& densityCentroid, const Point& focus, const Polygon& poly) {
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
        Point origin = bases[myBasesId[0]];
        Point sectionVector = (endExplorationPoint - origin) / numTotalSection;
        int numSection = (idsInMyPart.size() - idInMyPart + 1) / 2;
        Point explorationPoint = endExplorationPoint - sectionVector * numSection;
        double rotateAngleRad = (pi / 3) * ((partId % 2 == 1)? -1: 1);
        if ((idsInMyPart.size() - idInMyPart) % 2 == 1) {
            explorationPoint = explorationPoint + sectionVector.rotate(rotateAngleRad);
        }
        return explorationPoint;
    }

    void setCVTCBF(const json& config) {
        cvt = CVT(n, world.boundary);
        for (auto &[id, pos2d]: comm->_othersPos) {
            if (id == this->id) continue;
            cvt.pt[id] = pos2d;
        }
        cvt.pt[this->id] = model->xy();
        cvt.cal_poly();
        Point densityCentroid = gridWorld.getCentroidInPolygon(cvt.pl[this->id]);

        double heading = this->model->getStateVariable("yawRad");
        Point focus = model->xy() + cvtFrontFocusDistance * Point(std::cos(heading), std::sin(heading));

        cvt.ct[this->id] = getExplorationPoint(densityCentroid, focus, cvt.pl[this->id]);

        if (cvtExplorationMode == "cvt") {
            cvt.ct[this->id] = world.boundary.clip(cvt.ct[this->id]);
        } else {
            cvt.ct[this->id] = world.boundary.clip(getMyExplorationPoint(getEndTargetPoint()));
        }

        Point cvtCenter = cvt.ct[this->id];

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
            cbfSlack[cvtDistanceCBF.name] = cvtDistanceCBF;
        }

        if (config.contains("cvt-yaw") && config["cvt-yaw"]["on"]) {
            CBF cvtYawCBF;
            cvtYawCBF.name = config["cvt-yaw"]["name"];
            cvtYawCBF.h = [cvtCenter, config, this](VectorXd x, double t) {
                Point myPosition = this->model->extractXYFromVector(x);
                double k_yaw = config["cvt-yaw"]["k_yaw"];
                double desired_yaw = atan2(cvtCenter.y - myPosition.y, cvtCenter.x - myPosition.x);
                double current_yaw = this->model->extractFromVector(x, "yawRad");
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
            // Per-solve fresh optimiser to avoid OsqpEigen heap corruption
            // from repeated clearSolver+initSolver on a long-lived Solver.
            optimiser = createOptimiser(
                settings.at("optimiser").get<std::string>(),
                settings.at("cbfs").at("objective-function"));

            auto f = model->f();
            auto g = model->g();
            auto x = model->getX();

            int uSize = model->uSize();
            int slackSize = cbfSlack.size();
            int totalSize = uSize + slackSize;

            optimiser->start(totalSize, uSize);

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
                            {"alpha_pow", cbf.getAlphaPower()},
                            {"alpha_sat", cbf.getAlphaSat()}
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
                Eigen::VectorXd coe(totalSize);
                coe << uCoe, sCoe;
                double constraintConst = cbf.constraintConstWithoutTime(f, g, x, runtime);

                optimiser->addLinearConstraint(coe, -constraintConst);

                jsonCBFSlack.emplace_back(json{
                        {"name",  cbf.name},
                        {"coe",   model->control2Json(uCoe)},
                        {"const", constraintConst}
                });
                ++cnt;
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
            model->setControlInput(u);
            opt["result"] = model->control2Json(u);
            opt["slacks"] = result.tail(slackSize);
            for (size_t index = 0; index < hardConstraintCoefficients.size(); ++index) {
                opt["cbfNoSlack"][index]["residual"] =
                    opt["cbfNoSlack"][index]["const"].get<double>() +
                    hardConstraintCoefficients[index].dot(u);
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
        updatedGridWorld = json::array();
        json searchSettings = settings["searching"];
        std::string method = searchSettings["method"];
        json params = searchSettings[method];
        if (method == "front-sector") {
            for (auto &[id, position2D]: comm->_othersPos) {
                params["centerAngleRad"] = comm->_othersYawRad[id];
                auto updatedFor1 = gridWorld.setValueInSectorRing(
                    position2D,
                    params,
                    true, true
                );
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
            json cvtJson = {{"num", cvt.pl[id].n + 1}};
            for (int i = 1; i <= cvt.pl[id].n; i++) {
                cvtJson["pos"].push_back({cvt.pl[id].p[i].x, cvt.pl[id].p[i].y});
            }
            cvtJson["pos"].push_back({cvt.pl[id].p[1].x, cvt.pl[id].p[1].y});
            cvtJson["center"] = {cvt.ct[id].x, cvt.ct[id].y};
            robotJson["cvt"] = cvtJson;
        }

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
