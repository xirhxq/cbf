#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "cbf/cbf"
#include "world/world"
#include "models/models"
#include "optimisers/optimisers"
#include "communicators/communicators"
#include "bridge/HocbfFeasibilityGuard.hpp"
#include <array>
#include <cmath>
#include <limits>

typedef std::pair<int, Point> intPoint;

class Robot {
public:
    int id = 0;
    int n;
    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    std::unordered_map<std::string, SecondOrderCBF> secondOrderCbfNoSlack;
    std::unordered_map<std::string, SecondOrderCBF> secondOrderCbfSlack;
    bool hocbfFeasibilitySlackEnabled = false;
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

    std::string searchMethod;
    json searchParams;

    std::vector<Point> bases;
    std::vector<int> myBasesId;

    std::set<int> myNeighboursId;
    bool hasBridgeFormationOverride = false;
    std::vector<int> bridgeAnchorIds;
    std::vector<int> bridgeBaseIds;

    // External velocity data (for simulation environment)
    double externalVx = 0.0;
    double externalVy = 0.0;
    bool hasExternalVelocity = false;
    Eigen::VectorXd nominalControlOverride;
    bool hasNominalControlOverride = false;
    json nominalGuardDiagnostic = json::object();
    bool hasNominalGuardDiagnostic = false;

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

        if (settings.contains("searching")) {
            const json &searchSettings = settings["searching"];
            searchMethod = searchSettings.value("method", "");
            if (!searchMethod.empty() && searchSettings.contains(searchMethod)) {
                searchParams = searchSettings[searchMethod];
            }
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

        if (settings["initial"].contains("velocity") &&
            settings["initial"]["velocity"].contains("values")) {
            auto velocities = settings["initial"]["velocity"]["values"];
            if (id - 1 < velocities.size()) {
                model->setStateVariable("vx", velocities[id - 1][0]);
                model->setStateVariable("vy", velocities[id - 1][1]);
            }
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
        double alpha_coe = config.value("alpha/coe", 0.1);
        int alpha_pow = config.value("alpha/pow", 1);
        energyCBF.setAlphaClassK(alpha_coe, alpha_pow);

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

    void getCovariance(json &config) {
        if (!enablePositionCovariance) return;

        myCovarianceFormation = {
            {"id", id},
            {"anchorIds", json::array()},
            {"baseIds", json::array()}
        };

        Point p = this->model->xy();
        std::vector<Point> anchorPoints;
        std::vector<Eigen::Matrix2d> anchorCovariances;

        double maxRange = config["max-range"];

        for (int i = 0; i < bases.size(); i++) {
            Point& base = bases[i];
            if (base.distance_to(p) > maxRange) continue;

            anchorPoints.push_back(base);
            anchorCovariances.push_back(Eigen::Matrix2d::Zero());
            myCovarianceFormation["baseIds"].push_back(i);
        }

        for (auto &[otherId, otherPos] : comm->_othersPos) {
            if (myNeighboursId.find(otherId) == myNeighboursId.end() && otherPos.distance_to(p) > maxRange) continue;
            if (!isSamePartAsMe(otherId)) continue;
            if (getIdInPart(otherId) >= getIdInPart(id)) continue;

            anchorPoints.push_back(otherPos);
            anchorCovariances.push_back(comm->_othersPositionCovariance[otherId]);
            myCovarianceFormation["anchorIds"].push_back(otherId);
        }

        std::vector<double> angles;
        std::vector<double> total_variances;

        for (int i = 0; i < anchorPoints.size(); ++i) {
            double angle = anchorPoints[i].angle_to(p);
            double distance = p.distance_to(anchorPoints[i]);
            double ranging_unc = rangingUncertaintyFunction(distance);

            // ρ² = cos²θ·cov(0,0) + sin²θ·cov(1,1) + 2cosθ·sinθ·cov(0,1) + σ²_ranging
            Eigen::Matrix2d anchor_cov = anchorCovariances[i];
            double position_var = (std::cos(angle) * std::cos(angle) * anchor_cov(0, 0) +
                                  std::sin(angle) * std::sin(angle) * anchor_cov(1, 1) +
                                  2.0 * std::cos(angle) * std::sin(angle) * anchor_cov(0, 1));

            double total_variance = position_var + ranging_unc * ranging_unc;

            angles.push_back(angle);
            total_variances.push_back(total_variance);
        }

        int n_anchors = angles.size();
        if (n_anchors < 2) {
            throw std::invalid_argument(
                "#" + std::to_string(this->id) +
                " has less than two anchor points for covariance calculation"
            );
        }

        Eigen::MatrixXd J(n_anchors, 2);
        Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(n_anchors, n_anchors);

        for (int i = 0; i < n_anchors; ++i) {
            J(i, 0) = std::cos(angles[i]);
            J(i, 1) = std::sin(angles[i]);
            Sigma(i, i) = total_variances[i];
        }

        // Φ = J^T * Σ^(-1) * J (Fisher Information Matrix)
        Eigen::Matrix2d Phi;
        try {
            Phi = J.transpose() * Sigma.inverse() * J;
            positionCovariance = Phi.inverse();
        } catch (...) {
            throw std::invalid_argument("Covariance calculation failed");
        }
    }

    void setupFormation() {
        // Setup formation information for constraint violation detection
        // This should always be called, regardless of whether comm-fixed CBF is enabled
        // Uses the already-initialized myNeighboursId and myBasesId

        auto commConfig = settings["cbfs"]["without-slack"]["comm-fixed"];
        int minOffset = commConfig.value("min-neighbour-id-offset", -2);

        myFormation = {
            {"id", id},
            {"anchorPoints", json::array()},
            {"anchorIds", json::array()},
            {"baseIds", json::array()}
        };

        if (hasBridgeFormationOverride) {
            for (int baseId : bridgeBaseIds) {
                if (baseId >= 0 && baseId < bases.size()) {
                    auto base = bases[baseId];
                    myFormation["anchorPoints"].push_back({base.x, base.y});
                    myFormation["baseIds"].push_back(baseId);
                }
            }
            for (int anchorId : bridgeAnchorIds) {
                if (anchorId != id) {
                    myFormation["anchorIds"].push_back(anchorId);
                }
            }
            return;
        }

        // Add base stations (from already-initialized myBasesId)
        for (int i = 0; i < myBasesId.size(); i++) {
            int baseId = myBasesId[i];
            if (i > -idInMyPart - minOffset) continue;
            auto base = bases[baseId];
            myFormation["anchorPoints"].push_back({base.x, base.y});
            myFormation["baseIds"].push_back(baseId);
        }

        // Add robot neighbours (from already-initialized myNeighboursId)
        for (auto &otherId: myNeighboursId) {
            myFormation["anchorIds"].push_back(otherId);
        }
    }

    void setFixedCommCBF(json& config) {
        // This function only creates the CBF constraints
        // Formation information is already set by setupFormation()
        double maxRange = config["max-range"];
        double rangeTighteningMargin = config.value("range-tightening-margin", 0.0);
        if (!std::isfinite(rangeTighteningMargin) || rangeTighteningMargin < 0.0) {
            throw std::invalid_argument("comm-fixed.range-tightening-margin must be non-negative and finite");
        }
        if (rangeTighteningMargin >= maxRange) {
            throw std::invalid_argument("comm-fixed.range-tightening-margin must be smaller than max-range");
        }
        double effectiveMaxRange = maxRange - rangeTighteningMargin;

        std::vector<Point> formationPoints;
        std::vector<Point> formationVels;
        std::vector<std::string> anchorCBFNames;
        std::vector<double> formationUncertainties;

        for (auto &baseId : myFormation["baseIds"]) {
            int id = baseId.get<int>();
            auto base = bases[id];
            formationPoints.push_back(base);
            formationVels.emplace_back(0, 0);
            anchorCBFNames.emplace_back("base-" + std::to_string(id));
            formationUncertainties.push_back(0);
        }

        for (auto &anchorId : myFormation["anchorIds"]) {
            int otherId = anchorId.get<int>();
            formationPoints.push_back(comm->_othersPos[otherId]);
            formationVels.push_back(comm->_othersVel[otherId]);
            anchorCBFNames.push_back("#" + std::to_string(otherId));
            formationUncertainties.push_back(uncertaintyFromCovarianceFunction(comm->_othersPositionCovariance[otherId]));
        }

        getCovariance(config);
        double myUncertainty = uncertaintyFromCovarianceFunction(positionCovariance);

        for (int i = 0; i < formationPoints.size(); i++) {
            auto otherPoint = formationPoints[i];
            auto otherVel = config["compensate-velocity"] ? formationVels[i] : Point(0, 0);
            double k = config["k"];
            bool isAnchor = anchorCBFNames[i].find("base-") != std::string::npos;
            auto otherUncertainty = formationUncertainties[i];
            bool considerUncertainty = config.value("consider-uncertainty", true);

            auto fixedFormationCommH = [this, otherPoint, otherVel, effectiveMaxRange, k, isAnchor, myUncertainty, otherUncertainty, considerUncertainty](VectorXd x, double t) {
                Point myPosition = model->extractXYFromVector(x);
                double distance = myPosition.distance_to(otherPoint);
                double h = k * (effectiveMaxRange - distance);

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
                Point myPosition = model->extractXYFromVector(x);
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
            auto dhdt = [this, otherPoint, otherVel, k](VectorXd x, double t) -> double {
                Point myPosition = model->extractXYFromVector(x);
                Point diff = myPosition - otherPoint;
                double distance = diff.len();
                if (distance < 1e-8) {
                    return 0.0;
                }
                double dotProduct = diff * otherVel;
                return k * dotProduct / distance;
            };

            CBF commCBF;
            commCBF.name = "fixedCommCBF(" + anchorCBFNames[i] + ")";
            commCBF.h = fixedFormationCommH;
            commCBF.dhdx_analytical = dhdx;
            commCBF.dhdt_analytical = dhdt;
            double alpha_coe = config.value("alpha/coe", 0.1);
            int alpha_pow = config.value("alpha/pow", 1);
            commCBF.setAlphaClassK(alpha_coe, alpha_pow);
            cbfNoSlack.cbfs[commCBF.name] = commCBF;
        }
    }

    void setSecondOrderFixedCommCBF(json& config) {
        double maxRange = config["max-range"];
        double rangeTighteningMargin = config.value("range-tightening-margin", 0.0);
        if (!std::isfinite(rangeTighteningMargin) || rangeTighteningMargin < 0.0) {
            throw std::invalid_argument("comm-fixed.range-tightening-margin must be non-negative and finite");
        }
        if (rangeTighteningMargin >= maxRange) {
            throw std::invalid_argument("comm-fixed.range-tightening-margin must be smaller than max-range");
        }
        double effectiveMaxRange = maxRange - rangeTighteningMargin;
        double k = config.contains("k") ? static_cast<double>(config["k"]) : 1.0;
        bool considerUncertainty = config.value("consider-uncertainty", true);
        double lambda1 = secondOrderLambda1();
        double lambda2 = secondOrderLambda2();
        double sampledDataReserve = secondOrderSampledDataReserve();
        bool useStateDependentReserve = false;
        double stateReserveVelocityGain = 0.0;
        double stateReserveSampleTime = settings.value("execute", json::object()).value("time-step", 0.0);
        double stateReserveAccelerationGain = 0.0;
        double stateReserveNeighborAccelerationBound = 0.0;
        double stateReserveMax = std::numeric_limits<double>::infinity();
        if (config.contains("state-dependent-reserve")) {
            const json& stateReserveConfig = config["state-dependent-reserve"];
            useStateDependentReserve = stateReserveConfig.value("enabled", false);
            stateReserveVelocityGain = stateReserveConfig.value("velocity-gain", 0.0);
            stateReserveSampleTime = stateReserveConfig.value("sample-time", stateReserveSampleTime);
            stateReserveAccelerationGain = stateReserveConfig.value("acceleration-gain", 0.0);
            stateReserveNeighborAccelerationBound = stateReserveConfig.value("neighbor-acceleration-bound", 0.0);
            stateReserveMax = stateReserveConfig.value("max-reserve", stateReserveMax);
            const std::array<std::pair<const char*, double>, 5> stateReserveValues = {{
                {"comm-fixed.state-dependent-reserve.velocity-gain", stateReserveVelocityGain},
                {"comm-fixed.state-dependent-reserve.sample-time", stateReserveSampleTime},
                {"comm-fixed.state-dependent-reserve.acceleration-gain", stateReserveAccelerationGain},
                {"comm-fixed.state-dependent-reserve.neighbor-acceleration-bound", stateReserveNeighborAccelerationBound},
                {"comm-fixed.state-dependent-reserve.max-reserve", stateReserveMax}
            }};
            for (const auto& [name, value] : stateReserveValues) {
                if (!std::isfinite(value) || value < 0.0) {
                    throw std::invalid_argument(std::string(name) + " must be non-negative and finite");
                }
            }
        }

        std::vector<Point> formationPoints;
        std::vector<VectorXd> formationVels;
        std::vector<VectorXd> formationAccs;
        std::vector<std::string> anchorCBFNames;
        std::vector<double> formationUncertainties;

        for (auto &baseId : myFormation["baseIds"]) {
            int id = baseId.get<int>();
            auto base = bases[id];
            formationPoints.push_back(base);
            formationVels.push_back(VectorXd::Zero(2));
            formationAccs.push_back(VectorXd::Zero(2));
            anchorCBFNames.emplace_back("base-" + std::to_string(id));
            formationUncertainties.push_back(0.0);
        }

        for (auto &anchorId : myFormation["anchorIds"]) {
            int otherId = anchorId.get<int>();
            formationPoints.push_back(comm->_othersPos[otherId]);
            formationVels.push_back(
                    comm->_othersVel.find(otherId) == comm->_othersVel.end()
                    ? VectorXd::Zero(2)
                    : comm->_othersVel[otherId]);
            formationAccs.push_back(
                    comm->_othersAcc.find(otherId) == comm->_othersAcc.end()
                    ? VectorXd::Zero(2)
                    : comm->_othersAcc[otherId]);
            anchorCBFNames.push_back("#" + std::to_string(otherId));
            formationUncertainties.push_back(uncertaintyFromCovarianceFunction(comm->_othersPositionCovariance[otherId]));
        }

        getCovariance(config);
        double myUncertainty = uncertaintyFromCovarianceFunction(positionCovariance);

        for (int i = 0; i < formationPoints.size(); i++) {
            Point otherPoint = formationPoints[i];
            VectorXd otherVelocity = config.value("compensate-velocity", true)
                                     ? formationVels[i]
                                     : VectorXd::Zero(2);
            VectorXd otherAcceleration = formationAccs[i];
            bool isAnchor = anchorCBFNames[i].find("base-") != std::string::npos;
            double otherUncertainty = formationUncertainties[i];
            double uncertainty = 0.0;
            if (considerUncertainty) {
                uncertainty = isAnchor ? myUncertainty : myUncertainty + otherUncertainty;
            }

            SecondOrderCBF commCBF;
            commCBF.name = "secondOrderFixedCommCBF(" + anchorCBFNames[i] + ")";
            commCBF.lambda1 = lambda1;
            commCBF.k1 = lambda1 + lambda2;
            commCBF.k0 = lambda1 * lambda2;
            commCBF.sampledDataReserve = sampledDataReserve;
            auto evaluateSharedRow = [
                this,
                otherPoint,
                otherVelocity,
                otherAcceleration,
                effectiveMaxRange,
                k,
                uncertainty,
                lambda1,
                lambda2
            ](const VectorXd &x) {
                PairwiseSecondOrderState2D selfState{
                    model->extractXYFromVector(x),
                    extractPlanarVelocityFromState(x),
                    Eigen::Vector2d::Zero()};
                PairwiseSecondOrderState2D referenceState{
                    otherPoint,
                    Eigen::Vector2d(otherVelocity(0), otherVelocity(1)),
                    Eigen::Vector2d(otherAcceleration(0), otherAcceleration(1))};
                return buildPairwiseSecondOrderRow(
                    selfState,
                    referenceState,
                    {PairwiseSecondOrderBarrierKind::CommunicationUpper,
                     effectiveMaxRange,
                     uncertainty,
                     k,
                     lambda1,
                     lambda2,
                     0.0});
            };
            commCBF.h = [evaluateSharedRow](const VectorXd &x, double) {
                return evaluateSharedRow(x).h;
            };
            commCBF.hdot = [evaluateSharedRow](const VectorXd &x, double) {
                return evaluateSharedRow(x).hdot;
            };
            commCBF.hddotConst = [evaluateSharedRow](const VectorXd &x, double) {
                return evaluateSharedRow(x).hddotConst;
            };
            commCBF.uCoe = [this, evaluateSharedRow](const VectorXd &x, double) {
                const auto row = evaluateSharedRow(x);
                VectorXd coe = VectorXd::Zero(model->uSize());
                coe(0) = row.uCoe(0);
                coe(1) = row.uCoe(1);
                return coe;
            };
            if (useStateDependentReserve) {
                commCBF.stateDependentReserve = [
                    this,
                    otherVelocity,
                    stateReserveVelocityGain,
                    stateReserveSampleTime,
                    stateReserveAccelerationGain,
                    stateReserveNeighborAccelerationBound,
                    stateReserveMax
                ](const VectorXd &x, double) {
                    VectorXd myVelocity = extractPlanarVelocityFromState(x);
                    double reserve = stateReserveVelocityGain * (myVelocity - otherVelocity).norm() * stateReserveSampleTime
                                     + 0.5 * stateReserveAccelerationGain * stateReserveNeighborAccelerationBound
                                     * stateReserveSampleTime * stateReserveSampleTime;
                    return std::min(reserve, stateReserveMax);
                };
            }

            secondOrderCbfNoSlack[commCBF.name] = commCBF;
        }
    }

    void setSafetyCBF(const json& config) {
        if (settings["num"] == 1) return;

        double myUncertainty = uncertaintyFromCovarianceFunction(positionCovariance);
        bool considerUncertainty = config.value("consider-uncertainty", true);
        double safeDistance = config["safe-distance"];
        double tighteningMargin = config.value("safe-distance-tightening-margin", 0.0);
        if (!std::isfinite(tighteningMargin) || tighteningMargin < 0.0) {
            throw std::invalid_argument("safety.safe-distance-tightening-margin must be non-negative and finite");
        }
        double effectiveSafeDistance = safeDistance + tighteningMargin;

        auto safetyH = [&, config, myUncertainty, considerUncertainty, effectiveSafeDistance](VectorXd x, double t) {
            Point myPosition = model->extractXYFromVector(x);

            double k = config.contains("k") ? static_cast<double>(config["k"]) : 1.0;

            double h = inf;
            for (auto &[id, pos2d]: comm->_othersPos) {
                if (this->id == id) continue;

                double otherUncertainty = 0.0;
                if (enablePositionCovariance && comm->_othersPositionCovariance.find(id) != comm->_othersPositionCovariance.end()) {
                    const Eigen::Matrix2d& otherCov = comm->_othersPositionCovariance[id];
                    otherUncertainty = uncertaintyFromCovarianceFunction(otherCov);
                }

                double distance = myPosition.distance_to(pos2d);
                double robust_h = k * (distance - effectiveSafeDistance - (considerUncertainty ? myUncertainty + otherUncertainty : 0));

                h = std::min(h, robust_h);
            }
            return h;
        };

        CBF safetyCBF;
        safetyCBF.name = "safetyCBF";
        safetyCBF.h = safetyH;
        cbfNoSlack.cbfs[safetyCBF.name] = safetyCBF;
    }

    bool isSecondOrderCbfEnabled() const {
        if (settings.value("model", "") != "DoubleIntegrate2D") {
            return false;
        }
        if (!settings.contains("cbfs") || !settings["cbfs"].contains("high-order")) {
            return false;
        }
        return settings["cbfs"]["high-order"].value("enabled", false);
    }

    double secondOrderLambda1() const {
        return settings["cbfs"]["high-order"].value("lambda1", 1.0);
    }

    double secondOrderLambda2() const {
        return settings["cbfs"]["high-order"].value("lambda2", 1.0);
    }

    double secondOrderSampledDataReserve() const {
        double reserve = settings["cbfs"]["high-order"].value("sampled-data-reserve", 0.0);
        if (!std::isfinite(reserve) || reserve < 0.0) {
            throw std::invalid_argument("cbfs.high-order.sampled-data-reserve must be a non-negative finite number");
        }
        return reserve;
    }

    bool hasSecondOrderAccelerationBound() const {
        return isSecondOrderCbfEnabled()
               && settings["cbfs"]["high-order"].contains("acceleration-bound");
    }

    double secondOrderAccelerationBound() const {
        double bound = settings["cbfs"]["high-order"].value("acceleration-bound", 0.0);
        if (!std::isfinite(bound) || bound < 0.0) {
            throw std::invalid_argument("cbfs.high-order.acceleration-bound must be a non-negative finite number");
        }
        return bound;
    }

    bool isHocbfFeasibilitySlackEnabled() const {
        if (!isSecondOrderCbfEnabled()) {
            return false;
        }
        const json& highOrder = settings["cbfs"]["high-order"];
        if (!highOrder.contains("feasibility-slack")) {
            return false;
        }
        return highOrder["feasibility-slack"].value("enabled", false);
    }

    void addSecondOrderAccelerationBounds(json &jsonControlBounds) {
        if (!hasSecondOrderAccelerationBound()) {
            return;
        }

        double bound = secondOrderAccelerationBound();
        const std::array<std::string, 2> names = {"ax", "ay"};
        for (int axis = 0; axis < 2 && axis < model->uSize(); ++axis) {
            VectorXd lower = VectorXd::Zero(model->uSize());
            lower(axis) = 1.0;
            optimiser->addLinearConstraint(lower, -bound);
            jsonControlBounds.emplace_back(json{
                    {"name", names[axis] + "-lower"},
                    {"coe", model->control2Json(lower)},
                    {"rhs", -bound}
            });

            VectorXd upper = VectorXd::Zero(model->uSize());
            upper(axis) = -1.0;
            optimiser->addLinearConstraint(upper, -bound);
            jsonControlBounds.emplace_back(json{
                    {"name", names[axis] + "-upper"},
                    {"coe", model->control2Json(upper)},
                    {"rhs", -bound}
            });
        }
    }

    VectorXd extractPlanarVelocityFromState(const VectorXd &x) const {
        VectorXd velocity(2);
        velocity << model->extractFromVector(x, "vx"), model->extractFromVector(x, "vy");
        return velocity;
    }

    void setSecondOrderSafetyCBF(const json& config) {
        if (settings["num"] == 1) return;

        double safeDistance = config["safe-distance"];
        double tighteningMargin = config.value("safe-distance-tightening-margin", 0.0);
        if (!std::isfinite(tighteningMargin) || tighteningMargin < 0.0) {
            throw std::invalid_argument("safety.safe-distance-tightening-margin must be non-negative and finite");
        }
        double effectiveSafeDistance = safeDistance + tighteningMargin;
        double k = config.contains("k") ? static_cast<double>(config["k"]) : 1.0;
        bool considerUncertainty = config.value("consider-uncertainty", true);
        double myUncertainty = uncertaintyFromCovarianceFunction(positionCovariance);
        double lambda1 = secondOrderLambda1();
        double lambda2 = secondOrderLambda2();
        double sampledDataReserve = secondOrderSampledDataReserve();

        bool usePairStateReserve = false;
        double pairStateReserveDistance = 0.0;
        double pairStateReserveRadial = 0.0;
        double pairStateReserveVelocityGain = 0.0;
        double pairStateReserveSampleTime = settings.value("execute", json::object()).value("time-step", 0.0);
        double pairStateReserveMax = std::numeric_limits<double>::infinity();
        if (config.contains("pair-state-reserve")) {
            const json& pairReserveConfig = config["pair-state-reserve"];
            usePairStateReserve = pairReserveConfig.value("enabled", false);
            pairStateReserveDistance = pairReserveConfig.value("distance-threshold", 0.0);
            pairStateReserveRadial = pairReserveConfig.value("radial-threshold", 0.0);
            pairStateReserveVelocityGain = pairReserveConfig.value("velocity-gain", 0.0);
            pairStateReserveSampleTime = pairReserveConfig.value("sample-time", pairStateReserveSampleTime);
            pairStateReserveMax = pairReserveConfig.value("max-reserve", pairStateReserveMax);
            const std::array<std::pair<const char*, double>, 5> pairReserveValues = {{
                {"safety.pair-state-reserve.distance-threshold", pairStateReserveDistance},
                {"safety.pair-state-reserve.radial-threshold", pairStateReserveRadial},
                {"safety.pair-state-reserve.velocity-gain", pairStateReserveVelocityGain},
                {"safety.pair-state-reserve.sample-time", pairStateReserveSampleTime},
                {"safety.pair-state-reserve.max-reserve", pairStateReserveMax}
            }};
            for (const auto& [name, value] : pairReserveValues) {
                if (!std::isfinite(value) || value < 0.0) {
                    throw std::invalid_argument(std::string(name) + " must be non-negative and finite");
                }
            }
        }

        for (auto &[otherId, otherPoint]: comm->_othersPos) {
            if (this->id == otherId) continue;
            Point otherPosition = otherPoint;

            double otherUncertainty = 0.0;
            if (enablePositionCovariance && comm->_othersPositionCovariance.find(otherId) != comm->_othersPositionCovariance.end()) {
                const Eigen::Matrix2d& otherCov = comm->_othersPositionCovariance[otherId];
                otherUncertainty = uncertaintyFromCovarianceFunction(otherCov);
            }

            VectorXd otherVelocity = VectorXd::Zero(2);
            if (comm->_othersVel.find(otherId) != comm->_othersVel.end()) {
                otherVelocity = comm->_othersVel[otherId];
            }

            VectorXd otherAcceleration = VectorXd::Zero(2);
            if (comm->_othersAcc.find(otherId) != comm->_othersAcc.end()) {
                otherAcceleration = comm->_othersAcc[otherId];
            }

            double uncertainty = considerUncertainty ? myUncertainty + otherUncertainty : 0.0;

            SecondOrderCBF safetyCBF;
            safetyCBF.name = "secondOrderSafetyCBF(#" + std::to_string(otherId) + ")";
            safetyCBF.lambda1 = lambda1;
            safetyCBF.k1 = lambda1 + lambda2;
            safetyCBF.k0 = lambda1 * lambda2;
            safetyCBF.sampledDataReserve = sampledDataReserve;
            auto evaluateSharedRow = [
                this,
                otherPosition,
                otherVelocity,
                otherAcceleration,
                effectiveSafeDistance,
                k,
                uncertainty,
                lambda1,
                lambda2
            ](const VectorXd &x) {
                PairwiseSecondOrderState2D selfState{
                    model->extractXYFromVector(x),
                    extractPlanarVelocityFromState(x),
                    Eigen::Vector2d::Zero()};
                PairwiseSecondOrderState2D referenceState{
                    otherPosition,
                    Eigen::Vector2d(otherVelocity(0), otherVelocity(1)),
                    Eigen::Vector2d(otherAcceleration(0), otherAcceleration(1))};
                return buildPairwiseSecondOrderRow(
                    selfState,
                    referenceState,
                    {PairwiseSecondOrderBarrierKind::CollisionLower,
                     effectiveSafeDistance,
                     uncertainty,
                     k,
                     lambda1,
                     lambda2,
                     0.0});
            };
            safetyCBF.h = [evaluateSharedRow](const VectorXd &x, double) {
                return evaluateSharedRow(x).h;
            };
            safetyCBF.hdot = [evaluateSharedRow](const VectorXd &x, double) {
                return evaluateSharedRow(x).hdot;
            };
            safetyCBF.hddotConst = [evaluateSharedRow](const VectorXd &x, double) {
                return evaluateSharedRow(x).hddotConst;
            };
            safetyCBF.uCoe = [this, evaluateSharedRow](const VectorXd &x, double) {
                const auto row = evaluateSharedRow(x);
                VectorXd coe = VectorXd::Zero(model->uSize());
                coe(0) = row.uCoe(0);
                coe(1) = row.uCoe(1);
                return coe;
            };
            if (usePairStateReserve) {
                safetyCBF.stateDependentReserve = [
                    this,
                    otherPosition,
                    otherVelocity,
                    k,
                    pairStateReserveDistance,
                    pairStateReserveRadial,
                    pairStateReserveVelocityGain,
                    pairStateReserveSampleTime,
                    pairStateReserveMax
                ](const VectorXd &x, double) {
                    Point myPosition = model->extractXYFromVector(x);
                    VectorXd myVelocity = extractPlanarVelocityFromState(x);
                    auto terms = computePairwiseDistanceKinematics(myPosition, otherPosition, myVelocity, otherVelocity);
                    double radial = terms.radialVelocity;
                    if (!(terms.distance < pairStateReserveDistance && radial < -pairStateReserveRadial)) {
                        return 0.0;
                    }
                    double closingRate = std::max(0.0, -radial);
                    double reserve = pairStateReserveVelocityGain * closingRate * pairStateReserveSampleTime;
                    if (reserve > pairStateReserveMax) {
                        reserve = pairStateReserveMax;
                    }
                    return reserve;
                };
            }

            secondOrderCbfNoSlack[safetyCBF.name] = safetyCBF;
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

    Eigen::Vector2d coverageNominalAcceleration(
            double positionGain,
            double velocityGain,
            double accelerationHalfBox) {
        return coverageNominalAcceleration(
            model->xy(),model->getVelocity().head<2>(),positionGain,
            velocityGain,accelerationHalfBox);
    }

    Eigen::Vector2d coverageNominalAcceleration(
            const Point& estimatedPosition,
            const Eigen::Vector2d& estimatedVelocity,
            double positionGain,
            double velocityGain,
            double accelerationHalfBox) {
        if (!std::isfinite(positionGain) || positionGain < 0.0 ||
            !std::isfinite(velocityGain) || velocityGain < 0.0 ||
            !std::isfinite(accelerationHalfBox) || accelerationHalfBox <= 0.0 ||
            !std::isfinite(estimatedPosition.x) ||
            !std::isfinite(estimatedPosition.y) ||
            !estimatedVelocity.allFinite()) {
            throw std::invalid_argument("invalid coverage nominal gains");
        }
        const Point target = gridWorld.getNearestUnexploredPointInPolygon(
            world.boundary, estimatedPosition, estimatedPosition);
        Eigen::Vector2d nominal =
            positionGain * (target - estimatedPosition).vec() -
            velocityGain * estimatedVelocity;
        nominal.x() = std::max(-accelerationHalfBox,
                              std::min(accelerationHalfBox, nominal.x()));
        nominal.y() = std::max(-accelerationHalfBox,
                              std::min(accelerationHalfBox, nominal.y()));
        return nominal;
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
            double cvt_alpha_coe = config["cvt"].value("alpha/coe", 1.0);
            int cvt_alpha_pow = config["cvt"].value("alpha/pow", 1);
            cvtDistanceCBF.setAlphaClassK(cvt_alpha_coe, cvt_alpha_pow);
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
            double cvt_yaw_alpha_coe = config["cvt-yaw"].value("alpha/coe", 1.0);
            int cvt_yaw_alpha_pow = config["cvt-yaw"].value("alpha/pow", 1);
            cvtYawCBF.setAlphaClassK(cvt_yaw_alpha_coe, cvt_yaw_alpha_pow);
            cbfSlack[cvtYawCBF.name] = cvtYawCBF;
        }
    }

    void setBridgeFormationOverride(const std::vector<int> &anchorIds, const std::vector<int> &baseIds) {
        bridgeAnchorIds = anchorIds;
        bridgeBaseIds = baseIds;
        hasBridgeFormationOverride = true;
    }

    void clearBridgeFormationOverride() {
        bridgeAnchorIds.clear();
        bridgeBaseIds.clear();
        hasBridgeFormationOverride = false;
    }

    void clearPerStepNoSlackCbfs() {
        for (auto it = cbfNoSlack.cbfs.begin(); it != cbfNoSlack.cbfs.end();) {
            const std::string &name = it->first;
            bool isPerStepCbf = name == "commCBF"
                                || name == "safetyCBF"
                                || name.find("fixedCommCBF(") == 0;
            if (isPerStepCbf) {
                it = cbfNoSlack.cbfs.erase(it);
            } else {
                ++it;
            }
        }
    }

    void postsetCBF() {
        auto cbfConfig = settings["cbfs"];
        bool useSecondOrder = isSecondOrderCbfEnabled();

        setupFormation();
        clearPerStepNoSlackCbfs();
        secondOrderCbfNoSlack.clear();

        if (cbfConfig["without-slack"]["comm-fixed"]["on"]) {
            if (useSecondOrder) {
                setSecondOrderFixedCommCBF(cbfConfig["without-slack"]["comm-fixed"]);
            } else {
                setFixedCommCBF(cbfConfig["without-slack"]["comm-fixed"]);
            }
        }
        if (cbfConfig["without-slack"]["comm-auto"]["on"]) setCommunicationAutoCBF(cbfConfig["without-slack"]["comm-auto"]);
        if (cbfConfig["with-slack"]["cvt"]["on"] || cbfConfig["with-slack"]["cvt-yaw"]["on"]) setCVTCBF(cbfConfig["with-slack"]);
        if (cbfConfig["without-slack"]["safety"]["on"]) {
            if (useSecondOrder) {
                setSecondOrderSafetyCBF(cbfConfig["without-slack"]["safety"]);
            } else {
                setSafetyCBF(cbfConfig["without-slack"]["safety"]);
            }
        }

        hocbfFeasibilitySlackEnabled = isHocbfFeasibilitySlackEnabled();
        if (hocbfFeasibilitySlackEnabled) {
            for (auto& [name, cbf] : secondOrderCbfNoSlack) {
                secondOrderCbfSlack[name] = std::move(cbf);
            }
            secondOrderCbfNoSlack.clear();
        } else {
            secondOrderCbfSlack.clear();
        }
    }

    void optimise() {
        VectorXd uNominal(model->uSize());
        if (hasNominalControlOverride) {
            uNominal = nominalControlOverride;
        } else {
            uNominal.setZero();
        }
        opt = {
                {"nominal",    model->control2Json(uNominal)},
                {"result",     model->control2Json(uNominal)},
                {"cbfNoSlack", json::array()},
                {"cbfSlack",   json::array()},
                {"controlBounds", json::array()},
                {"nominalGuard", hasNominalGuardDiagnostic ? nominalGuardDiagnostic : json{{"enabled", false}}}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array(), jsonSecondOrderCBFNoSlack = json::array();
        json jsonControlBounds = json::array();
        std::unordered_map<std::string, CBFConstraintEvaluation> cbfNoSlackEvaluations;
        double chargeRate = 1.0;
        if (world.isCharging(model->xy(), chargeRate) && model->getStateVariable("battery") < model->BATTERY_MAX) {
            model->startCharge();
            model->setChargeRate(chargeRate);
            uNominal.setZero();
            model->setControlInput(uNominal);
        } else {
            optimiser->clear();

            auto f = model->f();
            auto g = model->g();
            auto x = model->getX();

            int uSize = model->uSize();
            int cvtSlackSize = cbfSlack.size();
            int hocbfSlackSize = secondOrderCbfSlack.size();
            int slackSize = cvtSlackSize + hocbfSlackSize;
            int totalSize = uSize + slackSize;

            optimiser->start(totalSize, uSize);

            optimiser->setObjective(uNominal);
            addSecondOrderAccelerationBounds(jsonControlBounds);
            opt["controlBounds"] = jsonControlBounds;

            std::string cbf_method = settings["cbfs"]["without-slack"].value("method", "all");

            if (cbf_method == "all") {
                for (auto &[name, cbf]: cbfNoSlack.cbfs) {
                    auto evaluation = cbf.evaluateConstraint(f, g, x, runtime);
                    cbfNoSlackEvaluations[name] = evaluation;
                    optimiser->addLinearConstraint(evaluation.uCoe, -evaluation.constWithTime);
                    jsonCBFNoSlack.emplace_back(json{
                            {"name",  cbf.name},
                            {"coe",   model->control2Json(evaluation.uCoe)},
                            {"const", evaluation.constWithTime}
                    });
                }
            } else if (cbf_method == "min") {
                if (!cbfNoSlack.cbfs.empty()) {
                    auto evaluation = cbfNoSlack.evaluateConstraint(f, g, x, runtime);

                    optimiser->addLinearConstraint(evaluation.uCoe, -evaluation.constWithTime);

                    jsonCBFNoSlack.emplace_back(json{
                            {"name",  cbfNoSlack.getName()},
                            {"coe",   model->control2Json(evaluation.uCoe)},
                            {"const", evaluation.constWithTime}
                    });
                }
            }
            else {
                throw std::runtime_error("unknown cbfs.without-slack.method");
            }
            opt["cbfNoSlack"] = jsonCBFNoSlack;

            std::unordered_map<std::string, SecondOrderCBFConstraintEvaluation> secondOrderEvaluations;
            for (auto &[name, cbf]: secondOrderCbfNoSlack) {
                auto evaluation = cbf.evaluateConstraint(x, runtime);
                secondOrderEvaluations[name] = evaluation;
                optimiser->addLinearConstraint(evaluation.uCoe, -evaluation.constTerm);

                jsonSecondOrderCBFNoSlack.emplace_back(json{
                        {"name",  cbf.name},
                        {"coe",   model->control2Json(evaluation.uCoe)},
                        {"const", evaluation.constTerm},
                        {"h", evaluation.h},
                        {"hdot", evaluation.hdot},
                        {"psi1", evaluation.psi1},
                        {"sampledDataReserve", evaluation.sampledDataReserve},
                        {"hocbf", evaluation.constTerm}
                });
            }
            opt["hocbfNoSlack"] = jsonSecondOrderCBFNoSlack;

            int cnt = 0;
            for (auto &[name, cbf]: cbfSlack) {
                auto evaluation = cbf.evaluateConstraint(f, g, x, runtime);
                Eigen::VectorXd coe = makeSlackConstraintCoefficients(evaluation.uCoe, slackSize, cnt);

                optimiser->addLinearConstraint(coe, -evaluation.constWithoutTime);

                jsonCBFSlack.emplace_back(json{
                        {"name",  cbf.name},
                        {"coe",   model->control2Json(evaluation.uCoe)},
                        {"const", evaluation.constWithoutTime}
                });
                ++cnt;
            }

            json jsonSecondOrderCBFSlack = json::array();
            for (auto &[name, cbf]: secondOrderCbfSlack) {
                auto evaluation = cbf.evaluateConstraint(x, runtime);
                secondOrderEvaluations[name] = evaluation;
                Eigen::VectorXd coe = makeSlackConstraintCoefficients(evaluation.uCoe, slackSize, cnt);

                optimiser->addLinearConstraint(coe, -evaluation.constTerm);

                jsonSecondOrderCBFSlack.emplace_back(json{
                        {"name",  cbf.name},
                        {"coe",   model->control2Json(evaluation.uCoe)},
                        {"const", evaluation.constTerm},
                        {"h", evaluation.h},
                        {"hdot", evaluation.hdot},
                        {"psi1", evaluation.psi1},
                        {"sampledDataReserve", evaluation.sampledDataReserve},
                        {"hocbf", evaluation.constTerm}
                });
                ++cnt;
            }
            opt["hocbfSlack"] = jsonSecondOrderCBFSlack;
            opt["cbfSlack"] = jsonCBFSlack;

            auto result = optimiser->solve();

            auto u = result.head(uSize);
            model->setControlInput(u);
            opt["result"] = model->control2Json(u);
            opt["slacks"] = result.tail(slackSize);
            for (auto &item: opt["hocbfNoSlack"]) {
                std::string name = item.at("name").get<std::string>();
                if (secondOrderEvaluations.find(name) != secondOrderEvaluations.end()) {
                    item["hocbf"] = secondOrderEvaluations.at(name).value(u);
                }
            }
            for (int i = 0; i < static_cast<int>(opt["hocbfSlack"].size()); ++i) {
                auto &item = opt["hocbfSlack"][i];
                const std::string name = item.at("name").get<std::string>();
                if (secondOrderEvaluations.find(name) == secondOrderEvaluations.end()) {
                    continue;
                }
                const double physicalMargin =
                        secondOrderEvaluations.at(name).value(u);
                const double slack = result(uSize + cvtSlackSize + i);
                item["hocbf"] = physicalMargin;
                item["slack"] = slack;
                item["hocbf_with_slack"] = physicalMargin + slack;
            }

            try {
                json solver_status = optimiser->getStatus();
                opt["solver_info"] = solver_status;

                std::string status = solver_status.value("status", "unknown");
                if (status == "optimal") {
                    opt["status"] = "success";
                } else {
                    opt["status"] = "failed";
                    opt["error"] = solver_status.value("error", status);
                }
            } catch (...) {
                opt["status"] = "failed";
                opt["error"] = "Status check failed";
                opt["solver_info"] = {{"status", "error"}};
            }

            if (settings["debug"]["opt-cbc"]) {
                for (auto &[name, cbf]: cbfNoSlack.cbfs) {
                    for (auto &item : opt["cbfNoSlack"]) {
                        if (item["name"] == cbf.name) {
                            auto evaluation = cbfNoSlackEvaluations.at(name);
                            item["lhs"] = evaluation.hdot(u);
                            item["rhs"] = -cbf.alpha(evaluation.h);
                            item["lfh"] = evaluation.drift + evaluation.dhdt;
                            item["dhdt"] = evaluation.dhdt;
                            item["lgh"] = evaluation.uCoe;
                            double dt = settings["execute"]["time-step"];
                            item["expected-position"] = model->xy().vec() + u * dt;
                            item["expected-h"] = evaluation.hdot(u) * dt + evaluation.h;
                            break;
                        }
                    }
                }
            }

            // cbfNoSlack.checkInequality(f, g, x, u, runtime);
        }
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

    void applySecondOrderNominalFeasibilityGuard(double tolerance = 1.0e-9) {
        nominalGuardDiagnostic = {
                {"enabled", true},
                {"active", false},
                {"feasible", false},
                {"projection_norm", 0.0},
                {"polygon_vertices", 0},
                {"margin_before", nullptr},
                {"margin_after", nullptr}
        };
        hasNominalGuardDiagnostic = true;

        if (!hasNominalControlOverride || model->uSize() < 2 || !hasSecondOrderAccelerationBound()) {
            return;
        }

        std::vector<BridgeHocbfHalfspace2D> constraints;
        VectorXd x = model->getX();
        for (auto &[name, cbf]: secondOrderCbfNoSlack) {
            auto evaluation = cbf.evaluateConstraint(x, runtime);
            if (evaluation.uCoe.size() < 2) {
                continue;
            }
            constraints.push_back({evaluation.uCoe(0), evaluation.uCoe(1), -evaluation.constTerm});
        }

        BridgeHocbfProjectionResult result = projectBridgeHocbfNominalAcceleration(
                nominalControlOverride(0),
                nominalControlOverride(1),
                secondOrderAccelerationBound(),
                constraints,
                tolerance);

        nominalGuardDiagnostic = {
                {"enabled", true},
                {"active", result.active},
                {"feasible", result.feasible},
                {"projection_norm", result.projection_norm},
                {"polygon_vertices", result.vertex_count},
                {"margin_before", std::isfinite(result.margin_before) ? json(result.margin_before) : json(nullptr)},
                {"margin_after", std::isfinite(result.margin_after) ? json(result.margin_after) : json(nullptr)}
        };

        if (result.feasible && result.active) {
            nominalControlOverride(0) = result.projected_ax;
            nominalControlOverride(1) = result.projected_ay;
        }
    }

    void setNominalControlOverride(const Eigen::VectorXd &control) {
        if (control.size() != model->uSize()) {
            throw std::invalid_argument("Nominal control override size mismatch");
        }
        nominalControlOverride = control;
        hasNominalControlOverride = true;
    }

    void clearNominalControlOverride() {
        hasNominalControlOverride = false;
        nominalControlOverride.resize(0);
    }

    void output() const {
        std::cout << "Robot " << id << ": ";
        model->output();
    }

    void updateGridWorld() {
        updatedGridWorld = json::array();
        json params = searchParams;
        if (searchMethod == "front-sector") {
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
        else if (searchMethod == "front-cone") {
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
        else if (searchMethod == "downward") {
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
        VectorXd x = model->getX();
        json robotJson = {{"state", model->state2Json(x)},
                          {"id",    id}};
        if (!cbfNoSlack.cbfs.empty()) {
            json cbfNoSlackJson;
            for (auto &[name, cbf]: cbfNoSlack.cbfs) {
                cbfNoSlackJson[cbf.name] = cbf.h(x, runtime);
            }
            // cbfNoSlackJson[cbfNoSlack.getName()] = cbfNoSlack.h(model->getX(), runtime);
            robotJson["cbfNoSlack"] = cbfNoSlackJson;
        }
        {
            json cbfSlackJson;
            for (auto &[name, cbf]: cbfSlack) {
                cbfSlackJson[cbf.name] = cbf.h(x, runtime);
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

        // Calculate current distance from origin and compute uncertainty
        Point currentPos = model->xy();
        double currentDistFromOrigin = currentPos.distance_to(Point(0, 0));
        robotJson["uncertainty"] = uncertaintyFromCovarianceFunction(positionCovariance);
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
