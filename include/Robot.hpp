#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "cbf/cbf"
#include "cbf/CBFConfig.hpp"
#include "world/world"
#include "models/models"
#include "optimisers/optimisers"
#include "communicators/communicators"

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

    json fixedLocalizationReferences() const {
        json references = {
            {"anchorIds", json::array()},
            {"baseIds", json::array()}
        };

        const auto commConfig =
            settings["cbfs"]["without-slack"]["comm-fixed"];
        const int minOffset =
            commConfig.value("min-neighbour-id-offset", -2);
        const int maximumBaseIndex =
            -idInMyPart - minOffset;

        for (std::size_t index = 0; index < myBasesId.size(); ++index) {
            if (static_cast<int>(index) > maximumBaseIndex) {
                continue;
            }
            references["baseIds"].push_back(myBasesId[index]);
        }

        for (int otherId : myNeighboursId) {
            if (!isSamePartAsMe(otherId)) {
                continue;
            }
            if (getIdInPart(otherId) >= idInMyPart) {
                continue;
            }
            references["anchorIds"].push_back(otherId);
        }
        return references;
    }

    void getCovariance() {
        if (!enablePositionCovariance) return;

        const json references = fixedLocalizationReferences();
        myCovarianceFormation = {
            {"id", id},
            {"anchorIds", references.at("anchorIds")},
            {"baseIds", references.at("baseIds")}
        };

        Point p = this->model->xy();
        std::vector<Point> anchorPoints;
        std::vector<Eigen::Matrix2d> anchorCovariances;

        for (const json& baseIdJson : references.at("baseIds")) {
            const int baseId = baseIdJson.get<int>();
            if (baseId < 0 || baseId >= static_cast<int>(bases.size())) {
                throw std::invalid_argument(
                    "#" + std::to_string(id)
                    + " has an invalid fixed base reference "
                    + std::to_string(baseId)
                );
            }
            anchorPoints.push_back(bases.at(baseId));
            anchorCovariances.push_back(Eigen::Matrix2d::Zero());
        }

        for (const json& anchorIdJson : references.at("anchorIds")) {
            const int otherId = anchorIdJson.get<int>();
            const auto positionIt = comm->_othersPos.find(otherId);
            const auto covarianceIt =
                comm->_othersPositionCovariance.find(otherId);
            if (positionIt == comm->_othersPos.end()
                || covarianceIt == comm->_othersPositionCovariance.end()) {
                throw std::invalid_argument(
                    "#" + std::to_string(id)
                    + " is missing fixed localization data for #"
                    + std::to_string(otherId)
                );
            }
            anchorPoints.push_back(positionIt->second);
            anchorCovariances.push_back(covarianceIt->second);
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

            if (!std::isfinite(total_variance) || total_variance <= 0.0) {
                throw std::invalid_argument(
                    "#" + std::to_string(id)
                    + " has invalid covariance validity: total variance for fixed reference "
                    + std::to_string(i)
                    + " must be finite and positive"
                );
            }

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
        Phi = J.transpose() * Sigma.inverse() * J;

        // A relative eigenvalue floor of 1e-12 limits the FIM condition
        // number to 1e12 while avoiding any regularization of its values.
        constexpr double relativeSpectralThreshold = 1e-12;
        if (!Phi.allFinite()) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance geometry: FIM contains non-finite values"
            );
        }
        const double phiScale = Phi.cwiseAbs().maxCoeff();
        if ((Phi - Phi.transpose()).cwiseAbs().maxCoeff()
            > relativeSpectralThreshold * phiScale) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance geometry: FIM is not symmetric"
            );
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> phiEigenSolver(Phi);
        if (phiEigenSolver.info() != Eigen::Success
            || !phiEigenSolver.eigenvalues().allFinite()) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance geometry: FIM eigenvalues are invalid"
            );
        }
        const double minPhiEigenvalue = phiEigenSolver.eigenvalues().minCoeff();
        const double maxPhiEigenvalue = phiEigenSolver.eigenvalues().maxCoeff();
        if (maxPhiEigenvalue <= 0.0
            || minPhiEigenvalue
               <= relativeSpectralThreshold * maxPhiEigenvalue) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance geometry: FIM is not positive definite or exceeds the condition limit"
            );
        }

        positionCovariance = Phi.inverse();
        if (!positionCovariance.allFinite()) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance validity: covariance contains non-finite values"
            );
        }
        const double covarianceScale = positionCovariance.cwiseAbs().maxCoeff();
        if ((positionCovariance - positionCovariance.transpose()).cwiseAbs().maxCoeff()
            > relativeSpectralThreshold * covarianceScale) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance validity: covariance is not symmetric"
            );
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> covarianceEigenSolver(
            positionCovariance
        );
        if (covarianceEigenSolver.info() != Eigen::Success
            || !covarianceEigenSolver.eigenvalues().allFinite()
            || covarianceEigenSolver.eigenvalues().minCoeff()
               < -relativeSpectralThreshold * covarianceScale) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance validity: covariance is not positive semidefinite"
            );
        }

        const double scalarEpsilon =
            uncertaintyFromCovarianceFunction(positionCovariance);
        if (!std::isfinite(scalarEpsilon) || scalarEpsilon < 0.0) {
            throw std::invalid_argument(
                "#" + std::to_string(id)
                + " has invalid covariance validity: configured scalar epsilon is not finite and nonnegative"
            );
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
        getCovariance();
        updateUncertaintyHistory(uncertaintyFromCovarianceFunction(positionCovariance), dt);
    }

    void setupFormation() {
        // Setup formation information for constraint violation detection
        // This should always be called, regardless of whether comm-fixed CBF is enabled
        // Uses the already-initialized myNeighboursId and myBasesId

        const json references = fixedLocalizationReferences();
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

    void setFixedCommCBF(json& config) {
        // This function only creates the CBF constraints
        // Formation information is already set by setupFormation()
        double maxRange = config["max-range"];

        std::vector<Point> formationPoints;
        std::vector<Point> formationVels;
        std::vector<std::string> anchorCBFNames;
        std::vector<double> formationUncertainties;
        std::vector<double> formationUncertaintyRates;

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
            formationUncertainties.push_back(uncertaintyFromCovarianceFunction(comm->_othersPositionCovariance[otherId]));
            auto rateIt = comm->_othersUncertaintyRate.find(otherId);
            formationUncertaintyRates.push_back(
                rateIt == comm->_othersUncertaintyRate.end() ? 0.0 : rateIt->second
            );
        }

        double myUncertainty = currentUncertainty;
        std::string uncertaintyRateMode = "off";
        if (settings["cbfs"].contains("uncertainty-rate")) {
            uncertaintyRateMode =
                settings["cbfs"]["uncertainty-rate"].value("mode", "off");
        }
        bool useUncertaintyRate =
            uncertaintyRateMode == "backward-difference-positive";

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
                ? uncertaintyRate + otherUncertaintyRate
                : 0.0;

            auto fixedFormationCommH = [this, otherPoint, otherVel, maxRange, k, isAnchor, myUncertainty, otherUncertainty, considerUncertainty](VectorXd x, double t) {
                Point myPosition = model->extractXYFromVector(x);
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
            auto dhdt = [this, otherPoint, otherVel, k, uncertaintyRateSum](VectorXd x, double t) -> double {
                Point myPosition = model->extractXYFromVector(x);
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

        for (auto it = cbfNoSlack.cbfs.begin(); it != cbfNoSlack.cbfs.end();) {
            if (it->first == "safetyCBF" ||
                it->first.rfind("safetyCBF(#", 0) == 0) {
                it = cbfNoSlack.cbfs.erase(it);
            } else {
                ++it;
            }
        }

        double myUncertainty = currentUncertainty;
        bool considerUncertainty = config.value("consider-uncertainty", true);
        double safeDistance = config["safe-distance"];
        double k = config.value("k", 1.0);
        std::string safetyMode = config.value("mode", "minimum");
        std::string uncertaintyRateMode = "off";
        if (settings["cbfs"].contains("uncertainty-rate")) {
            uncertaintyRateMode =
                settings["cbfs"]["uncertainty-rate"].value("mode", "off");
        }
        bool useUncertaintyRate =
            considerUncertainty &&
            uncertaintyRateMode == "backward-difference-positive";

        struct SafetyPair {
            int id;
            Point position;
            Point velocity;
            double uncertainty;
            double uncertaintyRate;
            double currentRobustH;
        };
        std::vector<SafetyPair> safetyPairs;
        Point myPosition = model->xy();
        for (auto &[otherId, otherPosition]: comm->_othersPos) {
            if (id == otherId) continue;

            double otherUncertainty = 0.0;
            auto covarianceIt = comm->_othersPositionCovariance.find(otherId);
            if (enablePositionCovariance &&
                covarianceIt != comm->_othersPositionCovariance.end()) {
                otherUncertainty =
                    uncertaintyFromCovarianceFunction(covarianceIt->second);
            }

            Point otherVelocity(0.0, 0.0);
            auto velocityIt = comm->_othersVel.find(otherId);
            if (velocityIt != comm->_othersVel.end()) {
                otherVelocity = Point(velocityIt->second);
            }

            double otherUncertaintyRate = 0.0;
            auto rateIt = comm->_othersUncertaintyRate.find(otherId);
            if (rateIt != comm->_othersUncertaintyRate.end()) {
                otherUncertaintyRate = rateIt->second;
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
                ? uncertaintyRate + pair.uncertaintyRate
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
                Point currentPosition = model->extractXYFromVector(x);
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
                Point currentPosition = model->extractXYFromVector(x);
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
                Point currentPosition = model->extractXYFromVector(x);
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

        if (cbfConfig["without-slack"]["comm-fixed"]["on"]) setFixedCommCBF(cbfConfig["without-slack"]["comm-fixed"]);
        if (cbfConfig["without-slack"]["comm-auto"]["on"]) setCommunicationAutoCBF(cbfConfig["without-slack"]["comm-auto"]);
        if (cbfConfig["with-slack"]["cvt"]["on"] || cbfConfig["with-slack"]["cvt-yaw"]["on"]) setCVTCBF(cbfConfig["with-slack"]);
        if (cbfConfig["without-slack"]["safety"]["on"]) setSafetyCBF(cbfConfig["without-slack"]["safety"]);
    }

    void optimise() {
        VectorXd uNominal = VectorXd::Zero(model->uSize());
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
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
        std::vector<Eigen::VectorXd> hardConstraintCoefficients;
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
            int slackSize = cbfSlack.size();
            int totalSize = uSize + slackSize;

            optimiser->start(totalSize, uSize);

            if (inputLimitsEnabled) {
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

            if (cbf_method == "all") {
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
