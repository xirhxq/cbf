#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "cbf/cbf"
#include "world/world"
#include "models/models"
#include "optimisers/optimisers"
#include "communicators/communicators"

typedef std::pair<int, Point> intPoint;

class Robot {
public:
    int id = 0;
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
    json updatedGridWorld;
    std::string folderName;
    std::string filename;
    CVT cvt;
    double runtime;
    std::function<double(double)> uncertaintyFunction;
public:

    Robot() = default;

    Robot(int id, json &settings)
            : id(id),
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
#ifdef ENABLE_GUROBI
        if (settings["optimiser"] == "Gurobi") {
            optimiser = std::make_unique<Gurobi>(settings["cbfs"]["objective-function"]);
        } else
#endif
#ifdef ENABLE_HIGHS
        if (settings["optimiser"] == "HiGHS") {
            optimiser = std::make_unique<HiGHS>(settings["cbfs"]["objective-function"]);
        } else
#endif
        {
            throw std::invalid_argument("Invalid optimiser type");
        }
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
        auto xLim = world.boundary.get_x_limit(1.1), yLim = world.boundary.get_y_limit(1.1);
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

    void setFixedCommCBF(const json& config) {
        int n = settings["num"];
        double maxRange = config["max-range"];
        std::string mode = config.value("mode", "scissor");

        auto getFormationInfo = [&](int robotId) -> std::pair<int, int> {
            if (mode == "scissor") {
                int idInPart = (robotId - 1) % (n / 2) + 1;
                int partId = robotId > n / 2 ? 2 : 1;
                return {partId, idInPart};
            } else if (mode == "chain") {
                return {1, robotId};
            } else {
                throw std::invalid_argument("Unknown formation mode: " + mode);
            }
        };

        auto [partId, idInPart] = getFormationInfo(id);

        myFormation = {
            {"id",           id},
            {"anchorPoints", json::array()},
            {"anchorIds",    json::array()}
        };

        std::vector<Point> formationPoints;
        std::vector<Point> formationVels;
        std::vector<std::string> anchorNames;

        int minOffset = config["min-neighbour-id-offset"];
        int maxOffset = config["max-neighbour-id-offset"];

        if (config.contains("bases") && !config["bases"].empty()) {
            std::vector<Point> bases = getPointsFromJson(config["bases"]);

            if (mode == "chain") {
                for (int baseIdx = 0; baseIdx < bases.size(); baseIdx++) {
                    int virtualId = -baseIdx;

                    if (virtualId >= this->id + minOffset && virtualId <= this->id + maxOffset) {
                        myFormation["anchorPoints"].push_back({bases[baseIdx].x, bases[baseIdx].y});
                        formationPoints.push_back(bases[baseIdx]);
                        formationVels.emplace_back(0, 0);
                        anchorNames.emplace_back("base-" + std::to_string(virtualId));
                    }
                }
            } else if (mode == "scissor") {
                if (idInPart == 1 && partId <= 2 && minOffset <= -1) {
                    myFormation["anchorPoints"].push_back({bases[partId].x, bases[partId].y});
                    formationPoints.push_back(bases[partId]);
                    formationVels.emplace_back(0, 0);
                    anchorNames.emplace_back("base-" + std::to_string(partId));
                }

                if (idInPart <= 2 && minOffset <= -2) {
                    myFormation["anchorPoints"].push_back({bases[0].x, bases[0].y});
                    formationPoints.push_back(bases[0]);
                    formationVels.emplace_back(0, 0);
                    anchorNames.emplace_back("base-0");
                }
            }
        }

        for (auto &[otherId, otherPos]: comm->_othersPos) {
            if (otherId == this->id) continue;

            if (otherId < this->id + minOffset) continue;
            if (otherId > this->id + maxOffset) continue;

            auto [otherPartId, otherIdInPart] = getFormationInfo(otherId);

            if (otherPartId != partId) continue;

            myFormation["anchorIds"].push_back(otherId);
            formationPoints.push_back(otherPos);
            auto otherVel = comm->_othersVel[otherId];
            formationVels.emplace_back(otherVel);
            anchorNames.push_back("#" + std::to_string(otherId));
        }

        for (int i = 0; i < formationPoints.size(); i++) {
            auto otherPoint = formationPoints[i];
            auto otherVel = config["compensate-velocity"] ? formationVels[i] : Point(0, 0);
            double k = config["k"];
            bool isAnchor = anchorNames[i].find("base-") != std::string::npos;  // Check if this is an anchor point

            auto fixedFormationCommH = [this, otherPoint, otherVel, maxRange, k, isAnchor](VectorXd x, double t) {
                Point myPosition = model->extractXYFromVector(x);

                // Calculate distance between robots/anchor
                double distance = myPosition.distance_to(otherPoint);
                double h = k * (maxRange - distance);

                // Robust CBF: ĥ = h - lε, where l = k for comm CBF
                // Use distance from origin for uncertainty calculation
                Point origin(0, 0);
                double myDistFromOrigin = myPosition.distance_to(origin);
                double myUncertainty = uncertaintyFunction(myDistFromOrigin);

                double robust_h;
                if (isAnchor) {
                    // Robot-to-anchor communication: only robot uncertainty
                    robust_h = h - k * myUncertainty;
                } else {
                    // Robot-to-robot communication: sum of both uncertainties
                    double otherDistFromOrigin = otherPoint.distance_to(origin);
                    double otherUncertainty = uncertaintyFunction(otherDistFromOrigin);
                    robust_h = h - k * (myUncertainty + otherUncertainty);
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
            commCBF.name = "fixedCommCBF(" + anchorNames[i] + ")";
            commCBF.h = fixedFormationCommH;
            commCBF.dhdx_analytical = dhdx;
            commCBF.dhdt_analytical = dhdt;
            double alpha_coe = config.value("alpha/coe", 0.1);
            int alpha_pow = config.value("alpha/pow", 1);
            commCBF.setAlphaClassK(alpha_coe, alpha_pow);
            cbfNoSlack.cbfs[commCBF.name] = commCBF;
        }
    }

    void setSafetyCBF(const json& config) {
        if (settings["num"] == 1) return;
        auto safetyH = [&, config](VectorXd x, double t) {
            Point myPosition = model->extractXYFromVector(x);

            double safeDistance = config["safe-distance"];
            double k = config.contains("k") ? static_cast<double>(config["k"]) : 1.0;

            double h = inf;
            for (auto &[id, pos2d]: comm->_othersPos) {
                if (this->id == id) continue;
                h = std::min(
                        h,
                        k * (myPosition.distance_to(pos2d) - safeDistance)
                );
            }
            return h;
        };

        CBF safetyCBF;
        safetyCBF.name = "safetyCBF";
        safetyCBF.h = safetyH;
        cbfNoSlack.cbfs[safetyCBF.name] = safetyCBF;
    }

    void setCVTCBF(const json& config) {
        cvt = CVT(settings["num"], world.boundary);
        for (auto &[id, pos2d]: comm->_othersPos) {
            if (id == this->id) continue;
            cvt.pt[id] = pos2d;
        }
        cvt.pt[this->id] = model->xy();
        cvt.cal_poly();
        Point densityCentroid = gridWorld.getCentroidInPolygon(cvt.pl[this->id]);

        std::string explorationMode = "intelligent";
        if (config.contains("cvt") && config["cvt"].contains("exploration-mode")) {
            explorationMode = config["cvt"]["exploration-mode"];
        }

        if (explorationMode == "centroid") {
            cvt.ct[this->id] = densityCentroid;
        } else if (explorationMode == "intelligent") {
            if (gridWorld.isExplored(densityCentroid)) {
                cvt.ct[this->id] = gridWorld.getNearestUnexploredPointInPolygon(cvt.pl[this->id], densityCentroid);
            } else {
                cvt.ct[this->id] = densityCentroid;
            }
        } else if (explorationMode == "nearest-unexplored") {
            cvt.ct[this->id] = gridWorld.getNearestUnexploredPointInPolygon(cvt.pl[this->id], densityCentroid);
        } else {
            throw std::invalid_argument("Unknown exploration-mode: " + explorationMode);
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

    void postsetCBF() {
        auto cbfConfig = settings["cbfs"];

        if (cbfConfig["without-slack"]["comm-fixed"]["on"]) setFixedCommCBF(cbfConfig["without-slack"]["comm-fixed"]);
        if (cbfConfig["without-slack"]["comm-auto"]["on"]) setCommunicationAutoCBF(cbfConfig["without-slack"]["comm-auto"]);
        if (cbfConfig["with-slack"]["cvt"]["on"] || cbfConfig["with-slack"]["cvt-yaw"]["on"]) setCVTCBF(cbfConfig["with-slack"]);
        if (cbfConfig["without-slack"]["safety"]["on"]) setSafetyCBF(cbfConfig["without-slack"]["safety"]);
    }

    void optimise() {
        VectorXd uNominal(model->uSize());
        opt = {
                {"nominal",    model->control2Json(uNominal)},
                {"result",     model->control2Json(uNominal)},
                {"cbfNoSlack", json::array()},
                {"cbfSlack",   json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
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
                            {"const", constraintConstWithTime}
                    });
                }
            } else if (cbf_method == "min") {
                if (!cbfNoSlack.cbfs.empty()) {
                    VectorXd uCoe = cbfNoSlack.constraintUCoe(f, g, x, runtime);
                    double constraintConstWithTime = cbfNoSlack.constraintConstWithTime(f, g, x, runtime);

                    optimiser->addLinearConstraint(uCoe, -constraintConstWithTime);

                    jsonCBFNoSlack.emplace_back(json{
                            {"name",  cbfNoSlack.getName()},
                            {"coe",   model->control2Json(uCoe)},
                            {"const", constraintConstWithTime}
                    });
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

            auto u = result.head(uSize);
            model->setControlInput(u);
            opt["result"] = model->control2Json(u);
            opt["slacks"] = result.tail(slackSize);

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

        // Calculate current distance from origin and compute uncertainty
        Point currentPos = model->xy();
        double currentDistFromOrigin = currentPos.distance_to(Point(0, 0));
        robotJson["uncertainty"] = uncertaintyFunction(currentDistFromOrigin);
        {
            robotJson["opt"] = opt;
        }
        return robotJson;
    }

    double getUncertaintyAtDistance(double distance) {
        return uncertaintyFunction(distance);
    }

};


#endif //CBF_ROBOT_HPP
