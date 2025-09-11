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
    json myFormation;
    json updatedGridWorld;
    std::string folderName;
    std::string filename;
    CVT cvt;
    double runtime;
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
            model = std::make_unique<SingleIntegrate2D>();
        } else if (settings["model"] == "DoubleIntegrate2D") {
            model = std::make_unique<DoubleIntegrate2D>();
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
        auto xLim = world.boundary.get_x_limit(1), yLim = world.boundary.get_y_limit(1);
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

            double batteryLevel = model->extractFromVector(x, "battery");
            double normalizedBattery = (batteryLevel - model->BATTERY_MIN) / (model->BATTERY_MAX - model->BATTERY_MIN) * 100.0;
            return normalizedBattery - rho(model->extractXYFromVector(x));
        };

        CBF energyCBF;
        energyCBF.name = config["name"];
        energyCBF.h = batteryH;
        energyCBF.alpha = [](double _h) { return _h; };
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
            return kp * deltaHeadingRad;
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

    void setCommScissorCBF(const json& config) {
        int n = settings["num"];
        double maxRange = config["max-range"];
        auto getIdInPart = [&](int id) { return (id - 1) % (n / 2) + 1; };
        auto getPartId = [&](int id) { return id > n / 2 ? 2 : 1; };

        std::vector<Point> bases = getPointsFromJson(config["bases"]);

        int partId = getPartId(id);
        int idInPart = getIdInPart(id);

        myFormation = {
            {"id",           id},
            {"anchorPoints", json::array()},
            {"anchorIds",    json::array()}
        };
        std::vector<Point> formationPoints;
        std::vector<Point> formationVels;
        std::vector<std::string> anchorNames;

        if (idInPart == 1) {
            myFormation["anchorPoints"].push_back({bases[partId - 1].x, bases[partId - 1].y});
            formationPoints.push_back(bases[partId - 1]);
            formationVels.emplace_back(0, 0);
            anchorNames.emplace_back("base #" + std::to_string(partId));
        }
        if (idInPart <= 2) {
            myFormation["anchorPoints"].push_back({bases[bases.size() - 1].x, bases[bases.size() - 1].y});
            formationPoints.push_back(bases[bases.size() - 1]);
            formationVels.emplace_back(0, 0);
            anchorNames.emplace_back("base #" + std::to_string(bases.size()));
        }

        for (auto &[id, otherPos]: comm->_othersPos) {
            if (getPartId(id) != getPartId(this->id)) continue;
            if (id < this->id + static_cast<int>(config["min-neighbour-id-offset"])) continue;
            if (id > this->id + static_cast<int>(config["max-neighbour-id-offset"])) continue;
            myFormation["anchorIds"].push_back(id);
            formationPoints.push_back(otherPos);

            auto otherVel = comm->_othersVel[id];
            formationVels.emplace_back(otherVel);
            anchorNames.push_back("#" + std::to_string(id));
        }

        for (int i = 0; i < formationPoints.size(); i++) {
            auto otherPoint = formationPoints[i];
            auto otherVel = config["compensate-velocity"] ? formationVels[i] : Point(0, 0);
            double k = config["k"];
            auto fixedFormationCommH = [this, otherPoint, otherVel, maxRange, k](VectorXd x, double t) {
                Point myPosition = model->extractXYFromVector(x);
                double h = k * (
                        maxRange -
                        myPosition.distance_to(otherPoint + otherVel * (t - this->runtime))
                );
                return h;
            };

            CBF commCBF;
            commCBF.name = "commCBF(" + anchorNames[i] + ")";
            commCBF.h = fixedFormationCommH;
            cbfNoSlack.cbfs[commCBF.name] = commCBF;
        }
    }

    void setCommChainCBF(const json& config) {
        int n = settings["num"];
        double maxRange = config["max-range"];

        Point base(config["base"][0], config["base"][1]);

        myFormation = {
            {"id",           id},
            {"anchorPoints", json::array()},
            {"anchorIds",    json::array()}
        };
        std::vector<Point> formationPoints;
        std::vector<Point> formationVels;
        std::vector<std::string> anchorNames;

        if (id == 1) {
            myFormation["anchorPoints"].push_back({base.x, base.y});
            formationPoints.push_back(base);
            formationVels.emplace_back(0, 0);
            anchorNames.emplace_back("base");
        }

        for (auto &[id, otherPos]: comm->_othersPos) {
            if (id == this->id) continue;
            if (id < this->id + static_cast<int>(config["min-neighbour-id-offset"])) continue;
            if (id > this->id + static_cast<int>(config["max-neighbour-id-offset"])) continue;
            myFormation["anchorIds"].push_back(id);
            formationPoints.push_back(otherPos);

            auto otherVel = comm->_othersVel[id];
            formationVels.emplace_back(otherVel);
            anchorNames.push_back("#" + std::to_string(id));
        }

        for (int i = 0; i < formationPoints.size(); i++) {
            auto otherPoint = formationPoints[i];
            auto otherVel = config["compensate-velocity"] ? formationVels[i] : Point(0, 0);
            double k = config["k"];
            auto fixedFormationCommH = [this, otherPoint, otherVel, maxRange, k](VectorXd x, double t) {
                Point myPosition = model->extractXYFromVector(x);
                double h = k * (
                        maxRange -
                        myPosition.distance_to(otherPoint + otherVel * (t - this->runtime))
                );
                return h;
            };

            CBF commCBF;
            commCBF.name = "commCBF(" + anchorNames[i] + ")";
            commCBF.h = fixedFormationCommH;
            cbfNoSlack.cbfs[commCBF.name] = commCBF;
        }
    }

    void setSafetyCBF(const json& config) {
        if (settings["num"] == 1) return;
        auto safetyH = [&, config](VectorXd x, double t) {
            Point myPosition = model->xy();

            double safeDistance = config["safe-distance"];
            double k = config["k"];

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
        for (int i = 1; i <= cvt.n; i++) {
            cvt.ct[i] = gridWorld.getCentroidInPolygon(cvt.pl[i]);
        }

        Point cvtCenter = cvt.ct[this->id];

        if (config.contains("cvt") && config["cvt"]["on"]) {
            CBF cvtDistanceCBF;
            cvtDistanceCBF.name = config["cvt"]["name"];
            cvtDistanceCBF.h = [cvtCenter, config, this](VectorXd x, double t) {
                Point myPosition = this->model->extractXYFromVector(x);
                double kp = config["cvt"]["kp"];
                auto xLim = world.boundary.get_x_limit(1);
                auto yLim = world.boundary.get_y_limit(1);
                double maxDistance = sqrt(pow(xLim.second - xLim.first, 2) + pow(yLim.second - yLim.first, 2));
                double distance = cvtCenter.distance_to(myPosition);
                return -kp * (distance / maxDistance);
            };
            cvtDistanceCBF.alpha = [](double h) { return h; };
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
            cvtYawCBF.alpha = [](double h) { return h; };
            cbfSlack[cvtYawCBF.name] = cvtYawCBF;
        }
    }

    void postsetCBF() {
        auto cbfConfig = settings["cbfs"];
        if (cbfConfig["without-slack"]["comm-scissor"]["on"]) setCommScissorCBF(cbfConfig["without-slack"]["comm-scissor"]);
        if (cbfConfig["without-slack"]["comm-chain"]["on"]) setCommChainCBF(cbfConfig["without-slack"]["comm-chain"]);
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
        if (world.isCharging(model->xy()) && model->getStateVariable("battery") < model->BATTERY_MAX) {
            model->startCharge();
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

            optimiser->start(totalSize);

            optimiser->setObjective(uNominal);

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

            // if (!cbfNoSlack.cbfs.empty()) {
            //     VectorXd uCoe = cbfNoSlack.constraintUCoe(f, g, x, runtime);
            //     double constraintConstWithTime = cbfNoSlack.constraintConstWithTime(f, g, x, runtime);
            //
            //     optimiser->addLinearConstraint(uCoe, -constraintConstWithTime);
            //
            //     jsonCBFNoSlack.emplace_back(json{
            //             {"name",  cbfNoSlack.getName()},
            //             {"coe",   model->control2Json(uCoe)},
            //             {"const", constraintConstWithTime}
            //     });
            // }
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

            if (settings["debug"]["opt-cbc"]) {
                for (auto &[name, cbf]: cbfNoSlack.cbfs) {
                    for (auto &item : opt["cbfNoSlack"]) {
                        if (item["name"] == cbf.name) {
                            item["lhs"] = cbf.hdot(f, g, x, u, runtime);
                            item["rhs"] = -cbf.alpha(cbf.h(x, runtime));
                            item["lfh"] = cbf.dhdx(x, runtime).dot(VectorXd(f)) + cbf.dhdt(x, runtime);
                            item["dhdt"] = cbf.dhdt(x, runtime);
                            item["lgh"] = cbf.constraintUCoe(f, g, x, runtime);
                            item["expected-position"] = model->xy().vec() + u * 0.02;
                            item["expected-h"] = cbf.hdot(f, g, x, u, runtime) * 0.02 + cbf.h(x, runtime);
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

        {
            robotJson["opt"] = opt;
        }
        return robotJson;
    }

};


#endif //CBF_ROBOT_HPP
