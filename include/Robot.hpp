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
            optimiser = std::make_unique<Gurobi>();
        } else
#endif
#ifdef ENABLE_HIGHS
        if (settings["optimiser"] == "HiGHS") {
            optimiser = std::make_unique<HiGHS>();
        } else
#endif
        {
            throw std::invalid_argument("Invalid optimiser type");
        }
        model->setStateVariable("battery", 20.0 * (rand() % 100) / 100 + 10);
        model->setYawDeg(180);
        cbfSlack.clear();
        cbfNoSlack.cbfs.clear();
        comm = std::make_unique<CommunicatorCentral>(settings);
    }

    void setupInitialPosition() {
        std::string method = settings["initial-position"]["method"];
        if (method == "random-in-world") {
             model->setPosition2D(world.getRandomPoint());
        } else if (method == "random-in-polygon") {
            Polygon poly = Polygon(getPointsFromJson(settings["initial-position"]["polygon"]));
            model->setPosition2D(poly.get_random_point());
        } else if (method == "specified") {
            std::vector<Point> initialPositions = getPointsFromJson(settings["initial-position"]["positions"]);
            model->setPosition2D(initialPositions[id - 1]);
        } else {
            throw std::invalid_argument("Invalid method for setting initial position");
        }
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
        auto batteryH = [&](VectorXd x, double t) {
            std::function<double(Point)> minDistanceToChargingStations = [&](Point myPosition) {
                return (
                        world.distanceToChargingStations(myPosition)
                        / world.chargingStations[world.nearestChargingStation(myPosition)].second
                );
            };

            Point myPosition = model->extractXYFromVector(x);

            double h = inf;
            h = std::min(
                    h,
                    model->extractFromVector(x, "battery") - log(minDistanceToChargingStations(myPosition))
            );
            return h;
        };

        CBF energyCBF;
        energyCBF.name = "energyCBF";
        energyCBF.h = batteryH;
        energyCBF.alpha = [](double _h) { return _h; };
        cbfNoSlack.cbfs[energyCBF.name] = energyCBF;
    }

    void setYawCBF() {
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
        CBF yawCBF;
        yawCBF.name = "yawCBF";
        yawCBF.h = [&](VectorXd x, double t) {
            Point myPosition = model->extractXYFromVector(x);
            double headingRad = headingToNearestTarget(myPosition, t);
            double deltaHeadingRad = headingRad - model->extractFromVector(x, "yawRad");
            deltaHeadingRad = atan2(sin(deltaHeadingRad), cos(deltaHeadingRad));
            double kp = 5;

            double h = inf;
            h = std::min(h, kp * deltaHeadingRad);
            return h;
        };
        cbfSlack[yawCBF.name] = yawCBF;
    }

    void presetCBF() {
        if (settings["cbfs"]["without-slack"]["energy"]) setEnergyCBF();
        if (settings["cbfs"]["with-slack"]["yaw"]) setYawCBF();
    }

    void setCommunicationAutoCBF() {
        Point origin(0, 0);
        double maxCommRange = 8;

        std::vector<intPoint> neighbours;
        neighbours.clear();

        for (auto &[id, pos2d]: comm->position2D) {
            if (id == this->id) continue;
            if (model->xy().distance_to(pos2d) > 1.5 * maxCommRange) continue;
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

        auto autoFormationCommH = [this, formationPoints, maxCommRange](VectorXd x, double t) {
            Point myPosition = model->extractXYFromVector(x);

            double h = inf;
            for (auto &point: formationPoints) {
                h = std::min(
                        h,
                        100 / maxCommRange * (
                                maxCommRange -
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

    void setCommunicationFixedCBF() {
        int n = settings["num"];
        auto partId = [&](int id) { return (id - 1) % (n / 2) + 1; };
        auto isSecondPart = [&](int id) { return id > (n / 2); };

        Point origin(0, 0);
        Point baseOfMyPart = Point(-3 + 6 * isSecondPart(id), 0);
        int idInPart = partId(id);

        myFormation = {
            {"id",           id},
            {"anchorPoints", json::array()},
            {"anchorIds",    json::array()}
        };
        std::vector<Point> formationPoints;

        if (idInPart == 1) {
            myFormation["anchorPoints"].push_back({baseOfMyPart.x, baseOfMyPart.y});
            formationPoints.push_back(baseOfMyPart);
        }
        if (idInPart <= 2) {
            myFormation["anchorPoints"].push_back({origin.x, origin.y});
            formationPoints.push_back(origin);
        }

        for (auto &[id, pos2d]: comm->position2D) {
            if (isSecondPart(id) != isSecondPart(this->id)) continue;
            if (partId(id) <= idInPart) continue;
            if (partId(id) > idInPart + 2) continue;
            myFormation["anchorIds"].push_back(id);
            formationPoints.push_back(pos2d);
        }

        auto fixedFormationCommH = [this, formationPoints](VectorXd x, double t) {
            double maxCommDistance = 8.5;
            Point myPosition = model->extractXYFromVector(x);

            double h = inf;
            for (auto &point: formationPoints) {
                h = std::min(
                        h,
                        (
                                maxCommDistance * maxCommDistance -
                                myPosition.distance_to(point) * myPosition.distance_to(point)
                        )
                );
            }
            return h;
        };

        CBF commCBF;
        commCBF.name = "commCBF";
        commCBF.h = fixedFormationCommH;
        cbfNoSlack.cbfs[commCBF.name] = commCBF;
    }

    void setSafetyCBF() {
        auto safetyH = [&](VectorXd x, double t) {
            Point myPosition = model->xy();

            double safeDistance = 3;

            double h = inf;
            for (auto &[id, pos2d]: comm->position2D) {
                if (this->id == id) continue;
                h = std::min(
                        h,
                        0.5 * (myPosition.distance_to(pos2d) - safeDistance)
                );
            }
            return h;
        };

        CBF safetyCBF;
        safetyCBF.name = "safetyCBF";
        safetyCBF.h = safetyH;
        cbfNoSlack.cbfs[safetyCBF.name] = safetyCBF;
    }

    void setCVTCBF() {
        cvt = CVT(settings["num"], world.boundary);
        for (auto &[id, pos2d]: comm->position2D) {
            if (id == this->id) continue;
            cvt.pt[id] = pos2d;
        }
        cvt.pt[this->id] = model->xy();
        cvt.cal_poly();
        for (int i = 1; i <= cvt.n; i++) {
            cvt.ct[i] = gridWorld.getCentroidInPolygon(cvt.pl[i]);
        }

        double kp = 5.0;
        CBF cvtCBF;
        cvtCBF.name = "cvtCBF";
        Point cvtCenter = cvt.ct[this->id];
        cvtCBF.h = [cvtCenter, kp, this](VectorXd x, double t) {
            Point myPosition = this->model->extractXYFromVector(x);
            return -kp * cvtCenter.distance_to(myPosition);
        };
        cvtCBF.alpha = [](double h) { return h; };
        cbfSlack[cvtCBF.name] = cvtCBF;
    }

    void postsetCBF() {
        auto cbfConfig = settings["cbfs"];
        if (cbfConfig["without-slack"]["comm-fixed"]) setCommunicationFixedCBF();
        if (cbfConfig["without-slack"]["comm-auto"]) setCommunicationAutoCBF();
        if (cbfConfig["with-slack"]["cvt"]) setCVTCBF();
        if (cbfConfig["without-slack"]["safety"]) setSafetyCBF();
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
        if (world.isCharging(model->xy()) && model->getStateVariable("battery") < 100.0) {
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
        updatedGridWorld.clear();
        double tol = 2;
        for (auto &[id, position2D]: comm->position2D) {
            updatedGridWorld.push_back(gridWorld.setValueInCircle(position2D, tol, true, true));
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
            cbfNoSlackJson[cbfNoSlack.getName()] = cbfNoSlack.h(model->getX(), runtime);
            robotJson["cbfNoSlack"] = cbfNoSlackJson;
        }
        {
            json cbfSlackJson;
            for (auto &[name, cbf]: cbfSlack) {
                cbfSlackJson[cbf.name] = cbf.h(model->getX(), runtime);
            }
            robotJson["cbfSlack"] = cbfSlackJson;
        }

        if (settings["cbfs"]["with-slack"]["cvt"]) {
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
