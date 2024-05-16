#ifndef CBF_SWARM_HPP
#define CBF_SWARM_HPP

#include "utils.h"
#include "World.hpp"
#include "Robot.hpp"
#include "GridWorld.hpp"

#include <fstream>
#include <iomanip>

class Swarm {
public:
    int n;
    double runtime = 0.0;
    std::vector<Robot> robots;
    World world;
    std::ofstream ofstream;
    json data;
    json stepData;
    CVT cvt;
    GridWorld gridWorld;
    json updatedGridWorld;
    json config;
    std::string folderName;
    std::string filename;
public:
    Swarm(const json &settings)
            : config(settings),
              n(settings["swarm"]["num"]),
              world(settings["world"]),
              robots(n, Robot(settings["swarm"]["dim"])),
              gridWorld(computeGridWorld(settings["world"])),
              runtime(0.0) {
        for (int i = 0; i < n; i++) {
            robots[i].id = i + 1;
            robots[i].G(2, 2) = 0;
            robots[i].G(3, 3) = 1;
            robots[i].state.setBattery(20.0 * (rand() % 100) / 100 + 10);
            robots[i].state.setYawDeg(180);
            robots[i].F << 0, 0, -1, 0;
            robots[i].cbfSlack.clear();
            robots[i].cbfNoSlack.cbfs.clear();
        }
        setupInitialPosition();
    }

    static GridWorld computeGridWorld(const json &worldSettings) {
        World tmpWorld(worldSettings);
        auto worldXLimit = tmpWorld.boundary.get_x_limit(1.0), worldYLimit = tmpWorld.boundary.get_y_limit(1.0);
        int xNum = (worldXLimit.second - worldXLimit.first) / double(worldSettings["spacing"]);
        int yNum = (worldYLimit.second - worldYLimit.first) / double(worldSettings["spacing"]);
        return {worldXLimit, xNum, worldYLimit, yNum};
    }

    void output() {
        printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, runtime);
        for (auto &robot: robots) {
            printf("Robot %d: (%.4lf, %.4lf, %.4lf, %.4lf)\n", robot.id, robot.state.x(), robot.state.y(),
                   robot.state.battery(), robot.state.yawDeg());
        }
        printf("--------------\n");
    }

    void setupInitialPosition() {
        auto settings = config["swarm"]["initialPosition"];
        std::string method = settings["method"];
        if (method == "randomAll") {
            for (auto &robot: robots) {
                robot.state.setPosition(world.getRandomPoint());
            }
        } else if (method == "randomInPolygon") {
            Polygon poly = Polygon(getPointsFromJson(settings["polygon"]));
            for (auto &robot: robots) {
                robot.state.setPosition(poly.get_random_point());
            }
        } else if (method == "specified") {
            std::vector<Point> initialPositions = getPointsFromJson(settings["positions"]);
            auto pointIt = initialPositions.begin();
            for (Robot &robot: robots) {
                robot.state.setPosition(*pointIt);
                ++pointIt;
            }
        } else {
            throw std::invalid_argument("Invalid method for setting initial position");
        }
    }

    void setEnergyCBF() {
        for (auto &robot: robots) {
            auto batteryH = [&](VectorXd x, double t) {
                State state(x);
                std::function<double(Point, World)> minDistanceToChargingStations = [&](Point myPosition, World world) {
                    return (
                            world.distanceToChargingStations(myPosition)
                            / world.chargingStations[world.nearestChargingStation(myPosition)].second
                    );
                };

                Point myPosition = state.xy();

                double h = inf;
                h = std::min(
                        h,
                        state.battery() - log(minDistanceToChargingStations(myPosition, world))
                );
                return h;
            };

            CBF energyCBF;
            energyCBF.name = "energyCBF";
            energyCBF.h = batteryH;
            energyCBF.alpha = [](double _h) { return _h; };
            energyCBF.controlVariable.resize(robot.state.X.size());
            energyCBF.controlVariable << 1, 1, 1, 1;
            robot.cbfNoSlack.cbfs[energyCBF.name] = energyCBF;
        }
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
        for (auto &robot: robots) {
            CBF yawCBF;
            yawCBF.name = "yawCBF";
            yawCBF.h = [&](VectorXd x, double t) {
                State state(x);
                Point myPosition = state.xy();
                double headingRad = headingToNearestTarget(myPosition, t);
                double deltaHeadingRad = headingRad - state.yawRad();
                deltaHeadingRad = atan2(sin(deltaHeadingRad), cos(deltaHeadingRad));
                double kp = 5;

                double h = inf;
                h = std::min(
                        h,
                        kp * deltaHeadingRad
                );
                return h;
            };
            yawCBF.controlVariable.resize(robot.state.X.size());
            yawCBF.controlVariable << 0, 0, 0, 1;
            robot.cbfSlack[yawCBF.name] = yawCBF;
        }
    }

    void setCommunicationAutoCBF() {
        for (auto &robot: robots) {
            auto autoFormationCommH = [&](VectorXd x, double t) {
                State state(x);
                double maxCommRange = 8;
                Point myPosition = state.xy();
                Point origin(0, 0);

                auto robotDistanceToOrigin = [&](Robot r) {
                    return r.state.xy().distance_to(origin);
                };

                std::vector<Robot> robotsInCommRange;
                for (auto &otherRobot: robots) {
                    if (robot.id == otherRobot.id) continue;
                    if (robot.state.xy().distance_to(otherRobot.state.xy()) > maxCommRange) continue;
                    if (robotDistanceToOrigin(otherRobot) > robotDistanceToOrigin(robot)) continue;
                    robotsInCommRange.push_back(otherRobot);
                }
                std::sort(
                        robotsInCommRange.begin(), robotsInCommRange.end(),
                        [&](Robot a, Robot b) {
                            return robotDistanceToOrigin(a) < robotDistanceToOrigin(b);
                        }
                );

                auto formationRobots = std::vector<Robot>(
                        robotsInCommRange.end() - std::min(2, (int) robotsInCommRange.size()),
                        robotsInCommRange.end()
                );

                std::vector<Point> formationPoints(formationRobots.size());
                for (auto &otherRobot: formationRobots) {
                    formationPoints.push_back(otherRobot.state.xy());
                }
                if (formationPoints.size() < 2) {
                    formationPoints.push_back(origin);
                }

                double h = inf;
                for (auto &point: formationPoints) {
                    h = std::min(
                            h,
                            0.5 * (
                                    maxCommRange -
                                    myPosition.distance_to(point)
                            )
                    );
                }
                return h;
            };

            CBF commCBF;
            commCBF.name = "commCBF";
            commCBF.h = autoFormationCommH;
            commCBF.controlVariable.resize(robot.state.X.size());
            commCBF.controlVariable << 1, 1, 1, 1;
            robot.cbfNoSlack.cbfs[commCBF.name] = commCBF;
        }
    }

    void setCommunicationFixedCBF() {
        for (auto &robot: robots) {
            auto fixedFormationCommH = [&](VectorXd x, double t) {
                State state(x);
                auto partId = [&](int id) { return (id - 1) % (n / 2) + 1; };
                auto isSecondPart = [&](int id) { return id > (n / 2); };

                double maxCommDistance = 8.5;
                Point myPosition = state.xy();
                Point baseOfMyPart = Point(-3 + 6 * isSecondPart(robot.id), 0);
                Point origin(0, 0);


                int idInPart = partId(robot.id);

                std::vector<Point> formationPoints;
                if (idInPart == 1) formationPoints.push_back(baseOfMyPart);
                if (idInPart <= 2) formationPoints.push_back(origin);
                for (auto &other: robots) {
                    if (isSecondPart(robot.id) != isSecondPart(other.id)) continue;
                    if (partId(other.id) <= idInPart) continue;
                    if (partId(other.id) > idInPart + 2) continue;
                    formationPoints.push_back(other.state.xy());
                }

                double h = inf;
                for (auto &point: formationPoints) {
                    h = std::min(
                            h,
                            0.5 * (
                                    maxCommDistance -
                                    myPosition.distance_to(point)
                            )
                    );
                }
                return h;
            };

            CBF commCBF;
            commCBF.name = "commCBF";
            commCBF.h = fixedFormationCommH;
            commCBF.controlVariable.resize(robot.state.X.size());
            commCBF.controlVariable << 1, 1, 1, 1;
            robot.cbfNoSlack.cbfs[commCBF.name] = commCBF;
        }
    }

    void setSafetyCBF() {
        for (auto &robot: robots) {
            auto safetyH = [&](VectorXd x, double t) {
                State state(x);
                Point myPosition = state.xy();

                double safeDistance = 3;

                double h = inf;
                for (auto &otherRobot: robots) {
                    if (robot.id == otherRobot.id) continue;
                    h = std::min(
                            h,
                            0.5 * (myPosition.distance_to(otherRobot.state.xy()) - safeDistance)
                    );
                }
                return h;
            };

            CBF safetyCBF;
            safetyCBF.name = "safetyCBF";
            safetyCBF.h = safetyH;
            safetyCBF.controlVariable.resize(robot.state.X.size());
            safetyCBF.controlVariable << 1, 1, 1, 1;
            robot.cbfNoSlack.cbfs[safetyCBF.name] = safetyCBF;
        }
    }

    void endLog() {
        ofstream << std::fixed << std::setprecision(6) << data;
        ofstream.close();
    }

    void logParams() {
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
            swarmJson["num"] = n;
            for (auto &robot: robots) {
                json robotJson;
                robotJson["id"] = robot.id;
                robotJson["stateEncode"] = robot.state.stateEncodeJson();
                swarmJson["robots"].push_back(robotJson);
            }
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
        data["para"] = paraJson;
        data["config"] = config;
    }

    void logOnce() {
        stepData["runtime"] = runtime;
        {
            json robotsJson = json::array();
            for (auto &robot: robots) {
                json robotJson = {{"state", robot.state.toJson()}, {"id", robot.id}};
                if (!robot.cbfNoSlack.cbfs.empty()) {
                    json cbfNoSlackJson;
                    for (auto &[name, cbf]: robot.cbfNoSlack.cbfs) {
                        cbfNoSlackJson[cbf.name] = cbf.h(robot.state.X, runtime);
                    }
                    cbfNoSlackJson[robot.cbfNoSlack.getName()] = robot.cbfNoSlack.h(robot.state.X, runtime);
                    robotJson["cbfNoSlack"] = cbfNoSlackJson;
                }
                {
                    json cbfSlackJson;
                    for (auto &[name, cbf]: robot.cbfSlack) {
                        cbfSlackJson[cbf.name] = cbf.h(robot.state.X, runtime);
                    }
                    robotJson["cbfSlack"] = cbfSlackJson;
                }

                if (config["cbfs"]["with-slack"]["cvt"]) {
                    json cvtJson = {{"num", cvt.pl[robot.id].n + 1}};
                    for (int i = 1; i <= cvt.pl[robot.id].n; i++) {
                        cvtJson["pos"].push_back({cvt.pl[robot.id].p[i].x, cvt.pl[robot.id].p[i].y});
                    }
                    cvtJson["pos"].push_back({cvt.pl[robot.id].p[1].x, cvt.pl[robot.id].p[1].y});
                    cvtJson["center"] = {cvt.ct[robot.id].x, cvt.ct[robot.id].y};
                    robotJson["cvt"] = cvtJson;
                }

                robotsJson.emplace_back(robotJson);
            }

            stepData["robots"] = robotsJson;
        }
        for (auto i: world.targets) {
            if (i.visibleAtTime(runtime)) {
                stepData["targets"].push_back({
                                                      {"x", i.pos(runtime).x},
                                                      {"y", i.pos(runtime).y},
                                                      {"k", i.densityParams["k"]},
                                                      {"r", i.densityParams["r"]}
                                              });
            }
        }
        stepData["update"] = updatedGridWorld;
        updatedGridWorld.clear();
        data["state"].push_back(stepData);
        stepData.clear();
    }

    void stepTimeForward(double dt) {
        for (auto &robot: robots) {
            VectorXd u{3};
            u << 0, 0, 0;
            robot.stepTimeForward(u, runtime, dt, world);
        }
        runtime += dt;
    }

    void calCVT() {
        cvt = CVT(n, world.boundary);
        for (auto &robot: robots) {
            cvt.pt[robot.id] = robot.state.xy();
        }
        cvt.cal_poly();
        for (int i = 1; i <= cvt.n; i++) {
            cvt.ct[i] = gridWorld.getCentroidInPolygon(cvt.pl[i]);
        }
    }

    void setCVTCBF() {
        for (auto &robot: robots) {
            double kp = 5.0;
            CBF cvtCBF;
            cvtCBF.controlVariable.resize(robot.state.X.size());
            cvtCBF.controlVariable << 1, 1, 1, 1;
            cvtCBF.name = "cvtCBF";
            Point cvtCenter = cvt.ct[robot.id];
            cvtCBF.h = [cvtCenter, kp](VectorXd x, double t) {
                State state(x);
                Point myPosition = state.xy();
                return -kp * cvtCenter.distance_to(myPosition);
            };
            cvtCBF.alpha = [](double h) { return h; };
            robot.cbfSlack[cvtCBF.name] = cvtCBF;
        }
    }

    void cvtForward(double dt) {
        json optimisationData = {};
        for (auto &robot: robots) {
            VectorXd u;
            u.resize(robot.state.X.size());
            u.setZero();
            json robotData = robot.stepTimeForward(u, runtime, dt, world);
            optimisationData += robotData;
        }
        for (auto &j: stepData["robots"]) {
            j["opt"] = optimisationData[j["id"]];
        }
        runtime += dt;
    }

    std::vector<Point> getPositions() {
        std::vector<Point> positions;
        for (auto &robot: robots) {
            positions.push_back(robot.state.xy());
        }
        return positions;
    }

    void getXLimit(double _x[], double inflation = 1.2) {
        world.boundary.get_x_limit(_x, inflation);
    }

    void getYLimit(double *_y, double inflation = 1.2) {
        world.boundary.get_y_limit(_y, inflation);
    }

    void updateGridWorld() {
        updatedGridWorld = json::array();
        for (auto &robot: robots) {
            double tol = 2;
            Point visibleBoundary[4] = {
                    {robot.state.x() - tol, robot.state.y() - tol},
                    {robot.state.x() + tol, robot.state.y() - tol},
                    {robot.state.x() + tol, robot.state.y() + tol},
                    {robot.state.x() - tol, robot.state.y() + tol}
            };
//            updatedGridWorld.push_back(gridWorld.setValueInPolygon(Polygon(4, visibleBoundary), true, true));
            updatedGridWorld.push_back(gridWorld.setValueInCircle(robot.state.xy(), tol, true, true));
        }
    }

    void gridWorldOutput() {
        system("clear");
//    std::cout << "\033[2J\033[H";
        std::vector<std::vector<char> > charMap;
        int yNum = 60, xNum = 150;
        charMap.resize(yNum);
        for (auto &a: charMap) {
            a.resize(xNum);
        }
        for (int j = yNum - 1; j >= 0; j--) {
            for (int i = 0; i < xNum; i++) {
                charMap[j][i] = (gridWorld.getValue(i * gridWorld.xNum / xNum, j * gridWorld.yNum / yNum) == true)
                                ? ' ' : '.';
            }
        }
        for (auto &robot: robots) {
            charMap[gridWorld.getNumInYLim(robot.state.y()) * yNum / gridWorld.yNum]
            [gridWorld.getNumInXLim(robot.state.x()) * xNum / gridWorld.xNum] = 'a' + robot.id - 1;
        }
        for (int j = yNum - 1; j >= 0; j--) {
            for (int i = 0; i < xNum; i++) {
                if (isalpha(charMap[j][i])) {
                    std::cout << RED;
                    printf("%c", charMap[j][i]);
                    std::cout << GREEN;
                } else {
                    printf("%c", charMap[j][i]);
                }
            }
            printf("\n");
        }
    }

    void presetCBF() {
        auto cbfConfig = config["cbfs"];
        if (cbfConfig["without-slack"]["energy"]) setEnergyCBF();
        if (cbfConfig["with-slack"]["yaw"]) setYawCBF();
    }

    void initLog() {
        mkdir("../data", 0777);
        time_t now = time(nullptr);
        tm *t = localtime(&now);
        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(2) << t->tm_mon + 1 << "-"
            << std::setw(2) << t->tm_mday << "_"
            << std::setw(2) << t->tm_hour << "-"
            << std::setw(2) << t->tm_min;
        folderName = oss.str();
        if (mkdir(("../data/" + folderName).c_str(), 0777) == -1) {
            std::cerr << "Error :  " << strerror(errno) << std::endl;
        }
        filename = "../data/" + folderName + "/data.json";
        ofstream.open(filename, std::ios::app);
    }

    void postsetCBF() {
        auto cbfConfig = config["cbfs"];
        if (cbfConfig["without-slack"]["communicationFixed"]) setCommunicationFixedCBF();
        if (cbfConfig["without-slack"]["communicationAuto"]) setCommunicationAutoCBF();
        if (cbfConfig["with-slack"]["cvt"]) calCVT(), setCVTCBF();
        if (cbfConfig["without-slack"]["safety"]) setSafetyCBF();
    }

    void run() {
        initLog();
        logParams();
        output();
        presetCBF();

        auto settings = config["execute"];

        double tTotal = settings["tTotal"], tStep = settings["tStep"];
        while (runtime < tTotal) {
            if (!settings["gridInTerminal"]) {
                printf("\r%.2lf seconds elapsed...", runtime);
            } else {
                printf("%.2lf seconds elapsed...\n", runtime);
            }
            updateGridWorld();
            postsetCBF();
            logOnce();
            cvtForward(tStep);
            if (settings["gridInTerminal"]) gridWorldOutput();
        }

        printf("\nAfter %.4lf seconds\n", tTotal);
        output();
        endLog();

        printf("Data saved in %s\n", filename.c_str());

    }
};


#endif //CBF_SWARM_HPP