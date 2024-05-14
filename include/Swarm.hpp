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
    double spacing = 0.1;
    std::ofstream ofstream;
    json data;
    CVT cvt;
    GridWorld gridWorld;
    json updatedGridWorld;
public:
    Swarm(int n, Point *initialPosition, World world) : n(n), world(world), robots(n, Robot(3)) {
        for (int i = 0; i < n; i++) {
            robots[i].id = i + 1;
            robots[i].state.setPosition(initialPosition[i]);
            robots[i].state.setBattery(1.0 * (rand() % 40) / 40 + 10);
            robots[i].F << 0, 0, -1;
        }
        auto worldXLimit = world.boundary.get_x_limit(1.0), worldYLimit = world.boundary.get_y_limit(1.0);
        int xNum = (worldXLimit.second - worldXLimit.first) / spacing;
        int yNum = (worldYLimit.second - worldYLimit.first) / spacing;
        gridWorld = GridWorld(worldXLimit, xNum, worldYLimit, yNum);
        runtime = 0.0;
    }

    Swarm(int n, World world) : n(n), world(world), robots(n, Robot(4)) {
        auto worldXLimit = world.boundary.get_x_limit(1.0), worldYLimit = world.boundary.get_y_limit(1.0);
        int xNum = (worldXLimit.second - worldXLimit.first) / spacing;
        int yNum = (worldYLimit.second - worldYLimit.first) / spacing;
        gridWorld = GridWorld(worldXLimit, xNum, worldYLimit, yNum);
        runtime = 0.0;
        for (int i = 0; i < n; i++) {
            robots[i].id = i + 1;
            robots[i].G(2, 2) = 0;
            robots[i].G(3, 3) = 1;
            robots[i].state.setBattery(20.0 * (rand() % 100) / 100 + 10);
            robots[i].state.setYawDeg(180);
            robots[i].F << 0, 0, -1, 0;
            robots[i].cbfSlack.clear();
            robots[i].cbfNoSlack.clear();
        }
        randomInitialPosition();
    }

    void output() {
        printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, runtime);
        for (auto &robot: robots) {
            printf("Robot %d: (%.4lf, %.4lf, %.4lf, %.4lf)\n", robot.id, robot.state.x(), robot.state.y(),
                   robot.state.battery(), robot.state.yawDeg());
        }
        printf("--------------\n");
    }

    void randomInitialPosition() {
        for (auto &robot: robots) {
            robot.state.setPosition(world.getRandomPoint());
        }
    }

    void randomInitialPosition(Polygon poly) {
        for (auto &robot: robots) {
            robot.state.setPosition(poly.get_random_point());
        }
    }

    void setInitialPosition(std::vector<Point> initialPositions) {
        for (int i = 0; i < n; i++) {
            robots[i].state.setPosition(initialPositions[i]);
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
            robot.cbfNoSlack["energyCBF"] = energyCBF;
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
            robot.cbfNoSlack["yawCBF"] = yawCBF;
        }
    }

    void setCommCBF() {
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

                std::vector<Point> formationPoints;
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
            robot.cbfNoSlack["commCBF"] = commCBF;
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
            robot.cbfNoSlack["safetyCBF"] = safetyCBF;
        }
    }

    void initLogPath(char *_p) {
        ofstream.open(_p, std::ios::app);
    }

    void endLog() {
        ofstream << std::fixed << std::setprecision(6) << data;
        ofstream.close();
    }

    void logParams() {
        data["para"]["number"] = n;
        double x_lim[2], y_lim[2];
        getXLimit(x_lim), getYLimit(y_lim);
        data["para"]["lim"]["x"] = {x_lim[0], x_lim[1]};
        data["para"]["lim"]["y"] = {y_lim[0], y_lim[1]};
        getXLimit(x_lim, 1.0), getYLimit(y_lim, 1.0);
        for (int i = 1; i <= world.boundary.n; i++) {
            data["para"]["world"].push_back({
                                                    {"x", world.boundary.p[i].x},
                                                    {"y", world.boundary.p[i].y}
                                            });
        }
        data["para"]["world"].push_back({
                                                {"x", world.boundary.p[1].x},
                                                {"y", world.boundary.p[1].y}
                                        });
        data["para"]["charge"]["num"] = world.chargingStations.size();
        for (auto &i: world.chargingStations) {
            data["para"]["charge"]["pos"].push_back({
                                                            {"x", i.first.x},
                                                            {"y", i.first.y}
                                                    });
            data["para"]["charge"]["dist"].push_back(i.second);
        }
        for (auto &i: world.targets) {
            data["para"]["target"].push_back({
                                                     {"k", i.densityParams["k"]},
                                                     {"r", i.densityParams["r"]}
                                             });
        }
        data["para"]["grid_world"] = {
                {"x_num", gridWorld.xNum},
                {"y_num", gridWorld.yNum},
                {"x_lim", {gridWorld.xLim.first, gridWorld.xLim.second}},
                {"y_lim", {gridWorld.yLim.first, gridWorld.yLim.second}}
        };
    }

    void logOnce() {
        json stepData;
        stepData["runtime"] = runtime;
        for (auto &robot: robots) {
            stepData["robot"].push_back({
                                                 {"x",    robot.state.x()},
                                                 {"y",    robot.state.y()},
                                                 {"battery", robot.state.battery()},
                                                 {"yawRad",  robot.state.yawRad()}
                                         });
            for (auto &cbf: robot.cbfNoSlack) {
                stepData["robot"].back()[cbf.first] = cbf.second.h(robot.state.X, runtime);
            }
            for (auto &cbf: robot.cbfSlack) {
                stepData["robot"].back()[cbf.first] = cbf.second.h(robot.state.X, runtime);
            }
            stepData["cvt"].push_back({
                                              {"num", cvt.pl[robot.id].n + 1}
                                      });
            for (int i = 1; i <= cvt.pl[robot.id].n; i++) {
                stepData["cvt"].back()["pos"].push_back({
                                                                {"x", cvt.pl[robot.id].p[i].x},
                                                                {"y", cvt.pl[robot.id].p[i].y}
                                                        });
            }
            stepData["cvt"].back()["pos"].push_back({
                                                            {"x", cvt.pl[robot.id].p[1].x},
                                                            {"y", cvt.pl[robot.id].p[1].y}
                                                    });
            stepData["cvt"].back()["center"] = {
                    {"x", cvt.ct[robot.id].x},
                    {"y", cvt.ct[robot.id].y}
            };
        }
        for (auto i: world.targets) {
            if (i.visibleAtTime(runtime)) {
                stepData["target"].push_back({
                                                     {"x", i.pos(runtime).x},
                                                     {"y", i.pos(runtime).y},
                                                     {"k", i.densityParams["k"]},
                                                     {"r", i.densityParams["r"]}
                                             });
            }
        }
        stepData["update"] = updatedGridWorld;
        updatedGridWorld = false;
        data["state"].push_back(stepData);
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
//        cvt.cal_centroid([&](const Point &_p) {
//            return world.getDensity(runtime)(_p);
//        }, spacing);
        for (int i = 1; i <= cvt.n; i++) {
            cvt.ct[i] = gridWorld.getCentroidInPolygon(cvt.pl[i]);
        }
    }

    void setCVTCBF() {
#ifdef TIMER_ON
        clock_t begin, end;
        double cvtCBFTime = 0.0;
#endif
        for (auto &robot: robots) {
            double kp = 5.0;
            CBF cvtCBF;
            cvtCBF.controlVariable.resize(robot.state.X.size());
            cvtCBF.controlVariable << 1, 1, 1, 1;
            cvtCBF.name = "cvtCBF";
            Point cvtCenter = cvt.ct[robot.id];
#ifdef TIMER_ON
            begin = clock();
#endif
            cvtCBF.h = [cvtCenter, kp](VectorXd x, double t) {
                State state(x);
                Point myPosition = state.xy();
                return -kp * cvtCenter.distance_to(myPosition);
            };
#ifdef TIMER_ON
            end = clock();
            cvtCBFTime += double(end - begin) / CLOCKS_PER_SEC;
#endif
            cvtCBF.alpha = [](double h) { return h; };
            robot.cbfSlack[cvtCBF.name] = cvtCBF;
        }
#ifdef TIMER_ON
        printf("cvtCBFTime: %f ", cvtCBFTime);
#endif
    }

    void cvtForward(double dt) {
        json optimisationData;
        for (auto &robot: robots) {
            VectorXd u;
            u.resize(robot.state.X.size());
            u.setZero();
            json robotData = robot.stepTimeForward(u, runtime, dt, world);
            optimisationData.push_back(robotData);
        }
        auto sz = data["state"].size();
        data["state"][sz - 1]["opt"] = optimisationData;
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
};


#endif //CBF_SWARM_HPP