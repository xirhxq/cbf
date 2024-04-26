#ifndef CBF_MAIN_SWARM_HPP
#define CBF_MAIN_SWARM_HPP

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
    Robot robots[maxp];
    World world;
    double spacing = 0.1;
    std::ofstream ofstream;
    json data;
    CVT cvt;
    GridWorld gridWorld;
    json updatedGridWorld;
public:
    Swarm(int n, Point *initialPosition, World world) : n(n), world(world) {
        for (int i = 1; i <= n; i++) {
            robots[i] = Robot(3);
            robots[i].id = i;
            robots[i].X << initialPosition[i - 1].x, initialPosition[i - 1].y, 1.0 * (rand() % 40) / 40 + 10;
            robots[i].F << 0, 0, -1;
        }
        auto worldXLimit = world.boundary.get_x_limit(1.0), worldYLimit = world.boundary.get_y_limit(1.0);
        int xNum = (worldXLimit.second - worldXLimit.first) / spacing;
        int yNum = (worldYLimit.second - worldYLimit.first) / spacing;
        gridWorld = GridWorld(worldXLimit, xNum, worldYLimit, yNum);
        runtime = 0.0;
    }

    Swarm(int n, World world) : n(n), world(world) {
        auto worldXLimit = world.boundary.get_x_limit(1.0), worldYLimit = world.boundary.get_y_limit(1.0);
        int xNum = (worldXLimit.second - worldXLimit.first) / spacing;
        int yNum = (worldYLimit.second - worldYLimit.first) / spacing;
        gridWorld = GridWorld(worldXLimit, xNum, worldYLimit, yNum);
        runtime = 0.0;
        for (int i = 1; i <= n; i++) {
            robots[i] = Robot(4);
            robots[i].id = i;
            robots[i].G(2, 2) = 0;
            robots[i].G(3, 3) = 1;
            robots[i].setBattery(20.0 * (rand() % 100) / 100 + 10);
            robots[i].X(3) = pi;
//        r[i].set_battery(i * 10 + 10);
//        r[i].F << 0, 0, -1;
            robots[i].F << 0, 0, -1, 0;
            robots[i].cbfSlack.clear();
            robots[i].cbfNoSlack.clear();
        }
        randomInitialPosition();
    }

    void output() {
        printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, runtime);
        for (int i = 1; i <= n; i++) {
            printf("Robot #%d: ", i);
            robots[i].output();
        }
        printf("--------------\n");
    }

    void randomInitialPosition() {
        for (int i = 1; i <= n; i++) {
            robots[i].setPosition(world.getRandomPoint());
        }
    }

    void randomInitialPosition(Polygon poly) {
        for (int i = 1; i <= n; i++) {
            robots[i].setPosition(poly.get_random_point());
        }
    }

    void setInitialPosition(std::vector<Point> initialPositions) {
        for (int i = 1; i <= n; i++) {
            robots[i].setPosition(initialPositions[i - 1]);
        }
    }

    void setEnergyCBF() {
        std::function<double(Point, World)> minDistanceToChargingStations = [=](Point myPosition, World world) {
            return (
                    world.distanceToChargingStations(myPosition)
                    / world.chargingStations[world.nearestChargingStation(myPosition)].second
            );
        };
        auto robotDistanceToBase = [=](int id) {
            return robots[id].xy().len();
        };
        auto partId = [=](int id) { return (id - 1) % (n / 2) + 1; };
        auto isSecondPart = [=](int x) { return x > (n / 2); };
        for (int i = 1; i <= n; i++) {
            auto autoFormationCommH = [=](VectorXd state, double t) {
                std::vector<int> idsInCommRange;
                for (int j = 1; j <= n; j++) {
                    if (i == j) continue;
                    if (robots[i].xy().distance_to(robots[j].xy()) > 8) continue;
                    idsInCommRange.push_back(j);
                }
                std::sort(idsInCommRange.begin(), idsInCommRange.end(), [=](int a, int b) {
                    return robotDistanceToBase(a) < robotDistanceToBase(b);
                });
                double h = inf;
                int indexOfI = std::distance(
                        idsInCommRange.begin(),
                        std::find(idsInCommRange.begin(), idsInCommRange.end(), i)
                );
                if (indexOfI < 2) {
                    h = std::min(
                            h,
                            0.5 * (8 - Point(
                                    state(robots[i].xIndex),
                                    state(robots[i].yIndex)
                            ).distance_to(Point(0, 0))
                            ));
                }
                for (int id = std::max(0, indexOfI - 2); id < indexOfI; id++) {
                    int j = idsInCommRange[id];
                    h = std::min(
                            h,
                            0.5 * (8 - Point(
                                    state(robots[i].xIndex),
                                    state(robots[i].yIndex))
                                    .distance_to(robots[j].xy())
                            ));
                }
                return h;
            };
            auto fixedFormationCommH = [=](VectorXd state, double t) {
                double h = inf;
                double maxCommDistance = 8.5;
                int idInPart = partId(i);
                if (idInPart == 1) {
                    h = std::min(
                            h,
                            0.5 * (maxCommDistance - Point(
                                    state(robots[i].xIndex),
                                    state(robots[i].yIndex))
                                    .distance_to(Point(-3 + 6 * isSecondPart(i), 0))
                            ));
                }
                if (idInPart <= 2) {
                    h = std::min(
                            h,
                            0.5 * (maxCommDistance - Point(
                                    state(robots[i].xIndex),
                                    state(robots[i].yIndex))
                                    .distance_to(Point(0, 0))
                            ));
                }
                for (int id = std::max(1, idInPart - 2); id <= idInPart + 2 && id <= n / 2; id++) {
                    if (id <= idInPart) continue;
                    h = std::min(
                            h,
                            0.5 * (maxCommDistance - Point(
                                    state(robots[i].xIndex),
                                    state(robots[i].yIndex))
                                    .distance_to(robots[id + isSecondPart(i) * n / 2].xy())
                            ));
                }
                return h;
            };
            auto batteryH = [=](VectorXd state, double t) {
                double h = inf;
                h = std::min(
                        h,
                        state(robots[i].batteryIndex) -
                        log(minDistanceToChargingStations(
                                Point(
                                        state(robots[i].xIndex),
                                        state(robots[i].yIndex)
                                ),
                                world)
                        ));
                return h;
            };
            auto safetyH = [=](VectorXd state, double t) {
                double h = inf;
                for (int j = 1; j <= n; j++) {
                    if (i == j) continue;
                    h = std::min(
                            h,
                            0.5 * (Point(
                                    state(robots[i].xIndex),
                                    state(robots[i].yIndex)
                            ).distance_to(robots[j].xy()) - 3)
                    );
                }
                return h;
            };
            auto energy_h = [=](VectorXd state, double t) {
                double h = inf;
                h = std::min(h, batteryH(state, t));
                return h;
            };
            CBF energyCBF;
            energyCBF.name = "energyCBF";
            energyCBF.h = energy_h;
            energyCBF.alpha = [](double _h) { return _h; };
            energyCBF.controlVariable.resize(robots[i].X.size());
            energyCBF.controlVariable << 1, 1, 1, 1;
            robots[i].cbfNoSlack["energy_cbf"] = energyCBF;
        }
    }

    void setYawCBF() {
        std::function<double(Point, double)> headingToTarget = [=](Point myPosition, double t) {
            double res = -1, headingRad = 0.0;
            for (auto i: world.targets) {
                if (i.visibleAtTime(t)) {
                    if (res == -1 || myPosition.distance_to(i.pos(t)) < res) {
                        res = myPosition.distance_to(i.pos(t));
                        headingRad = myPosition.angle_to(i.pos(t));
                    }
                }
            }
            return headingRad;
        };
        for (int i = 1; i <= n; i++) {
            std::function<double(VectorXd, double)> cameraAngle = [=](VectorXd x, double t) {
                Point pos{x(robots[i].xIndex), x(robots[i].yIndex)};
                double res = -1, dYawRad = 0.0;
                for (auto tar: world.targets) {
                    if (!tar.visibleAtTime(t)) continue;
                    double dis = pos.distance_to(tar.pos(t));
//                if (dis < 0.5) continue;
                    if (res == -1 || dis < res) {
                        res = dis;
                        dYawRad = x(robots[i].cameraIndex) - pos.angle_to(tar.pos(t));
                    }
                }
//            double delta_angle = _x(r[i].camera_ord)
//                                 - angle_to_center(Point(_x(r[i].x_ord),
//                                                     _x(r[i].y_ord)),
//                                                   _t);
                if (dYawRad <= -pi) dYawRad += 2 * pi;
                else if (dYawRad >= pi) dYawRad -= 2 * pi;
//            std::cout << "delta_angle = " << delta_angle << std::endl;
                return -5 * abs(dYawRad);
            };
            CBF yawCBF;
            yawCBF.name = "yawCBF";
            yawCBF.h = cameraAngle;
            yawCBF.controlVariable.resize(robots[i].X.size());
            yawCBF.controlVariable << 0, 0, 0, 1;
//        camera_cbf.ctrl_var << 1, 1, 1, 1;
//        camera_cbf.alpha = [](double _h) {return _h;};
            robots[i].cbfSlack["camera_cbf"] = yawCBF;

        }
    }

    void setCommCBF() {
        CBF commCBF;
        commCBF.controlVariable.resize(robots[1].X.size());
        commCBF.controlVariable << 1, 1, 1, 1;

        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= n; j++) {
                if (i == j) continue;
                if ((i > 3) != (j > 3)) continue;
                commCBF.name = "CommTo" + std::to_string(j);
                commCBF.h = [=](VectorXd _x, double _t) {
                    return 2 - Point(_x(robots[i].xIndex), _x(robots[i].yIndex)).distance_to(robots[j].xy());
                };
                robots[i].cbfSlack[commCBF.name] = commCBF;
            }
        }
    }

    void setSafetyCBF() {
        CBF safetyCBF;
        safetyCBF.controlVariable.resize(robots[1].X.size());
        safetyCBF.controlVariable << 1, 1, 1, 1;

        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= n; j++) {
                if (i == j) continue;
                safetyCBF.name = "SafetyTo" + std::to_string(j);
                safetyCBF.h = [=](VectorXd x, double t) {
                    return 5.0 * (Point(x(robots[i].xIndex), x(robots[i].yIndex)).distance_to(robots[j].xy()) - 3);
                };
                robots[i].cbfSlack[safetyCBF.name] = safetyCBF;
            }
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
        json tmp_j;
        tmp_j["runtime"] = runtime;
        for (int i = 1; i <= n; i++) {
            tmp_j["robot"][i - 1] = {
                    {"x",      robots[i].x()},
                    {"y",      robots[i].y()},
                    {"batt",   robots[i].batt()},
                    {"camera", robots[i].camera()}
            };
            for (auto cbf: robots[i].cbfNoSlack) {
                auto h = cbf.second;
                tmp_j["robot"][i - 1][cbf.first] = h.h(robots[i].X, runtime);
            }
            for (auto cbf: robots[i].cbfSlack) {
                auto h = cbf.second;
                tmp_j["robot"][i - 1][cbf.first] = h.h(robots[i].X, runtime);
            }
            tmp_j["cvt"][i - 1]["num"] = cvt.pl[i].n + 1;
            for (int j = 1; j <= cvt.pl[i].n; j++) {
                tmp_j["cvt"][i - 1]["pos"].push_back({{"x", cvt.pl[i].p[j].x},
                                                      {"y", cvt.pl[i].p[j].y}});
            }
            tmp_j["cvt"][i - 1]["pos"].push_back({{"x", cvt.pl[i].p[1].x},
                                                  {"y", cvt.pl[i].p[1].y}});
            tmp_j["cvt"][i - 1]["center"] = {{"x", cvt.ct[i].x},
                                             {"y", cvt.ct[i].y}};
        }
        for (auto i: world.targets) {
            if (i.visibleAtTime(runtime)) {
                tmp_j["target"].push_back({
                                                  {"x", i.pos(runtime).x},
                                                  {"y", i.pos(runtime).y},
                                                  {"k", i.densityParams["k"]},
                                                  {"r", i.densityParams["r"]}
                                          });
            }
        }
        tmp_j["update"] = updatedGridWorld;
        data["state"].push_back(tmp_j);
    }

    void stepTimeForward(double dt) {
        for (int i = 1; i <= n; i++) {
            VectorXd u{3};
            u << 0, 0, 0;
            robots[i].stepTimeForward(u, runtime, dt, world);
        }
        runtime += dt;
    }

    void calCVT() {
        cvt = CVT(n, world.boundary);
        for (int i = 1; i <= n; i++) {
            cvt.pt[i] = robots[i].xy();
        }
        cvt.cal_poly();
//    c.cal_centroid([=](const Point &_p) {
//        return wd.get_dens(runtime)(_p);
//    }, spacing);
        for (int i = 1; i <= cvt.n; i++) {
            cvt.ct[i] = gridWorld.getCentroidInPolygon(cvt.pl[i]);
        }
    }

    void setCVTCBF() {
        for (int i = 1; i <= n; i++) {
            CBF cvtCBF;
            cvtCBF.controlVariable.resize(robots[i].X.size());
            cvtCBF.controlVariable << 1, 1, 1, 1;
            cvtCBF.name = "cvt_cbf";
            cvtCBF.h = [=](VectorXd x, double t) {
                return -5.0 * cvt.ct[i].distance_to(Point(x(robots[i].xIndex), x(robots[i].yIndex)));
            };
            cvtCBF.alpha = [](double h) { return h; };
            robots[i].cbfSlack[cvtCBF.name] = cvtCBF;
        }
    }

    void cvtForward(double _t) {
        json optimisationData;
        for (int i = 1; i <= n; i++) {
            Point up = ((cvt.ct[i] - robots[i].xy()) * 5).saturation(1.0);
            VectorXd u;
            u.resize(robots[i].X.size());
            u.setZero();
//        u(r[i].x_ord) = up.x;
//        u(r[i].y_ord) = up.y;
            json robotData = robots[i].stepTimeForward(u, runtime, _t, world);
            optimisationData.push_back(robotData);
        }
        int sz = data["state"].size();
        data["state"][sz - 1]["opt"] = optimisationData;
        runtime += _t;
    }

    std::vector<Point> getPositions() {
        std::vector<Point> positions;
        for (int i = 1; i <= n; i++) {
            positions.push_back(robots[i].xy());
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
        for (int i = 1; i <= n; i++) {
            double tol = 2;
            Point visibleBoundary[4] = {
                    {robots[i].x() - tol, robots[i].y() - tol},
                    {robots[i].x() + tol, robots[i].y() - tol},
                    {robots[i].x() + tol, robots[i].y() + tol},
                    {robots[i].x() - tol, robots[i].y() + tol}
            };
//            updatedGridWorld.push_back(gridWorld.setValueInPolygon(Polygon(4, visibleBoundary), true, true));
            updatedGridWorld.push_back(gridWorld.setValueInCircle(robots[i].xy(), tol, true, true));
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
        for (int i = 1; i <= n; i++) {
            charMap[gridWorld.getNumInYLim(robots[i].y()) * yNum / gridWorld.yNum]
            [gridWorld.getNumInXLim(robots[i].x()) * xNum / gridWorld.xNum] = 'a' + i - 1;
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


#endif //CBF_MAIN_SWARM_HPP