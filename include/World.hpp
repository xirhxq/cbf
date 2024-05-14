#ifndef CBF_WORLD_HPP
#define CBF_WORLD_HPP

#include <utility>

#include "utils.h"
#include "Target.hpp"

class World {
public:
    Polygon boundary;
    std::vector<std::pair<Point, double>> chargingStations;
    std::vector<Target> targets;
//    std::vector<std::function<Point (double)>> target_pos;
//    std::vector<std::pair<std::function<double (Point, double)>, std::pair<double, double>>> dens;

public:
    World() {}

    World(Polygon boundary) : boundary(std::move(boundary)) {}

    World(Polygon boundary, std::vector<Point> chargingStationsPosition) : boundary(std::move(boundary)) {
        for (auto _c: chargingStationsPosition) {
            chargingStations.emplace_back(std::make_pair(_c, 0.3));
        }
    }

    explicit World(json settings) {
        boundary = Polygon(getPointsFromJson(settings["boundary"]));
        for (auto &c: settings["charge"]) {
            chargingStations.emplace_back(
                    Point(c[0], c[1]),
                    c.contains("r") ? double(c["r"]) : 0.3
            );
        }
    }

    Point getRandomPoint() {
        return boundary.get_random_point();
    }

    int nearestChargingStation(Point point) {
        assert(!chargingStations.empty());
        double minDistance = point.distance_to(chargingStations[0].first);
        int id = 0;
        for (int i = 1; i < chargingStations.size(); i++) {
            if (point.distance_to(chargingStations[i].first) < minDistance) {
                minDistance = point.distance_to(chargingStations[i].first);
                id = i;
            }
        }
        return id;
    }

    double distanceToChargingStations(Point point) {
        return point.distance_to(chargingStations[nearestChargingStation(Point(point))].first);
    }

    bool isCharging(Point point) {
        for (int i = 0; i < chargingStations.size(); i++) {
            if (point.distance_to(chargingStations[i].first) <= chargingStations[i].second) return true;
        }
        return false;
    }

    std::function<double(Point)> getDensity(double t) {
        return [=](Point p_) {
            double res = eps;
            for (auto target: targets) {
                if (target.visibleAtTime(t)) {
                    res += exp((
                                       pow(-fabs((p_ - target.pos(t)).len() - target.densityParams["r"]), 3)
                                       + 2
                               ) * 10);
                }
            }
            return res;
        };
    }


    Point loop(double xMin, double xMax, double yMin, double yMax, double t, double v, double bias = 0.0) {
        t += bias;
        double dxt = (xMax - xMin) / v, dyt = (yMax - yMin) / v;
        double dt = 2 * dxt + 2 * dyt;
        t += dt;
        while (t >= dt) t -= dt;
        if (t < dxt) {
            return {xMin + v * t, yMin};
        } else if (t < dxt + dyt) {
            return {xMax, yMin + (t - dxt) * v};
        } else if (t < 2 * dxt + dyt) {
            return {xMax - (t - dxt - dyt) * v, yMax};
        } else {
            return {xMin, yMax - (t - 2 * dxt - dyt) * v};
        }
    }

};


#endif //CBF_WORLD_HPP
