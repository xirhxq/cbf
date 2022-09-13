//
// Created by xirhxq on 2022/3/4.
//

#ifndef CBF_MAIN_WORLD_H
#define CBF_MAIN_WORLD_H

#include "utils.h"
#include "computing_geometry/Polygon.h"
#include "Target.h"

class World {
public:
    Polygon w;
    std::vector<std::pair<Point, double>> charge_place;
    std::vector<Target> target;
//    std::vector<std::function<Point (double)>> target_pos;
//    std::vector<std::pair<std::function<double (Point, double)>, std::pair<double, double>>> dens;

public:
    World();
    World(Polygon);
    World(Polygon, std::vector<Point>);

    int nearest_charge_place(Point);
    double dist_to_charge_place(Point);
    bool is_charging(Point);
    Point get_random_point();
    std::function<double (Point)> get_dens(double);

};


#endif //CBF_MAIN_WORLD_H
