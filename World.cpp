//
// Created by xirhxq on 2022/3/4.
//

#include "World.h"

World::World() {}

World::World(Polygon _w) {
    w = _w;
}

World::World(Polygon _w, std::vector<Point> _v) {
    w = _w;
    for (auto _c: _v){
        charge_place.emplace_back(std::make_pair(_c, 0.3));
    }
}

Point World::get_random_point() {
    return w.get_random_point();
}

int World::nearest_charge_place(Point _p) {
    assert(!charge_place.empty());
    double res = _p.distance_to(charge_place[0].first);
    int id = 0;
    for (int i = 1; i < charge_place.size(); i++){
        if (_p.distance_to(charge_place[i].first) < res){
            res = _p.distance_to(charge_place[i].first);
            id = i;
        }
    }
    return id;
}

double World::dist_to_charge_place(Point _p) {
    return _p.distance_to(charge_place[nearest_charge_place(Point(_p))].first);
}

bool World::is_charging(Point _p) {
    for (int i = 0; i < charge_place.size(); i++){
        if (_p.distance_to(charge_place[i].first) <= charge_place[i].second) return true;
    }
    return false;
}

std::function<double (Point)> World::get_dens(double _t) {
    return [=](Point p_){
        double res = 0.0;
        int vis_cnt = 0;
        for (auto i: target){
            if (i.visible(_t)){
                vis_cnt++;
                res += exp(-fabs((p_ - i.pos(_t)).len() - i.den_para["r"]) * i.den_para["k"]);
            }
        }
        if (vis_cnt > 0) return res;
        else return 1.0;
    };
}


Point loop(double x_min, double x_max, double y_min, double y_max, double t, double v, double bias = 0.0){
    t += bias;
    double dxt = (x_max - x_min) / v, dyt = (y_max - y_min) / v;
    double dt = 2 * dxt + 2 * dyt;
    t += dt;
    while (t >= dt) t -= dt;
    if (t < dxt){
        return {x_min + v * t, y_min};
    }
    else if (t < dxt + dyt){
        return {x_max, y_min + (t - dxt) * v};
    }
    else if (t < 2 * dxt + dyt){
        return {x_max - (t - dxt - dyt) * v, y_max};
    }
    else {
        return {x_min, y_max - (t - 2 * dxt - dyt) * v};
    }
}