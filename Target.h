//
// Created by xirhxq on 2022/9/13.
//

#ifndef CBF_MAIN_TARGET_H
#define CBF_MAIN_TARGET_H

#include "utils.h"
#include "computing_geometry/Point.h"

enum class TargetType{
    Static,
    LoopRect,
};

class Target {
public:
    TargetType type;
    std::vector<std::pair<double ,double>> vis_time;
    std::function<Point (double)> pos;
    std::map<std::string, double> den_para;

    Target();
    void set_visible_time(double);
    bool visible(double);

    static Target make_static_target(Point _p);
    static Target make_loop_rectangle_target(Point _min, Point _max, double _v, double _bias = 0.0);
};


#endif //CBF_MAIN_TARGET_H
