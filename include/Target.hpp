#ifndef CBF_MAIN_TARGET_HPP
#define CBF_MAIN_TARGET_HPP

#include "utils.h"

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
    Target() {
        type = TargetType::Static;
        pos = [](double _t){
            return Point(0, 0);
        };
        vis_time = {{0, inf}};
        den_para = {{"x", 0.0}, {"y", 0.0}};
    }

    bool visible(double _t) {
        for (auto v: vis_time){
            if (_t >= v.first && _t <= v.second) return true;
        }
        return false;
    }

    double time_gap_to_vis(double _t) {
        double res = fabs(_t - vis_time[0].first);
        for (auto v: vis_time){
            res = std::min(res, fabs(_t - v.first));
            res = std::min(res, fabs(_t - v.second));
        }
        return res;
    }

    Target make_static_target(Point _p) {
        Target res;
        res.pos = [_p](double _t){
            return Point(_p.x, _p.y);
        };
        res.den_para = {{"x", _p.x},
                        {"y", _p.y},
                        {"k", 10},
                        {"r", 3}};
        return res;
    }

    Point loop_rect(double x_min, double x_max, double y_min, double y_max, double t, double v, double bias = 0.0){
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

    Target make_loop_rectangle_target(Point _min, Point _max, double _v, double _bias = 0.0) {
        Target res;
        res.type = TargetType::LoopRect;
        res.pos = [=] (double t_) {
            return loop_rect(_min.x, _max.x,
                             _min.y, _max.y,
                             t_, _v, _bias);
        };
        res.den_para = {
                {"k", 10},
                {"r", 1.0}
        };
        return res;
    }
};


#endif //CBF_MAIN_TARGET_HPP
