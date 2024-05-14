#ifndef CBF_TARGET_HPP
#define CBF_TARGET_HPP

#include "utils.h"

enum class TargetType{
    Static,
    LoopRect,
};

class Target {
public:
    TargetType type;
    std::vector<std::pair<double ,double>> visibleTimeRanges;
    std::function<Point (double)> pos;
    std::map<std::string, double> densityParams;
    Target() {
        type = TargetType::Static;
        pos = [](double t){
            return Point(0, 0);
        };
        visibleTimeRanges = {{0, inf}};
        densityParams = {{"x", 0.0}, {"y", 0.0}};
    }

    bool visibleAtTime(double t) {
        for (auto v: visibleTimeRanges){
            if (t >= v.first && t <= v.second) return true;
        }
        return false;
    }

    double timeToVisible(double t) {
        double res = fabs(t - visibleTimeRanges[0].first);
        for (auto v: visibleTimeRanges){
            res = std::min(res, fabs(t - v.first));
            res = std::min(res, fabs(t - v.second));
        }
        return res;
    }

    static Target makeStaticTarget(Point point) {
        Target res;
        res.pos = [point](double t){
            return Point(point.x, point.y);
        };
        res.densityParams = {{"x", point.x},
                             {"y", point.y},
                             {"k", 10},
                             {"r", 3}};
        return res;
    }

    static Target makeLoopRectangleTarget(Point minPoint, Point maxPoint, double velocity, double bias = 0.0) {
        Target res;
        res.type = TargetType::LoopRect;
        res.pos = [=] (double t) {
            t += bias;
            double dxt = (maxPoint.x - minPoint.x) / velocity, dyt = (maxPoint.y - minPoint.y) / velocity;
            double dt = 2 * dxt + 2 * dyt;
            t += dt;
            while (t >= dt) t -= dt;
            if (t < dxt){
                return Point(minPoint.x + velocity * t, minPoint.y);
            }
            else if (t < dxt + dyt){
                return Point(maxPoint.x, minPoint.y + (t - dxt) * velocity);
            }
            else if (t < 2 * dxt + dyt){
                return Point(maxPoint.x - (t - dxt - dyt) * velocity, maxPoint.y);
            }
            else {
                return Point(minPoint.x, maxPoint.y - (t - 2 * dxt - dyt) * velocity);
            }
        };
        res.densityParams = {
                {"k", 10},
                {"r", 1.0}
        };
        return res;
    }
};


#endif //CBF_TARGET_HPP
