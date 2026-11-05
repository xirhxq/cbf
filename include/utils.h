#ifndef CBF_UTILS_H
#define CBF_UTILS_H

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include <set>
#include <iomanip>
#include <iostream>
#include <utility>
#include <sys/stat.h>

#include "nlohmann/json.hpp"
#include "Eigen/Dense"

#include "ComputingGeometry/ComputingGeometry"


using namespace Eigen;
using json = nlohmann::json;

typedef std::pair<double, double> pd;

inline unsigned int resolveRandomSeed(const json &settings, unsigned int fallback) {
    const json execute = settings.value("execute", json::object());
    return execute.value("random-seed", fallback);
}

inline unsigned int seedRandomFromConfig(const json &settings, unsigned int fallback) {
    const unsigned int seed = resolveRandomSeed(settings, fallback);
    srand(seed);
    return seed;
}

std::vector<Point> getPointsFromJson(const json &j) {
    std::vector<Point> points;
    for (auto &point : j) {
        points.emplace_back(point[0], point[1]);
    }
    return points;
}

std::string vecToString(const VectorXd& vec) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << "(";
    for (int i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i < vec.size() - 1) {
            oss << ", ";
        }
    }
    oss << ")";
    return oss.str();
}

#endif //CBF_UTILS_H
