#ifndef CBF_UTILS_H
#define CBF_UTILS_H

#include <cmath>
#include <fstream>
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
