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

#define RESET   "\033[0m"
#define BLACK   "\033[30m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"
#define BOLDBLACK   "\033[1m\033[30m"
#define BOLDRED     "\033[1m\033[31m"
#define BOLDGREEN   "\033[1m\033[32m"
#define BOLDYELLOW  "\033[1m\033[33m"
#define BOLDBLUE    "\033[1m\033[34m"
#define BOLDMAGENTA "\033[1m\033[35m"
#define BOLDCYAN    "\033[1m\033[36m"
#define BOLDWHITE   "\033[1m\033[37m"

#endif //CBF_UTILS_H
