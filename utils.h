// Created by XIRHXQ on 12/29/21.
#ifndef CBF_MAIN_UTILS_H
#define CBF_MAIN_UTILS_H

#include <iostream>
#include <cmath>
#include "nlohmann/json.hpp"
#include "Eigen/Dense"
#include "gurobi_c++.h"

using namespace Eigen;
using json = nlohmann::json;

typedef std::pair<double, double> pd;

const double eps = 1e-8;
const double inf = 1e20;
const double pi = acos(-1.0);
const int maxp = 110;
const int mat_max = 15;

inline int index_before(int n, int id, int d){
    int res = id - (d % n);
    if (res < 1) res += n;
    return res;
}

inline int index_after(int n, int id, int d){
    int res = id + (d % n);
    if (res > n) res -= n;
    return res;
}

inline int sgn(double val){
    if (fabs(val) < eps) return 0;
    if (val < 0) return -1;
    else return 1;
}

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#endif //CBF_MAIN_UTILS_H
