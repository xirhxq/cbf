//
// Created by xirhxq on 2022/3/7.
//

#ifndef CBF_MAIN_CBF_H
#define CBF_MAIN_CBF_H

#include "computing_geometry/Line.h"

class CBF{
public:
    double delta = 0.001;
    std::function<double(double)> alpha = [](double _h) {return 0.1 * pow(_h, 3);};
    std::function<double(VectorXd, double)> h;

public:
    CBF();

    double dh(VectorXd, double, int);

    double dhdt(VectorXd, double);
    VectorXd dhdx(VectorXd, double);
    VectorXd constraint_u_coe(VectorXd&, MatrixXd&, VectorXd&, double);
    double constraint_const(VectorXd&, MatrixXd&, VectorXd&, double);
};

#endif //CBF_MAIN_CBF_H
