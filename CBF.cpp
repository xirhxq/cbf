//
// Created by xirhxq on 2022/3/7.
//

#include "CBF.h"

#include <utility>

CBF::CBF() {
}

//double CBF::h(VectorXd _v) {
//    return _v(2) - log(dist_to_charge_place(Point(_v)) / charge_dist[nearest_charge_place(Point(_v))]);
//}

double CBF::dh(VectorXd _x, double _t, int _i){
    VectorXd x_plus_dx = _x, x_minus_dx = _x;
    x_plus_dx(_i) += delta;
    x_minus_dx(_i) -= delta;
    return (h(x_plus_dx, _t) - h(x_minus_dx, _t)) / 2.0 / delta;
}

double CBF::dhdt(VectorXd _x, double _t) {
    return (h(_x, _t + delta) - h(_x, _t - delta)) / 2.0 / delta;
}

VectorXd CBF::dhdx(VectorXd _x, double _t) {
    VectorXd res = _x;
    for (int i = 0; i < _x.size(); i++){
        res(i) = dh(_x, _t, i);
    }
    return res;
}

VectorXd CBF::constraint_u_coe(VectorXd& _f, MatrixXd& _g, VectorXd& _x, double _t) {
    return dhdx(_x, _t).transpose() * _g;
}

double CBF::constraint_const(VectorXd & _f, MatrixXd & _g, VectorXd & _x, double _t) {
//    printf("h: %lf\tdhdt: %lf\t dhdx.dot(_f): %lf\talpha(h): %lf\n", h(_x, _t),
//           dhdt(_x, _t), dhdx(_x, _t).dot(_f), alpha(h(_x, _t)));
    return dhdt(_x, _t) + dhdx(_x, _t).dot(_f) + alpha(h(_x, _t));
}
