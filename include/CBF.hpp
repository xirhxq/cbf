#ifndef CBF_MAIN_CBF_HPP
#define CBF_MAIN_CBF_HPP

//#define CBF_DEBUG

#include "utils.h"
#include <utility>

class CBF{
public:
    std::string name;
    double delta = 0.001;
    std::function<double(double)> alpha = [](double _h) {return 0.1 * pow(_h, 3);};
    std::function<double(VectorXd, double)> h;
    VectorXd ctrl_var;

public:
    CBF(){}

    double dh(VectorXd _x, double _t, int _i){
        VectorXd x_plus_dx = _x, x_minus_dx = _x;
        x_plus_dx(_i) += delta;
        x_minus_dx(_i) -= delta;
        return (h(x_plus_dx, _t) - h(x_minus_dx, _t)) / 2.0 / delta;
    }

    double dhdt(VectorXd _x, double _t) {
        return (h(_x, _t + delta) - h(_x, _t - delta)) / 2.0 / delta;
    }

    VectorXd dhdx(VectorXd _x, double _t) {
        VectorXd res = _x;
        for (int i = 0; i < _x.size(); i++){
            res(i) = dh(_x, _t, i);
        }
        return res;
    }

    VectorXd constraint_u_coe(VectorXd& _f, MatrixXd& _g, VectorXd& _x, double _t) {
        VectorXd v = dhdx(_x, _t).transpose() * _g;
#ifdef CBF_DEBUG
        std::cout << "dhdx = " << std::endl << dhdx(_x, _t) << std::endl;
    std::cout << "v = " << v << std::endl;
    std::cout << "ret = " << v.cwiseProduct(ctrl_var) << std::endl;
#endif
        return v.cwiseProduct(ctrl_var);
    }

    double constraint_const_with_time(VectorXd & _f, MatrixXd & _g, VectorXd & _x, double _t) {
#ifdef CBF_DEBUG
        printf("h: %lf\tdhdt: %lf\t dhdx.dot(_f): %lf\talpha(h): %lf\n", h(_x, _t),
           dhdt(_x, _t), dhdx(_x, _t).dot(_f), alpha(h(_x, _t)));
#endif
        return dhdt(_x, _t) + dhdx(_x, _t).dot(_f) + alpha(h(_x, _t));
    }

    double constraint_const_without_time(VectorXd & _f, MatrixXd & _g, VectorXd & _x, double _t) {
#ifdef CBF_DEBUG
        printf("h: %lf\tdhdx.dot(_f): %lf\talpha(h): %lf\n", h(_x, _t),
           dhdx(_x, _t).dot(_f), alpha(h(_x, _t)));
#endif
        return dhdx(_x, _t).dot(_f) + alpha(h(_x, _t));
    }
};

#endif //CBF_MAIN_CBF_HPP
