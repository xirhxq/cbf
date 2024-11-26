#ifndef CBF_CBF_HPP
#define CBF_CBF_HPP

#include "utils.h"

class CBF{
public:
    std::string name;
    double delta = 0.001;
    std::function<double(double)> alpha = [](double h) {return 0.1 * pow(h, 3);};
    std::function<double(VectorXd, double)> h;

public:
    CBF(){}

    double dh(VectorXd x, double t, int i){
        VectorXd nxt = x, pre = x;
        nxt(i) += delta;
        pre(i) -= delta;
        return (h(nxt, t) - h(pre, t)) / 2.0 / delta;
    }

    double dhdt(VectorXd x, double t) {
        return (h(x, t + delta) - h(x, t - delta)) / 2.0 / delta;
    }

    VectorXd dhdx(VectorXd x, double t) {
        VectorXd res = x;
        for (int i = 0; i < x.size(); i++){
            res(i) = dh(x, t, i);
        }
        return res;
    }

    VectorXd constraintUCoe(const VectorXd& f, const MatrixXd& g, const VectorXd& x, double t) {
        VectorXd v = dhdx(x, t).transpose() * g;
        return v;
    }

    double constraintConstWithTime(const VectorXd & f, const MatrixXd & g, const VectorXd & x, double t) {
        return dhdt(x, t) + dhdx(x, t).dot(f) + alpha(h(x, t));
    }

    double constraintConstWithoutTime(const VectorXd & f, const MatrixXd & g, const VectorXd & x, double t) {
        return dhdx(x, t).dot(f) + alpha(h(x, t));
    }
};

#endif //CBF_CBF_HPP
