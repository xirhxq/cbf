#ifndef CBF_SECOND_ORDER_CBF_HPP
#define CBF_SECOND_ORDER_CBF_HPP

#include "utils.h"
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

struct SecondOrderCBFConstraintEvaluation {
    VectorXd uCoe;
    double h = 0.0;
    double hdot = 0.0;
    double hddotConst = 0.0;
    double k0 = 0.0;
    double k1 = 0.0;
    double lambda1 = 0.0;
    double sampledDataReserve = 0.0;
    double stateDependentReserve = 0.0;
    double totalReserve = 0.0;
    double constTerm = 0.0;
    double psi1 = 0.0;

    double value(const VectorXd &u) const {
        return uCoe.dot(u) + constTerm;
    }
};

class SecondOrderCBF {
public:
    std::string name;
    double k0 = 1.0;
    double k1 = 2.0;
    double lambda1 = 1.0;
    double sampledDataReserve = 0.0;
    std::function<double(const VectorXd&, double)> h;
    std::function<double(const VectorXd&, double)> hdot;
    std::function<double(const VectorXd&, double)> hddotConst;
    std::function<VectorXd(const VectorXd&, double)> uCoe;
    std::function<double(const VectorXd&, double)> stateDependentReserve;

    SecondOrderCBFConstraintEvaluation evaluateConstraint(const VectorXd &x, double t) const {
        if (!h || !hdot || !hddotConst || !uCoe) {
            throw std::logic_error("SecondOrderCBF is missing one or more callbacks");
        }
        if (!std::isfinite(sampledDataReserve) || sampledDataReserve < 0.0) {
            throw std::invalid_argument("SecondOrderCBF sampledDataReserve must be a non-negative finite number");
        }

        SecondOrderCBFConstraintEvaluation evaluation;
        evaluation.uCoe = uCoe(x, t);
        evaluation.h = h(x, t);
        evaluation.hdot = hdot(x, t);
        evaluation.hddotConst = hddotConst(x, t);
        evaluation.k0 = k0;
        evaluation.k1 = k1;
        evaluation.lambda1 = lambda1;
        evaluation.sampledDataReserve = sampledDataReserve;
        evaluation.stateDependentReserve = stateDependentReserve ? stateDependentReserve(x, t) : 0.0;
        if (!std::isfinite(evaluation.stateDependentReserve) || evaluation.stateDependentReserve < 0.0) {
            throw std::invalid_argument("SecondOrderCBF stateDependentReserve must be a non-negative finite number");
        }
        evaluation.totalReserve = sampledDataReserve + evaluation.stateDependentReserve;
        evaluation.constTerm = evaluation.hddotConst + k1 * evaluation.hdot + k0 * evaluation.h - evaluation.totalReserve;
        evaluation.psi1 = evaluation.hdot + lambda1 * evaluation.h;
        return evaluation;
    }
};

#endif
