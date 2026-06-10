#ifndef CBF_CBF_HPP
#define CBF_CBF_HPP

#include "utils.h"
#include <optional>

struct CBFConstraintEvaluation {
    VectorXd gradient;
    VectorXd uCoe;
    double h = 0.0;
    double dhdt = 0.0;
    double drift = 0.0;
    double constWithTime = 0.0;
    double constWithoutTime = 0.0;

    double hdot(const VectorXd &u) const {
        return dhdt + drift + uCoe.dot(u);
    }
};

class CBF{
public:
    std::string name;
    double delta = 0.001;
    std::function<double(double)> alpha;
    std::function<double(VectorXd, double)> h;

    std::optional<std::function<VectorXd(VectorXd, double)>> dhdx_analytical;
    std::optional<std::function<double(VectorXd, double)>> dhdt_analytical;

private:
    // Class-k function parameters: alpha(h) = c * h^k
    double alpha_c = 0.1;
    int alpha_k = 3;

public:
    CBF() {
        setAlphaClassK(0.1, 3);
    }

    // Set alpha as a class-k function: alpha(h) = c * h^k
    void setAlphaClassK(double coefficient, int power) {
        if (power <= 0 || power % 2 == 0) {
            throw std::invalid_argument("Power must be a positive odd integer for class-k functions");
        }
        alpha_c = coefficient;
        alpha_k = power;
        alpha = [coefficient, power](double h) { return coefficient * std::pow(h, power); };
    }

    void setAlphaCustom(std::function<double(double)> custom_alpha) {
        alpha = custom_alpha;
    }

    double getAlphaCoefficient() const { return alpha_c; }
    int getAlphaPower() const { return alpha_k; }

    double dh(VectorXd x, double t, int i){
        VectorXd nxt = x, pre = x;
        nxt(i) += delta;
        pre(i) -= delta;
        return (h(nxt, t) - h(pre, t)) / 2.0 / delta;
    }

    double dhdt(VectorXd x, double t) {
        if (dhdt_analytical.has_value()) {
            return dhdt_analytical.value()(x, t);
        } else {
            return (h(x, t + delta) - h(x, t - delta)) / 2.0 / delta;
        }
    }

    VectorXd dhdx(VectorXd x, double t) {
        if (dhdx_analytical.has_value()) {
            return dhdx_analytical.value()(x, t);
        } else {
            VectorXd res = x;
            for (int i = 0; i < x.size(); i++){
                res(i) = dh(x, t, i);
            }
            return res;
        }
    }

    VectorXd constraintUCoe(const VectorXd& f, const MatrixXd& g, const VectorXd& x, double t) {
        VectorXd v = dhdx(x, t).transpose() * g;
        return v;
    }

    CBFConstraintEvaluation evaluateConstraint(const VectorXd & f, const MatrixXd & g, const VectorXd & x, double t) {
        CBFConstraintEvaluation evaluation;
        evaluation.gradient = dhdx(x, t);
        evaluation.uCoe = evaluation.gradient.transpose() * g;
        evaluation.h = h(x, t);
        evaluation.dhdt = dhdt(x, t);
        evaluation.drift = evaluation.gradient.dot(f);
        double alphaH = alpha(evaluation.h);
        evaluation.constWithTime = evaluation.dhdt + evaluation.drift + alphaH;
        evaluation.constWithoutTime = evaluation.drift + alphaH;
        return evaluation;
    }

    double constraintConstWithTime(const VectorXd & f, const MatrixXd & g, const VectorXd & x, double t) {
        return dhdt(x, t) + dhdx(x, t).dot(f) + alpha(h(x, t));
    }

    double constraintConstWithoutTime(const VectorXd & f, const MatrixXd & g, const VectorXd & x, double t) {
        return dhdx(x, t).dot(f) + alpha(h(x, t));
    }

    double hdot(const VectorXd & f, const MatrixXd & g, const VectorXd & x, const VectorXd & u, double t) {
        return evaluateConstraint(f, g, x, t).hdot(u);
    }

    void checkInequality(const VectorXd & f, const MatrixXd & g, const VectorXd & x, const VectorXd & u, double t) {
        auto evaluation = evaluateConstraint(f, g, x, t);
        std::cout << std::setprecision(4)
            << "Checking " << name << ": "
            << evaluation.hdot(u) << " >= "
            << -alpha(evaluation.h) << std::endl;
    }
};

#endif //CBF_CBF_HPP
