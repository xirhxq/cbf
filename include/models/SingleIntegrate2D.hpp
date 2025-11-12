#ifndef SINGLE_INTEGRATE_2D_HPP
#define SINGLE_INTEGRATE_2D_HPP

#include "BaseModel.hpp"

class SingleIntegrate2D : public BaseModel {
private:

public:
    SingleIntegrate2D(json &settings) : BaseModel(settings) {
        xMap = {{"x", 0}, {"y", 1}, {"battery", 2}, {"yawRad", 3}};
        uMap = {{"vx", 0}, {"vy", 1}, {"yawRateRad", 2}};

        X = Eigen::VectorXd::Zero(4);
        u = Eigen::VectorXd::Zero(3);

        F = Eigen::VectorXd::Zero(4);
        F[xMap["battery"]] = -dischargeRate;

        A = Eigen::MatrixXd::Zero(4, 4);

        B = Eigen::MatrixXd::Zero(4, 3);
        B(xMap["x"], uMap["vx"]) = 1.0;
        B(xMap["y"], uMap["vy"]) = 1.0;
        B(xMap["yawRad"], uMap["yawRateRad"]) = 1.0;
    }

    void output() const override {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "SingleIntegrate2D @ (x: " << X[0]
                  << ", y: " << X[1]
                  << ", battery: " << X[2]
                  << ", yawDeg: " << X[3] * 180 / M_PI << ")" << std::endl;
        std::cout << std::fixed << std::setprecision(6);
    }
};

#endif