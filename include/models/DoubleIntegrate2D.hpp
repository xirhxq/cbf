#ifndef DOUBLE_INTEGRATE_2D_HPP
#define DOUBLE_INTEGRATE_2D_HPP

#include "BaseModel.hpp"

class DoubleIntegrate2D : public BaseModel {
public:
    DoubleIntegrate2D(json &settings) : BaseModel(settings) {
        xMap = {{"x", 0}, {"y", 1}, {"vx", 2}, {"vy", 3}, {"battery", 4}, {"yawRad", 5}};
        uMap = {{"ax", 0}, {"ay", 1}, {"yawRateRad", 2}};

        X = Eigen::VectorXd::Zero(6);
        u = Eigen::VectorXd::Zero(3);

        F = Eigen::VectorXd::Zero(6);
        F(xMap["battery"]) = -dischargeRate;

        A = Eigen::MatrixXd ::Zero(6, 6);
        A(xMap["x"], xMap["vx"]) = 1.0;
        A(xMap["y"], xMap["vy"]) = 1.0;

        B = Eigen::MatrixXd::Zero(6, 3);
        B(xMap["vx"], uMap["ax"]) = 1.0;
        B(xMap["vy"], uMap["ay"]) = 1.0;
        B(xMap["yawRad"], uMap["yawRateRad"]) = 1.0;

    }

    void output() const override {
        std::cout << "DoubleIntegrate2D @ (x: " << X[0]
                  << ", y: " << X[1]
                  << ", vx: " << X[2]
                  << ", vy: " << X[3]
                  << ", battery: " << X[4]
                  << ", yawDeg: " << X[5] * 180 / M_PI << ")" << std::endl;
    }
};

#endif