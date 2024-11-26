#ifndef DOUBLE_INTEGRATE_2D_HPP
#define DOUBLE_INTEGRATE_2D_HPP

#include "BaseModel.hpp"

class DoubleIntegrate2D : public BaseModel {
public:
    DoubleIntegrate2D() {
        stateIndexMap = {{"x", 0}, {"y", 1}, {"vx", 2}, {"vy", 3}, {"battery", 4}, {"yawRad", 5}};
        controlIndexMap = {{"ax", 0}, {"ay", 1}, {"yawRateRad", 2}};

        X = Eigen::VectorXd::Zero(6);
        u = Eigen::VectorXd::Zero(3);

        F = Eigen::VectorXd::Zero(6);
        F(stateIndexMap["battery"]) = -1.0;

        A = Eigen::MatrixXd ::Zero(6, 6);
        A(0, 2) = 1.0; // x <- vx
        A(1, 3) = 1.0; // y <- vy

        B = Eigen::MatrixXd::Zero(6, 3);
        B(2, 0) = 1.0; // vx <- ax
        B(3, 1) = 1.0; // vy <- ay
        B(5, 2) = 1.0; // yaw <- yawRateRad

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