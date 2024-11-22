#ifndef SINGLE_INTEGRATE_2D_HPP
#define SINGLE_INTEGRATE_2D_HPP

#include "BaseModel.hpp"

class SingleIntegrate2D : public BaseModel {
public:
    SingleIntegrate2D() {
        X = Eigen::VectorXd::Zero(4);
        u = Eigen::VectorXd::Zero(3);
        A = Eigen::MatrixXd::Zero(4, 4);
        B = Eigen::MatrixXd::Zero(4, 3);

        B(0, 0) = 1.0; // x <- vx
        B(1, 1) = 1.0; // y <- vy
        B(3, 2) = 1.0; // yaw <- yaw_rate

        stateIndexMap = {{"x", 0}, {"y", 1}, {"battery", 2}, {"yawRad", 3}};
        controlIndexMap = {{"vx", 0}, {"vy", 1}, {"yaw_rate", 2}};
    }

    void output() const override {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "SingleIntegrate2D @ (x: " << X[0]
                  << ", y: " << X[1]
                  << ", battery: " << X[2]
                  << ", yawDeg: " << X[3] * 180 / M_PI << ")" << std::endl;
    }
};

#endif