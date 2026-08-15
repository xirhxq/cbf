#ifndef DOUBLE_INTEGRATE_2D_HPP
#define DOUBLE_INTEGRATE_2D_HPP

#include "BaseModel.hpp"

struct DoubleIntegratorPlanarState {
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
};

inline DoubleIntegratorPlanarState propagateDoubleIntegratorPlanarZoh(
    const Eigen::Vector2d &position,
    const Eigen::Vector2d &velocity,
    const Eigen::Vector2d &acceleration,
    double dt) {
    if (!position.allFinite() || !velocity.allFinite() ||
        !acceleration.allFinite() || !std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument(
            "Double-integrator ZOH propagation requires finite inputs and dt >= 0");
    }

    return {
        position + velocity * dt + 0.5 * acceleration * dt * dt,
        velocity + acceleration * dt,
    };
}

inline double wrapDoubleIntegratorYawRad(double angle) {
    if (!std::isfinite(angle))
        throw std::invalid_argument("yaw angle must be finite");
    double wrapped=std::fmod(angle+M_PI,2.0*M_PI);
    if (wrapped<0.0) wrapped+=2.0*M_PI;
    return wrapped-M_PI;
}

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

    void stepTimeForward(double dt) override {
        checkCharge();

        const Eigen::Vector2d position(X[xMap["x"]], X[xMap["y"]]);
        const Eigen::Vector2d velocity(X[xMap["vx"]], X[xMap["vy"]]);
        const Eigen::Vector2d acceleration(u[uMap["ax"]], u[uMap["ay"]]);
        const auto next = propagateDoubleIntegratorPlanarZoh(
            position, velocity, acceleration, dt);

        X[xMap["x"]] = next.position(0);
        X[xMap["y"]] = next.position(1);
        X[xMap["vx"]] = next.velocity(0);
        X[xMap["vy"]] = next.velocity(1);
        X[xMap["battery"]] += F[xMap["battery"]] * dt;
        X[xMap["yawRad"]] = wrapDoubleIntegratorYawRad(
            X[xMap["yawRad"]] + u[uMap["yawRateRad"]] * dt);
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
