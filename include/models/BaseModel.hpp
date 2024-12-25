#ifndef BASE_MODEL_HPP
#define BASE_MODEL_HPP

#include "utils.h"

class BaseModel {
protected:
    Eigen::VectorXd X, u;
    Eigen::VectorXd F;
    Eigen::MatrixXd A, B;
    std::unordered_map<std::string, int> xMap;
    std::unordered_map<std::string, int> uMap;
public:
    virtual ~BaseModel() = default;

    auto getX() const {
        return X;
    }

    Eigen::MatrixXd f() const {
        if (F.size() != X.size() || A.cols() != X.size() || A.rows() != F.size()) {
            throw std::logic_error("State or matrix dimensions do not match");
        }
        return A * X + F;
    }

    Eigen::MatrixXd g() const {
        if (B.rows() != X.size() || B.cols() != u.size()) {
            throw std::logic_error("State or control dimensions do not match");
        }
        return B;
    }

    void setStateVector(const Eigen::VectorXd &stateInput) {
        if (stateInput.size() != X.size()) {
            throw std::invalid_argument("State size mismatch");
        }
        X = stateInput;
    }

    int getStateSize() const {
        return X.size();
    }

    int uSize() const {
        return u.size();
    }

    double getStateVariable(const std::string &name) const {
        auto it = xMap.find(name);
        if (it != xMap.end()) {
            return X[it->second];
        }
        throw std::invalid_argument("Invalid state variable name: " + name);
    }

    Point xy() {
        if (xMap.find("x") == xMap.end()) {
            throw std::invalid_argument("Invalid state variable name: x");
        } else if (xMap.find("y") == xMap.end()) {
            throw std::invalid_argument("Invalid state variable name: y");
        } else {
            return Point(X[xMap["x"]], X[xMap["y"]]);
        }
    }

    void setStateVariable(const std::string &name, double value) {
        auto it = xMap.find(name);
        if (it != xMap.end()) {
            X[it->second] = value;
            return;
        }
        throw std::invalid_argument("Invalid state variable name: " + name);
    }

    void setPosition2D(Point p) {
        setStateVariable("x", p.x);
        setStateVariable("y", p.y);
    }

    void setYawDeg(double yaw) {
        setStateVariable("yawRad", yaw * M_PI / 180.0);
    }

    void setControlInput(const Eigen::VectorXd &controlInput) {
        if (controlInput.size() != u.size()) {
            throw std::invalid_argument("Control input size mismatch");
        }
        u = controlInput;
    }

    Eigen::VectorXd getControlInput() const {
        return u;
    }

    double extractFromVector(const Eigen::VectorXd &input, const std::string &name) const {
        auto it = xMap.find(name);
        if (it != xMap.end() && it->second < input.size()) {
            return input[it->second];
        }
        throw std::invalid_argument("Invalid state variable name or index out of bounds: " + name);
    }

    Point extractXYFromVector(const Eigen::VectorXd &input) {
        return Point(extractFromVector(input, "x"), extractFromVector(input, "y"));
    }

    double extractFromControl(const Eigen::VectorXd &controlInput, const std::string &name) const {
        auto it = uMap.find(name);
        if (it != uMap.end() && it->second < controlInput.size()) {
            return controlInput[it->second];
        }
        throw std::invalid_argument("Invalid control variable name or index out of bounds: " + name);
    }

    void startCharge() {
        if (xMap.find("battery") == xMap.end()) {
            throw std::invalid_argument("Invalid state variable name: battery");
        }
        F[xMap["battery"]] = 10.0;
    }

    void stopCharge() {
        if (xMap.find("battery") == xMap.end()) {
            throw std::invalid_argument("Invalid state variable name: battery");
        }
        F[xMap["battery"]] = -1.0;
    }

    void checkCharge() {
        if (xMap.find("battery") == xMap.end()) {
            throw std::invalid_argument("Invalid state variable name: battery");
        }
        if (X[xMap["battery"]] >= 100.0) {
            stopCharge();
        }
    }

    void stepTimeForward(double dt) {
        checkCharge();

        X += (f() + g() * u) * dt;
    }

    json state2Json() const {
        json j;
        for (const auto &[name, index] : xMap) {
            j[name] = X[index];
        }
        return j;
    }

    json state2Json(VectorXd x) {
        json j;
        for (const auto &[name, index] : xMap) {
            j[name] = x[index];
        }
        return j;
    }

    json control2Json() const {
        json j;
        for (const auto &[name, index] : uMap) {
            j[name] = u[index];
        }
        return j;
    }

    json control2Json(VectorXd u) {
        json j;
        for (const auto &[name, index] : uMap) {
            j[name] = u[index];
        }
        return j;
    }

    virtual void output() const = 0;
};

#endif