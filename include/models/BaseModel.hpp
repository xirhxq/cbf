#ifndef BASE_MODEL_HPP
#define BASE_MODEL_HPP

#include "utils.h"

using json = nlohmann::json;

class BaseModel {
protected:
    Eigen::VectorXd X, u;
    Eigen::MatrixXd A, B;
    std::unordered_map<std::string, int> stateIndexMap;
    std::unordered_map<std::string, int> controlIndexMap;
    std::unordered_map<std::string, double> miscVariables = {
            {"chargeRate", 10.0},
            {"dischargeRate", 1.0},
            {"batteryUpperLimit", 100.0},
            {"batteryLowerLimit", 0.0}
    };
    bool isCharge = false;

public:
    virtual ~BaseModel() = default;

    auto getX() const {
        return X;
    }

    auto f() const {
        if (A.rows() != X.size() || A.cols() != X.size()) {
            throw std::logic_error("State or matrix dimensions do not match");
        }
        Eigen::VectorXd ret = A * X;
        return ret;
    }

    auto g() const {
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
        auto it = stateIndexMap.find(name);
        if (it != stateIndexMap.end()) {
            return X[it->second];
        }
        throw std::invalid_argument("Invalid state variable name: " + name);
    }

    Point xy() {
        if (stateIndexMap.find("x") == stateIndexMap.end()) {
            throw std::invalid_argument("Invalid state variable name: x");
        } else if (stateIndexMap.find("y") == stateIndexMap.end()) {
            throw std::invalid_argument("Invalid state variable name: y");
        } else {
            return Point(X[stateIndexMap["x"]], X[stateIndexMap["y"]]);
        }
    }

    void setStateVariable(const std::string &name, double value) {
        auto it = stateIndexMap.find(name);
        if (it != stateIndexMap.end()) {
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
        auto it = stateIndexMap.find(name);
        if (it != stateIndexMap.end() && it->second < input.size()) {
            return input[it->second];
        }
        throw std::invalid_argument("Invalid state variable name or index out of bounds: " + name);
    }

    double extractFromVector(const Eigen::VectorXd &input, int index) const {
        if (index >= 0 && index < input.size()) {
            return input[index];
        }
        throw std::out_of_range("Index out of bounds in extractFromVector");
    }

    Point extractXYFromVector(const Eigen::VectorXd &input) {
        return Point(extractFromVector(input, "x"), extractFromVector(input, "y"));
    }

    double extractFromControl(const Eigen::VectorXd &controlInput, const std::string &name) const {
        auto it = controlIndexMap.find(name);
        if (it != controlIndexMap.end() && it->second < controlInput.size()) {
            return controlInput[it->second];
        }
        throw std::invalid_argument("Invalid control variable name or index out of bounds: " + name);
    }

    double extractFromControl(const Eigen::VectorXd &controlInput, int index) const {
        if (index >= 0 && index < controlInput.size()) {
            return controlInput[index];
        }
        throw std::out_of_range("Index out of bounds in extractFromControl");
    }

    double getMiscVariable(const std::string &name) const {
        auto it = miscVariables.find(name);
        if (it != miscVariables.end()) {
            return it->second;
        }
        throw std::invalid_argument("Invalid misc variable name: " + name);
    }

    void setMiscVariable(const std::string &name, double value) {
        auto it = miscVariables.find(name);
        if (it != miscVariables.end()) {
            miscVariables[name] = value;
            return;
        }
        throw std::invalid_argument("Invalid misc variable name: " + name);
    }

    void startCharge() {
        isCharge = true;
    }

    void stopCharge(){
        isCharge = false;
    }

    void stepTimeForward(double dt) {
        if (A.rows() != X.size() || A.cols() != X.size() || X.size() != B.rows() || u.size() != B.cols()) {
            throw std::logic_error("State, control input, or matrix dimensions do not match");
        }

        X += (A * X + B * u) * dt;

        auto it = stateIndexMap.find("battery");
        if (it != stateIndexMap.end()) {
            int batteryIndex = it->second;
            double rate = isCharge ? getMiscVariable("chargeRate") : -getMiscVariable("dischargeRate");
            X[batteryIndex] += rate * dt;
            X[batteryIndex] = std::clamp(X[batteryIndex], getMiscVariable("batteryLowerLimit"), getMiscVariable("batteryUpperLimit"));
            if (X[batteryIndex] >= getMiscVariable("batteryUpperLimit")) {
                stopCharge();
            }
        }
    }

    json toJson() const {
        json j;
        for (const auto &[name, index] : stateIndexMap) {
            j[name] = X[index];
        }
        return j;
    }

    json toJson(VectorXd x) {
        json j;
        for (const auto &[name, index] : stateIndexMap) {
            j[name] = x[index];
        }
        return j;
    }

    virtual void output() const = 0;
};

#endif