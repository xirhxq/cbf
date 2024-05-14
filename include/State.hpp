#ifndef CBF_STATE_H
#define CBF_STATE_H

#include "utils.h"
#include "ComputingGeometry/ComputingGeometry"

class State {
private:
    int xIndex = 0;
    int yIndex = 1;
    int batteryIndex = 2;
    int yawIndex = 3;
public:
    VectorXd X;

    State() = default;

    State(int dimension) {
        X.resize(dimension);
    }

    State(VectorXd X) : X(std::move(X)) {}

    void setPosition(Point position) {
        X(xIndex) = position.x;
        X(yIndex) = position.y;
    }

    void setBattery(double battery) {
        if (X.size() > 2) {
            X(batteryIndex) = battery;
        }
    }

    void setYawRad(double yawRad) {
        if (X.size() > 3) {
            X(yawIndex) = yawRad;
        }
    }

    void setYawDeg(double yawDeg) {
        if (X.size() > 3) {
            X(yawIndex) = yawDeg * pi / 180;
        }
    }

    double x() {
        return X(xIndex);
    }

    double y() {
        return X(yIndex);
    }

    double battery() {
        return X(batteryIndex);
    }

    double yawRad() {
        return X(yawIndex);
    }

    double yawDeg() {
        return X(yawIndex) * 180 / pi;
    }

    Point xy() const {
        return {X(xIndex), X(yIndex)};
    }

    json toJson() {
        json j;
        j["x"] = X(xIndex);
        j["y"] = X(yIndex);
        j["battery"] = X(batteryIndex);
        j["yawRad"] = X(yawIndex);
        j["raw"] = X;
        return j;
    }

    json stateEncodeJson() {
        json j;
        j["x"] = xIndex;
        j["y"] = yIndex;
        j["battery"] = batteryIndex;
        j["yawRad"] = yawIndex;
        return j;
    }
};

#endif //CBF_STATE_H
