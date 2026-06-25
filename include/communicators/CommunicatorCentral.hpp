#ifndef CBF_COMMUNICATOR_CENTRAL_H
#define CBF_COMMUNICATOR_CENTRAL_H

#include "utils.h"
#include "CommunicatorBase.hpp"
#include <Eigen/Dense>

class CommunicatorCentral : public CommunicatorBase {
public:
    CommunicatorCentral(json settings) : CommunicatorBase(settings) {}

    void sendPosition2D(int id, const Point pos2d) override {
        _othersPos[id] = pos2d;
    }

    void receivePosition2D(int id, const Point pos2d) override {
        _othersPos[id] = pos2d;
    }

    void sendVelocity2D(int id, const VectorXd velocity2D) override {
        _othersVel[id] = velocity2D;
    }

    void receiveVelocity2D(int id, const VectorXd velocity2D) override {
        _othersVel[id] = velocity2D;
    }

    void sendAcceleration2D(int id, const VectorXd acceleration2D) override {
        _othersAcc[id] = acceleration2D;
    }

    void receiveAcceleration2D(int id, const VectorXd acceleration2D) override {
        _othersAcc[id] = acceleration2D;
    }

    void sendYawRad(int id, double yawRad) override {
        _othersYawRad[id] = yawRad;
    }

    void receiveYawRad(int id, double yawRad) override {
        _othersYawRad[id] = yawRad;
    }

    void sendBatteryLevel(int id, double batteryLevel) override {
        _othersBatteryLevel[id] = batteryLevel;
    }

    void receiveBatteryLevel(int id, double batteryLevel) override {
        _othersBatteryLevel[id] = batteryLevel;
    }

    void sendPositionCovariance(int id, const Eigen::Matrix2d& covariance) override {
        _othersPositionCovariance[id] = covariance;
    }

    void receivePositionCovariance(int id, const Eigen::Matrix2d& covariance) override {
        _othersPositionCovariance[id] = covariance;
    }
};

#endif //CBF_COMMUNICATOR_CENTRAL_H
