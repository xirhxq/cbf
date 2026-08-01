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

    void sendUncertaintyRate(int id, double uncertaintyRate) override {
        _othersUncertaintyRate[id] = uncertaintyRate;
    }

    void receiveUncertaintyRate(int id, double uncertaintyRate) override {
        _othersUncertaintyRate[id] = uncertaintyRate;
    }

    void sendEndpointCertificateSnapshot(
        int id,
        const cbf2026::EndpointCertificateSnapshot& snapshot
    ) override {
        storeEndpointCertificateSnapshot(id, snapshot);
    }

    void receiveEndpointCertificateSnapshot(
        int id,
        const cbf2026::EndpointCertificateSnapshot& snapshot
    ) override {
        storeEndpointCertificateSnapshot(id, snapshot);
    }

private:
    void storeEndpointCertificateSnapshot(
        int id,
        const cbf2026::EndpointCertificateSnapshot& snapshot
    ) {
        cbf2026::validateEndpointCertificateSnapshot(id, snapshot);
        _othersEndpointCertificateSnapshots[id] = snapshot;
        _othersEpsilon[id] = snapshot.epsilon;
        _othersBarNu[id] = snapshot.barNu;
        _othersCovarianceRateBound[id] = snapshot.covarianceRateBound;
        _othersCertificateSnapshotVersion[id] = snapshot.snapshotVersion;
        _othersAllocationVersion[id] = snapshot.allocationVersion;
    }
};

#endif //CBF_COMMUNICATOR_CENTRAL_H
