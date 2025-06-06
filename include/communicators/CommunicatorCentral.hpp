#ifndef CBF_COMMUNICATOR_CENTRAL_H
#define CBF_COMMUNICATOR_CENTRAL_H

#include "utils.h"
#include "CommunicatorBase.hpp"

class CommunicatorCentral : public CommunicatorBase {
public:
    CommunicatorCentral(json settings) : CommunicatorBase(settings) {}

    void sendPosition2D(int id, const Point pos2d) override {
        _othersPos[id] = pos2d;
    }

    void receivePosition2D(int id, const Point pos2d) override {
        if (id == this->id) return;
        _othersPos[id] = pos2d;
    }

    void sendVelocity2D(int id, const VectorXd velocity2D) override {
        _othersVel[id] = velocity2D;
    }

    void receiveVelocity2D(int id, const VectorXd velocity2D) override {
        if (id == this->id) return;
        _othersVel[id] = velocity2D;
    }
};

#endif //CBF_COMMUNICATOR_CENTRAL_H
