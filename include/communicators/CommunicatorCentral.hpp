#ifndef CBF_COMMUNICATOR_CENTRAL_H
#define CBF_COMMUNICATOR_CENTRAL_H

#include "utils.h"
#include "CommunicatorBase.hpp"

class CommunicatorCentral : public CommunicatorBase {
public:
    CommunicatorCentral(json settings) : CommunicatorBase(settings) {}

    void sendPosition2D(int id, const Point pos2d) override {
        position2D[id] = pos2d;
    }

    void receivePosition2D(int id, const Point pos2d) override {
        if (id == this->id) return;
        position2D[id] = pos2d;
    }
};

#endif //CBF_COMMUNICATOR_CENTRAL_H
