#ifndef CBF_COMMUNICATOR_BASE_HPP
#define CBF_COMMUNICATOR_BASE_HPP

#include "utils.h"
#include <string>
#include <unordered_map>

class CommunicatorBase {
public:
    explicit CommunicatorBase(json settings){
        id = settings["id"];
        all = settings["all"].get<std::vector<int>>();
    }
    virtual ~CommunicatorBase() = default;

    virtual void sendPosition2D(int id, Point position2D) = 0;
    virtual void receivePosition2D(int id, Point position2D) = 0;

    virtual void sendVelocity2D(int id, const VectorXd velocity2D) = 0;
    virtual void receiveVelocity2D(int id, const VectorXd velocity2D) = 0;

    virtual void sendYawRad(int id, double yawRad) = 0;
    virtual void receiveYawRad(int id, double yawRad) = 0;

    void output() {
        for (auto &i : all) {
            std::cout << "Robot " << i << ": ";
            std::cout << "Pos (" << std::setprecision(4) << _othersPos[i].x << ", " << _othersPos[i].y << "), ";
            std::cout << "Vel" <<  vecToString(_othersVel[i]) << ", ";
            std::cout << "YawDeg" << std::setprecision(2) << _othersYawRad[i] * 180 / M_PI << std::endl;
        }
    }

protected:
    json settings;
    int id;
    std::vector<int> all;

public:
    std::unordered_map<int, Point> _othersPos;
    std::unordered_map<int, VectorXd> _othersVel;
    std::unordered_map<int, double> _othersYawRad;
};

#endif // CBF_COMMUNICATOR_BASE_HPP