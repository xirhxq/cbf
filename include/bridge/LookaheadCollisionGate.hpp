#ifndef CBF_LOOKAHEAD_COLLISION_GATE_HPP
#define CBF_LOOKAHEAD_COLLISION_GATE_HPP

#include <vector>

struct BridgeLookaheadNeighbourState2D {
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
};

inline bool bridgeLookaheadDistanceClosingTrigger(
        double selfX,
        double selfY,
        double selfVx,
        double selfVy,
        const std::vector<BridgeLookaheadNeighbourState2D> &neighbours,
        double distanceThreshold) {
    if (distanceThreshold <= 0.0) {
        return false;
    }
    const double squaredThreshold = distanceThreshold * distanceThreshold;
    for (const auto &neighbour : neighbours) {
        const double dx = neighbour.x - selfX;
        const double dy = neighbour.y - selfY;
        if (dx * dx + dy * dy >= squaredThreshold) {
            continue;
        }
        const double relativeVx = selfVx - neighbour.vx;
        const double relativeVy = selfVy - neighbour.vy;
        if (dx * relativeVx + dy * relativeVy > 0.0) {
            return true;
        }
    }
    return false;
}

#endif
