#ifndef CBF_BRIDGE_HOCBF_FEASIBILITY_GUARD_HPP
#define CBF_BRIDGE_HOCBF_FEASIBILITY_GUARD_HPP

#include "utils.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

struct BridgeHocbfHalfspace2D {
    double ax = 0.0;
    double ay = 0.0;
    double rhs = 0.0;
};

struct BridgeHocbfProjectionResult {
    bool feasible = false;
    bool active = false;
    double projected_ax = 0.0;
    double projected_ay = 0.0;
    double projection_norm = 0.0;
    double margin_before = std::numeric_limits<double>::quiet_NaN();
    double margin_after = std::numeric_limits<double>::quiet_NaN();
    int vertex_count = 0;
};

struct BridgeHocbfCertifiedEndpoint2D {
    bool valid = false;
    bool repaired = false;
    double accelX = 0.0;
    double accelY = 0.0;
    double repairMix = 0.0;
    double repairNorm = 0.0;
    double marginBefore = std::numeric_limits<double>::quiet_NaN();
    double marginAfter = std::numeric_limits<double>::quiet_NaN();
};

inline double bridgeHocbfHalfspaceMargin(
        const BridgeHocbfHalfspace2D &constraint,
        double ax,
        double ay) {
    return constraint.ax * ax + constraint.ay * ay - constraint.rhs;
}

inline double bridgeHocbfMinMargin(
        const std::vector<BridgeHocbfHalfspace2D> &constraints,
        double ax,
        double ay) {
    if (constraints.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double value = std::numeric_limits<double>::infinity();
    for (const auto &constraint: constraints) {
        value = std::min(value, bridgeHocbfHalfspaceMargin(constraint, ax, ay));
    }
    return value;
}

inline std::vector<Point> clipBridgeHocbfPolygon(
        const std::vector<Point> &polygon,
        const BridgeHocbfHalfspace2D &constraint,
        double tolerance) {
    std::vector<Point> output;
    if (polygon.empty()) {
        return output;
    }

    auto inside = [&](const Point &point) {
        return bridgeHocbfHalfspaceMargin(constraint, point.x, point.y) >= -tolerance;
    };

    for (size_t i = 0; i < polygon.size(); ++i) {
        const Point current = polygon[i];
        const Point previous = polygon[(i + polygon.size() - 1) % polygon.size()];
        bool currentInside = inside(current);
        bool previousInside = inside(previous);
        double currentValue = bridgeHocbfHalfspaceMargin(constraint, current.x, current.y);
        double previousValue = bridgeHocbfHalfspaceMargin(constraint, previous.x, previous.y);

        if (currentInside != previousInside) {
            double denom = previousValue - currentValue;
            if (std::abs(denom) > 1.0e-12) {
                double ratio = previousValue / denom;
                output.emplace_back(
                        previous.x + ratio * (current.x - previous.x),
                        previous.y + ratio * (current.y - previous.y));
            }
        }

        if (currentInside) {
            output.push_back(current);
        }
    }

    return output;
}

inline Point closestPointOnBridgeHocbfSegment(const Point &point, const Point &a, const Point &b) {
    Point ab = b - a;
    double denom = ab.x * ab.x + ab.y * ab.y;
    if (denom <= 1.0e-12) {
        return a;
    }

    double t = ((point.x - a.x) * ab.x + (point.y - a.y) * ab.y) / denom;
    t = std::max(0.0, std::min(1.0, t));
    return Point(a.x + t * ab.x, a.y + t * ab.y);
}

inline BridgeHocbfProjectionResult projectBridgeHocbfNominalAcceleration(
        double nominal_ax,
        double nominal_ay,
        double acceleration_bound,
        const std::vector<BridgeHocbfHalfspace2D> &constraints,
        double tolerance = 1.0e-9) {
    BridgeHocbfProjectionResult result;
    result.projected_ax = nominal_ax;
    result.projected_ay = nominal_ay;
    result.margin_before = bridgeHocbfMinMargin(constraints, nominal_ax, nominal_ay);

    if (!std::isfinite(acceleration_bound) || acceleration_bound < 0.0) {
        return result;
    }

    std::vector<Point> polygon = {
            Point(-acceleration_bound, -acceleration_bound),
            Point(acceleration_bound, -acceleration_bound),
            Point(acceleration_bound, acceleration_bound),
            Point(-acceleration_bound, acceleration_bound)
    };
    for (const auto &constraint: constraints) {
        polygon = clipBridgeHocbfPolygon(polygon, constraint, tolerance);
        if (polygon.empty()) {
            result.vertex_count = 0;
            return result;
        }
    }

    result.feasible = true;
    result.vertex_count = static_cast<int>(polygon.size());

    Point nominal(nominal_ax, nominal_ay);
    if (result.margin_before >= -tolerance
        && nominal_ax >= -acceleration_bound - tolerance
        && nominal_ax <= acceleration_bound + tolerance
        && nominal_ay >= -acceleration_bound - tolerance
        && nominal_ay <= acceleration_bound + tolerance) {
        result.margin_after = result.margin_before;
        return result;
    }

    Point best = polygon.front();
    double bestDistance = (best - nominal).len();
    for (size_t i = 0; i < polygon.size(); ++i) {
        Point vertex = polygon[i];
        double vertexDistance = (vertex - nominal).len();
        if (vertexDistance < bestDistance) {
            best = vertex;
            bestDistance = vertexDistance;
        }

        Point edgePoint = closestPointOnBridgeHocbfSegment(
                nominal,
                polygon[i],
                polygon[(i + 1) % polygon.size()]);
        double edgeDistance = (edgePoint - nominal).len();
        if (edgeDistance < bestDistance) {
            best = edgePoint;
            bestDistance = edgeDistance;
        }
    }

    result.active = true;
    result.projected_ax = best.x;
    result.projected_ay = best.y;
    result.projection_norm = bestDistance;
    result.margin_after = bridgeHocbfMinMargin(constraints, best.x, best.y);
    return result;
}

inline BridgeHocbfCertifiedEndpoint2D certifyBridgeHocbfEndpointTowardWitness(
        double candidateX,
        double candidateY,
        double witnessX,
        double witnessY,
        double accelerationBound,
        const std::vector<BridgeHocbfHalfspace2D> &constraints,
        double maximumRepairNorm = 1.0e-10) {
    BridgeHocbfCertifiedEndpoint2D result;
    result.accelX = candidateX;
    result.accelY = candidateY;
    result.marginBefore = bridgeHocbfMinMargin(
            constraints, candidateX, candidateY);
    if (!std::isfinite(candidateX) || !std::isfinite(candidateY)
        || !std::isfinite(witnessX) || !std::isfinite(witnessY)
        || !std::isfinite(accelerationBound) || accelerationBound < 0.0
        || !std::isfinite(maximumRepairNorm) || maximumRepairNorm < 0.0
        || std::abs(candidateX) > accelerationBound
        || std::abs(candidateY) > accelerationBound
        || std::abs(witnessX) > accelerationBound
        || std::abs(witnessY) > accelerationBound) {
        return result;
    }

    const double witnessMargin = bridgeHocbfMinMargin(
            constraints, witnessX, witnessY);
    if (witnessMargin < 0.0) {
        return result;
    }
    if (result.marginBefore >= 0.0) {
        result.valid = true;
        result.marginAfter = result.marginBefore;
        return result;
    }

    double requiredMix = 0.0;
    for (const auto &constraint : constraints) {
        const double candidateMargin = bridgeHocbfHalfspaceMargin(
                constraint, candidateX, candidateY);
        if (candidateMargin >= 0.0) {
            continue;
        }
        const double endpointMargin = bridgeHocbfHalfspaceMargin(
                constraint, witnessX, witnessY);
        const double improvement = endpointMargin - candidateMargin;
        if (!(improvement > 0.0)) {
            return result;
        }
        requiredMix = std::max(
                requiredMix, -candidateMargin / improvement);
    }
    requiredMix = std::min(1.0, std::max(0.0, requiredMix));

    const double witnessDistance = std::hypot(
            witnessX - candidateX, witnessY - candidateY);
    if (!(witnessDistance > 0.0)) {
        return result;
    }

    const double maximumRepairMix = std::min(
            1.0,
            std::nextafter(
                    maximumRepairNorm / witnessDistance, 0.0));
    if (requiredMix > maximumRepairMix) {
        return result;
    }

    auto pointAtMix = [&](double mix) {
        return Point(
                std::fma(mix, witnessX - candidateX, candidateX),
                std::fma(mix, witnessY - candidateY, candidateY));
    };
    double lowerMix = requiredMix;
    double upperMix = maximumRepairMix;
    Point repaired = pointAtMix(lowerMix);
    double repairedMargin = bridgeHocbfMinMargin(
            constraints, repaired.x, repaired.y);
    if (repairedMargin < 0.0) {
        repaired = pointAtMix(upperMix);
        repairedMargin = bridgeHocbfMinMargin(
                constraints, repaired.x, repaired.y);
        if (repairedMargin < 0.0) {
            return result;
        }
        for (int iteration = 0; iteration < 64; ++iteration) {
            const double middleMix = lowerMix
                    + 0.5 * (upperMix - lowerMix);
            if (middleMix <= lowerMix || middleMix >= upperMix) {
                break;
            }
            const Point middle = pointAtMix(middleMix);
            if (bridgeHocbfMinMargin(
                        constraints, middle.x, middle.y) >= 0.0) {
                upperMix = middleMix;
                repaired = middle;
                repairedMargin = bridgeHocbfMinMargin(
                        constraints, repaired.x, repaired.y);
            } else {
                lowerMix = middleMix;
            }
        }
    } else {
        upperMix = lowerMix;
    }

    const double repairNorm = std::hypot(
            repaired.x - candidateX,
            repaired.y - candidateY);
    if (repairNorm > maximumRepairNorm) {
        return result;
    }
    result.valid = true;
    result.repaired = true;
    result.accelX = repaired.x;
    result.accelY = repaired.y;
    result.repairMix = upperMix;
    result.repairNorm = repairNorm;
    result.marginAfter = repairedMargin;
    return result;

}

#endif
