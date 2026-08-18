#pragma once

#include "grand_finale/Task10p10CompletionFixture.hpp"

#include <limits>
#include <stdexcept>
#include <string>

namespace gf {

enum class Task10p11lPairKind { MobileMobile, MobileFixed };

struct Task10p11lSeparationAudit {
    bool valid = false;
    std::string scenario_id;
    Task10p11lPairKind minimum_kind = Task10p11lPairKind::MobileMobile;
    NodeId first = 0;
    NodeId second = 0;
    double minimum_distance_m = std::numeric_limits<double>::infinity();
    double required_distance_m = 0.0;
    double margin_m = -std::numeric_limits<double>::infinity();
};

inline Task10p11lSeparationAudit auditTask10p11lInitialSeparation(
    const Task10p10Scenario& scenario,double required_distance_m) {
    if (!std::isfinite(required_distance_m) || required_distance_m<=0.0 ||
        scenario.mobile_ids.size()!=scenario.mobile_positions.size()) {
        throw std::invalid_argument("invalid Task 10.11l separation request");
    }
    Task10p11lSeparationAudit result;
    result.scenario_id=scenario.id;
    result.required_distance_m=required_distance_m;
    const auto consider=[&](Task10p11lPairKind kind,NodeId first,NodeId second,
                            const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
        if (!a.allFinite() || !b.allFinite())
            throw std::invalid_argument("non-finite scenario position");
        const double distance=(a-b).norm();
        if (distance<result.minimum_distance_m) {
            result.minimum_kind=kind;
            result.first=first;
            result.second=second;
            result.minimum_distance_m=distance;
        }
    };
    for (std::size_t first=0;first<scenario.mobile_ids.size();++first) {
        for (std::size_t second=first+1;second<scenario.mobile_ids.size();++second)
            consider(Task10p11lPairKind::MobileMobile,
                scenario.mobile_ids[first],scenario.mobile_ids[second],
                scenario.mobile_positions[first],scenario.mobile_positions[second]);
        for (const auto& [fixed,position] : scenario.fixed_positions)
            consider(Task10p11lPairKind::MobileFixed,
                scenario.mobile_ids[first],fixed,
                scenario.mobile_positions[first],position);
    }
    result.margin_m=result.minimum_distance_m-required_distance_m;
    result.valid=result.margin_m>=0.0;
    return result;
}

}  // namespace gf
