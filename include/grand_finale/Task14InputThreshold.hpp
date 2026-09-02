#pragma once

#include "grand_finale/CanonicalHardRows.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace gf {

struct Task14InputBoundResult {
    bool valid=false;
    double bound_mps2=std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector2d witness=Eigen::Vector2d::Zero();
    std::string reason;
};

struct Task14DiskGammaResult {
    bool valid=false;
    double gamma=-std::numeric_limits<double>::infinity();
    Eigen::Vector2d control=Eigen::Vector2d::Zero();
};

namespace task14_input_detail {

struct Halfspace {
    Eigen::Vector2d coefficient=Eigen::Vector2d::Zero();
    double constant=0.0;
};

inline std::vector<Halfspace> ownerHalfspaces(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,double gamma) {
    std::vector<Halfspace> result;
    for (const auto& row:rows) {
        if (row.owner!=owner || !row.participates_in_gamma) continue;
        result.push_back({row.control_coefficient,row.constant-gamma});
    }
    return result;
}

inline bool feasible(const std::vector<Halfspace>& rows,
                     const Eigen::Vector2d& value,double tolerance) {
    for (const auto& row:rows)
        if (row.coefficient.dot(value)+row.constant<
            -tolerance*(1.0+row.coefficient.norm()*value.norm()+
                        std::abs(row.constant))) return false;
    return value.allFinite();
}

inline Task14InputBoundResult minimumNorm(
    const std::vector<Halfspace>& rows,double tolerance) {
    Task14InputBoundResult result;
    if (!std::isfinite(tolerance) || tolerance<0.0) {
        result.reason="invalid_tolerance";
        return result;
    }
    for (const auto& row:rows) {
        if (!row.coefficient.allFinite() || !std::isfinite(row.constant)) {
            result.reason="nonfinite_halfspace";
            return result;
        }
        if (row.coefficient.squaredNorm()<=1e-24 &&
            row.constant< -tolerance) {
            result.reason="constant_infeasible_halfspace";
            return result;
        }
    }
    std::optional<Eigen::Vector2d> best;
    const auto consider=[&](const Eigen::Vector2d& candidate) {
        if (!candidate.allFinite() || !feasible(rows,candidate,tolerance))
            return;
        if (!best.has_value() ||
            candidate.squaredNorm()<best->squaredNorm()-1e-18 ||
            (std::abs(candidate.squaredNorm()-best->squaredNorm())<=1e-18 &&
             (candidate.x()<best->x() ||
              (candidate.x()==best->x()&&candidate.y()<best->y()))))
            best=candidate;
    };
    consider(Eigen::Vector2d::Zero());
    for (const auto& row:rows) {
        const double scale=row.coefficient.squaredNorm();
        if (scale<=1e-24) continue;
        consider(-row.constant*row.coefficient/scale);
    }
    for (std::size_t first=0;first<rows.size();++first) {
        for (std::size_t second=first+1;second<rows.size();++second) {
            Eigen::Matrix2d matrix;
            matrix.row(0)=rows[first].coefficient.transpose();
            matrix.row(1)=rows[second].coefficient.transpose();
            const double determinant=matrix.determinant();
            const double scale=rows[first].coefficient.norm()*
                rows[second].coefficient.norm();
            if (scale<=1e-24 || std::abs(determinant)<=1e-12*scale)
                continue;
            consider(matrix.fullPivLu().solve(Eigen::Vector2d(
                -rows[first].constant,-rows[second].constant)));
        }
    }
    if (!best.has_value()) {
        result.reason="hard_row_intersection_empty";
        return result;
    }
    result.valid=true;
    result.witness=*best;
    result.bound_mps2=best->norm();
    result.reason="minimum_horizontal_norm_witness";
    return result;
}

}  // namespace task14_input_detail

inline Task14InputBoundResult task14MinimumHorizontalNorm(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,double gamma,
    double tolerance=1e-10) {
    return task14_input_detail::minimumNorm(
        task14_input_detail::ownerHalfspaces(rows,owner,gamma),tolerance);
}

inline Task14InputBoundResult task14MinimumAxisBox(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,double gamma,
    double maximum_bound,double tolerance=1e-8) {
    Task14InputBoundResult result;
    if (!std::isfinite(maximum_bound) || maximum_bound<0.0 ||
        !std::isfinite(tolerance) || tolerance<=0.0) {
        result.reason="invalid_search_interval";
        return result;
    }
    const auto solve=[&](double bound) {
        const auto value=solveCanonicalGammaStar(rows,owner,bound);
        return value;
    };
    const auto high=solve(maximum_bound);
    if (!high.valid || high.gamma<gamma-tolerance) {
        result.reason="infeasible_at_maximum_bound";
        return result;
    }
    double lower=0.0,upper=maximum_bound;
    BridgeGammaStarSolution2D witness=high;
    while (upper-lower>tolerance) {
        const double middle=0.5*(lower+upper);
        const auto candidate=solve(middle);
        if (candidate.valid && candidate.gamma>=gamma) {
            upper=middle;
            witness=candidate;
        } else lower=middle;
    }
    const auto final=solve(upper);
    if (final.valid) witness=final;
    result.valid=true;
    result.bound_mps2=upper;
    result.witness={witness.accelX,witness.accelY};
    result.reason="minimum_per_axis_half_box_witness";
    return result;
}

inline Task14DiskGammaResult task14SolveDiskGamma(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,double radius,
    double tolerance=1e-8) {
    Task14DiskGammaResult result;
    if (!std::isfinite(radius) || radius<0.0 ||
        !std::isfinite(tolerance) || tolerance<=0.0) return result;
    const auto owner_rows=task14_input_detail::ownerHalfspaces(rows,owner,0.0);
    if (owner_rows.empty()) {
        result.valid=true;
        result.gamma=std::numeric_limits<double>::infinity();
        return result;
    }
    double lower=std::numeric_limits<double>::infinity();
    double upper=std::numeric_limits<double>::infinity();
    for (const auto& row:owner_rows) {
        lower=std::min(lower,row.constant-radius*row.coefficient.norm());
        upper=std::min(upper,row.constant+radius*row.coefficient.norm());
    }
    Eigen::Vector2d best=Eigen::Vector2d::Zero();
    for (int iteration=0;iteration<100 && upper-lower>tolerance;++iteration) {
        const double middle=0.5*(lower+upper);
        const auto candidate=task14MinimumHorizontalNorm(
            rows,owner,middle,tolerance*0.1);
        if (candidate.valid && candidate.bound_mps2<=radius+tolerance) {
            lower=middle;
            best=candidate.witness;
        } else upper=middle;
    }
    const auto final=task14MinimumHorizontalNorm(
        rows,owner,lower,tolerance*0.1);
    if (!final.valid || final.bound_mps2>radius+tolerance) return result;
    result.valid=true;
    result.gamma=lower;
    result.control=final.witness;
    return result;
}

}  // namespace gf
