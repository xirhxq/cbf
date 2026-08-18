#pragma once

#include "grand_finale/CanonicalHocbfQpController.hpp"

#include <optional>
#include <string>
#include <vector>

namespace gf {

struct Task10p11lSNearBoundaryCase {
    std::string id;
    CanonicalQpRequest request;
    bool exact_feasible = false;
    std::optional<double> expected_gamma_mps2;
};

struct Task10p11lSExactCaseResult {
    bool feasible = false;
    Eigen::Vector2d control = Eigen::Vector2d::Zero();
    double objective = std::numeric_limits<double>::infinity();
    double gamma_mps2 = -std::numeric_limits<double>::infinity();
};

namespace task10p11l_s_family_detail {

inline CanonicalHardRow affine(
    const char* id,CanonicalHardRowKind kind,
    const Eigen::Vector2d& coefficient,double constant,bool gamma=true) {
    return CanonicalHardRow{id,kind,11,std::nullopt,coefficient,coefficient,
        constant,1.0,gamma};
}

inline std::vector<CanonicalHardRow> inputRows() {
    return {
        affine("input:11:ax:lower",CanonicalHardRowKind::InputBox,{1,0},4,false),
        affine("input:11:ax:upper",CanonicalHardRowKind::InputBox,{-1,0},4,false),
        affine("input:11:ay:lower",CanonicalHardRowKind::InputBox,{0,1},4,false),
        affine("input:11:ay:upper",CanonicalHardRowKind::InputBox,{0,-1},4,false),
    };
}

inline Task10p11lSNearBoundaryCase makeCase(
    std::string id,const Eigen::Vector2d& nominal,
    std::vector<CanonicalHardRow> physical,bool feasible,
    std::optional<double> gamma) {
    auto input=inputRows();
    physical.insert(physical.end(),input.begin(),input.end());
    std::sort(physical.begin(),physical.end(),[](const auto& lhs,const auto& rhs) {
        return lhs.id<rhs.id;
    });
    return {std::move(id),CanonicalQpRequest{
        SolverProfile::OpenSource,11,1,1,SupervisorMode::Search,
        nominal,4.0,std::move(physical),1.0e-7},feasible,gamma};
}

}  // namespace task10p11l_s_family_detail

inline std::vector<Task10p11lSNearBoundaryCase>
task10p11lSNearBoundaryFamily() {
    using namespace task10p11l_s_family_detail;
    return {
        makeCase("clear_positive_margin",{-2.0,0.5},
            {affine("collision:clear",CanonicalHardRowKind::Collision,
                    Eigen::Vector2d::UnitX(),1.0)},true,5.0),
        makeCase("near_zero_positive",{0.0,0.0},
            {affine("collision:near-zero",CanonicalHardRowKind::Collision,
                    Eigen::Vector2d::UnitX(),-3.99999)},true,1.0e-5),
        makeCase("exact_zero_boundary",{0.0,0.0},
            {affine("collision:zero",CanonicalHardRowKind::Collision,
                    Eigen::Vector2d::UnitX(),-4.0)},true,0.0),
        makeCase("exact_negative_infeasible",{0.0,0.0},
            {affine("collision:negative",CanonicalHardRowKind::Collision,
                    Eigen::Vector2d::UnitX(),-4.001)},false,-0.001),
        makeCase("single_active_row",{-1.0,0.25},
            {affine("collision:single",CanonicalHardRowKind::Collision,
                    Eigen::Vector2d::UnitX(),0.0)},true,4.0),
        makeCase("multi_active_collision_reference_input",{5.0,5.0},
            {affine("collision:multi",CanonicalHardRowKind::Collision,
                    Eigen::Vector2d::UnitX(),-4.0),
             affine("reference:multi",CanonicalHardRowKind::ReferenceDistance,
                    Eigen::Vector2d::UnitY(),-4.0)},true,0.0),
    };
}

inline Task10p11lSExactCaseResult evaluateTask10p11lSExactCase(
    const Task10p11lSNearBoundaryCase& item) {
    const auto projection=evaluateProgressCompatibility(
        item.request.rows,item.request.owner,item.request.nominal,
        item.request.acceleration_half_box,
        {std::numeric_limits<double>::max(),0.0,
         item.request.residual_tolerance,true});
    const auto gamma=solveCanonicalGammaStar(
        item.request.rows,item.request.owner,item.request.acceleration_half_box);
    Task10p11lSExactCaseResult result;
    result.feasible=projection.polytope_nonempty;
    if (projection.polytope_nonempty) {
        result.control=projection.projection;
        result.objective=(result.control-item.request.nominal).squaredNorm();
    }
    if (gamma.valid) result.gamma_mps2=gamma.gamma;
    return result;
}

}  // namespace gf
