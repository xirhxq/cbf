#pragma once

#include "grand_finale/CanonicalGammaStarFeedback.hpp"

#include <cmath>
#include <string>

namespace gf {

struct GammaFeedbackEquivalenceResult {
    bool equivalent = false;
    std::string first_mismatch;
};

namespace gamma_feedback_equivalence_detail {

inline bool close(double lhs,double rhs,double tolerance) {
    if (std::isinf(lhs) || std::isinf(rhs)) return lhs==rhs;
    return std::isfinite(lhs) && std::isfinite(rhs) &&
        std::abs(lhs-rhs)<=tolerance;
}

inline bool sameRow(const CanonicalHardRow& lhs,const CanonicalHardRow& rhs,
                    double tolerance) {
    return lhs.id==rhs.id && lhs.kind==rhs.kind && lhs.owner==rhs.owner &&
        lhs.peer==rhs.peer &&
        (lhs.normal-rhs.normal).norm()<=tolerance &&
        (lhs.control_coefficient-rhs.control_coefficient).norm()<=tolerance &&
        close(lhs.constant,rhs.constant,tolerance) &&
        close(lhs.responsibility,rhs.responsibility,tolerance) &&
        lhs.participates_in_gamma==rhs.participates_in_gamma &&
        close(lhs.barrier_h,rhs.barrier_h,tolerance) &&
        close(lhs.barrier_psi1,rhs.barrier_psi1,tolerance) &&
        close(lhs.barrier_hdot,rhs.barrier_hdot,tolerance) &&
        close(lhs.coefficient_uncertainty_reserve,
              rhs.coefficient_uncertainty_reserve,tolerance) &&
        lhs.tube_provenance==rhs.tube_provenance &&
        close(lhs.position_uncertainty_reserve_m,
              rhs.position_uncertainty_reserve_m,tolerance) &&
        close(lhs.velocity_uncertainty_reserve_mps,
              rhs.velocity_uncertainty_reserve_mps,tolerance);
}

}

inline GammaFeedbackEquivalenceResult compareCanonicalGammaFeedbackBatches(
    const CanonicalGammaFeedbackBatchResult& lhs,
    const CanonicalGammaFeedbackBatchResult& rhs,double tolerance) {
    using namespace gamma_feedback_equivalence_detail;
    const auto fail=[](std::string path) {
        return GammaFeedbackEquivalenceResult{false,std::move(path)};
    };
    if (lhs.valid!=rhs.valid) return fail("valid");
    if (lhs.reason!=rhs.reason) return fail("reason");
    if (lhs.current_rows.size()!=rhs.current_rows.size())
        return fail("current_rows.size");
    for (std::size_t index=0;index<lhs.current_rows.size();++index)
        if (!sameRow(lhs.current_rows[index],rhs.current_rows[index],tolerance))
            return fail("current_rows["+std::to_string(index)+"]");
    if (lhs.stages.size()!=rhs.stages.size()) return fail("stages.size");
    for (const auto& [owner,a] : lhs.stages) {
        const auto found=rhs.stages.find(owner);
        if (found==rhs.stages.end()) return fail("stages.owner");
        const auto& b=found->second;
        if (a.valid!=b.valid || a.reason!=b.reason ||
            !close(a.current_gamma,b.current_gamma,tolerance) ||
            (a.task_projection-b.task_projection).norm()>tolerance ||
            (a.maximum_margin_control-b.maximum_margin_control).norm()>tolerance ||
            a.current_row_ids!=b.current_row_ids ||
            a.candidates.size()!=b.candidates.size())
            return fail("stages["+std::to_string(owner)+"]");
        for (std::size_t index=0;index<a.candidates.size();++index)
            if ((a.candidates[index]-b.candidates[index]).norm()>tolerance)
                return fail("stages["+std::to_string(owner)+"].candidates");
    }
    if (lhs.selections.size()!=rhs.selections.size())
        return fail("selections.size");
    for (const auto& [owner,a] : lhs.selections) {
        const auto found=rhs.selections.find(owner);
        if (found==rhs.selections.end()) return fail("selections.owner");
        const auto& b=found->second;
        if (a.valid!=b.valid || a.reason!=b.reason ||
            (a.selected_control-b.selected_control).norm()>tolerance ||
            !close(a.nominal_predicted_gamma,b.nominal_predicted_gamma,tolerance) ||
            !close(a.maximum_margin_candidate_predicted_gamma,
                   b.maximum_margin_candidate_predicted_gamma,tolerance) ||
            !close(a.selected_predicted_gamma,b.selected_predicted_gamma,tolerance) ||
            !close(a.controllable_predicted_gamma_range,
                   b.controllable_predicted_gamma_range,tolerance) ||
            a.intervened!=b.intervened ||
            a.dominant_row!=b.dominant_row ||
            a.fallback_reason!=b.fallback_reason)
            return fail("selections["+std::to_string(owner)+"]");
    }
    if (lhs.selected_controls.size()!=rhs.selected_controls.size())
        return fail("selected_controls.size");
    for (const auto& [owner,control] : lhs.selected_controls) {
        const auto found=rhs.selected_controls.find(owner);
        if (found==rhs.selected_controls.end()) return fail("selected_controls.owner");
        if ((control-found->second).norm()>tolerance)
            return fail("selected_controls["+std::to_string(owner)+"]");
    }
    return {true,{}};
}

}  // namespace gf
