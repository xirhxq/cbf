#pragma once

#include "grand_finale/Task10p11xRecoveryCampaign.hpp"

#include <limits>

namespace gf {

enum class Task10p11aaGraphDepth { H1, H2FiniteWitness };

struct Task10p11aaGraphEndpoint {
    bool feasible=false;
    std::string status="not_solved";
    std::map<NodeId,Eigen::Vector2d> controls;
    double margin_mps2=-std::numeric_limits<double>::infinity();
    double recomputed_gamma_mps2=-std::numeric_limits<double>::infinity();
    double minimum_full_row_residual_mps2=
        -std::numeric_limits<double>::infinity();
};

struct Task10p11aaGraphCandidate {
    std::size_t index=0;
    double alpha=0.0;
    std::map<NodeId,Eigen::Vector2d> controls;
    bool current_feasible=false;
    double current_full_row_residual_mps2=
        -std::numeric_limits<double>::infinity();
    double current_gamma_mps2=-std::numeric_limits<double>::infinity();
    std::size_t current_row_count=0;
    bool successor_feasible=false;
    double successor_gamma_mps2=-std::numeric_limits<double>::infinity();
    double successor_full_row_residual_mps2=
        -std::numeric_limits<double>::infinity();
    std::size_t successor_row_count=0;
    bool predecessor_witness=false;
    std::size_t predecessor_witness_index=0;
    double coverage_deviation_l2_mps2=0.0;
    bool independent_current_residual_recomputed=false;
    bool independent_successor_residual_recomputed=false;
};

struct Task10p11aaGraphDecision {
    bool valid=false;
    std::string reason;
    Task10p11aaGraphDepth depth=Task10p11aaGraphDepth::H1;
    Task10p11aaGraphEndpoint current_endpoint;
    std::vector<Task10p11aaGraphCandidate> candidates;
    std::size_t selected_index=0;
    std::map<NodeId,Eigen::Vector2d> selected_controls;
    std::string selection;
    std::string predecessor_semantics=
        "not_applicable_for_h1";
    bool recursive_feasibility_claimed=false;
};

namespace task10p11aa_detail {

constexpr double kTolerance=1.0e-8;

inline double gammaResidual(const Task10p11sRows28d& problem,
                            const Eigen::VectorXd& controls) {
    double gamma=std::numeric_limits<double>::infinity();
    for (const auto& row:problem.rows)
        if (row.participates_in_gamma)
            gamma=std::min(gamma,row.residual(controls));
    return gamma;
}

struct FullStateAudit {
    Task10p11sRows28d problem;
    task10p11w_detail::RestrictedResult margin;
    double recomputed_gamma=-std::numeric_limits<double>::infinity();
    double minimum_residual=-std::numeric_limits<double>::infinity();
    bool feasible=false;
};

inline FullStateAudit fullStateAudit(
    const CanonicalHardRowRequest& request,const Eigen::VectorXd& nominal) {
    FullStateAudit result;
    const auto rows=buildCanonicalHardRows(request);
    result.problem=buildTask10p11sRows28d(rows,request.mobile_ids,true);
    const std::set<NodeId> all(request.mobile_ids.begin(),
                               request.mobile_ids.end());
    result.margin=task10p11w_detail::solveRestricted(
        result.problem,nominal,nominal,all,true);
    if (result.margin.feasible) {
        const auto residual=task10p11w_detail::minimumResidual(
            result.problem,result.margin.controls);
        result.minimum_residual=residual.first;
        result.recomputed_gamma=gammaResidual(
            result.problem,result.margin.controls);
        result.feasible=result.minimum_residual>=-kTolerance &&
            result.recomputed_gamma>=-kTolerance;
    }
    return result;
}

inline JointEstimateSnapshot predictEstimate(
    const nlohmann::json& snapshot,const JointEstimateSnapshot& estimate,
    const std::map<NodeId,Eigen::Vector2d>& controls) {
    const auto& parameters=snapshot.at("successor_parameters");
    return predictNoMeasurementSnapshot(estimate,controls,
        parameters.at("dt_s").get<double>(),
        parameters.at("estimator_acceleration_variance").get<double>());
}

inline bool finitePredecessorWitness(
    const nlohmann::json& snapshot,
    const JointEstimateSnapshot& estimate1,
    const CanonicalHardRowRequest& request1,
    const Task10p11sRows28d& problem1,
    const Eigen::VectorXd& coverage1,
    const task10p11w_detail::RestrictedResult& maximum1,
    std::size_t& witness_index) {
    if (!maximum1.feasible) return false;
    for (std::size_t index=0;index<9;++index) {
        const double alpha=static_cast<double>(index)/8.0;
        const Eigen::VectorXd control1=(1.0-alpha)*coverage1+
            alpha*maximum1.controls;
        if (task10p11w_detail::minimumResidual(
                problem1,control1).first < -kTolerance) continue;
        const auto map1=task10p11sControlMap(request1.mobile_ids,control1);
        const auto estimate2=predictEstimate(snapshot,estimate1,map1);
        const auto request2=task10p11x_detail::requestAtEstimate(
            snapshot,estimate2);
        const auto state2=fullStateAudit(request2,control1);
        if (state2.feasible) {
            witness_index=index;
            return true;
        }
    }
    return false;
}

}  // namespace task10p11aa_detail

inline std::map<NodeId,Eigen::Vector2d>
task10p11aaAppliedControlsFromCheckpoint(const nlohmann::json& checkpoint) {
    const auto& encoded=checkpoint.at("task10p11z").at("decision")
        .at("applied_controls");
    std::map<NodeId,Eigen::Vector2d> controls;
    for (auto iterator=encoded.begin();iterator!=encoded.end();++iterator) {
        const auto value=iterator.value().get<std::vector<double>>();
        if (value.size()!=2) throw std::runtime_error("invalid applied control");
        controls.emplace(static_cast<NodeId>(std::stoull(iterator.key())),
            Eigen::Vector2d(value[0],value[1]));
    }
    if (controls.size()!=14)
        throw std::runtime_error("checkpoint applied controls incomplete");
    return controls;
}

inline Task10p11aaGraphDecision task10p11aaDecideGraphOracle(
    const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& baseline_controls,
    Task10p11aaGraphDepth depth) {
    using namespace task10p11aa_detail;
    Task10p11aaGraphDecision result;
    result.depth=depth;
    result.predecessor_semantics=depth==Task10p11aaGraphDepth::H1
        ?"not_applicable_for_h1"
        :"finite_9x9_full_pair_witness_not_unrestricted_predecessor";
    try {
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        if (baseline_controls.size()!=request.mobile_ids.size())
            throw std::runtime_error("distributed_baseline_incomplete");
        const auto current_rows=buildCanonicalHardRows(request);
        const auto current_problem=buildTask10p11sRows28d(
            current_rows,request.mobile_ids,true);
        const auto baseline=task10p11sOrderedControls(
            request.mobile_ids,baseline_controls);
        const std::set<NodeId> all(request.mobile_ids.begin(),
                                   request.mobile_ids.end());
        const auto maximum=task10p11w_detail::solveRestricted(
            current_problem,baseline,baseline,all,true);
        result.current_endpoint.status=maximum.status;
        result.current_endpoint.feasible=maximum.feasible;
        result.current_endpoint.margin_mps2=maximum.margin;
        if (!maximum.feasible)
            throw std::runtime_error("current_full_pair_maximum_margin_unavailable:"+
                                     maximum.status);
        result.current_endpoint.controls=task10p11sControlMap(
            request.mobile_ids,maximum.controls);
        result.current_endpoint.recomputed_gamma_mps2=gammaResidual(
            current_problem,maximum.controls);
        result.current_endpoint.minimum_full_row_residual_mps2=
            task10p11w_detail::minimumResidual(
                current_problem,maximum.controls).first;

        const auto estimate0=task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        for (std::size_t index=0;index<9;++index) {
            Task10p11aaGraphCandidate candidate;
            candidate.index=index;
            candidate.alpha=static_cast<double>(index)/8.0;
            const Eigen::VectorXd controls=(1.0-candidate.alpha)*baseline+
                candidate.alpha*maximum.controls;
            candidate.controls=task10p11sControlMap(
                request.mobile_ids,controls);
            candidate.coverage_deviation_l2_mps2=(controls-baseline).norm();
            candidate.current_row_count=current_problem.rows.size();
            candidate.current_full_row_residual_mps2=
                task10p11w_detail::minimumResidual(
                    current_problem,controls).first;
            candidate.current_gamma_mps2=gammaResidual(
                current_problem,controls);
            candidate.current_feasible=
                candidate.current_full_row_residual_mps2>=-kTolerance;
            candidate.independent_current_residual_recomputed=true;

            const auto estimate1=predictEstimate(
                snapshot,estimate0,candidate.controls);
            const auto request1=task10p11x_detail::requestAtEstimate(
                snapshot,estimate1);
            const auto state1=fullStateAudit(request1,controls);
            candidate.successor_row_count=state1.problem.rows.size();
            candidate.successor_gamma_mps2=state1.recomputed_gamma;
            candidate.successor_full_row_residual_mps2=
                state1.minimum_residual;
            candidate.successor_feasible=state1.feasible;
            candidate.independent_successor_residual_recomputed=
                state1.margin.feasible;
            if (depth==Task10p11aaGraphDepth::H2FiniteWitness &&
                candidate.successor_feasible) {
                const auto coverage1=solveTask10p11sQp(
                    state1.problem,controls);
                if (coverage1.feasible)
                    candidate.predecessor_witness=finitePredecessorWitness(
                        snapshot,estimate1,request1,state1.problem,
                        coverage1.controls,state1.margin,
                        candidate.predecessor_witness_index);
            }
            result.candidates.push_back(std::move(candidate));
        }

        std::optional<std::size_t> admissible;
        for (std::size_t index=0;index<result.candidates.size();++index) {
            const auto& candidate=result.candidates[index];
            const bool depth_gate=depth==Task10p11aaGraphDepth::H1 ||
                candidate.predecessor_witness;
            if (candidate.current_feasible && candidate.successor_feasible &&
                depth_gate) {
                admissible=index;
                break;
            }
        }
        if (admissible.has_value()) {
            result.selected_index=*admissible;
            result.selection=depth==Task10p11aaGraphDepth::H1
                ?"least_coverage_deviation_successor_full_pair_feasible"
                :"least_coverage_deviation_finite_predecessor_witness";
            result.valid=true;
            result.reason="selected_admissible_fixed_homotopy_candidate";
        } else if (depth==Task10p11aaGraphDepth::H1) {
            std::optional<std::size_t> best;
            for (std::size_t index=0;index<result.candidates.size();++index) {
                const auto& candidate=result.candidates[index];
                if (!candidate.current_feasible) continue;
                if (!best.has_value() || candidate.successor_gamma_mps2>
                    result.candidates[*best].successor_gamma_mps2)
                    best=index;
            }
            if (!best.has_value())
                throw std::runtime_error("no_current_full_row_feasible_candidate");
            result.selected_index=*best;
            result.selection="maximum_predicted_global_margin_fallback";
            result.valid=result.candidates[*best].successor_gamma_mps2>=
                -kTolerance &&
                result.candidates[*best].successor_full_row_residual_mps2>=
                -kTolerance;
            result.reason=result.valid
                ?"selected_maximum_predicted_global_margin"
                :"maximum_predicted_global_margin_negative_fail_closed";
        } else {
            result.reason="no_finite_predecessor_witness_fail_closed";
        }
        result.selected_controls=
            result.candidates.at(result.selected_index).controls;
    } catch (const std::exception& error) {
        result.valid=false;
        result.reason=error.what();
    }
    return result;
}

inline nlohmann::json task10p11aaGraphDecisionJson(
    const Task10p11aaGraphDecision& value) {
    nlohmann::json candidates=nlohmann::json::array();
    for (const auto& candidate:value.candidates) {
        candidates.push_back({{"index",candidate.index},
            {"alpha",candidate.alpha},{"current_feasible",candidate.current_feasible},
            {"current_full_row_residual_mps2",task10p11w_detail::number(
                candidate.current_full_row_residual_mps2)},
            {"current_global_gamma_mps2",task10p11w_detail::number(
                candidate.current_gamma_mps2)},
            {"successor_feasible",candidate.successor_feasible},
            {"successor_global_gamma_mps2",task10p11w_detail::number(
                candidate.successor_gamma_mps2)},
            {"successor_full_row_residual_mps2",task10p11w_detail::number(
                candidate.successor_full_row_residual_mps2)},
            {"predecessor_witness",candidate.predecessor_witness},
            {"predecessor_witness_index",candidate.predecessor_witness_index},
            {"coverage_deviation_l2_mps2",candidate.coverage_deviation_l2_mps2},
            {"current_row_count",candidate.current_row_count},
            {"successor_row_count",candidate.successor_row_count}});
    }
    return {{"valid",value.valid},{"reason",value.reason},
        {"depth",value.depth==Task10p11aaGraphDepth::H1?"H1":"H2"},
        {"selection",value.selection},{"selected_index",value.selected_index},
        {"predecessor_semantics",value.predecessor_semantics},
        {"candidates",std::move(candidates)},
        {"development_centralized_oracle",true},
        {"recursive_feasibility_claimed",false}};
}

}  // namespace gf
