#pragma once

#include "grand_finale/Task10p11afSuccessorRecovery.hpp"

#include <chrono>
#include <set>

namespace gf {

struct Task10p11agFrozenProtocol {
    double tau_mps2=22.0;
    double dt_s=0.1;
    double feasibility_tolerance_mps2=1.0e-8;
    double oracle_time_limit_s=12.0*60.0*60.0;
    std::string initialization="P3";
    bool fixed_topology=true;
    bool full_domain_u0=true;
    bool full_domain_u1=true;
    bool recursive_feasibility_claimed=false;
};

inline Task10p11agFrozenProtocol task10p11agFrozenProtocol() { return {}; }

namespace task10p11ag_detail {

constexpr double kTolerance=1.0e-8;

struct Evaluation {
    bool solved=false;
    bool witness=false;
    std::string status="not_solved";
    Eigen::VectorXd u0;
    Eigen::VectorXd u1;
    JointEstimateSnapshot x1;
    Task10p11sRows28d successor_problem;
    double current_minimum_residual=-std::numeric_limits<double>::infinity();
    std::string current_limiting_row;
    double successor_gamma=-std::numeric_limits<double>::infinity();
    double successor_minimum_residual=-std::numeric_limits<double>::infinity();
    std::string successor_limiting_row;
};

struct Context {
    nlohmann::json snapshot;
    CanonicalHardRowRequest current_request;
    Task10p11sRows28d current_problem;
    Eigen::VectorXd distributed;
    JointEstimateSnapshot x0;
    std::set<NodeId> all;
};

inline Context makeContext(const nlohmann::json& snapshot,
                           const Eigen::VectorXd& distributed) {
    Context result;
    result.snapshot=snapshot;
    const auto validation=validateTask10p11sSnapshot(snapshot);
    if (!validation.complete)
        throw std::invalid_argument("Task10p11ag requires complete packed snapshot");
    result.current_request=task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    if (distributed.size()!=static_cast<Eigen::Index>(
            2*result.current_request.mobile_ids.size()))
        throw std::invalid_argument("distributed command is not 28D");
    result.current_problem=buildTask10p11sRows28d(
        buildCanonicalHardRows(result.current_request),
        result.current_request.mobile_ids,true);
    result.distributed=distributed;
    result.x0=task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    result.all.insert(result.current_request.mobile_ids.begin(),
                      result.current_request.mobile_ids.end());
    return result;
}

inline Evaluation evaluate(const Context& context,const Eigen::VectorXd& u0) {
    Evaluation result;
    result.u0=u0;
    try {
        const auto current=task10p11w_detail::minimumResidual(
            context.current_problem,u0);
        result.current_minimum_residual=current.first;
        result.current_limiting_row=current.second;
        const auto controls=task10p11sControlMap(
            context.current_request.mobile_ids,u0);
        result.x1=task10p11aa_detail::predictEstimate(
            context.snapshot,context.x0,controls);
        const auto request1=task10p11x_detail::requestAtEstimate(
            context.snapshot,result.x1);
        result.successor_problem=buildTask10p11sRows28d(
            buildCanonicalHardRows(request1),request1.mobile_ids,true);
        const auto maximum=task10p11w_detail::solveRestricted(
            result.successor_problem,u0,u0,context.all,true);
        result.status=maximum.status;
        if (!maximum.feasible || maximum.controls.size()!=u0.size()) return result;
        result.u1=maximum.controls;
        result.successor_gamma=task10p11aa_detail::gammaResidual(
            result.successor_problem,result.u1);
        const auto successor=task10p11w_detail::minimumResidual(
            result.successor_problem,result.u1);
        result.successor_minimum_residual=successor.first;
        result.successor_limiting_row=successor.second;
        result.solved=true;
        result.witness=result.current_minimum_residual>=-kTolerance &&
            result.successor_gamma>=-kTolerance &&
            result.successor_minimum_residual>=-kTolerance;
    } catch (const std::exception& error) {
        result.status=std::string("error:")+error.what();
    }
    return result;
}

#ifdef ENABLE_GUROBI
inline std::optional<Eigen::VectorXd> linearTrustStep(
    const Context& context,const Eigen::VectorXd& center,
    const Eigen::VectorXd& gradient,double radius,
    bool minimize_deviation,double linearized_gamma) {
    try {
        GRBEnv environment(true);
        task10p11t_detail::configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        std::vector<GRBVar> variable;
        variable.reserve(static_cast<std::size_t>(center.size()));
        for (Eigen::Index index=0;index<center.size();++index) {
            const double lower=std::max(-context.current_request.acceleration_half_box,
                                        center(index)-radius);
            const double upper=std::min(context.current_request.acceleration_half_box,
                                        center(index)+radius);
            variable.push_back(model.addVar(lower,upper,0.0,GRB_CONTINUOUS));
        }
        for (const auto& row:context.current_problem.rows) {
            GRBLinExpr residual=row.constant;
            for (Eigen::Index index=0;index<center.size();++index)
                if (row.coefficient(index)!=0.0)
                    residual+=row.coefficient(index)*variable[
                        static_cast<std::size_t>(index)];
            model.addConstr(residual>=0.0);
        }
        if (minimize_deviation) {
            GRBLinExpr tangent=linearized_gamma;
            for (Eigen::Index index=0;index<center.size();++index)
                tangent+=gradient(index)*(variable[
                    static_cast<std::size_t>(index)]-center(index));
            model.addConstr(tangent>=2.0e-8);
            GRBQuadExpr objective=0.0;
            for (Eigen::Index index=0;index<center.size();++index)
                objective+=(variable[static_cast<std::size_t>(index)]-
                    context.distributed(index))*(variable[
                    static_cast<std::size_t>(index)]-
                    context.distributed(index));
            model.setObjective(objective,GRB_MINIMIZE);
        } else {
            GRBLinExpr objective=0.0;
            for (Eigen::Index index=0;index<center.size();++index)
                objective+=gradient(index)*(variable[
                    static_cast<std::size_t>(index)]-center(index));
            model.setObjective(objective,GRB_MAXIMIZE);
        }
        model.optimize();
        if (model.get(GRB_IntAttr_Status)!=GRB_OPTIMAL) return std::nullopt;
        Eigen::VectorXd result(center.size());
        for (Eigen::Index index=0;index<center.size();++index)
            result(index)=variable[static_cast<std::size_t>(index)]
                .get(GRB_DoubleAttr_X);
        return result;
    } catch (...) {
        return std::nullopt;
    }
}
#endif

inline Eigen::VectorXd finiteDifferenceGradient(
    const Context& context,const Evaluation& center,
    const std::chrono::steady_clock::time_point& deadline,
    std::size_t& evaluations) {
    Eigen::VectorXd gradient=Eigen::VectorXd::Zero(center.u0.size());
    const double epsilon=1.0e-4;
    for (Eigen::Index index=0;index<center.u0.size();++index) {
        if (std::chrono::steady_clock::now()>=deadline) break;
        Eigen::VectorXd perturbed=center.u0;
        const double direction=perturbed(index)+epsilon<=
                context.current_request.acceleration_half_box?1.0:-1.0;
        perturbed(index)+=direction*epsilon;
        const auto value=evaluate(context,perturbed);
        ++evaluations;
        if (value.solved)
            gradient(index)=direction*(value.successor_gamma-
                center.successor_gamma)/epsilon;
    }
    return gradient;
}

inline nlohmann::json controlsJson(const std::vector<NodeId>& owners,
                                   const Eigen::VectorXd& controls) {
    return task10p11w_detail::controlsJson(owners,controls);
}

inline nlohmann::json residualsJson(const Task10p11sRows28d& problem,
                                    const Eigen::VectorXd& controls,
                                    double active_tolerance=1.0e-7) {
    nlohmann::json rows=nlohmann::json::array();
    for (const auto& row:problem.rows) {
        const double residual=row.residual(controls);
        rows.push_back({{"row_id",row.id},{"residual_mps2",residual},
            {"participates_in_gamma",row.participates_in_gamma},
            {"active",std::abs(residual)<=active_tolerance},
            {"coupled_mobile_pair",row.coupled_mobile_pair}});
    }
    return rows;
}

}  // namespace task10p11ag_detail

struct Task10p11agFullDomainResult {
    bool valid=false;
    bool witness_found=false;
    bool strict_infeasibility_proven=false;
    bool timed_out=false;
    std::string classification="undetermined";
    std::string reason;
    double solve_time_s=0.0;
    std::size_t nonlinear_evaluations=0;
    std::size_t outer_iterations=0;
    double best_successor_gamma_mps2=-std::numeric_limits<double>::infinity();
    double deviation_l2_mps2=std::numeric_limits<double>::infinity();
    task10p11ag_detail::Evaluation witness;
    nlohmann::json trace=nlohmann::json::array();
};

inline Task10p11agFullDomainResult solveTask10p11agFullDomainPredecessor(
    const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& distributed_controls,
    double time_limit_s=12.0*60.0*60.0) {
    using namespace task10p11ag_detail;
    Task10p11agFullDomainResult result;
    const auto started=std::chrono::steady_clock::now();
    const auto deadline=started+std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(std::chrono::duration<double>(
            time_limit_s));
    try {
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const Eigen::VectorXd distributed=task10p11sOrderedControls(
            request.mobile_ids,distributed_controls);
        const Context context=makeContext(snapshot,distributed);
        Evaluation best=evaluate(context,distributed);
        ++result.nonlinear_evaluations;
        if (!best.solved)
            throw std::runtime_error("initial successor margin unavailable:"+
                                     best.status);
        result.trace.push_back({{"iteration",0},{"source","distributed"},
            {"successor_gamma_mps2",best.successor_gamma},
            {"current_residual_mps2",best.current_minimum_residual}});

        const auto current_maximum=task10p11w_detail::solveRestricted(
            context.current_problem,distributed,distributed,context.all,true);
        if (current_maximum.feasible) {
            const auto endpoint=evaluate(context,current_maximum.controls);
            ++result.nonlinear_evaluations;
            if (endpoint.solved && endpoint.successor_gamma>
                    best.successor_gamma)
                best=endpoint;
            result.trace.push_back({{"iteration",0},
                {"source","current_full_pair_maximum_margin_start"},
                {"successor_gamma_mps2",endpoint.solved?
                    nlohmann::json(endpoint.successor_gamma):nlohmann::json(nullptr)},
                {"current_residual_mps2",endpoint.current_minimum_residual}});
        }

        double radius=2.0;
        for (std::size_t iteration=1;iteration<=30 &&
                std::chrono::steady_clock::now()<deadline && !best.witness;
                ++iteration) {
            ++result.outer_iterations;
            const auto gradient=finiteDifferenceGradient(
                context,best,deadline,result.nonlinear_evaluations);
            if (!gradient.allFinite() || gradient.norm()<1.0e-10) break;
#ifdef ENABLE_GUROBI
            const auto proposed=linearTrustStep(
                context,best.u0,gradient,radius,false,best.successor_gamma);
#else
            const std::optional<Eigen::VectorXd> proposed=std::nullopt;
#endif
            if (!proposed.has_value()) break;
            Evaluation candidate=evaluate(context,*proposed);
            ++result.nonlinear_evaluations;
            bool accepted=candidate.solved &&
                candidate.current_minimum_residual>=-kTolerance &&
                candidate.successor_gamma>best.successor_gamma+1.0e-10;
            if (!accepted) {
                Eigen::VectorXd line=*proposed;
                for (int reduction=0;reduction<8 && !accepted;++reduction) {
                    line=0.5*(line+best.u0);
                    candidate=evaluate(context,line);
                    ++result.nonlinear_evaluations;
                    accepted=candidate.solved &&
                        candidate.current_minimum_residual>=-kTolerance &&
                        candidate.successor_gamma>best.successor_gamma+1.0e-10;
                }
            }
            result.trace.push_back({{"iteration",iteration},
                {"source","full_28d_trust_region"},
                {"accepted",accepted},{"trust_radius_mps2",radius},
                {"successor_gamma_mps2",candidate.solved?
                    nlohmann::json(candidate.successor_gamma):nlohmann::json(nullptr)},
                {"current_residual_mps2",candidate.current_minimum_residual}});
            if (accepted) {
                best=std::move(candidate);
                radius=std::min(4.0,1.5*radius);
            } else {
                radius*=0.35;
                if (radius<1.0e-5) break;
            }
        }

        if (best.witness) {
            // First remove unnecessary deviation along the deterministic segment
            // to U^0.  Then use a full-28D local SQP refinement.  The saved
            // independent residual, rather than local optimality, certifies the
            // existence claim.
            Evaluation feasible=best;
            Evaluation infeasible=evaluate(context,distributed);
            ++result.nonlinear_evaluations;
            for (int iteration=0;iteration<45 &&
                    std::chrono::steady_clock::now()<deadline;++iteration) {
                const Eigen::VectorXd midpoint=0.5*(feasible.u0+infeasible.u0);
                const auto value=evaluate(context,midpoint);
                ++result.nonlinear_evaluations;
                if (value.witness) feasible=value; else infeasible=value;
            }
            for (std::size_t iteration=0;iteration<8 &&
                    std::chrono::steady_clock::now()<deadline;++iteration) {
                const auto gradient=finiteDifferenceGradient(
                    context,feasible,deadline,result.nonlinear_evaluations);
#ifdef ENABLE_GUROBI
                const auto proposed=linearTrustStep(context,feasible.u0,
                    gradient,0.5,true,feasible.successor_gamma);
#else
                const std::optional<Eigen::VectorXd> proposed=std::nullopt;
#endif
                if (!proposed.has_value()) break;
                auto value=evaluate(context,*proposed);
                ++result.nonlinear_evaluations;
                if (!value.witness) {
                    Eigen::VectorXd low=*proposed;
                    Eigen::VectorXd high=feasible.u0;
                    for (int line=0;line<35;++line) {
                        const Eigen::VectorXd midpoint=0.5*(low+high);
                        const auto trial=evaluate(context,midpoint);
                        ++result.nonlinear_evaluations;
                        if (trial.witness) { high=midpoint; value=trial; }
                        else low=midpoint;
                    }
                }
                if (value.witness &&
                    (value.u0-distributed).norm()<
                        (feasible.u0-distributed).norm()-1.0e-9)
                    feasible=value;
                else break;
            }
            result.witness=std::move(feasible);
            result.witness_found=true;
            result.valid=true;
            result.classification="feasible_witness";
            result.reason="independently_recomputed_current_and_successor_full_rows_nonnegative";
            result.best_successor_gamma_mps2=result.witness.successor_gamma;
            result.deviation_l2_mps2=(result.witness.u0-distributed).norm();
        } else {
            result.valid=true;
            result.witness=best;
            result.best_successor_gamma_mps2=best.successor_gamma;
            result.classification="undetermined";
            result.reason="no_feasible_witness_and_no_global_infeasibility_upper_bound";
        }
        result.timed_out=std::chrono::steady_clock::now()>=deadline;
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    result.solve_time_s=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-started).count();
    return result;
}

inline nlohmann::json task10p11agFullDomainJson(
    const Task10p11agFullDomainResult& result,
    const std::vector<NodeId>& owners) {
    using namespace task10p11ag_detail;
    nlohmann::json witness=nullptr;
    if (result.witness.solved) {
        witness={{"U0",controlsJson(owners,result.witness.u0)},
            {"x1",task10p11s_capture_detail::estimateJson(result.witness.x1)},
            {"U1",controlsJson(owners,result.witness.u1)},
            {"current_minimum_residual_mps2",
                result.witness.current_minimum_residual},
            {"current_limiting_row_id",result.witness.current_limiting_row},
            {"successor_global_gamma_mps2",result.witness.successor_gamma},
            {"successor_minimum_residual_mps2",
                result.witness.successor_minimum_residual},
            {"successor_limiting_row_id",
                result.witness.successor_limiting_row},
            {"current_all_1113_residuals",nullptr},
            {"successor_all_1113_residuals",residualsJson(
                result.witness.successor_problem,result.witness.u1)}};
    }
    return {{"protocol","task10p11ag-full-domain-predecessor-v1"},
        {"valid",result.valid},{"classification",result.classification},
        {"reason",result.reason},{"witness_found",result.witness_found},
        {"strict_infeasibility_proven",result.strict_infeasibility_proven},
        {"timed_out",result.timed_out},{"solve_time_s",result.solve_time_s},
        {"nonlinear_evaluations",result.nonlinear_evaluations},
        {"outer_iterations",result.outer_iterations},
        {"best_successor_gamma_mps2",task10p11w_detail::number(
            result.best_successor_gamma_mps2)},
        {"coverage_control_deviation_l2_mps2",task10p11w_detail::number(
            result.deviation_l2_mps2)},
        {"witness",std::move(witness)},{"optimization_trace",result.trace},
        {"model",{{"current_control_dimension",28},
            {"successor_control_dimension",28},
            {"full_pair_row_count",1113},
            {"uncertainty_reserve","once_per_coupled_mobile_pair"},
            {"non_gamma_rows","hard_not_relaxed"},
            {"fixed_topology",true}}},
        {"claim_boundary",{{"positive_witness_proves_existence",true},
            {"negative_local_search_proves_infeasibility",false},
            {"recursive_feasibility_claimed",false},
            {"production_14_owner_centralized_controller",false}}}};
}

}  // namespace gf
