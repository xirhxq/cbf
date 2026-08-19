#pragma once

#include "grand_finale/Task10p11wConflictComponentOracle.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <functional>
#include <set>
#include <vector>

namespace gf {

struct Task10p11zFarkasCertificate {
    bool valid=false;
    double contradiction=0.0;
    double stationarity_max_abs=0.0;
    std::vector<std::pair<std::string,double>> support;
};

struct Task10p11zComponentAttempt {
    std::set<NodeId> component;
    bool current_feasible=false;
    task10p11w_detail::RestrictedResult current;
    Task10p11zFarkasCertificate infeasibility_certificate;
    std::vector<std::string> iis_rows;
    bool successor_performed=false;
    bool successor_feasible=false;
    task10p11w_detail::RestrictedResult successor;
    Task10p11sQpResult successor_full;
    Task10p11zFarkasCertificate successor_infeasibility_certificate;
    std::vector<std::string> successor_iis_rows;
};

struct Task10p11zCurrentResidualAudit {
    std::size_t row_count=0;
    double minimum_residual_mps2=0.0;
    std::string limiting_row_id;
};

struct Task10p11zSuccessorResidualAudit {
    std::size_t current_row_count=0;
    std::size_t successor_row_count=0;
    double current_minimum_residual_mps2=0.0;
    double successor_minimum_residual_mps2=0.0;
    std::string current_limiting_row_id;
    std::string successor_limiting_row_id;
};

struct Task10p11zComponentGate {
    bool valid=false;
    std::string reason;
    std::string snapshot_conditioning=
        "two_applied_4d_prefix_cycles";
    std::size_t enumeration_size_limit=0;
    bool pair_2_4_current_feasible=false;
    bool full_28d_current_feasible=false;
    std::set<NodeId> current_minimum_component;
    std::set<NodeId> successor_minimum_component;
    Task10p11zCurrentResidualAudit current_selected;
    Task10p11zSuccessorResidualAudit successor_selected;
    std::vector<Task10p11zComponentAttempt> attempts;
    std::map<NodeId,std::set<NodeId>> connected_graph;
};

namespace task10p11z_component_detail {

constexpr double kTolerance=1.0e-8;

inline std::map<NodeId,std::set<NodeId>> coupledGraph(
    const Task10p11sRows28d& problem) {
    std::map<NodeId,std::set<NodeId>> graph;
    for (NodeId owner:problem.mobile_ids) graph[owner];
    for (const auto& row:problem.rows) {
        if (!row.coupled_mobile_pair) continue;
        const auto owners=task10p11w_detail::rowOwners(
            row,problem.mobile_ids);
        if (owners.size()!=2) continue;
        const NodeId first=*owners.begin();
        const NodeId second=*std::next(owners.begin());
        graph[first].insert(second);
        graph[second].insert(first);
    }
    return graph;
}

inline bool connected(const std::set<NodeId>& component,
    const std::map<NodeId,std::set<NodeId>>& graph) {
    if (component.empty()) return false;
    std::set<NodeId> reached{*component.begin()};
    std::vector<NodeId> frontier{*component.begin()};
    while (!frontier.empty()) {
        const NodeId owner=frontier.back();
        frontier.pop_back();
        for (NodeId peer:graph.at(owner))
            if (component.count(peer)!=0 && reached.insert(peer).second)
                frontier.push_back(peer);
    }
    return reached==component;
}

inline std::vector<std::set<NodeId>> connectedComponentsAtSize(
    const std::vector<NodeId>& owners,std::size_t size,
    const std::map<NodeId,std::set<NodeId>>& graph) {
    std::vector<NodeId> optional;
    for (NodeId owner:owners)
        if (owner!=2 && owner!=4) optional.push_back(owner);
    std::vector<std::set<NodeId>> output;
    if (size<2 || size-2>optional.size()) return output;
    std::vector<NodeId> chosen;
    std::function<void(std::size_t,std::size_t)> enumerate=
        [&](std::size_t offset,std::size_t remaining) {
            if (remaining==0) {
                std::set<NodeId> component{2,4};
                component.insert(chosen.begin(),chosen.end());
                if (connected(component,graph)) output.push_back(component);
                return;
            }
            if (optional.size()-offset<remaining) return;
            for (std::size_t index=offset;
                 index+remaining<=optional.size();++index) {
                chosen.push_back(optional[index]);
                enumerate(index+1,remaining-1);
                chosen.pop_back();
            }
        };
    enumerate(0,size-2);
    return output;
}

inline std::pair<Eigen::MatrixXd,Eigen::VectorXd> reducedSystem(
    const Task10p11sRows28d& problem,const Eigen::VectorXd& frozen,
    const std::set<NodeId>& component) {
    std::vector<std::size_t> component_indices;
    for (std::size_t index=0;index<problem.mobile_ids.size();++index)
        if (component.count(problem.mobile_ids[index])!=0)
            component_indices.push_back(index);
    Eigen::MatrixXd coefficients(problem.rows.size(),
        static_cast<Eigen::Index>(2*component_indices.size()));
    Eigen::VectorXd constants(problem.rows.size());
    coefficients.setZero();
    for (std::size_t row_index=0;row_index<problem.rows.size();++row_index) {
        const auto& row=problem.rows[row_index];
        double constant=row.constant;
        std::size_t variable_index=0;
        for (std::size_t owner_index=0;
             owner_index<problem.mobile_ids.size();++owner_index) {
            const bool inside=component.count(
                problem.mobile_ids[owner_index])!=0;
            for (int axis=0;axis<2;++axis) {
                const double coefficient=row.coefficient(
                    static_cast<Eigen::Index>(2*owner_index+axis));
                if (inside) {
                    coefficients(static_cast<Eigen::Index>(row_index),
                        static_cast<Eigen::Index>(2*variable_index+axis))=
                        coefficient;
                } else {
                    constant+=coefficient*frozen(
                        static_cast<Eigen::Index>(2*owner_index+axis));
                }
            }
            if (inside) ++variable_index;
        }
        constants(static_cast<Eigen::Index>(row_index))=constant;
    }
    return {coefficients,constants};
}

inline Task10p11zFarkasCertificate farkasCertificate(
    const Task10p11sRows28d& problem,const Eigen::VectorXd& frozen,
    const std::set<NodeId>& component) {
    Task10p11zFarkasCertificate result;
#ifdef ENABLE_GUROBI
    const auto [coefficients,constants]=reducedSystem(
        problem,frozen,component);
    try {
        GRBEnv environment(true);
        task10p11t_detail::configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        std::vector<GRBVar> multiplier;
        multiplier.reserve(problem.rows.size());
        for (std::size_t index=0;index<problem.rows.size();++index)
            multiplier.push_back(model.addVar(
                0.0,GRB_INFINITY,0.0,GRB_CONTINUOUS));
        GRBLinExpr normalization=0.0;
        for (auto& value:multiplier) normalization+=value;
        model.addConstr(normalization==1.0);
        for (Eigen::Index column=0;column<coefficients.cols();++column) {
            GRBLinExpr stationarity=0.0;
            for (Eigen::Index row=0;row<coefficients.rows();++row)
                stationarity+=coefficients(row,column)*
                    multiplier[static_cast<std::size_t>(row)];
            model.addConstr(stationarity==0.0);
        }
        GRBLinExpr objective=0.0;
        for (Eigen::Index row=0;row<constants.size();++row)
            objective+=constants(row)*multiplier[static_cast<std::size_t>(row)];
        model.setObjective(objective,GRB_MINIMIZE);
        model.optimize();
        if (model.get(GRB_IntAttr_Status)!=GRB_OPTIMAL) return result;
        Eigen::VectorXd weights(constants.size());
        for (Eigen::Index row=0;row<weights.size();++row) {
            weights(row)=multiplier[static_cast<std::size_t>(row)]
                .get(GRB_DoubleAttr_X);
            if (weights(row)>1.0e-9)
                result.support.emplace_back(
                    problem.rows[static_cast<std::size_t>(row)].id,
                    weights(row));
        }
        result.contradiction=constants.dot(weights);
        result.stationarity_max_abs=coefficients.cols()==0?0.0:
            (coefficients.transpose()*weights).cwiseAbs().maxCoeff();
        result.valid=result.contradiction< -1.0e-9 &&
            result.stationarity_max_abs<=kTolerance &&
            !result.support.empty();
    } catch (...) {
        result.valid=false;
    }
#else
    (void)problem;(void)frozen;(void)component;
#endif
    return result;
}

inline std::vector<std::string> iisRows(
    const Task10p11sRows28d& problem,const Eigen::VectorXd& frozen,
    const std::set<NodeId>& component) {
    std::vector<std::string> result;
#ifdef ENABLE_GUROBI
    const auto [coefficients,constants]=reducedSystem(
        problem,frozen,component);
    try {
        GRBEnv environment(true);
        task10p11t_detail::configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        std::vector<GRBVar> variables;
        for (Eigen::Index column=0;column<coefficients.cols();++column)
            variables.push_back(model.addVar(
                -GRB_INFINITY,GRB_INFINITY,0.0,GRB_CONTINUOUS));
        std::vector<GRBConstr> constraints;
        constraints.reserve(problem.rows.size());
        for (Eigen::Index row=0;row<coefficients.rows();++row) {
            GRBLinExpr expression=constants(row);
            for (Eigen::Index column=0;column<coefficients.cols();++column)
                expression+=coefficients(row,column)*
                    variables[static_cast<std::size_t>(column)];
            constraints.push_back(model.addConstr(expression>=0.0));
        }
        GRBLinExpr zero_objective=0.0;
        model.setObjective(zero_objective,GRB_MINIMIZE);
        model.optimize();
        if (model.get(GRB_IntAttr_Status)!=GRB_INFEASIBLE) return result;
        model.computeIIS();
        for (std::size_t index=0;index<constraints.size();++index)
            if (constraints[index].get(GRB_IntAttr_IISConstr)!=0)
                result.push_back(problem.rows[index].id);
    } catch (...) {
        result.clear();
    }
#else
    (void)problem;(void)frozen;(void)component;
#endif
    return result;
}

inline Task10p11zCurrentResidualAudit currentAudit(
    const Task10p11sRows28d& problem,const Eigen::VectorXd& controls) {
    const auto residual=task10p11w_detail::minimumResidual(problem,controls);
    return {problem.rows.size(),residual.first,residual.second};
}

}  // namespace task10p11z_component_detail

inline Task10p11zComponentGate runTask10p11zComponentGate(
    const std::filesystem::path& checkpoint,std::size_t size_limit) {
    using namespace task10p11z_component_detail;
    Task10p11zComponentGate gate;
    gate.enumeration_size_limit=size_limit;
    if (size_limit<2 || size_limit>=14) {
        gate.reason="component_size_limit_must_be_between_2_and_13";
        return gate;
    }
    try {
        const auto frame=task10p11w_detail::readFrame(checkpoint);
        if (std::abs(frame.time_s-133.0)>1.0e-8) {
            gate.reason="checkpoint_is_not_133p0";
            return gate;
        }
        gate.connected_graph=coupledGraph(frame.problem);
        gate.full_28d_current_feasible=frame.full.feasible &&
            frame.full.minimum_residual>=-kTolerance;
        std::optional<Eigen::VectorXd> current_selected_controls;
        std::optional<Eigen::VectorXd> successor_selected_controls;
        std::optional<task10p11w_detail::SuccessorAudit> successor_selected;
        for (std::size_t size=2;size<=size_limit;++size) {
            for (const auto& component:connectedComponentsAtSize(
                     frame.request.mobile_ids,size,gate.connected_graph)) {
                Task10p11zComponentAttempt attempt;
                attempt.component=component;
                attempt.current=task10p11w_detail::solveRestricted(
                    frame.problem,frame.nominal,frame.frozen,component,false);
                attempt.current_feasible=attempt.current.feasible &&
                    attempt.current.minimum_residual>=-kTolerance;
                if (!attempt.current_feasible) {
                    attempt.infeasibility_certificate=farkasCertificate(
                        frame.problem,frame.frozen,component);
                    attempt.iis_rows=iisRows(
                        frame.problem,frame.frozen,component);
                } else {
                    const auto successor=task10p11w_detail::successorAudit(
                        frame.snapshot,attempt.current.controls,component,
                        frame.request.mobile_ids);
                    attempt.successor_performed=true;
                    attempt.successor=successor.component;
                    attempt.successor_full=successor.full;
                    attempt.successor_feasible=successor.component.feasible &&
                        successor.component.minimum_residual>=-kTolerance &&
                        successor.full.feasible &&
                        successor.full.minimum_residual>=-kTolerance;
                    if (!attempt.successor.feasible) {
                        const auto successor_rows=buildCanonicalHardRows(
                            successor.request);
                        const auto successor_frozen=
                            task10p11w_detail::frozenLocalControls(
                                successor_rows,successor.request,
                                task10p11sControlMap(frame.request.mobile_ids,
                                    attempt.current.controls));
                        attempt.successor_infeasibility_certificate=
                            farkasCertificate(successor.problem,
                                successor_frozen,component);
                        attempt.successor_iis_rows=iisRows(
                            successor.problem,successor_frozen,component);
                    }
                    if (gate.current_minimum_component.empty()) {
                        gate.current_minimum_component=component;
                        current_selected_controls=attempt.current.controls;
                    }
                    if (attempt.successor_feasible &&
                        gate.successor_minimum_component.empty()) {
                        gate.successor_minimum_component=component;
                        successor_selected_controls=attempt.current.controls;
                        successor_selected=successor;
                    }
                }
                if (component==std::set<NodeId>{2,4})
                    gate.pair_2_4_current_feasible=attempt.current_feasible;
                gate.attempts.push_back(std::move(attempt));
                if (!gate.successor_minimum_component.empty()) break;
            }
            if (!gate.successor_minimum_component.empty()) break;
        }
        if (gate.current_minimum_component.empty()) {
            gate.reason="no_current_component_within_size_limit";
            return gate;
        }
        if (gate.successor_minimum_component.empty()) {
            gate.reason="no_successor_component_within_size_limit";
            return gate;
        }
        gate.current_selected=currentAudit(
            frame.problem,*current_selected_controls);
        const auto current_residual=currentAudit(
            frame.problem,*successor_selected_controls);
        const auto successor_residual=currentAudit(
            successor_selected->problem,
            successor_selected->component.controls);
        gate.successor_selected={current_residual.row_count,
            successor_residual.row_count,
            current_residual.minimum_residual_mps2,
            successor_residual.minimum_residual_mps2,
            current_residual.limiting_row_id,
            successor_residual.limiting_row_id};
        gate.valid=gate.full_28d_current_feasible &&
            !gate.pair_2_4_current_feasible &&
            gate.current_selected.minimum_residual_mps2>=-kTolerance &&
            gate.successor_selected.current_minimum_residual_mps2>=-kTolerance &&
            gate.successor_selected.successor_minimum_residual_mps2>=-kTolerance;
        gate.reason=gate.valid?"component_gate_passed":
            "component_gate_residual_or_full_oracle_failed";
    } catch (const std::exception& error) {
        gate.reason=error.what();
    }
    return gate;
}

inline nlohmann::json task10p11zFarkasJson(
    const Task10p11zFarkasCertificate& value) {
    nlohmann::json support=nlohmann::json::array();
    for (const auto& [row,multiplier]:value.support)
        support.push_back({{"row_id",row},{"multiplier",multiplier}});
    return {{"valid",value.valid},{"contradiction",value.contradiction},
        {"stationarity_max_abs",value.stationarity_max_abs},
        {"support",std::move(support)}};
}

inline nlohmann::json task10p11zRestrictedSummaryJson(
    const task10p11w_detail::RestrictedResult& value) {
    return {{"status",value.status},{"feasible",value.feasible},
        {"objective",task10p11w_detail::number(value.objective)},
        {"margin_mps2",task10p11w_detail::number(value.margin)},
        {"minimum_full_row_residual_mps2",
            task10p11w_detail::number(value.minimum_residual)},
        {"limiting_row_id",value.limiting_row}};
}

inline nlohmann::json task10p11zComponentGateJson(
    const Task10p11zComponentGate& gate) {
    nlohmann::json attempts=nlohmann::json::array();
    for (const auto& attempt:gate.attempts) {
        attempts.push_back({{"component",task10p11w_detail::idsJson(
                attempt.component)},
            {"current_feasible",attempt.current_feasible},
            {"current",task10p11zRestrictedSummaryJson(attempt.current)},
            {"infeasibility_certificate",task10p11zFarkasJson(
                attempt.infeasibility_certificate)},
            {"iis_rows",attempt.iis_rows},
            {"successor_performed",attempt.successor_performed},
            {"successor_feasible",attempt.successor_feasible},
            {"successor_infeasibility_certificate",task10p11zFarkasJson(
                attempt.successor_infeasibility_certificate)},
            {"successor_iis_rows",attempt.successor_iis_rows}});
    }
    nlohmann::json graph=nlohmann::json::object();
    for (const auto& [owner,neighbors]:gate.connected_graph)
        graph[std::to_string(owner)]=std::vector<NodeId>(
            neighbors.begin(),neighbors.end());
    return {{"protocol","task10p11z-component-gate-v1"},
        {"valid",gate.valid},{"reason",gate.reason},
        {"snapshot_conditioning",gate.snapshot_conditioning},
        {"enumeration_size_limit",gate.enumeration_size_limit},
        {"pair_2_4_current_feasible",gate.pair_2_4_current_feasible},
        {"full_28d_current_feasible",gate.full_28d_current_feasible},
        {"current_protocol_minimum_component",
            task10p11w_detail::idsJson(gate.current_minimum_component)},
        {"successor_protocol_minimum_component",
            task10p11w_detail::idsJson(gate.successor_minimum_component)},
        {"current_selected",{{"row_count",gate.current_selected.row_count},
            {"minimum_residual_mps2",
                gate.current_selected.minimum_residual_mps2},
            {"limiting_row_id",gate.current_selected.limiting_row_id}}},
        {"successor_selected",{{"current_row_count",
                gate.successor_selected.current_row_count},
            {"successor_row_count",
                gate.successor_selected.successor_row_count},
            {"current_minimum_residual_mps2",
                gate.successor_selected.current_minimum_residual_mps2},
            {"successor_minimum_residual_mps2",
                gate.successor_selected.successor_minimum_residual_mps2},
            {"current_limiting_row_id",
                gate.successor_selected.current_limiting_row_id},
            {"successor_limiting_row_id",
                gate.successor_selected.successor_limiting_row_id}}},
        {"connected_graph",std::move(graph)},
        {"attempts",std::move(attempts)},
        {"gamma_star_role","diagnostic_not_parameter"},
        {"claim_boundary",{{"finite_snapshot_only",true},
            {"recursive_feasibility_claimed",false},
            {"production_14_owner_centralized_controller",false}}}};
}

}  // namespace gf
