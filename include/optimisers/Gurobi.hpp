#ifndef CBF_OPTIMISER_GUROBI_HPP
#define CBF_OPTIMISER_GUROBI_HPP

#include "utils.h"
#include "OptimiserBase.hpp"
#include "gurobi_c++.h"
#include <chrono>
#include <memory>
#include "diagnostics/CanonicalCaptureAudit.hpp"

class Gurobi : public OptimiserBase{
    GRBEnv env;
    GRBModel *model{};
    std::vector<GRBVar> vars;
    GRBQuadExpr obj = 0.0;
    int auxiliaryCount = 0;
    bool quadraticSlack = false;
    int commandBallRowCount = 0;

    mutable bool has_error = false;
    mutable std::string last_error_message;
    mutable int last_error_code = 0;
    mutable double last_solve_time_ms = 0.0;

public:
    Gurobi(json &settings): OptimiserBase(settings), env(true) {
        const std::string penalty = settings.value("slack-penalty", "linear");
        if (penalty != "linear" && penalty != "quadratic") throw std::invalid_argument("unknown slack penalty");
        quadraticSlack = penalty == "quadratic";
        env.set(GRB_IntParam_OutputFlag, 0);
        env.start();
        if(canonical::active)++canonical::environmentStartsCompleted;
    }

    void clear() override {
        vars.clear();
        delete model;
        model = nullptr;
        has_error = false;
        last_error_message = "";
        last_error_code = 0;
        last_solve_time_ms = 0.0;
        auxiliaryCount = 0;
        commandBallRowCount = 0;
    }

    void start(int total_size, int u_size) override {
        model = new GRBModel(env);
        std::vector<double> lowerBounds(total_size, -GRB_INFINITY);
        std::vector<double> upperBounds(total_size, GRB_INFINITY);
        std::vector<double> objCoeffs(total_size, 0.0);
        std::vector<char> varTypes(total_size, GRB_CONTINUOUS);
        GRBVar *allVars = model->addVars(
                lowerBounds.data(),
                upperBounds.data(),
                objCoeffs.data(),
                varTypes.data(),
                nullptr,
                total_size
        );
        vars.assign(allVars, allVars + total_size);
        // Set lower bounds for slack variables [u_size, total_size)
        for (int i = u_size; i < total_size; ++i) {
            vars[i].set(GRB_DoubleAttr_LB, 0.0);
        }
    }

    void setAuxiliaryVariableCount(int count) override {
        if (count < 0) throw std::invalid_argument("negative auxiliary variable count");
        auxiliaryCount = count;
    }

    void setObjective(Eigen::VectorXd &uNominal) override {
        obj = 0.0;
        for (int i = 0; i < uNominal.size(); i++) {
            obj += (vars[i] - uNominal[i]) * (vars[i] - uNominal[i]);
        }
        for (int i = uNominal.size(); i < static_cast<int>(vars.size()) - auxiliaryCount; i++) {
            if (quadraticSlack) obj += k_delta * vars[i] * vars[i];
            else obj += k_delta * vars[i];
        }
        model->setObjective(obj, GRB_MINIMIZE);
    }

    void addLinearConstraint(Eigen::VectorXd coe, double rhs) override {
        GRBLinExpr ln = 0.0;
        for (int i = 0; i < coe.size(); i++) {
            ln += coe[i] * vars[i];
        }
        model->addConstr(ln, '>', rhs);
    }

    void addPlanarNormCone(int auxiliaryIndex, const Eigen::Vector2d& offset) override {
        if (auxiliaryIndex < 3 || auxiliaryIndex >= static_cast<int>(vars.size()))
            throw std::invalid_argument("invalid planar cone index");
        model->set(GRB_DoubleParam_BarQCPConvTol, 1e-8);
        // Positive q with (vx-offset_x)^2+(vy-offset_y)^2 <= q^2.
        // Gurobi recognizes this as a convex second-order cone.
        GRBQuadExpr expression = (vars[0]-offset[0])*(vars[0]-offset[0])
            + (vars[1]-offset[1])*(vars[1]-offset[1])
            - vars[auxiliaryIndex]*vars[auxiliaryIndex];
        model->addQConstr(expression <= 0.0);
    }

    void addConcaveAuxiliaryRow(const Eigen::VectorXd& coefficient,double rhs,int auxiliaryIndex,double squareCoefficient) override {
        if(auxiliaryIndex<3 || auxiliaryIndex>=static_cast<int>(vars.size()) || !std::isfinite(squareCoefficient) || squareCoefficient>0)
            throw std::invalid_argument("invalid convex auxiliary row");
        model->set(GRB_IntParam_NumericFocus,3);
        GRBQuadExpr expression=squareCoefficient*vars[auxiliaryIndex]*vars[auxiliaryIndex];
        for(int i=0;i<coefficient.size();++i)expression+=coefficient[i]*vars[i];
        model->addQConstr(expression>=rhs);++commandBallRowCount;
    }

    void write(std::string filename) override {
        model->write(filename);
    }

    double canoExpressionAtPrimal() const {return obj.getValue();}
    double canoModelExpressionAtPrimal() const {return model->getObjective().getValue();}
    json sampledHoldPrimalDiagnostics() const {
        json d={{"linear_rows",json::array()},
                {"linear_residual_definition","nonnegative is feasible; solver row order"},
                {"feasibility_tol",model->getEnv().get(GRB_DoubleParam_FeasibilityTol)}};
        const int count=model->get(GRB_IntAttr_NumConstrs);
        std::unique_ptr<GRBConstr[]> rows(model->getConstrs());
        for(int i=0;i<count;++i) {
            const char sense=rows[i].get(GRB_CharAttr_Sense);
            const double slack=rows[i].get(GRB_DoubleAttr_Slack);
            const double residual=sense=='>'?-slack:(sense=='<'?slack:-std::abs(slack));
            if(!std::isfinite(residual))throw std::runtime_error("nonfinite diagnostic QP row residual");
            d["linear_rows"].push_back(residual);
        }
        return d;
    }
    double getObjectiveValue() const override {
        try {
            if (model && model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
                return model->get(GRB_DoubleAttr_ObjVal);
            }
        } catch (...) {
            return 0.0;
        }
        return 0.0;
    }

    virtual json getStatus() const override {
        json status;

        if (has_error) {
            status["status"] = "failed";
            status["error_code"] = last_error_code;
            status["error"] = last_error_message;
        } else if (model) {
            try {
                int grb_status = model->get(GRB_IntAttr_Status);
                switch (grb_status) {
                    case GRB_OPTIMAL:
                        status["status"] = "optimal";
                        break;
                    case GRB_INFEASIBLE:
                        status["status"] = "infeasible";
                        break;
                    case GRB_UNBOUNDED:
                        status["status"] = "unbounded";
                        break;
                    case GRB_INF_OR_UNBD:
                        status["status"] = "inf_or_unbounded";
                        break;
                    default:
                        status["status"] = "other";
                        break;
                }
                if (grb_status == GRB_OPTIMAL) {
                    status["objective_value"] = model->get(GRB_DoubleAttr_ObjVal);
                }
                if(commandBallRowCount && grb_status==GRB_OPTIMAL) {
                    status["objective_expression_value"]=obj.getValue();status["model_objective_expression_value"]=model->getObjective().getValue();
                    status["reported_ObjVal_vs_final_primal_difference"]=std::abs(model->get(GRB_DoubleAttr_ObjVal)-obj.getValue());
                    status["reported_ObjVal_matches_primal_at_1e_6"]=std::abs(model->get(GRB_DoubleAttr_ObjVal)-obj.getValue())<1e-6;
                    status["independent_full_numerical_optimality_certified"]=false;
                    status["command_ball_quadratic_row_count"]=commandBallRowCount;
                    status["raw_variables"]=json::array();for(const auto& variable:vars)status["raw_variables"].push_back({{"X",variable.get(GRB_DoubleAttr_X)},{"Obj",variable.get(GRB_DoubleAttr_Obj)},{"LB",variable.get(GRB_DoubleAttr_LB)},{"UB",variable.get(GRB_DoubleAttr_UB)}});
                }
                status["vars_count"] = model->get(GRB_IntAttr_NumVars);
                status["constraints_count"] = model->get(GRB_IntAttr_NumConstrs);
                if (model->get(GRB_IntAttr_NumQConstrs)>0)
                    {status["cone_constraints_count"] = model->get(GRB_IntAttr_NumQConstrs);
                    status["gurobi_status_code"]=grb_status;}
                status["solve_time_ms"] = last_solve_time_ms;
            } catch (...) {
                status["status"] = "error";
                status["error"] = "Failed to get model status";
            }
        } else {
            status["status"] = "not_initialized";
        }

        return status;
    }

    Eigen::VectorXd solve() override {
        canonical::forbid("Gurobi.solve before optimize API");
        try {
            auto start = std::chrono::high_resolution_clock::now();
            model->optimize();
            int status = model->get(GRB_IntAttr_Status);
            if (status == GRB_INF_OR_UNBD) {
                model->set(GRB_IntParam_DualReductions, 0);
                model->optimize();
                status = model->get(GRB_IntAttr_Status);
            }
            auto end = std::chrono::high_resolution_clock::now();

            // Calculate solve time in milliseconds
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            last_solve_time_ms = duration.count() / 1000.0;

            has_error = false;
            last_error_code = 0;
            last_error_message = "";
            if (status != GRB_OPTIMAL) {
                return Eigen::VectorXd::Zero(vars.size());
            }

            Eigen::VectorXd u(vars.size());
            for (int i = 0; i < vars.size(); i++) {
                u[i] = vars[i].get(GRB_DoubleAttr_X);
            }
            return u;
        } catch (GRBException e) {
            has_error = true;
            last_error_code = e.getErrorCode();
            last_error_message = "Gurobi error: " + std::string(e.getMessage());
            return Eigen::VectorXd::Zero(vars.size());
        } catch (...) {
            has_error = true;
            last_error_code = -1;
            last_error_message = "Unknown optimization error";
            return Eigen::VectorXd::Zero(vars.size());
        }
    }
};

#endif // CBF_OPTIMISER_GUROBI_HPP
