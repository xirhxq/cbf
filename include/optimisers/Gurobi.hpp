#ifndef CBF_OPTIMISER_GUROBI_HPP
#define CBF_OPTIMISER_GUROBI_HPP

#include "utils.h"
#include "OptimiserBase.hpp"
#include "gurobi_c++.h"

class Gurobi : public OptimiserBase{
    GRBEnv env;
    GRBModel *model{};
    std::vector<GRBVar> vars;
    GRBQuadExpr obj = 0.0;

public:
    Gurobi(json &settings): OptimiserBase(settings), env(true) {
        env.set(GRB_IntParam_OutputFlag, 0);
        env.start();
    }

    void clear() override {
        vars.clear();
        delete model;
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

    void setObjective(Eigen::VectorXd &uNominal) override {
        obj = 0.0;
        for (int i = 0; i < uNominal.size(); i++) {
            obj += (vars[i] - uNominal[i]) * (vars[i] - uNominal[i]);
        }
        for (int i = uNominal.size(); i < vars.size(); i++) {
            obj += k_delta * vars[i];
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

    void write(std::string filename) override {
        model->write(filename);
    }

    double getObjectiveValue() const override {
        if (model) {
            return model->get(GRB_DoubleAttr_ObjVal);
        }
        return 0.0;
    }

    Eigen::VectorXd solve() override {
        try {
            model->optimize();
            Eigen::VectorXd u(vars.size());
            for (int i = 0; i < vars.size(); i++) {
                u[i] = vars[i].get(GRB_DoubleAttr_X);
            }
            return u;
        } catch (GRBException e) {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        } catch (...) {
            std::cout << "Exception during optimization" << std::endl;
        }
        return Eigen::VectorXd::Zero(vars.size());
    }
};

#endif // CBF_OPTIMISER_GUROBI_HPP