#ifndef CBF_OPTIMISER_HIGHS_HPP
#define CBF_OPTIMISER_HIGHS_HPP

#include "Highs.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <stdexcept>
#include "OptimiserBase.hpp"

class HiGHS : public OptimiserBase {
    Highs highs;
    HighsModel model;
    int var_count = 0;
    int constraint_count = 0;

    Eigen::VectorXd solution;
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd rhs;
    std::vector<double> col_lower;
    std::vector<double> col_upper;
    std::vector<double> obj_coeffs;
    Eigen::SparseMatrix<double> Q;

    double infinity = highs.getInfinity();

public:
    HiGHS() {
        model.lp_.a_matrix_.format_ = MatrixFormat::kColwise;
        model.lp_.sense_ = ObjSense::kMinimize;
        highs.setOptionValue("output_flag", false);
        highs.setOptionValue("log_to_console", false);

    }

    void clear() override {
        var_count = 0;
        constraint_count = 0;
        solution.resize(0);
        A.resize(0, 0);
        rhs.resize(0);
        col_lower.clear();
        col_upper.clear();
        obj_coeffs.clear();
        Q.resize(0, 0);
        model = HighsModel();
    }

    void start(int size) override {
        var_count = size;
        col_lower.resize(size, -highs.getInfinity());
        col_upper.resize(size, highs.getInfinity());
        obj_coeffs.resize(size, 0.0);
    }

    void setObjective(Eigen::VectorXd &uNominal) override {
        Q.resize(var_count, var_count);
        Q.setZero();

        for (int i = 0; i < uNominal.size(); i++) {
            Q.insert(i, i) = 2.0;
            obj_coeffs[i] = -2.0 * uNominal[i];
        }

        for (int i = uNominal.size(); i < var_count; i++) {
            Q.insert(i, i) = 0.2;
        }

        Q.makeCompressed();
        model.hessian_.dim_ = var_count;
        model.hessian_.format_ = HessianFormat::kTriangular;
        model.hessian_.start_.assign(Q.outerIndexPtr(), Q.outerIndexPtr() + Q.outerSize() + 1);
        model.hessian_.index_.assign(Q.innerIndexPtr(), Q.innerIndexPtr() + Q.nonZeros());
        model.hessian_.value_.assign(Q.valuePtr(), Q.valuePtr() + Q.nonZeros());

        model.lp_.col_cost_ = obj_coeffs;
    }

    void addLinearConstraint(Eigen::VectorXd coe, double rhs_value) override {
        if (constraint_count == 0) {
            A.resize(1, var_count);
            rhs.resize(1);
        } else {
            A.conservativeResize(constraint_count + 1, var_count);
            rhs.conservativeResize(constraint_count + 1);
        }
        for (int i = 0; i < coe.size(); i++) {
            A.insert(constraint_count, i) = coe[i];
        }
        rhs[constraint_count] = rhs_value;
        constraint_count++;
    }

    void write(std::string filename) override {
        highs.writeModel(filename);
    }

    Eigen::VectorXd solve() override {

        A.makeCompressed();
        model.lp_.a_matrix_.start_.assign(A.outerIndexPtr(), A.outerIndexPtr() + A.outerSize() + 1);
        model.lp_.a_matrix_.index_.assign(A.innerIndexPtr(), A.innerIndexPtr() + A.nonZeros());
        model.lp_.a_matrix_.value_.assign(A.valuePtr(), A.valuePtr() + A.nonZeros());

        model.lp_.num_col_ = var_count;
        model.lp_.num_row_ = constraint_count;
        model.lp_.col_lower_ = col_lower;
        model.lp_.col_upper_ = col_upper;

        std::vector<double> row_upper(constraint_count, highs.getInfinity());
        model.lp_.row_lower_.assign(rhs.data(), rhs.data() + rhs.size());
        model.lp_.row_upper_ = row_upper;

        HighsStatus return_status = highs.passModel(model);

        if (return_status != HighsStatus::kOk) {
            throw std::runtime_error("Error in passing model to HiGHS");
        }

        return_status = highs.run();
        if (return_status != HighsStatus::kOk) {
            throw std::runtime_error("Error in solving the model with HiGHS");
        }

        const HighsSolution &highs_solution = highs.getSolution();
        solution.resize(var_count);
        for (int i = 0; i < var_count; i++) {
            solution[i] = highs_solution.col_value[i];
        }

        return solution;
    }
};

#endif // CBF_OPTIMISER_HIGHS_HPP
