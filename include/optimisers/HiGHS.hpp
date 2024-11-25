#ifndef CBF_OPTIMISER_HIGHS_HPP
#define CBF_OPTIMISER_HIGHS_HPP

#include "Highs.h"
#include <vector>
#include <Eigen/Dense>
#include <stdexcept>

class HiGHS {
    Highs highs;
    HighsModel model;

public:
    HiGHS() {
        highs.setOptionValue("output_flag", false);
    }

    void start(int num_cols, int num_rows) {
        model.lp_.num_col_ = num_cols;
        model.lp_.num_row_ = num_rows;
        model.lp_.sense_ = ObjSense::kMinimize;
        model.lp_.col_cost_.resize(num_cols, 0.0);
        model.lp_.col_lower_.resize(num_cols, -1.0e30);
        model.lp_.col_upper_.resize(num_cols, 1.0e30);
        model.lp_.row_lower_.resize(num_rows, -1.0e30);
        model.lp_.row_upper_.resize(num_rows, 1.0e30);
        model.lp_.a_matrix_.format_ = MatrixFormat::kColwise;
    }

    void setObjective(const Eigen::VectorXd& costs, double offset = 0) {
        if (costs.size() != model.lp_.num_col_)
            throw std::runtime_error("Objective size mismatch");
        model.lp_.offset_ = offset;
        model.lp_.col_cost_.assign(costs.data(), costs.data() + costs.size());
    }

    void setBounds(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds) {
        if (lower_bounds.size() != model.lp_.num_col_ || upper_bounds.size() != model.lp_.num_col_)
            throw std::runtime_error("Bounds size mismatch");
        model.lp_.col_lower_.assign(lower_bounds.data(), lower_bounds.data() + lower_bounds.size());
        model.lp_.col_upper_.assign(upper_bounds.data(), upper_bounds.data() + upper_bounds.size());
    }

    void addConstraint(const Eigen::VectorXd& coefficients, double lower, double upper) {
        if (coefficients.size() != model.lp_.num_col_)
            throw std::runtime_error("Constraint size mismatch");
        int row_index = model.lp_.num_row_++;
        model.lp_.row_lower_.push_back(lower);
        model.lp_.row_upper_.push_back(upper);
        for (int i = 0; i < coefficients.size(); ++i) {
            if (coefficients[i] != 0.0) {
                model.lp_.a_matrix_.index_.push_back(i);
                model.lp_.a_matrix_.value_.push_back(coefficients[i]);
            }
        }
        model.lp_.a_matrix_.start_.push_back(model.lp_.a_matrix_.index_.size());
    }

    Eigen::VectorXd solve() {
        if (highs.passModel(model) != HighsStatus::kOk)
            throw std::runtime_error("Failed to pass model to HiGHS");
        if (highs.run() != HighsStatus::kOk)
            throw std::runtime_error("HiGHS optimization failed");

        const auto& solution = highs.getSolution();
        return Eigen::Map<const Eigen::VectorXd>(solution.col_value.data(), solution.col_value.size());
    }

    double getObjectiveValue() const {
        return highs.getInfo().objective_function_value;
    }
};

#endif // CBF_OPTIMISER_HIGHS_HPP