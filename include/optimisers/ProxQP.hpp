#ifndef CBF_OPTIMISER_PROXQP_HPP
#define CBF_OPTIMISER_PROXQP_HPP

#include <Eigen/Dense>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include "OptimiserBase.hpp"

class ProxQP : public OptimiserBase {
    using QP = proxsuite::proxqp::dense::QP<double>;

    int var_count = 0;
    int u_size = 0;
    int constraint_count = 0;

    Eigen::VectorXd solution;
    Eigen::MatrixXd H;
    Eigen::MatrixXd C;
    Eigen::VectorXd gradient;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    std::unique_ptr<QP> qp;
    double objective_offset = 0.0;
    const double bound = 1.0e20;

    mutable bool has_error = false;
    mutable std::string last_error_message;
    mutable int last_error_code = 0;

    std::string statusString() const {
        if (!qp) {
            return "not_initialized";
        }

        switch (qp->results.info.status) {
            case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED:
                return "optimal";
            case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED:
                return "max_iter_reached";
            case proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
                return "primal_infeasible";
            case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
                return "closest_primal_feasible";
            case proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
                return "dual_infeasible";
            case proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN:
                return "not_run";
        }

        return "unknown";
    }

public:
    ProxQP(json &settings): OptimiserBase(settings) {}

    void clear() override {
        var_count = 0;
        u_size = 0;
        constraint_count = 0;
        solution.resize(0);
        H.resize(0, 0);
        C.resize(0, 0);
        gradient.resize(0);
        lowerBound.resize(0);
        upperBound.resize(0);
        qp.reset();
        objective_offset = 0.0;
        has_error = false;
        last_error_message = "";
        last_error_code = 0;
    }

    void start(int total_size, int u_sz) override {
        var_count = total_size;
        u_size = u_sz;
        gradient.resize(total_size);
        gradient.setZero();
    }

    void setObjective(Eigen::VectorXd &uNominal) override {
        H = Eigen::MatrixXd::Zero(var_count, var_count);
        gradient = Eigen::VectorXd::Zero(var_count);

        for (int i = 0; i < u_size; i++) {
            H(i, i) = 2.0;
            gradient[i] = -2.0 * uNominal[i];
        }
        for (int i = u_size; i < var_count; i++) {
            gradient[i] = k_delta;
        }

        objective_offset = uNominal.squaredNorm();
    }

    void addLinearConstraint(const Eigen::VectorXd &coe, double rhs_value) override {
        if (constraint_count == 0) {
            C.resize(1, var_count);
            lowerBound.resize(1);
            upperBound.resize(1);
        } else {
            C.conservativeResize(constraint_count + 1, var_count);
            lowerBound.conservativeResize(constraint_count + 1);
            upperBound.conservativeResize(constraint_count + 1);
        }

        C.row(constraint_count) = coe.transpose();
        lowerBound[constraint_count] = rhs_value;
        upperBound[constraint_count] = bound;
        constraint_count++;
    }

    void write(std::string filename) override {
        std::cout << "ProxQP model: " << filename << " (variables: " << var_count
                  << ", constraints: " << constraint_count << ")" << std::endl;
    }

    double getObjectiveValue() const override {
        if (solution.size() == 0) {
            return 0.0;
        }

        return 0.5 * solution.dot(H * solution) + gradient.dot(solution) + objective_offset;
    }

    json getStatus() const override {
        json status;

        if (has_error) {
            status["status"] = "failed";
            status["error_code"] = last_error_code;
            status["error"] = last_error_message;
            return status;
        }

        status["status"] = statusString();
        status["objective_value"] = getObjectiveValue();
        status["vars_count"] = var_count;
        status["constraints_count"] = constraint_count;

        if (qp) {
            status["iterations"] = qp->results.info.iter;
            status["primal_residual"] = qp->results.info.pri_res;
            status["dual_residual"] = qp->results.info.dua_res;
        }

        return status;
    }

    Eigen::VectorXd solve() override {
        try {
            Eigen::MatrixXd Aeq(0, var_count);
            Eigen::VectorXd beq(0);

            const int slack_count = var_count - u_size;
            const int total_constraints = constraint_count + slack_count;
            Eigen::MatrixXd C_full = Eigen::MatrixXd::Zero(total_constraints, var_count);
            Eigen::VectorXd lower_full(total_constraints);
            Eigen::VectorXd upper_full(total_constraints);

            if (constraint_count > 0) {
                C_full.topRows(constraint_count) = C;
                lower_full.head(constraint_count) = lowerBound;
                upper_full.head(constraint_count) = upperBound;
            }

            for (int i = 0; i < slack_count; i++) {
                const int row = constraint_count + i;
                const int col = u_size + i;
                C_full(row, col) = 1.0;
                lower_full[row] = 0.0;
                upper_full[row] = bound;
            }

            qp = std::make_unique<QP>(var_count, 0, total_constraints);
            qp->settings.eps_abs = 1e-8;
            qp->settings.eps_rel = 1e-8;
            qp->settings.max_iter = 10000;
            qp->settings.verbose = false;
            qp->init(H, gradient, Aeq, beq, C_full, lower_full, upper_full, true);
            qp->solve();

            if (qp->results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
                throw std::runtime_error("ProxQP solver did not find optimal solution: " + statusString());
            }

            solution = qp->results.x;
            for (int i = 0; i < solution.size(); i++) {
                if (!std::isfinite(solution[i])) {
                    throw std::runtime_error("ProxQP returned an invalid solution");
                }
            }

            has_error = false;
            last_error_code = 0;
            last_error_message = "";

            return solution;
        } catch (const std::exception& e) {
            has_error = true;
            last_error_code = -1;
            last_error_message = std::string("ProxQP error: ") + e.what();
            throw;
        } catch (...) {
            has_error = true;
            last_error_code = -2;
            last_error_message = "Unknown ProxQP error";
            throw;
        }
    }
};

#endif // CBF_OPTIMISER_PROXQP_HPP
