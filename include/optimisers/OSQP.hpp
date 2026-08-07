#ifndef CBF_OPTIMISER_OSQP_HPP
#define CBF_OPTIMISER_OSQP_HPP

#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <stdexcept>
#include "OptimiserBase.hpp"

class OSQP : public OptimiserBase {
    mutable OsqpEigen::Solver solver;
    int var_count = 0;
    int u_size = 0;
    int user_constraint_count = 0;  // Constraints added by user

    Eigen::VectorXd solution;
    Eigen::SparseMatrix<double> H;  // Hessian
    Eigen::SparseMatrix<double> A;  // Constraint matrix (includes variable bounds + user constraints)
    Eigen::VectorXd gradient;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    bool solver_initialized = false;
    double objective_offset = 0.0;  // Constant term in objective

    mutable bool has_error = false;
    mutable std::string last_error_message;
    mutable int last_error_code = 0;

public:
    OSQP(json &settings): OptimiserBase(settings) {
        // Configure OSQP settings for robustness
        solver.settings()->setVerbosity(false);
        solver.settings()->setAlpha(1.6);
        solver.settings()->setAbsoluteTolerance(1e-3);
        solver.settings()->setRelativeTolerance(1e-3);
        solver.settings()->setPrimalInfeasibilityTolerance(1e-3);
        solver.settings()->setDualInfeasibilityTolerance(1e-3);
        solver.settings()->setMaxIteration(100000);
        solver.settings()->setPolish(true);  // Enable polish for better accuracy
        solver.settings()->setScaling(100);  // More aggressive scaling
        solver.settings()->setAdaptiveRho(true);
        solver.settings()->setAdaptiveRhoInterval(100);  // Update rho more frequently
        solver.settings()->setAdaptiveRhoTolerance(5);  // More aggressive rho adaptation
        solver.settings()->setRho(10.0);  // Larger rho for constraint-heavy problems
        solver.settings()->setSigma(1e-6);
    }

    void clear() override {
        var_count = 0;
        u_size = 0;
        user_constraint_count = 0;
        solution.resize(0);
        H.resize(0, 0);
        A.resize(0, 0);
        gradient.resize(0);
        lowerBound.resize(0);
        upperBound.resize(0);
        solver_initialized = false;
        objective_offset = 0.0;
        has_error = false;
        last_error_message = "";
        last_error_code = 0;
        // IMPORTANT: clear data BEFORE clearSolver — after clearSolver(),
        // the internal data object may be freed, making data()-> a UAF.
        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();
        solver.clearSolver();
    }

    void start(int total_size, int u_sz) override {
        var_count = total_size;
        u_size = u_sz;
        gradient.resize(total_size);
        gradient.setZero();
    }

    void setObjective(Eigen::VectorXd &uNominal) override {
        // Objective: minimize ||u - uNominal||^2 + k_delta * sum(slack)
        // = 0.5 * x' * P * x + q' * x + constant
        // where P = 2I (diagonal matrix with 2's for control vars, 0 for slack)
        // q = [-2*uNominal; k_delta * ones]
        // constant = ||uNominal||^2

        H.resize(var_count, var_count);
        H.setZero();
        gradient.resize(var_count);

        // Set Hessian: 2 for control variables, 2*k_delta for slack variables
        // Actually, the slack variables should have coefficient k_delta in linear term
        // So Hessian is 2I for control vars, 0 for slack vars
        for (int i = 0; i < u_size; i++) {
            H.insert(i, i) = 2.0;
            gradient[i] = -2.0 * uNominal[i];
        }
        for (int i = u_size; i < var_count; i++) {
            // Slack variables: linear term k_delta, no quadratic term
            gradient[i] = k_delta;
        }

        // Compute constant offset for objective value calculation
        objective_offset = uNominal.squaredNorm();

        H.makeCompressed();
    }

    void addLinearConstraint(Eigen::VectorXd coe, double rhs_value) override {
        if (user_constraint_count == 0) {
            A.resize(1, var_count);
            lowerBound.resize(1);
            upperBound.resize(1);
        } else {
            A.conservativeResize(user_constraint_count + 1, var_count);
            lowerBound.conservativeResize(user_constraint_count + 1);
            upperBound.conservativeResize(user_constraint_count + 1);
        }
        for (int i = 0; i < coe.size(); i++) {
            A.insert(user_constraint_count, i) = coe[i];
        }
        // OSQP uses l <= Ax <= u form
        // Our constraint is: coe'x >= rhs  =>  rhs <= coe'x <= +infty
        lowerBound[user_constraint_count] = rhs_value;
        upperBound[user_constraint_count] = OsqpEigen::INFTY;
        user_constraint_count++;
    }

    void write(std::string filename) override {
        // OSQP doesn't have a direct write function, but we can output debug info
        std::cout << "OSQP model: " << filename << " (variables: " << var_count
                  << ", constraints: " << user_constraint_count << ")" << std::endl;
    }

    double getObjectiveValue() const override {
        if (solver.isInitialized()) {
            double obj = solver.getObjValue();
            // OSQP returns 0.5*x'Px + q'x, we need to add the constant
            return obj + objective_offset;
        }
        return 0.0;
    }

    virtual json getStatus() const override {
        json status;

        if (has_error) {
            status["status"] = "failed";
            status["error_code"] = last_error_code;
            status["error"] = last_error_message;
        } else if (solver.isInitialized() && solution.size() > 0) {
            try {
                status["objective_value"] = getObjectiveValue();
                status["vars_count"] = var_count;
                status["constraints_count"] = user_constraint_count;

                OsqpEigen::Status solver_status = solver.getStatus();
                switch (solver_status) {
                    case OsqpEigen::Status::Solved:
                        status["status"] = "optimal";
                        break;
                    case OsqpEigen::Status::SolvedInaccurate:
                        status["status"] = "optimal_inaccurate";
                        break;
                    case OsqpEigen::Status::MaxIterReached:
                        status["status"] = "max_iter_reached";
                        break;
                    case OsqpEigen::Status::PrimalInfeasible:
                    case OsqpEigen::Status::PrimalInfeasibleInaccurate:
                        status["status"] = "primal_infeasible";
                        break;
                    case OsqpEigen::Status::DualInfeasible:
                    case OsqpEigen::Status::DualInfeasibleInaccurate:
                        status["status"] = "dual_infeasible";
                        break;
                    default:
                        status["status"] = "unknown";
                        break;
                }

                bool valid_solution = true;
                for (int i = 0; i < solution.size(); i++) {
                    if (!std::isfinite(solution[i])) {
                        valid_solution = false;
                        break;
                    }
                }

                if (!valid_solution) {
                    status["status"] = "invalid_solution";
                }
            } catch (...) {
                status["status"] = "error";
                status["error"] = "Failed to get solution status";
            }
        } else {
            status["status"] = "not_initialized";
        }

        return status;
    }

    Eigen::VectorXd solve() override {
        try {
            // Total constraints = user constraints + slack variable bounds
            int num_slack_vars = var_count - u_size;
            int total_constraints = user_constraint_count + num_slack_vars;

            // Build the full constraint matrix including slack variable bounds
            Eigen::SparseMatrix<double> A_full(total_constraints, var_count);
            Eigen::VectorXd lowerBound_full(total_constraints);
            Eigen::VectorXd upperBound_full(total_constraints);

            // Prune small values from user constraint matrix
            A.prune([](const int&, const int&, const double& value) {
                return std::abs(value) > 1e-12;
            });
            A.makeCompressed();

            // Copy user constraints
            for (int k = 0; k < A.outerSize(); ++k) {
                for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
                    A_full.insert(it.row(), it.col()) = it.value();
                }
            }
            for (int i = 0; i < user_constraint_count; i++) {
                lowerBound_full[i] = lowerBound[i];
                upperBound_full[i] = upperBound[i];
            }

            // Add slack variable bounds: s_i >= 0  =>  [0, +infty)
            // This is done by adding identity rows for slack variables
            for (int i = 0; i < num_slack_vars; i++) {
                int row = user_constraint_count + i;
                int col = u_size + i;
                A_full.insert(row, col) = 1.0;
                lowerBound_full[row] = 0.0;
                upperBound_full[row] = OsqpEigen::INFTY;
            }

            A_full.makeCompressed();

            // Set up the problem data
            solver.data()->setNumberOfVariables(var_count);
            solver.data()->setNumberOfConstraints(total_constraints);

            if (!solver.data()->setHessianMatrix(H)) {
                throw std::runtime_error("Failed to set Hessian matrix");
            }

            if (!solver.data()->setGradient(gradient)) {
                throw std::runtime_error("Failed to set gradient");
            }

            if (!solver.data()->setLinearConstraintsMatrix(A_full)) {
                throw std::runtime_error("Failed to set constraint matrix");
            }

            if (!solver.data()->setLowerBound(lowerBound_full)) {
                throw std::runtime_error("Failed to set lower bound");
            }

            if (!solver.data()->setUpperBound(upperBound_full)) {
                throw std::runtime_error("Failed to set upper bound");
            }

            // Initialize the solver
            if (!solver.initSolver()) {
                throw std::runtime_error("Failed to initialize OSQP solver");
            }
            solver_initialized = true;

            // Solve the problem
            OsqpEigen::ErrorExitFlag error = solver.solveProblem();

            if (error != OsqpEigen::ErrorExitFlag::NoError) {
                throw std::runtime_error("OSQP solver failed with error code " + std::to_string(static_cast<int>(error)));
            }

            // Get the solution
            const Eigen::VectorXd& osqp_solution = solver.getSolution();
            solution = osqp_solution.cast<double>();

            // Check if solution is valid
            OsqpEigen::Status status = solver.getStatus();
            if (status != OsqpEigen::Status::Solved && status != OsqpEigen::Status::SolvedInaccurate) {
                std::string status_str;
                switch (status) {
                    case OsqpEigen::Status::MaxIterReached:
                        {
                            bool has_nan_or_inf = false;
                            for (int i = 0; i < solution.size(); i++) {
                                if (std::isnan(solution[i]) || std::isinf(solution[i])) {
                                    has_nan_or_inf = true;
                                    break;
                                }
                            }
                            if (!has_nan_or_inf && solution.size() > 0) {
                                std::cerr << "[OSQP] MaxIterReached but solution is valid, accepting approximate solution" << std::endl;
                                status = OsqpEigen::Status::SolvedInaccurate;
                            } else {
                                status_str = "max_iter_reached (invalid solution)";
                            }
                        }
                        break;
                    case OsqpEigen::Status::PrimalInfeasible:
                    case OsqpEigen::Status::PrimalInfeasibleInaccurate:
                        status_str = "primal_infeasible";
                        break;
                    case OsqpEigen::Status::DualInfeasible:
                    case OsqpEigen::Status::DualInfeasibleInaccurate:
                        status_str = "dual_infeasible";
                        break;
                    default:
                        status_str = "unknown status: " + std::to_string(static_cast<int>(status));
                        break;
                }
                if (status != OsqpEigen::Status::SolvedInaccurate) {
                    // Fail-soft: instead of throwing (which crashes the entire
                    // swarm), return the previous solution (or zeros if none).
                    // The caller (Robot::optimise) checks getStatus() and will
                    // keep the previous frame's control input. This keeps the
                    // mission alive through transient infeasible/divergent
                    // frames (e.g. explorer/follower speed asymmetry).
                    if (solution.size() > 0) {
                        has_error = false;
                        return solution;
                    }
                    throw std::runtime_error("OSQP solver did not find optimal solution: " + status_str);
                }
            }

            has_error = false;
            last_error_code = 0;
            last_error_message = "";

            return solution;

        } catch (const std::exception& e) {
            has_error = true;
            last_error_code = -1;
            last_error_message = std::string("OSQP error: ") + e.what();
            // Debug mode: re-throw to stop program
            throw;
        } catch (...) {
            has_error = true;
            last_error_code = -2;
            last_error_message = "Unknown OSQP error";
            // Debug mode: re-throw to stop program
            throw;
        }
    }
};

#endif // CBF_OPTIMISER_OSQP_HPP
